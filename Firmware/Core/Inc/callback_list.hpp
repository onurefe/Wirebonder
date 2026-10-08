/**
 * @author     Onur Efe
 *
 * Fixed-capacity callback registries with symmetric add/remove.
 *
 * Every callback in the firmware is a raw `void *context` + function pointer
 * pair. This header holds the one implementation of that pattern so listeners
 * can be attached and detached at runtime without the hand-rolled
 * array+count code each site used to carry.
 *
 * Concurrency contract
 * --------------------
 * The system is a bare-metal superloop. Every peripheral IRQ sits at NVIC
 * preempt priority 0, so no ISR can preempt another ISR; SysTick is at 15.
 *
 *   Mutation (add/remove/clear) holds InterruptLock for the whole operation.
 *   Masking on the main loop is therefore sufficient to make a mutation atomic
 *   with respect to every dispatch context. The std::atomic slot state carries
 *   no cross-core meaning here -- it exists to stop the compiler hoisting the
 *   callback/context stores past the store that publishes the slot.
 *
 *   Dispatch (invoke/invokeFirst) takes no lock. It walks slot indices up to a
 *   high-water bound and skips slots that are not Active.
 *
 * Removal tombstones the slot; it never compacts. Two things follow, and both
 * are relied upon by callers:
 *
 *   - A callback may remove itself, or any other listener, while a dispatch is
 *     in progress. The index walk is unaffected, so no listener is skipped and
 *     a listener removed earlier in the same dispatch is not called.
 *   - Registration order is preserved across removals. This matters for
 *     ArbiterList, whose first-true-wins semantics make order load-bearing.
 *
 * Whether an add() performed during a dispatch is observed by that same
 * dispatch is unspecified.
 */

#ifndef CALLBACK_LIST_HPP
#define CALLBACK_LIST_HPP

#include <atomic>
#include <cstdint>

#include "generic.h"

namespace callback_detail {

constexpr uint8_t kSlotFree   = 0U;
constexpr uint8_t kSlotActive = 1U;

// Shared slot storage and registration logic. Dispatch semantics live in the
// derived ListenerListN / ArbiterListN.
template <typename R, uint8_t Capacity, typename... Args>
class CallbackSlots {
public:
    using Callback = R (*)(void *, Args...);

    static constexpr uint8_t kCapacity = Capacity;

    CallbackSlots(const CallbackSlots &) = delete;
    CallbackSlots &operator=(const CallbackSlots &) = delete;

    // Registering the same (context, callback) twice is a no-op that reports
    // success -- several call sites re-register on every start().
    // Returns false only for a null callback or a full list.
    bool add(void *context, Callback callback)
    {
        if (callback == nullptr) {
            return false;
        }

        InterruptLock lock;

        uint8_t freeIndex = Capacity;

        for (uint8_t i = 0U; i < Capacity; i++) {
            if (m_slots[i].state.load(std::memory_order_relaxed) == kSlotActive) {
                if (m_slots[i].context == context && m_slots[i].callback == callback) {
                    return true;
                }
            } else if (freeIndex == Capacity) {
                freeIndex = i;
            }
        }

        if (freeIndex == Capacity) {
            return false;
        }

        m_slots[freeIndex].callback = callback;
        m_slots[freeIndex].context  = context;
        m_slots[freeIndex].state.store(kSlotActive, std::memory_order_release);

        if (freeIndex >= m_highWater.load(std::memory_order_relaxed)) {
            m_highWater.store(static_cast<uint8_t>(freeIndex + 1U), std::memory_order_release);
        }

        return true;
    }

    // Safe to call from inside a dispatch of this same list.
    bool remove(void *context, Callback callback)
    {
        if (callback == nullptr) {
            return false;
        }

        InterruptLock lock;

        for (uint8_t i = 0U; i < Capacity; i++) {
            if (m_slots[i].state.load(std::memory_order_relaxed) != kSlotActive) {
                continue;
            }
            if (m_slots[i].context != context || m_slots[i].callback != callback) {
                continue;
            }

            m_slots[i].state.store(kSlotFree, std::memory_order_release);
            m_slots[i].callback = nullptr;
            m_slots[i].context  = nullptr;
            shrinkHighWater();
            return true;
        }

        return false;
    }

    void clear()
    {
        InterruptLock lock;

        for (uint8_t i = 0U; i < Capacity; i++) {
            m_slots[i].state.store(kSlotFree, std::memory_order_release);
            m_slots[i].callback = nullptr;
            m_slots[i].context  = nullptr;
        }
        m_highWater.store(0U, std::memory_order_release);
    }

    bool contains(void *context, Callback callback) const
    {
        for (uint8_t i = 0U; i < Capacity; i++) {
            if (m_slots[i].state.load(std::memory_order_acquire) != kSlotActive) {
                continue;
            }
            if (m_slots[i].context == context && m_slots[i].callback == callback) {
                return true;
            }
        }
        return false;
    }

    uint8_t size() const
    {
        uint8_t count = 0U;
        for (uint8_t i = 0U; i < Capacity; i++) {
            if (m_slots[i].state.load(std::memory_order_acquire) == kSlotActive) {
                count++;
            }
        }
        return count;
    }

    bool isEmpty() const { return m_highWater.load(std::memory_order_acquire) == 0U; }

protected:
    CallbackSlots() = default;

    struct Slot {
        Callback              callback{nullptr};
        void                 *context{nullptr};
        std::atomic<uint8_t>  state{kSlotFree};
    };

    // Caller must already hold InterruptLock.
    void shrinkHighWater()
    {
        uint8_t high = 0U;
        for (uint8_t i = 0U; i < Capacity; i++) {
            if (m_slots[i].state.load(std::memory_order_relaxed) == kSlotActive) {
                high = static_cast<uint8_t>(i + 1U);
            }
        }
        m_highWater.store(high, std::memory_order_release);
    }

    Slot                 m_slots[Capacity];
    std::atomic<uint8_t> m_highWater{0U};
};

}  // namespace callback_detail

// Default slot count. Generous on purpose: overflow is silent at several call
// sites and the RAM is not scarce (12 bytes per slot).
constexpr uint8_t kDefaultCallbackCapacity = 8U;

// Fan-out list: every registered listener is called, in registration order.
template <uint8_t Capacity, typename... Args>
class ListenerListN : public callback_detail::CallbackSlots<void, Capacity, Args...> {
    using Base = callback_detail::CallbackSlots<void, Capacity, Args...>;

public:
    using Callback = typename Base::Callback;

    void invoke(Args... args) const
    {
        const uint8_t high = this->m_highWater.load(std::memory_order_acquire);

        for (uint8_t i = 0U; i < high; i++) {
            if (this->m_slots[i].state.load(std::memory_order_acquire) != callback_detail::kSlotActive) {
                continue;
            }

            Callback callback = this->m_slots[i].callback;
            void    *context  = this->m_slots[i].context;

            if (callback != nullptr) {
                callback(context, args...);
            }
        }
    }
};

// Arbitration list: registrants are polled in registration order and the first
// one that claims the call (returns true) wins. Used for the "first active
// controller wins" setpoint/duty/frequency controllers.
template <uint8_t Capacity, typename... Args>
class ArbiterListN : public callback_detail::CallbackSlots<bool, Capacity, Args...> {
    using Base = callback_detail::CallbackSlots<bool, Capacity, Args...>;

public:
    using Callback = typename Base::Callback;

    bool invokeFirst(Args... args) const
    {
        const uint8_t high = this->m_highWater.load(std::memory_order_acquire);

        for (uint8_t i = 0U; i < high; i++) {
            if (this->m_slots[i].state.load(std::memory_order_acquire) != callback_detail::kSlotActive) {
                continue;
            }

            Callback callback = this->m_slots[i].callback;
            void    *context  = this->m_slots[i].context;

            if (callback != nullptr && callback(context, args...)) {
                return true;
            }
        }

        return false;
    }
};

// A parameter pack has to come last, so the capacity cannot carry a default on
// the class templates themselves. These aliases are what call sites use.
template <typename... Args>
using ListenerList = ListenerListN<kDefaultCallbackCapacity, Args...>;

template <typename... Args>
using ArbiterList = ArbiterListN<kDefaultCallbackCapacity, Args...>;

#endif /* CALLBACK_LIST_HPP */
