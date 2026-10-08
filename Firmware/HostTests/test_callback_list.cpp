#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "callback_list.hpp"

namespace {

std::vector<std::string> g_log;

struct Sink {
    const char *name;
    explicit Sink(const char *n) : name(n) {}

    static void onValue(void *ctx, float v)
    {
        g_log.push_back(std::string(static_cast<Sink *>(ctx)->name) + "=" +
                        std::to_string(static_cast<int>(v)));
    }

    static void onOther(void *ctx, float)
    {
        g_log.push_back(std::string(static_cast<Sink *>(ctx)->name) + ":other");
    }
};

using FloatList = ListenerList<float>;

class CallbackListTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        g_log.clear();
        g_hostPrimask = 0U;
    }
};

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

TEST_F(CallbackListTest, RejectsNullCallback)
{
    FloatList list;
    Sink      a("a");

    EXPECT_FALSE(list.add(&a, nullptr));
    EXPECT_FALSE(list.remove(&a, nullptr));
    EXPECT_EQ(list.size(), 0U);
    EXPECT_TRUE(list.isEmpty());
}

// Several call sites re-register on every start(); a repeat must report success
// without consuming a second slot.
TEST_F(CallbackListTest, DuplicateRegistrationIsIdempotent)
{
    FloatList list;
    Sink      a("a");

    EXPECT_TRUE(list.add(&a, &Sink::onValue));
    EXPECT_TRUE(list.add(&a, &Sink::onValue));
    EXPECT_EQ(list.size(), 1U);

    g_log.clear();
    list.invoke(1.0f);
    EXPECT_EQ(g_log.size(), 1U);
}

TEST_F(CallbackListTest, SameContextDifferentCallbackAreDistinct)
{
    FloatList list;
    Sink      a("a");

    EXPECT_TRUE(list.add(&a, &Sink::onValue));
    EXPECT_TRUE(list.add(&a, &Sink::onOther));
    EXPECT_EQ(list.size(), 2U);
}

TEST_F(CallbackListTest, RejectsRegistrationWhenFull)
{
    FloatList         list;
    std::vector<Sink> sinks(FloatList::kCapacity + 4U, Sink("x"));

    for (uint8_t i = 0U; i < FloatList::kCapacity; i++) {
        EXPECT_TRUE(list.add(&sinks[i], &Sink::onValue)) << "slot " << i;
    }
    EXPECT_EQ(list.size(), FloatList::kCapacity);
    EXPECT_FALSE(list.add(&sinks[FloatList::kCapacity], &Sink::onValue));
}

TEST_F(CallbackListTest, DefaultCapacityIsEight)
{
    EXPECT_EQ(FloatList::kCapacity, 8U);
    EXPECT_EQ(kDefaultCallbackCapacity, 8U);
}

TEST_F(CallbackListTest, ExplicitCapacityIsHonoured)
{
    ListenerListN<2, float> small;
    Sink                    a("a"), b("b"), c("c");

    EXPECT_TRUE(small.add(&a, &Sink::onValue));
    EXPECT_TRUE(small.add(&b, &Sink::onValue));
    EXPECT_FALSE(small.add(&c, &Sink::onValue));
}

// ---------------------------------------------------------------------------
// Removal
// ---------------------------------------------------------------------------

TEST_F(CallbackListTest, RemoveUnregisteredReportsFailure)
{
    FloatList list;
    Sink      a("a");

    EXPECT_FALSE(list.remove(&a, &Sink::onValue));
}

TEST_F(CallbackListTest, RemoveDropsListenerAndIsNotRepeatable)
{
    FloatList list;
    Sink      a("a"), b("b");

    list.add(&a, &Sink::onValue);
    list.add(&b, &Sink::onValue);

    EXPECT_TRUE(list.remove(&a, &Sink::onValue));
    EXPECT_FALSE(list.remove(&a, &Sink::onValue));
    EXPECT_EQ(list.size(), 1U);
    EXPECT_FALSE(list.contains(&a, &Sink::onValue));
    EXPECT_TRUE(list.contains(&b, &Sink::onValue));

    list.invoke(1.0f);
    ASSERT_EQ(g_log.size(), 1U);
    EXPECT_EQ(g_log[0], "b=1");
}

TEST_F(CallbackListTest, FreedSlotIsReused)
{
    FloatList list;
    Sink      a("a"), b("b"), c("c");

    list.add(&a, &Sink::onValue);
    list.add(&b, &Sink::onValue);
    list.remove(&a, &Sink::onValue);
    EXPECT_TRUE(list.add(&c, &Sink::onValue));
    EXPECT_EQ(list.size(), 2U);

    // c took the freed slot 0, so it dispatches ahead of b.
    list.invoke(1.0f);
    ASSERT_EQ(g_log.size(), 2U);
    EXPECT_EQ(g_log[0], "c=1");
    EXPECT_EQ(g_log[1], "b=1");
}

// Tombstoning rather than compacting is what keeps ArbiterList's priority
// ordering stable when a mid-list registrant goes away.
TEST_F(CallbackListTest, RegistrationOrderSurvivesMiddleRemoval)
{
    FloatList list;
    Sink      a("a"), b("b"), c("c");

    list.add(&a, &Sink::onValue);
    list.add(&b, &Sink::onValue);
    list.add(&c, &Sink::onValue);
    list.remove(&b, &Sink::onValue);

    list.invoke(2.0f);
    ASSERT_EQ(g_log.size(), 2U);
    EXPECT_EQ(g_log[0], "a=2");
    EXPECT_EQ(g_log[1], "c=2");
}

TEST_F(CallbackListTest, ClearDropsEverything)
{
    FloatList list;
    Sink      a("a"), b("b");

    list.add(&a, &Sink::onValue);
    list.add(&b, &Sink::onValue);
    list.clear();

    EXPECT_EQ(list.size(), 0U);
    EXPECT_TRUE(list.isEmpty());
    list.invoke(1.0f);
    EXPECT_TRUE(g_log.empty());

    // Cleared list is still usable.
    EXPECT_TRUE(list.add(&a, &Sink::onValue));
    EXPECT_EQ(list.size(), 1U);
}

// ---------------------------------------------------------------------------
// Re-entrancy: this is the property that lets a command de-register itself
// from the callback it is currently running in.
// ---------------------------------------------------------------------------

FloatList *g_reentrantList = nullptr;

struct SelfRemover {
    static void run(void *ctx, float)
    {
        g_log.push_back("self");
        g_reentrantList->remove(ctx, &SelfRemover::run);
    }
};

struct Victim {
    static void run(void *, float) { g_log.push_back("victim"); }
};

struct Killer {
    void *victimCtx;
    static void run(void *ctx, float)
    {
        g_log.push_back("killer");
        g_reentrantList->remove(static_cast<Killer *>(ctx)->victimCtx, &Victim::run);
    }
};

TEST_F(CallbackListTest, ListenerRemovingItselfDoesNotSkipSuccessor)
{
    FloatList list;
    g_reentrantList = &list;
    int  selfCtx    = 0;
    Sink tail("tail");

    list.add(&selfCtx, &SelfRemover::run);
    list.add(&tail, &Sink::onValue);

    list.invoke(3.0f);
    ASSERT_EQ(g_log.size(), 2U);
    EXPECT_EQ(g_log[0], "self");
    EXPECT_EQ(g_log[1], "tail=3");
    EXPECT_EQ(list.size(), 1U);

    g_log.clear();
    list.invoke(3.0f);
    ASSERT_EQ(g_log.size(), 1U);
    EXPECT_EQ(g_log[0], "tail=3");

    g_reentrantList = nullptr;
}

TEST_F(CallbackListTest, ListenerRemovingLaterListenerPreventsThatCall)
{
    FloatList list;
    g_reentrantList = &list;
    int    victimCtx = 0;
    Killer killer{&victimCtx};
    Sink   tail("tail");

    list.add(&killer, &Killer::run);
    list.add(&victimCtx, &Victim::run);
    list.add(&tail, &Sink::onValue);

    list.invoke(4.0f);
    ASSERT_EQ(g_log.size(), 2U);
    EXPECT_EQ(g_log[0], "killer");
    EXPECT_EQ(g_log[1], "tail=4");

    g_reentrantList = nullptr;
}

// ---------------------------------------------------------------------------
// ArbiterList
// ---------------------------------------------------------------------------

using DutyArbiter = ArbiterList<float *>;

struct Controller {
    const char *name;
    bool        claims;
    float       value;

    static bool run(void *ctx, float *out)
    {
        auto *self = static_cast<Controller *>(ctx);
        g_log.push_back(std::string("poll:") + self->name);
        if (!self->claims) {
            return false;
        }
        *out = self->value;
        return true;
    }
};

TEST_F(CallbackListTest, ArbiterStopsAtFirstClaimant)
{
    DutyArbiter arb;
    Controller  passive{"passive", false, 0.0f};
    Controller  first{"first", true, 1.5f};
    Controller  second{"second", true, 9.9f};

    arb.add(&passive, &Controller::run);
    arb.add(&first, &Controller::run);
    arb.add(&second, &Controller::run);

    float out = 0.0f;
    EXPECT_TRUE(arb.invokeFirst(&out));
    EXPECT_FLOAT_EQ(out, 1.5f);
    ASSERT_EQ(g_log.size(), 2U);  // "second" is never polled
    EXPECT_EQ(g_log[0], "poll:passive");
    EXPECT_EQ(g_log[1], "poll:first");
}

TEST_F(CallbackListTest, ArbiterPromotesNextClaimantAfterRemoval)
{
    DutyArbiter arb;
    Controller  passive{"passive", false, 0.0f};
    Controller  first{"first", true, 1.5f};
    Controller  second{"second", true, 9.9f};

    arb.add(&passive, &Controller::run);
    arb.add(&first, &Controller::run);
    arb.add(&second, &Controller::run);
    EXPECT_TRUE(arb.remove(&first, &Controller::run));

    float out = 0.0f;
    EXPECT_TRUE(arb.invokeFirst(&out));
    EXPECT_FLOAT_EQ(out, 9.9f);
    ASSERT_EQ(g_log.size(), 2U);
    EXPECT_EQ(g_log[0], "poll:passive");
    EXPECT_EQ(g_log[1], "poll:second");
}

TEST_F(CallbackListTest, ArbiterLeavesOutputAloneWhenNobodyClaims)
{
    DutyArbiter arb;
    Controller  passive{"passive", false, 0.0f};
    arb.add(&passive, &Controller::run);

    float out = -1.0f;
    EXPECT_FALSE(arb.invokeFirst(&out));
    EXPECT_FLOAT_EQ(out, -1.0f);

    DutyArbiter empty;
    EXPECT_FALSE(empty.invokeFirst(&out));
    EXPECT_FLOAT_EQ(out, -1.0f);
}

// ---------------------------------------------------------------------------
// Signature shapes actually used by the firmware
// ---------------------------------------------------------------------------

TEST_F(CallbackListTest, SupportsMultiArgumentSignatures)
{
    struct Multi {
        static void run(void *, uint16_t *buf, uint32_t n)
        {
            g_log.push_back("multi:" + std::to_string(buf[0]) + "," + std::to_string(n));
        }
    };

    ListenerList<uint16_t *, uint32_t> raw;
    uint16_t                           buf[2] = {7U, 8U};

    raw.add(&buf, &Multi::run);
    raw.invoke(buf, 2U);
    ASSERT_EQ(g_log.size(), 1U);
    EXPECT_EQ(g_log[0], "multi:7,2");
}

TEST_F(CallbackListTest, SupportsZeroArgumentSignatures)
{
    struct Tick {
        static void run(void *) { g_log.push_back("tick"); }
    };

    ListenerList<> ticks;
    int            ctx = 0;

    ticks.add(&ctx, &Tick::run);
    ticks.invoke();
    EXPECT_EQ(g_log.size(), 1U);
}

// ---------------------------------------------------------------------------
// InterruptLock -- the registries' whole race-freedom argument rests on this.
// ---------------------------------------------------------------------------

TEST_F(CallbackListTest, InterruptLockRestoresPriorMask)
{
    g_hostPrimask = 0U;
    {
        InterruptLock outer;
        EXPECT_EQ(g_hostPrimask, 1U);
        {
            InterruptLock inner;
            EXPECT_EQ(g_hostPrimask, 1U);
        }
        // A nested lock must not unmask while the outer one is still live.
        EXPECT_EQ(g_hostPrimask, 1U);
    }
    EXPECT_EQ(g_hostPrimask, 0U);
}

TEST_F(CallbackListTest, InterruptLockTakenFromMaskedContextStaysMasked)
{
    g_hostPrimask = 1U;
    {
        InterruptLock lock;
        EXPECT_EQ(g_hostPrimask, 1U);
    }
    EXPECT_EQ(g_hostPrimask, 1U);
    g_hostPrimask = 0U;
}

TEST_F(CallbackListTest, MutationLeavesMaskAsItFoundIt)
{
    FloatList list;
    Sink      a("a");

    list.add(&a, &Sink::onValue);
    EXPECT_EQ(g_hostPrimask, 0U);
    list.remove(&a, &Sink::onValue);
    EXPECT_EQ(g_hostPrimask, 0U);
    list.clear();
    EXPECT_EQ(g_hostPrimask, 0U);
}

}  // namespace
