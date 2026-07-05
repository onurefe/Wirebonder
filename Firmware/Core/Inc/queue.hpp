/***
 * @author     Onur Efe (C++ Adaptation) & Refactored for ISR Safety
 */
#ifndef QUEUE_HPP
#define QUEUE_HPP

#include "generic.h"

template <typename T>
class Queue {
private:
    // CRITICAL: Volatile ensures the compiler knows the ISR can change these at any time
    volatile uint16_t tail; 
    volatile uint16_t head;
    uint16_t bufferSize;
    T* pContainer;

    /***
     * @Brief      Highly optimized wrap to avoid hardware division.
     * Executes in ~2 clock cycles on modern ARM Cortex MCUs.
     */
    uint16_t wrap(uint16_t value) const {
        if (value >= bufferSize) {
            return value - bufferSize;
        }
        return value;
        
        // POWER-OF-TWO OPTIMIZATION NOTE:
        // If bufferSize was EXACTLY a power of 2 (e.g., 16, 32, 64), 
        // you could delete the 'if' and just use:
        // return value & (bufferSize - 1);
    }

public:
    /***
     * @Brief      Constructor binds a statically allocated array to the queue.
     * @Params     container -> Pointer to the external data array.
     * capacityInNumOfItems -> Maximum usable items.
     */
    Queue(T* container, uint16_t capacityInNumOfItems)
        : tail(0), head(0), bufferSize(capacityInNumOfItems + 1), pContainer(container) {}

    /***
     * @Brief      Clears the buffer pointers.
     */
    void clear() {
        head = 0;
        tail = 0;
    }

    /***
     * @Brief      Enqueues an element. Safe from overwriting unread data.
     * @Returns    True if successful, False if queue is full.
     */
    bool enqueue(T element) {
        if (isFull()) {
            return false; // Prevent overflow corruption
        }

        // 1. Write the data FIRST
        pContainer[tail] = element;
        
        // 2. Calculate the next index
        uint16_t nextTail = wrap(tail + 1);
        
        // 3. Update the global pointer LAST (Atomic commit)
        tail = nextTail; 
        
        return true;
    }

    /***
     * @Brief      Dequeues an element.
     * @Returns    The element, or a zeroed/default element if empty.
     */
    T dequeue() {
        if (isEmpty()) {
            return T(); // Prevent underflow (Returns 0 for numeric types)
        }

        // 1. Read the data FIRST
        T element = pContainer[head];
        
        // 2. Calculate the next index
        uint16_t nextHead = wrap(head + 1);
        
        // 3. Update the global pointer LAST (Atomic commit)
        head = nextHead;

        return element;
    }

    /***
     * @Brief      Discards elements from the front of the queue.
     * (Renamed from 'remove' for semantic clarity)
     */
    void discard(uint16_t count) {
        if (count <= getElementCount()) {
            uint16_t nextHead = wrap(head + count);
            head = nextHead;
        } else {
            clear(); // If asked to discard more than we have, just clear it
        }
    }

    /***
     * @Brief      Searches for an element. Returns index relative to head, or 0xFFFF if not found.
     */
    uint16_t search(const T& element) const {
        uint16_t num_of_elements = getElementCount();
        for (uint16_t i = 0; i < num_of_elements; i++) {
            if (peek(i) == element) {
                return i;
            }
        }
        return 0xFFFF;
    }

    /***
     * @Brief      Peeks at an element based on its relative index from the head.
     */
    T peek(uint16_t elementIndex) const {
        if (elementIndex >= getElementCount()) {
            return T(); // Out of bounds safety
        }
        uint16_t element_position = wrap(head + elementIndex);
        return pContainer[element_position];
    }

    T& front() {
        return pContainer[head];
    }

    /***
     * @Brief      Checks if the buffer is empty.
     */
    bool isEmpty() const {
        return (head == tail); // Faster than calling getElementCount() == 0
    }

    /***
     * @Brief      Checks if the buffer is full.
     */
    bool isFull() const {
        return (wrap(tail + 1) == head); // Faster than calling getAvailableSpace() == 0
    }

    /***
     * @Brief      Returns the number of empty slots remaining.
     */
    uint16_t getAvailableSpace() const {
        return ((bufferSize - 1) - getElementCount());
    }

    /***
     * @Brief      Returns the number of elements currently in the buffer.
     */
    uint16_t getElementCount() const {
        // Cache the volatile variables locally to prevent torn reads during math
        uint16_t currentTail = tail;
        uint16_t currentHead = head;

        if (currentTail >= currentHead) {
            return currentTail - currentHead;
        } else {
            return currentTail + bufferSize - currentHead;
        }
    }
};

#endif // QUEUE_HPP