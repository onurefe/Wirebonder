#include <gtest/gtest.h>
#include "queue.hpp"

class QueueTest : public ::testing::Test {
protected:
    static constexpr uint16_t CAP = 4;
    int buf[CAP + 1];
    Queue<int> q{buf, CAP};
    void SetUp() override { q.clear(); }
};

TEST_F(QueueTest, InitiallyEmpty) {
    EXPECT_TRUE(q.isEmpty());
    EXPECT_FALSE(q.isFull());
    EXPECT_EQ(q.getElementCount(), 0u);
}

TEST_F(QueueTest, EnqueueDequeue) {
    q.enqueue(42);
    EXPECT_EQ(q.dequeue(), 42);
    EXPECT_TRUE(q.isEmpty());
}

TEST_F(QueueTest, FifoOrdering) {
    q.enqueue(1); q.enqueue(2); q.enqueue(3);
    EXPECT_EQ(q.dequeue(), 1);
    EXPECT_EQ(q.dequeue(), 2);
    EXPECT_EQ(q.dequeue(), 3);
}

TEST_F(QueueTest, OverflowReturnsFalse) {
    for (int i = 0; i < 4; i++) q.enqueue(i);
    EXPECT_TRUE(q.isFull());
    EXPECT_FALSE(q.enqueue(99));
    EXPECT_EQ(q.getElementCount(), 4u);
}

TEST_F(QueueTest, UnderflowReturnsDefault) {
    EXPECT_EQ(q.dequeue(), 0);
    EXPECT_TRUE(q.isEmpty());
}

TEST_F(QueueTest, WrapAround) {
    for (int i = 0; i < 4; i++) q.enqueue(i * 10);
    for (int i = 0; i < 4; i++) q.dequeue();
    for (int i = 0; i < 4; i++) EXPECT_TRUE(q.enqueue(i + 100));
    EXPECT_EQ(q.dequeue(), 100);
}

TEST_F(QueueTest, PeekDoesNotConsume) {
    q.enqueue(7); q.enqueue(8);
    EXPECT_EQ(q.peek(0), 7);
    EXPECT_EQ(q.peek(1), 8);
    EXPECT_EQ(q.getElementCount(), 2u);
}

TEST_F(QueueTest, SearchFound) {
    q.enqueue(10); q.enqueue(20); q.enqueue(30);
    EXPECT_EQ(q.search(20), 1u);
}

TEST_F(QueueTest, SearchNotFound) {
    q.enqueue(10);
    EXPECT_EQ(q.search(99), 0xFFFFu);
}

TEST_F(QueueTest, DiscardReducesCount) {
    q.enqueue(1); q.enqueue(2); q.enqueue(3);
    q.discard(2);
    EXPECT_EQ(q.getElementCount(), 1u);
    EXPECT_EQ(q.dequeue(), 3);
}

TEST_F(QueueTest, DiscardMoreThanAvail) {
    q.enqueue(1); q.enqueue(2);
    q.discard(10);
    EXPECT_TRUE(q.isEmpty());
}

TEST_F(QueueTest, AvailableSpace) {
    EXPECT_EQ(q.getAvailableSpace(), 4u);
    q.enqueue(1);
    EXPECT_EQ(q.getAvailableSpace(), 3u);
}
