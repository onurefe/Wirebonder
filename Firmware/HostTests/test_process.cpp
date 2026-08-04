#include <gtest/gtest.h>

#include "process.hpp"

namespace {

class TestProcess final : public Process {
public:
    void failNextStart() { m_failNextStart = true; }

    int starts{0};
    int executions{0};
    int stops{0};

private:
    void onStart() override
    {
        ++starts;
        if (m_failNextStart) {
            m_failNextStart = false;
            setProcessError();
        }
    }

    void onExecute() override { ++executions; }
    void onStop() override { ++stops; }

    bool m_failNextStart{false};
};

TEST(ProcessTest, GatesExecutionWithLifecycleState)
{
    TestProcess process;

    EXPECT_TRUE(process.isReady());
    process.execute();
    EXPECT_EQ(process.executions, 0);

    process.start();
    EXPECT_TRUE(process.isOperating());
    process.execute();
    EXPECT_EQ(process.executions, 1);

    process.stop();
    EXPECT_TRUE(process.isReady());
    process.execute();
    EXPECT_EQ(process.executions, 1);
    EXPECT_EQ(process.starts, 1);
    EXPECT_EQ(process.stops, 1);
}

TEST(ProcessTest, ErrorRequiresStopBeforeRestart)
{
    TestProcess process;
    process.failNextStart();

    process.start();
    EXPECT_TRUE(process.hasProcessError());
    process.execute();
    EXPECT_EQ(process.executions, 0);

    process.start();
    EXPECT_EQ(process.starts, 1);

    process.stop();
    EXPECT_TRUE(process.isReady());
    process.start();
    EXPECT_TRUE(process.isOperating());
    EXPECT_EQ(process.starts, 2);
}

} // namespace
