#ifndef PROCESS_HPP
#define PROCESS_HPP

#include <cstdint>

// Common lifecycle for every long-lived firmware component. Process state
// describes whether the component's infrastructure is available; physical
// work (motion, measurement, regulation, scanning, bonding, ...) is tracked by
// the component itself and is deliberately independent from this state.
class Process {
public:
    enum class State : uint8_t {
        READY,
        OPERATING,
        ERROR
    };

    virtual ~Process() = default;

    // Idempotent lifecycle entry points. A failed onStart() leaves the process
    // in ERROR through setProcessError(); stop() performs cleanup and returns
    // either OPERATING or ERROR to READY.
    void start()
    {
        if (m_state != State::READY) return;
        m_state = State::OPERATING;
        onStart();
    }

    void execute()
    {
        if (m_state == State::OPERATING) onExecute();
    }

    void stop()
    {
        if (m_state == State::READY) return;
        onStop();
        m_state = State::READY;
    }

    State getProcessState() const { return m_state; }
    bool isReady() const { return m_state == State::READY; }
    bool isOperating() const { return m_state == State::OPERATING; }
    bool hasProcessError() const { return m_state == State::ERROR; }

protected:
    Process() = default;
    Process(const Process&) = delete;
    Process& operator=(const Process&) = delete;

    virtual void onStart() = 0;
    virtual void onExecute() {}
    virtual void onStop() = 0;

    void setProcessError() { m_state = State::ERROR; }

private:
    State m_state{State::READY};
};

#endif /* PROCESS_HPP */
