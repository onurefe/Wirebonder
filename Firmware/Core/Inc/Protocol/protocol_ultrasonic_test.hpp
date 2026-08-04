#pragma once

#include "bonder_module.hpp"

// Ultrasonic-only diagnostic: scans the transducer, derives its operating
// point, transfers the configured first-bond energy, and publishes a report.
class UltrasonicTestProtocol final : public BonderProtocol {
public:
    const Instruction *getProtocolPtr() const override;
    uint8_t getProtocolSize() const override;
    bool requiresMotionControl() const override { return false; }

private:
    static const Instruction s_protocol[];
};
