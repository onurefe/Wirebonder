#pragma once

#include "Protocol/protocol_common.hpp"

// Ultrasonic bench test: one scan and one drive at the tail-assist settings,
// with the result reported.
class UltrasonicTestProtocol {
public:
    static const BonderModule::Instruction *getProtocolPtr();
    static uint8_t getProtocolSize();

private:
    static const BonderModule::Instruction s_protocol[];
};
