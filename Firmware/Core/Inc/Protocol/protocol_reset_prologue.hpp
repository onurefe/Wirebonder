#pragma once

#include "bonder_module.hpp"

class ProtocolResetPrologue final : public BonderProtocol {
public:
    const Instruction *getProtocolPtr() const override;
    uint8_t getProtocolSize() const override;
    // SETFORCE and ZMOVE below need the force coil and Z position loops
    // running, so this inherits the default requiresMotionControl() == true.

private:
    static const Instruction s_protocol[];
};