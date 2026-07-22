#pragma once

#include "bonder_module.hpp"

// Service force measurement: the operator lowers Z under tracking force,
// releases the left button to hold each configured bond force for five
// seconds, and the protocol restores Z under constant force between stages.
class ForceSetupProtocol final : public BonderProtocol {
public:
    const Instruction *getProtocolPtr() const override;
    uint8_t getProtocolSize() const override;

private:
    static const Instruction s_protocol[];
};
