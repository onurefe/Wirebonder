#pragma once

#include "bonder_module.hpp"

// Service force measurement. Waits for the left button, then holds the
// machine-wide setup tracking force (MachineSettingsData::
// forceSetupTrackingForce) while the operator drives Z down onto an external
// gauge; releasing the button ends the protocol and retracts to reset height.
// The reading the operator then enters becomes
// MachineSettingsData::forceCoilForceOffset.
class ForceSetupProtocol final : public BonderProtocol {
public:
    const Instruction *getProtocolPtr() const override;
    uint8_t getProtocolSize() const override;

private:
    static const Instruction s_protocol[];
};
