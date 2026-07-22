#pragma once

#include "bonder_module.hpp"

class TableTearBondingProtocol final : public BonderProtocol {
public:
    const BonderModule::Instruction *getProtocolPtr() const override;
    uint8_t getProtocolSize() const override;

private:
    static const Instruction s_protocol[];
};
