/*
 * Register Producer Tracking Table
 *
 * Authors: Tao Chen
 */

#include "mem/drop/rptb.hh"

RegisterPTB::RegisterPTB()
    : numEntries(TheISA::NUM_INTREGS)
{
    for (unsigned i = 0; i < numEntries; ++i) {
        ptb[i].valid = false;
    }
}

void
RegisterPTB::reset()
{
    for (unsigned i = 0; i < numEntries; ++i) {
        ptb[i].valid = false;
    }
}

bool
RegisterPTB::valid(int reg)
{
    if (ptb[reg].valid) {
        return true;
    } else {
        return false;
    }
}

Addr
RegisterPTB::lookup1(int reg)
{
    assert(reg < numEntries);

    if (ptb[reg].valid) {
        return ptb[reg].producerPC1;
    } else {
        return 0;
    }
}

Addr
RegisterPTB::lookup2(int reg)
{
    assert(reg < numEntries);

    if (ptb[reg].valid) {
        return ptb[reg].producerPC2;
    } else {
        return 0;
    }
}

void RegisterPTB::update(int reg, const Addr producerPC1, const Addr producerPC2)
{
	assert(reg < numEntries);

	ptb[reg].valid = true;
    ptb[reg].producerPC1 = producerPC1;
	ptb[reg].producerPC2 = producerPC2;
}
