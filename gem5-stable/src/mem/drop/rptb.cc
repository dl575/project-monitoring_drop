/*
 * Register Producer Tracking Table
 *
 * Authors: Tao Chen
 */

#include "mem/drop/rptb.hh"

#define SERIALIZE(x) os.write((const char*)&x, sizeof(x));
#define UNSERIALIZE(x) is.read((char*)&x, sizeof(x));

RegisterPTB::RegisterPTB()
    : numEntries(TheISA::NUM_INTREGS)
{
}

void
RegisterPTB::init()
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

void RegisterPTB::serialize(std::ostream &os)
{
    // serialize parameters
    SERIALIZE(numEntries)
    // serialize the actual table
    for (unsigned i = 0; i < numEntries; ++i) {
        PTBEntry &entry = ptb[i];
        SERIALIZE(entry.producerPC1)
        SERIALIZE(entry.producerPC2)
        SERIALIZE(entry.valid)
    }
}

void RegisterPTB::unserialize(std::istream &is)
{
    // unserialize parameters
    UNSERIALIZE(numEntries)
    // unserialize the actual table
    for (unsigned i = 0; i < numEntries; ++i) {
        PTBEntry &entry = ptb[i];
        UNSERIALIZE(entry.producerPC1)
        UNSERIALIZE(entry.producerPC2)
        UNSERIALIZE(entry.valid)
    }
}
