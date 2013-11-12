/*
 * Memory Producer Tracking Table
 *
 * Authors: Tao Chen
 */

#include "base/intmath.hh"
#include "mem/drop/mptb.hh"

MemoryPTB::MemoryPTB(unsigned _numEntries, unsigned _tagBits, unsigned _instShiftAmt)
	: numEntries(_numEntries), tagBits(_tagBits), instShiftAmt(_instShiftAmt)
{
    if (!isPowerOf2(numEntries)) {
        fatal("PTB entries is not a power of 2!");
    }
    
    ptb.resize(numEntries);
    
    for (unsigned i = 0; i < numEntries; ++i) {
        ptb[i].valid = false;
    }

    idxMask = numEntries - 1;

    tagMask = (1 << tagBits) - 1;

    tagShiftAmt = instShiftAmt + floorLog2(numEntries);
}

void
MemoryPTB::reset()
{
    for (unsigned i = 0; i < numEntries; ++i) {
        ptb[i].valid = false;
    }
}

inline
unsigned
MemoryPTB::getIndex(Addr addr)
{
    // Need to shift PC over by the word offset.
    return (addr >> instShiftAmt) & idxMask;
}

inline
Addr
MemoryPTB::getTag(Addr addr)
{
    return (addr >> tagShiftAmt) & tagMask;
}

bool
MemoryPTB::valid(Addr addr)
{
    unsigned ptb_idx = getIndex(addr);

    Addr tag = getTag(addr);

    assert(ptb_idx < numEntries);

    if (ptb[ptb_idx].valid
        && tag == ptb[ptb_idx].tag) {
        return true;
    } else {
        return false;
    }
}

Addr
MemoryPTB::lookup(Addr addr)
{
    unsigned ptb_idx = getIndex(addr);

    Addr tag = getTag(addr);

    assert(ptb_idx < numEntries);

    if (ptb[ptb_idx].valid
        && tag == ptb[ptb_idx].tag) {
        return ptb[ptb_idx].producerPC;
    } else {
        return 0;
    }
}

void MemoryPTB::update(Addr addr, const Addr producerPC)
{
	unsigned ptb_idx = getIndex(addr);

	assert(ptb_idx < numEntries);

	ptb[ptb_idx].valid = true;
	ptb[ptb_idx].producerPC = producerPC;
	ptb[ptb_idx].tag = getTag(addr);
}
