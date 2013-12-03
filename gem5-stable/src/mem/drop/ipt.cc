/*
 * Invalidation Priority Table
 *
 * Authors: Tao Chen
 */

#include "base/intmath.hh"
#include "mem/drop/ipt.hh"
#include "debug/Backtrack.hh"

#define SERIALIZE(x) os.write((const char*)&x, sizeof(x));
#define UNSERIALIZE(x) is.read((char*)&x, sizeof(x));

InvalidationPT::InvalidationPT(unsigned _numEntries, unsigned _tagBits, unsigned _instShiftAmt)
	: numEntries(_numEntries), tagBits(_tagBits), instShiftAmt(_instShiftAmt)
{
    if (!isPowerOf2(numEntries)) {
        fatal("IPT entries is not a power of 2!");
    }
    
    idxMask = numEntries - 1;

    tagMask = (1 << tagBits) - 1;

    tagShiftAmt = instShiftAmt + floorLog2(numEntries);
}

void
InvalidationPT::init()
{
    ipt.resize(numEntries);
    
    for (unsigned i = 0; i < numEntries; ++i) {
        ipt[i].valid = false;
    }
}

void
InvalidationPT::reset()
{
    for (unsigned i = 0; i < numEntries; ++i) {
        ipt[i].valid = false;
    }
}

inline
unsigned
InvalidationPT::getIndex(Addr addr)
{
    // Need to shift PC over by the word offset.
    return (addr >> instShiftAmt) & idxMask;
}

inline
Addr
InvalidationPT::getTag(Addr addr)
{
    return (addr >> tagShiftAmt) & tagMask;
}

bool
InvalidationPT::valid(Addr addr)
{
    unsigned ipt_idx = getIndex(addr);

    Addr tag = getTag(addr);

    assert(ipt_idx < numEntries);

    if (ipt[ipt_idx].valid
        && tag == ipt[ipt_idx].tag) {
        return true;
    } else {
        return false;
    }
}

bool
InvalidationPT::lookup(Addr addr)
{
    unsigned ipt_idx = getIndex(addr);

    Addr tag = getTag(addr);

    assert(ipt_idx < numEntries);

    if (ipt[ipt_idx].valid
        && tag == ipt[ipt_idx].tag) {
        return ipt[ipt_idx].priority;
    } else {
        return 0;
    }
}

void InvalidationPT::update(Addr addr, const bool priority)
{
	unsigned ipt_idx = getIndex(addr);

	assert(ipt_idx < numEntries);

	ipt[ipt_idx].valid = true;
	ipt[ipt_idx].priority = priority;
	ipt[ipt_idx].tag = getTag(addr);
}

void InvalidationPT::serialize(std::ostream &os)
{
    // serialize parameters
    SERIALIZE(numEntries)
    SERIALIZE(idxMask)
    SERIALIZE(tagBits)
    SERIALIZE(tagMask)
    SERIALIZE(instShiftAmt)
    SERIALIZE(tagShiftAmt)
    // serialize the actual table
    for (unsigned i = 0; i < numEntries; ++i) {
        PTEntry &entry = ipt[i];
        SERIALIZE(entry.tag)
        SERIALIZE(entry.priority)
        SERIALIZE(entry.valid)
        if (entry.valid && entry.priority)
            DPRINTF(Backtrack, "serialize important instruction 0x%08x\n",
                entry.tag << tagShiftAmt | i << instShiftAmt);
    }
}

void InvalidationPT::unserialize(std::istream &is)
{
    // unserialize parameters
    UNSERIALIZE(numEntries)
    UNSERIALIZE(idxMask)
    UNSERIALIZE(tagBits)
    UNSERIALIZE(tagMask)
    UNSERIALIZE(instShiftAmt)
    UNSERIALIZE(tagShiftAmt)
    // unserialize the actual table
    ipt.resize(numEntries);
    for (unsigned i = 0; i < numEntries; ++i) {
        PTEntry &entry = ipt[i];
        UNSERIALIZE(entry.tag)
        UNSERIALIZE(entry.priority)
        UNSERIALIZE(entry.valid)
        if (entry.valid && entry.priority)
            DPRINTF(Backtrack, "unserialize important instruction 0x%08x\n",
                entry.tag << tagShiftAmt | i << instShiftAmt);
    }
}
