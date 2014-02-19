/*
 * Invalidation Priority Table
 *
 * Authors: Tao Chen
 */

#include "base/intmath.hh"
#include "mem/drop/ipt.hh"
#include "debug/Backtrack.hh"

#define SERIALIZE(x) os.write((const char*)&(x), sizeof(x));
#define UNSERIALIZE(x) is.read((char*)&(x), sizeof(x));

InvalidationPT::InvalidationPT(bool _tagged, unsigned _numEntries, unsigned _entrySize, unsigned _tagBits, unsigned _instShiftAmt)
	: numEntries(_numEntries), entrySize(_entrySize), tagBits(_tagBits), instShiftAmt(_instShiftAmt)
{
    if (!isPowerOf2(numEntries)) {
        fatal("IPT entries is not a power of 2!");
    }
    if (!isPowerOf2(entrySize)) {
        fatal("IPT entry size is not a power of 2!");
    }
    
    tagged = _tagged;

    offsetMask = entrySize - 1;

    idxMask = numEntries - 1;

    tagMask = (1 << tagBits) - 1;

    indexShiftAmt = instShiftAmt + floorLog2(entrySize);

    tagShiftAmt = indexShiftAmt + floorLog2(numEntries);
}

void
InvalidationPT::init()
{
    ipt.resize(numEntries);
    
    for (unsigned i = 0; i < numEntries; ++i) {
        ipt[i].valid = false;
        ipt[i].priority.resize(entrySize);
        for (unsigned j = 0; j < entrySize; ++j) {
            ipt[i].priority[j] = false;
        }
    }
}

void
InvalidationPT::reset()
{
    for (unsigned i = 0; i < numEntries; ++i) {
        ipt[i].valid = false;
        for (unsigned j = 0; j < entrySize; ++j) {
            ipt[i].priority[j] = false;
        }
    }
}

inline
unsigned
InvalidationPT::getOffset(Addr addr)
{
    return (addr >> instShiftAmt) & offsetMask;
}

inline
unsigned
InvalidationPT::getIndex(Addr addr)
{
    // Need to shift PC over by the word offset.
    return (addr >> indexShiftAmt) & idxMask;
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
    assert(tagged);

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
    unsigned offset = getOffset(addr);
    unsigned ipt_idx = getIndex(addr);

    if (tagged) {
        Addr tag = getTag(addr);

        assert(ipt_idx < numEntries);

        if (ipt[ipt_idx].valid
            && tag == ipt[ipt_idx].tag) {
            return ipt[ipt_idx].priority[offset];
        } else {
            return 0;
        }
    } else {
        return ipt[ipt_idx].priority[offset];
    }
}

void InvalidationPT::update(Addr addr, const bool priority)
{
    unsigned offset = getOffset(addr);
	unsigned ipt_idx = getIndex(addr);
    unsigned tag = getTag(addr);

	assert(ipt_idx < numEntries);

    if (tagged) {
        if (ipt[ipt_idx].valid) {
            if (tag == ipt[ipt_idx].tag) {
                ipt[ipt_idx].priority[offset] = priority;
            } else {
                // replace entry
                ipt[ipt_idx].tag = getTag(addr);
                for (unsigned i = 0; i < entrySize; ++i) {
                    ipt[ipt_idx].priority[i] = false;
                }
                ipt[ipt_idx].priority[offset] = priority;
            }
        } else {
            ipt[ipt_idx].valid = true;
            ipt[ipt_idx].tag = getTag(addr);
            for (unsigned i = 0; i < entrySize; ++i) {
                ipt[ipt_idx].priority[i] = false;
            }
            ipt[ipt_idx].priority[offset] = priority;
        }
    } else {
        ipt[ipt_idx].priority[offset] = priority;
    }
}

void InvalidationPT::serialize(std::ostream &os)
{
    // serialize parameters
    SERIALIZE(numEntries)
    SERIALIZE(entrySize)
    SERIALIZE(offsetMask)
    SERIALIZE(idxMask)
    SERIALIZE(tagBits)
    SERIALIZE(tagMask)
    SERIALIZE(instShiftAmt)
    SERIALIZE(indexShiftAmt)
    SERIALIZE(tagShiftAmt)
    // serialize the actual table
    for (unsigned i = 0; i < numEntries; ++i) {
        PTEntry &entry = ipt[i];
        SERIALIZE(entry.tag)
        SERIALIZE(entry.valid)
        for (int j = 0; j < entrySize; ++j) {
            bool tmp = entry.priority[j];
            SERIALIZE(tmp)
        }
    }
}

void InvalidationPT::unserialize(std::istream &is)
{
    // unserialize parameters
    UNSERIALIZE(numEntries)
    UNSERIALIZE(entrySize)
    UNSERIALIZE(offsetMask)
    UNSERIALIZE(idxMask)
    UNSERIALIZE(tagBits)
    UNSERIALIZE(tagMask)
    UNSERIALIZE(instShiftAmt)
    UNSERIALIZE(indexShiftAmt)
    UNSERIALIZE(tagShiftAmt)
    // unserialize the actual table
    ipt.resize(numEntries);
    int numImportantInsts = 0;
    for (unsigned i = 0; i < numEntries; ++i) {
        PTEntry &entry = ipt[i];
        UNSERIALIZE(entry.tag)
        UNSERIALIZE(entry.valid)
        entry.priority.resize(entrySize);
        for (int j = 0; j < entrySize; ++j) {
            bool tmp;
            UNSERIALIZE(tmp)
            entry.priority[j] = tmp;
            if (entry.valid && entry.priority[j]) {
                numImportantInsts++;
            }
        }
    }
    DPRINTF(Backtrack, "unserialized %d important instructions\n", numImportantInsts);
}
