/*
 * Invalidation Metadata Table
 *
 * Authors: Tao Chen
 */

#include "base/intmath.hh"
#include "mem/drop/imt.hh"
#include "debug/Backtrack.hh"

#define SERIALIZE(x) os.write((const char*)&(x), sizeof(x));
#define UNSERIALIZE(x) is.read((char*)&(x), sizeof(x));

template <typename MetadataType>
InstructionMetadataTable<MetadataType>::InstructionMetadataTable(bool _tagged, unsigned _numEntries, unsigned _entrySize, unsigned _tagBits, unsigned _instShiftAmt)
	: numEntries(_numEntries), entrySize(_entrySize), tagBits(_tagBits), instShiftAmt(_instShiftAmt)
{
    if (!isPowerOf2(numEntries)) {
        fatal("IMT entries is not a power of 2!");
    }
    if (!isPowerOf2(entrySize)) {
        fatal("IMT entry size is not a power of 2!");
    }
    
    tagged = _tagged;

    offsetMask = entrySize - 1;

    idxMask = numEntries - 1;

    tagMask = (1 << tagBits) - 1;

    indexShiftAmt = instShiftAmt + floorLog2(entrySize);

    tagShiftAmt = indexShiftAmt + floorLog2(numEntries);
}

template <typename MetadataType>
void
InstructionMetadataTable<MetadataType>::init()
{
    imt.resize(numEntries);
    
    for (unsigned i = 0; i < numEntries; ++i) {
        imt[i].valid = false;
        imt[i].metadata.resize(entrySize);
        for (unsigned j = 0; j < entrySize; ++j) {
            imt[i].metadata[j] = NULL;
        }
    }
}

template <typename MetadataType>
void
InstructionMetadataTable<MetadataType>::reset()
{
    for (unsigned i = 0; i < numEntries; ++i) {
        imt[i].valid = false;
        for (unsigned j = 0; j < entrySize; ++j) {
            imt[i].metadata[j] = NULL;
        }
    }
}

template <typename MetadataType>
inline unsigned
InstructionMetadataTable<MetadataType>::getOffset(Addr addr)
{
    return (addr >> instShiftAmt) & offsetMask;
}

template <typename MetadataType>
inline unsigned
InstructionMetadataTable<MetadataType>::getIndex(Addr addr)
{
    // Need to shift PC over by the word offset.
    return (addr >> indexShiftAmt) & idxMask;
}

template <typename MetadataType>
inline Addr
InstructionMetadataTable<MetadataType>::getTag(Addr addr)
{
    return (addr >> tagShiftAmt) & tagMask;
}

template <typename MetadataType>
bool
InstructionMetadataTable<MetadataType>::valid(Addr addr)
{
    assert(tagged);

    unsigned imt_idx = getIndex(addr);

    Addr tag = getTag(addr);

    assert(imt_idx < numEntries);

    if (imt[imt_idx].valid
        && tag == imt[imt_idx].tag) {
        return true;
    } else {
        return false;
    }
}

template <typename MetadataType>
MetadataType*
InstructionMetadataTable<MetadataType>::lookup(Addr addr)
{
    unsigned offset = getOffset(addr);
    unsigned imt_idx = getIndex(addr);

    if (tagged) {
        Addr tag = getTag(addr);

        assert(imt_idx < numEntries);

        if (imt[imt_idx].valid
            && tag == imt[imt_idx].tag) {
            return imt[imt_idx].metadata[offset];
        } else {
            return 0;
        }
    } else {
        return imt[imt_idx].metadata[offset];
    }
}

template <typename MetadataType>
void
InstructionMetadataTable<MetadataType>::update(Addr addr, MetadataType *metadata)
{
    unsigned offset = getOffset(addr);
	unsigned imt_idx = getIndex(addr);
    unsigned tag = getTag(addr);

	assert(imt_idx < numEntries);

    if (tagged) {
        if (imt[imt_idx].valid) {
            if (tag == imt[imt_idx].tag) {
                imt[imt_idx].metadata[offset] = metadata;
            } else {
                // replace entry
                imt[imt_idx].tag = getTag(addr);
                for (unsigned i = 0; i < entrySize; ++i) {
                    imt[imt_idx].metadata[i] = NULL;
                }
                imt[imt_idx].metadata[offset] = metadata;
            }
        } else {
            imt[imt_idx].valid = true;
            imt[imt_idx].tag = getTag(addr);
            for (unsigned i = 0; i < entrySize; ++i) {
                imt[imt_idx].metadata[i] = NULL;
            }
            imt[imt_idx].metadata[offset] = metadata;
        }
    } else {
        imt[imt_idx].metadata[offset] = metadata;
    }
}

template <typename MetadataType>
void
InstructionMetadataTable<MetadataType>::serialize(std::ostream &os)
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
        Entry &entry = imt[i];
        SERIALIZE(entry.tag)
        SERIALIZE(entry.valid)
        for (int j = 0; j < entrySize; ++j) {
            MetadataType *tmp = entry.metadata[j];
            SERIALIZE(*tmp)
        }
    }
}

template <typename MetadataType>
void
InstructionMetadataTable<MetadataType>::unserialize(std::istream &is)
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
    imt.resize(numEntries);
    for (unsigned i = 0; i < numEntries; ++i) {
        Entry &entry = imt[i];
        UNSERIALIZE(entry.tag)
        UNSERIALIZE(entry.valid)
        entry.metadata.resize(entrySize);
        for (int j = 0; j < entrySize; ++j) {
            MetadataType *tmp = new MetadataType;
            UNSERIALIZE(*tmp)
            entry.metadata[j] = tmp;
        }
    }
}
