/*
 * Invalidation Metadata Table
 *
 * Authors: Tao Chen
 */

#ifndef __INSTRUCTION_METADATA_TABLE_HH__
#define __INSTRUCTION_METADATA_TABLE_HH__

#include <iostream>
#include <fstream>

#include "arch/types.hh"

template <typename MetadataType>
class InstructionMetadataTable
{
  private:
    struct Entry
    {
        Entry() : tag(0), valid(false)
        {}
        // tag of entry
        Addr tag;
        // the entry's metadata
        std::vector<MetadataType*> metadata;
        // whether the entry is valid
        bool valid;
    };

  public:
    /**
     * Creates a instruction metadata table
     * @param numEntries Number of entries
     * @param entrySize Size of entry
     * @param tagBits Number of bits in each tag
     * @param instShiftAmt Amount to shift instructions
     */
    InstructionMetadataTable(bool tagged, unsigned numEntries, unsigned entrySize, unsigned tagBits, unsigned instShiftAmt);

    void init();
    void reset();

    /**
     * Looks up a instruction address in the table, must call valid() first
     */
    MetadataType* lookup(Addr addr);

    /**
     * Checks if an instruction is in the table
     */
    bool valid(Addr addr);

    /**
     * Update the metadata of an instruction
     */
    void update(Addr addr, MetadataType *metadata);

    /**
     * Serialize the table to an output stream
     */
    void serialize(std::ostream &os);

    /**
     * Unserialize the table from an input stream
     */
    void unserialize(std::istream &is);

  private:
    /**
     * Return the offset within an entry
     */
    inline unsigned getOffset(Addr addr);

    /**
     * Return the index into the table
     */
    inline unsigned getIndex(Addr addr);

    /**
     * Return the tag bits for a given address
     */
    inline Addr getTag(Addr addr);

    /** The actual table */
    std::vector<Entry> imt;

    /** Whether the table has tags */
    bool tagged;

    /** The number of entries in the table. */
    unsigned numEntries;

    /** Intructions per entry **/
    unsigned entrySize;

    /** The offset mask. */
    unsigned offsetMask;

    /** The index mask. */
    unsigned idxMask;

    /** The number of tag bits per entry. */
    unsigned tagBits;

    /** The tag mask. */
    unsigned tagMask;

    /** Number of bits to shift PC for instruction. */
    unsigned instShiftAmt;

    /** Number of bits to shift PC when calculating index. */
    unsigned indexShiftAmt;

    /** Number of bits to shift PC when calculating tag. */
    unsigned tagShiftAmt;

  public:
    /**
     * Getters
     */
    unsigned getNumEntries() { return numEntries; }
    unsigned getEntrySize() { return entrySize; }
    unsigned getInstShiftAmt() { return instShiftAmt; }
};

#endif // __INSTRUCTION_METADATA_TABLE_HH__
