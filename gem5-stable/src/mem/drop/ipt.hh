/*
 * Invalidation Priority Table
 *
 * Authors: Tao Chen
 */

#ifndef __INVALIDATION_PT_HH__
#define __INVALIDATION_PT_HH__

#include <iostream>
#include <fstream>

#include "arch/types.hh"

class InvalidationPT
{
  private:
    struct PTEntry
    {
        PTEntry() : tag(0), priority(false), valid(false)
        {}
        // tag of entry
        Addr tag;
        // the entry's priority
        bool priority;
        // whether the entry is valid
        bool valid;
    };

  public:
    /**
     * Creates a invalidation priority table
     * @param numEntries Number of entries
     * @param tagBits Number of bits in each tag
     * @param instShiftAmt Amount to shift instructions
     */
    InvalidationPT(unsigned numEntries, unsigned tagBits, unsigned instShiftAmt);

    void init();
    void reset();

    /**
     * Looks up a memory address in the PTB, must call valid() first
     */
    bool lookup(Addr addr);

    /**
     * Checks if an instruction is in the PTB
     */
    bool valid(Addr addr);

    /**
     * Update the priority of an memory location
     */
    void update(Addr addr, const bool priority);

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
     * Return the index into the PTB
     */
    inline unsigned getIndex(Addr addr);

    /**
     * Return the tag bits for a given address
     */
    inline Addr getTag(Addr addr);

    /** The actual table */
    std::vector<PTEntry> ipt;

    /** The number of entries in the table. */
    unsigned numEntries;

    /** The index mask. */
    unsigned idxMask;

    /** The number of tag bits per entry. */
    unsigned tagBits;

    /** The tag mask. */
    unsigned tagMask;

    /** Number of bits to shift PC when calculating index. */
    unsigned instShiftAmt;

    /** Number of bits to shift PC when calculating tag. */
    unsigned tagShiftAmt;
};

#endif // __INVALIDATION_PT_HH__
