/*
 * Memory Producer Tracking Table
 *
 * Authors: Tao Chen
 */

#ifndef __MEMORY_PTB_HH__
#define __MEMORY_PTB_HH__

#include <iostream>
#include <fstream>

#include "arch/types.hh"

class MemoryPTB
{
  private:
    struct PTBEntry
    {
        PTBEntry() : tag(0), producerPC(0), valid(false)
        {}
        // tag of entry
        Addr tag;
        // the entry's producer
        Addr producerPC;
        // whether the entry is valid
        bool valid;
    };

  public:
    /**
     * Creates a memory producer tracking table
     * @param numEntries Number of entries
     * @param tagBits Number of bits in each tag
     * @param instShiftAmt Amount to shift instructions
     */
    MemoryPTB(unsigned numEntries, unsigned tagBits, unsigned instShiftAmt);

    void reset();

    /**
     * Looks up a memory address in the PTB, must call valid() first
     */
    Addr lookup(Addr addr);

    /**
     * Checks if an instruction is in the PTB
     */
    bool valid(Addr addr);

    /**
     * Update the producer of an memory location
     */
    void update(Addr addr, const Addr producerPC);

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

    /** The actual PTB */
    std::vector<PTBEntry> ptb;

    /** The number of entries in the PTB. */
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

#endif // __MEMORY_PTB_HH__