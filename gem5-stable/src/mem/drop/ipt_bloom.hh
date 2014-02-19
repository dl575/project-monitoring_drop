/*
 * Invalidation Priority Table Implemented with Bloom filter
 *
 * Authors: Tao Chen
 */

#ifndef __INVALIDATION_PT_BLOOM_HH__
#define __INVALIDATION_PT_BLOOM_HH__

#include <iostream>
#include <fstream>

#include "mem/drop/bloom_filter.hh"
#include "arch/types.hh"

class InvalidationPTBloom
{
  private:

  public:
    /**
     * Creates a invalidation priority table
     * @param capacity Projected element count
     * @param falsePositiveRate Max tolerable false positive rate
     */
    InvalidationPTBloom(unsigned capacity, double falsePositiveRate);

    void init();
    void reset();

    /**
     * Looks up an instruction address
     */
    bool lookup(Addr addr);

    /**
     * Update the priority of an instruction address
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

    /** The bloom filter */
    bloom_parameters parameters;
    bloom_filter *filter;
};

#endif // __INVALIDATION_PT_BLOOM_HH__
