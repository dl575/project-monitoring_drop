/*
 * Invalidation Priority Table Implemented with Bloom filter
 *
 * Authors: Tao Chen
 */

#include "base/intmath.hh"
#include "mem/drop/ipt_bloom.hh"
#include "debug/Backtrack.hh"

#define SERIALIZE(x) os.write((const char*)&(x), sizeof(x));
#define UNSERIALIZE(x) is.read((char*)&(x), sizeof(x));

InvalidationPTBloom::InvalidationPTBloom(unsigned capacity, double falsePositiveRate)
{
    parameters.projected_element_count = capacity;
    parameters.false_positive_probability = falsePositiveRate;
}

void
InvalidationPTBloom::init()
{
    if (!parameters) {
        fatal("Invalid set of bloom filter parameters!");
    }

    parameters.compute_optimal_parameters();

    // Instantiate Bloom Filter
    filter = new bloom_filter(parameters);
}

void
InvalidationPTBloom::reset()
{
    delete filter;
    filter = new bloom_filter(parameters);
}

bool
InvalidationPTBloom::lookup(Addr addr)
{
    return filter->contains(addr);
}

void InvalidationPTBloom::update(Addr addr, const bool priority)
{
    filter->insert(addr);
}

void InvalidationPTBloom::serialize(std::ostream &os)
{
    // serialize parameters
    parameters.serialize(os);
    // serialize the actual bloom filter
    filter->serialize(os);
}

void InvalidationPTBloom::unserialize(std::istream &is)
{
    // unserialize parameters
    parameters.unserialize(is);
    // unserialize the actual bloom filter
    filter = new bloom_filter();
    filter->unserialize(is);
}
