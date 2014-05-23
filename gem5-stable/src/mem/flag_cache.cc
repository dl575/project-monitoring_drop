/*
 * Copyright (c) 2010-2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Ron Dreslinski
 *          Ali Saidi
 *          Andreas Hansson
 */

/*
 * Based on simple_mem.cc
 *
 * Author: Daniel Lo
 */

#include "base/random.hh"
#include "mem/flag_cache.hh"
#include "debug/FlagCache.hh"

#include "arch/registers.hh"
#include "config/the_isa.hh"
#include "debug/MemoryAccess.hh"
//#include "mem/abstract_mem.hh"

using namespace std;

FlagCache::FlagCache(const Params* p) :
    AbstractMemory(p),
    lat(p->latency), lat_var(p->latency_var),
    addr(), last_access()
{
    for (size_t i = 0; i < p->port_port_connection_count; ++i) {
        ports.push_back(new MemoryPort(csprintf("%s-port-%d", name(), i),
                                       *this));
    }

    // Create flag array
    fa_size = p->flagarr_size;
    if (fa_size < 1) fa_size = 1;
    flag_array = new char[fa_size];
    for (int i = 0; i < fa_size; ++i){
        flag_array[i] = 0;
    }
    
    // Create bloom filter
    // unsigned capacity = p->bloom_cap;
    // float error_rate = p->bloom_err;
    // std::string bloom_file = p->bloom_file;
    // if (!(bloom = new_counting_bloom(capacity, error_rate, bloom_file.data()))) {
        // panic("Could not create bloom filter\n");
    // }
    
    // Create flag cache
    fc_size = p->flagcache_size;
    num_ways = p->flagcache_ways;
    if (fc_size < 1) fc_size = 1;
    if (num_ways < 1) num_ways = 1;
    cache_array = new cacheLine[fc_size];
    for(int i = 0; i < fc_size; ++i){
        cache_array[i].tags = new Addr[num_ways];
        cache_array[i].values = new char[num_ways];
        cache_array[i].num_valid = 0;
        cache_array[i].aliased = false;
    }
    
}

FlagCache::~FlagCache()
{
    delete [] flag_array;
    // free_counting_bloom(bloom);
    for (int i = 0; i < fc_size; ++i){
        delete [] cache_array[i].tags;
    }
    delete [] cache_array;
}

void
FlagCache::init()
{
    for (vector<MemoryPort*>::iterator p = ports.begin(); p != ports.end();
         ++p) {
        if (!(*p)->isConnected()) {
            fatal("FlagCache port %s is unconnected!\n", (*p)->name());
        } else {
            (*p)->sendRangeChange();
        }
    }
}

void
FlagCache::regStats()
{
    AbstractMemory::regStats();
    
    num_aliased
        .name(name()+".num_aliased")
        .desc("Number of flag cache accesses which aliased")
        ;

    numRegLoads
        .name(name() + ".flagRegLoads")
        .desc("Number of loads from flag register/array")
        ;

    numRegStores
        .name(name() + ".flagRegStores")
        .desc("Number of stores to flag register/array")
        ;

    numCacheLoads
        .name(name() + ".flagCacheLoads")
        .desc("Number of loads from flag cache")
        ;

    numCacheStores
        .name(name() + ".flagCacheStores")
        .desc("Number of stores to flag cache")
        ;
}

Tick
FlagCache::calculateLatency(PacketPtr pkt)
{
    if (pkt->memInhibitAsserted()) {
        return 0;
    } else {
        Tick latency = lat;
        if (lat_var != 0)
            latency += random_mt.random<Tick>(0, lat_var);
        return latency;
    }
}

Tick
FlagCache::doAtomicAccess(PacketPtr pkt)
{
    access(pkt);
    return calculateLatency(pkt);
}

void
FlagCache::doFunctionalAccess(PacketPtr pkt)
{
    /* Based on AbstractMemory::functionalAccess */
    
    // Check that address is within bounds
    assert(pkt->getAddr() >= range.start &&
           (pkt->getAddr() + pkt->getSize() - 1) <= range.end);
  
    // Read request
    if (pkt->isRead()) {
        if (pmemAddr){
            //Get address we are reading
            Addr read_addr = pkt->getAddr();
            //This is the data we will send
            uint64_t send_data = 0;
            
            //Set data based on address
            if (read_addr == FC_GET_ADDR) { 
                send_data = addr;
                DPRINTF(FlagCache, "Get flag cache address = %x\n", addr);
            // Read from flag array
            } else if (read_addr == FC_GET_FLAG_A){
                if (addr >= 0 && addr < fa_size){
                    // Lookup flag
                    send_data = flag_array[addr];
                    DPRINTF(FlagCache, "Flag array lookup @ %x: value = %d\n", addr, send_data);
                    numRegLoads++;
                }
            // Read from flag cache
            } else if (read_addr == FC_GET_FLAG_C) { 
                // Check bloom filter with address
                //bool found = counting_bloom_check(bloom, (const char *)&addr, sizeof(addr));
                //send_data = found;
                //DPRINTF(FlagCache, "Bloom filter lookup @ %x: found? %d\n", addr, found);
                Addr cA = cacheAddr(addr);
                // Get cache line
                cacheLine * cL = &cache_array[cA];
                // Associative lookup
                char value = 0;
                for (unsigned i = 0; i < cL->num_valid; ++i){
                    if (cL->tags[i] == addr){
                        value = cL->values[i];
                    }
                }
                send_data = (cL->aliased)? FC_MAXVAL : value;
                if (cL->aliased && curTick() != last_access) {
                    num_aliased++;
                    last_access = curTick();
                }
                DPRINTF(FlagCache, "Flag cache lookup @ %x -> %x: aliased? %d, value = %d, num_aliased = %d\n", addr, cA, cL->aliased, (char)send_data, num_aliased.value());
                numCacheLoads++;
            } else if (read_addr == FC_ALIASED) {
                send_data = num_aliased.value();
            } else {
              warn("Unrecognized read from flag cache read address %x\n", read_addr);
            }
            //Send data
            pkt->setData((uint8_t *)&send_data);
        }
        pkt->makeResponse();
    // Write request
    } else if (pkt->isWrite()) {
        if (pmemAddr) {
            // Get address we are writing
            Addr write_addr = pkt->getAddr();
            // Read data value
            uint64_t get_data = 0;
            pkt->writeData((uint8_t *)&get_data);
            // Write address register
            if (write_addr == FC_SET_ADDR) {
                addr = get_data;
                DPRINTF(FlagCache, "Set flag cache address = %x\n", addr);
            // Set flag
            } else if (write_addr == FC_SET_FLAG) {
                warn_once("Using FC_SET_FLAG is depricated. Use the new FlagCache interface.\n");
                // Set in flag array
                if (get_data == FC_ARRAY) {
                    if(addr >= 0 && addr < fa_size){
                        flag_array[addr] = FC_MAXVAL;
                        DPRINTF(FlagCache, "Flag array set @ %x\n", addr);
                        numRegStores++;
                    } else {
                        DPRINTF(FlagCache, "Flag array not set @ %x\n", addr);
                    }
                // Set in flag cache
                } else if (get_data == FC_CACHE) {
                    // Set bloom filter with address
                    // counting_bloom_add(bloom, (const char *)&addr, sizeof(addr));
                    // DPRINTF(FlagCache, "Bloom filter set @ %x\n", addr);
                    Addr cA = cacheAddr(addr);
                    // Get cache line
                    cacheLine * cL = &cache_array[cA];
                    // Cache isn't aliased
                    if (cL->num_valid <= num_ways) {
                        bool found = false;
                        // Associative lookup
                        for (unsigned i = 0; i < cL->num_valid; ++i){
                            if (cL->tags[i] == addr){
                                found = true;
                                cL->values[i] = FC_MAXVAL;
                            }
                        }
                        // Set cache line
                        if (!found){
                            // Cache has space
                            if (cL->num_valid < num_ways){
                                cL->tags[cL->num_valid] = addr;
                                cL->values[cL->num_valid] = FC_MAXVAL;
                                DPRINTF(FlagCache, "Flag cache set @ %x -> %x: tag set at way %d as %x\n", addr, cA, cL->num_valid, cL->tags[cL->num_valid]);
                            } else {
                                cL->aliased = true;
                                DPRINTF(FlagCache, "Flag cache set @ %x -> %x: cache is now aliased\n", addr, cA);
                            }
                            cL->num_valid++;
                        } else {
                            DPRINTF(FlagCache, "Flag cache set @ %x -> %x: tag exists\n", addr, cA);
                        }
                    } else {
                        DPRINTF(FlagCache, "Flag set @ %x -> %x: cache is aliased\n", addr, cA);
                    }
                    numCacheStores++;
                }
            // Clear flag
            } else if (write_addr == FC_CLEAR_FLAG) {
                warn_once("Using FC_CLEAR_FLAG is depricated. Use the new FlagCache interface.\n");
                // Clear in flag array
                if (get_data == FC_ARRAY) {
                    if(addr >= 0 && addr < fa_size){
                        flag_array[addr] = 0;
                        DPRINTF(FlagCache, "Flag array clear @ %x\n", addr);
                        numRegStores++;
                    } else {
                        DPRINTF(FlagCache, "Flag array not cleared @ %x\n", addr);
                    }
                // Clear in flag cache
                } else if (get_data == FC_CACHE) {
                    // Clear bloom filter with address
                    // counting_bloom_remove(bloom, (const char *)&addr, sizeof(addr));
                    // DPRINTF(FlagCache, "Bloom filter clear @ %x\n", addr);
                    Addr cA = cacheAddr(addr);
                    // Get cache line
                    cacheLine * cL = &cache_array[cA];
                    // Cache isn't aliased
                    if (cL->num_valid <= num_ways) {
                        bool found = false;
                        // Associative lookup
                        for (unsigned i = 0; i < cL->num_valid; ++i){
                            // Shift items back
                            if (found){
                                cL->tags[i-1] = cL->tags[i];
                                cL->values[i-1] = cL->values[i];
                            }
                            // Found tag
                            if (cL->tags[i] == addr){
                                found = true;
                                DPRINTF(FlagCache, "Flag cache clear @ %x -> %x: found tag at %d\n", addr, cA, i);
                            }
                        }
                        // Decrement number of valids
                        if (found){
                            cL->num_valid--;
                        } else {
                            DPRINTF(FlagCache, "Flag cache clear @ %x -> %x: did not find tag\n", addr, cA);
                        }
                    } else {
                        DPRINTF(FlagCache, "Flag cache clear @ %x -> %x: cache is aliased\n", addr, cA);
                    }
                    numCacheStores++;
                }
            } else if (write_addr == FC_SET_ARRAY) {
                // Set array value to the value set in get_data
                if(addr >= 0 && addr < fa_size){
                    flag_array[addr] = (get_data > FC_MAXVAL)? FC_MAXVAL : get_data;
                    DPRINTF(FlagCache, "Flag array set value %d @ %x\n", flag_array[addr], addr);
                    numRegStores++;
                } else {
                    DPRINTF(FlagCache, "Flag array not set @ %x\n", addr);
                }
            } else if (write_addr == FC_SET_CACHE) {
                // Set cache value to the value set in get_data
                Addr cA = cacheAddr(addr);
                // Get cache line
                cacheLine * cL = &cache_array[cA];
                // Cache isn't aliased
                if (cL->num_valid <= num_ways) {
                    // Perform clear operation - remove entry
                    if (get_data == 0){
                        bool found = false;
                        // Associative lookup
                        for (unsigned i = 0; i < cL->num_valid; ++i){
                            // Shift items back
                            if (found){
                                cL->tags[i-1] = cL->tags[i];
                                cL->values[i-1] = cL->values[i];
                            }
                            // Found tag
                            if (cL->tags[i] == addr){
                                found = true;
                                DPRINTF(FlagCache, "Flag cache set value 0 @ %x -> %x: remove tag at %d\n", addr, cA, i);
                            }
                        }
                        // Decrement number of valids
                        if (found){
                            cL->num_valid--;
                        } else {
                            DPRINTF(FlagCache, "Flag cache set value 0 @ %x -> %x: did not find tag\n", addr, cA);
                        }
                    // Write data to cache - update existing or add new
                    } else {
                        bool found = false;
                        // Make sure value does not exceed set maximum
                        char store_value = (get_data > FC_MAXVAL)? FC_MAXVAL : get_data;
                        // Associative lookup
                        for (unsigned i = 0; i < cL->num_valid; ++i){
                            // Update existing
                            if (cL->tags[i] == addr){
                                found = true;
                                cL->values[i] = store_value;
                            }
                        }
                        // Add new entry line
                        if (!found){
                            // Cache has space
                            if (cL->num_valid < num_ways){
                                cL->tags[cL->num_valid] = addr;
                                cL->values[cL->num_valid] = store_value;
                                DPRINTF(FlagCache, "Flag cache set value %d @ %x -> %x: add new tag at %d\n", store_value, addr, cA, cL->num_valid);
                            } else {
                                cL->aliased = true;
                                DPRINTF(FlagCache, "Flag cache set value %d @ %x -> %x: cache is now aliased\n", store_value, addr, cA);
                            }
                            cL->num_valid++;
                        } else {
                            DPRINTF(FlagCache, "Flag cache set value %d @ %x -> %x: update at existing tag\n", store_value, addr, cA);
                        }
                    }
                } else {
                    DPRINTF(FlagCache, "Flag cache set @ %x -> %x: cache is aliased\n", addr, cA);
                }
                numCacheStores++;
            } else {
              warn("Unknown address written to flag cache.");
            }
        }
        pkt->makeResponse();
    } else {
        panic("AbstractMemory: unimplemented functional command %s",
              pkt->cmdString());
    }

}

SlavePort &
FlagCache::getSlavePort(const std::string &if_name, int idx)
{
    if (if_name != "port") {
        return MemObject::getSlavePort(if_name, idx);
    } else {
        if (idx >= static_cast<int>(ports.size())) {
            fatal("FlagCache::getSlavePort: unknown index %d\n", idx);
        }

        return *ports[idx];
    }
}

unsigned int
FlagCache::drain(Event *de)
{
    int count = 0;
    for (vector<MemoryPort*>::iterator p = ports.begin(); p != ports.end();
         ++p) {
        count += (*p)->drain(de);
    }

    if (count)
        changeState(Draining);
    else
        changeState(Drained);
    return count;
}

FlagCache::MemoryPort::MemoryPort(const std::string& _name,
                                     FlagCache& _memory)
    : SimpleTimingPort(_name, &_memory), memory(_memory)
{ }

AddrRangeList
FlagCache::MemoryPort::getAddrRanges()
{
    AddrRangeList ranges;
    ranges.push_back(memory.getAddrRange());
    return ranges;
}

Tick
FlagCache::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return memory.doAtomicAccess(pkt);
}

void
FlagCache::MemoryPort::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(memory.name());

    if (!queue.checkFunctional(pkt)) {
        // Default implementation of SimpleTimingPort::recvFunctional()
        // calls recvAtomic() and throws away the latency; we can save a
        // little here by just not calculating the latency.
        memory.doFunctionalAccess(pkt);
    }

    pkt->popLabel();
}

bool
FlagCache::MemoryPort::recvTimingReq(PacketPtr pkt)
{
  // Use functional function to handle writing
  recvFunctional(pkt);
  // Indicate write accepted
  return true;
}

FlagCache*
FlagCacheParams::create()
{
    return new FlagCache(this);
}
