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
    lat(p->latency), lat_var(p->latency_var)
{
    for (size_t i = 0; i < p->port_port_connection_count; ++i) {
        ports.push_back(new MemoryPort(csprintf("%s-port-%d", name(), i),
                                       *this));
    }

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

    // Initialize cache
    for (unsigned i = 0; i < FC_SIZE; ++i){
        cache_array[i].init();
    }
    // Initialize address register
    addr = 0;
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
            } else if (read_addr == FC_GET_FLAG_A){
                Addr aA = arrayAddr(addr);
                // Lookup flag
                send_data = flag_array[aA];
                DPRINTF(FlagCache, "Flag array lookup @ %x -> %x: value? %d\n", addr, aA, send_data);
            } else if (read_addr == FC_GET_FLAG_C) { 
                Addr cA = cacheAddr(addr);
                // Get cache line
                cacheLine * cL = &cache_array[cA];
                // Associative lookup
                bool found = false;
                for (unsigned i = 0; i < cL->num_valid; ++i){
                    if (cL->tags[i] == addr){
                        found = true;
                    }
                }
                send_data = (cL->aliased || found);
                DPRINTF(FlagCache, "Flag cache lookup @ %x -> %x: found? %d, aliased? %d\n", addr, cA, found, cL->aliased);
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
                if (get_data == 0) {
                    Addr aA = arrayAddr(addr);
                    flag_array[aA] = true;
                    DPRINTF(FlagCache, "Flag array set @ %x -> %x\n", addr, aA);
                } else if (get_data == 1) {
                    Addr cA = cacheAddr(addr);
                    // Get cache line
                    cacheLine * cL = &cache_array[cA];
                    // Cache isn't aliased
                    if (cL->num_valid <= FC_NUM_WAYS) {
                        bool found = false;
                        // Associative lookup
                        for (unsigned i = 0; i < cL->num_valid; ++i){
                            if (cL->tags[i] == addr){
                                found = true;
                            }
                        }
                        // Set cache line
                        if (!found){
                            // Cache has space
                            if (cL->num_valid < FC_NUM_WAYS){
                                cL->tags[cL->num_valid] = addr;
                                DPRINTF(FlagCache, "Flag cache set @ %x -> %x: tag set at way %d as %x\n", addr, cA, cL->num_valid, cL->tags[cL->num_valid]);
                            } else {
                                cL->aliased = true;
                                DPRINTF(FlagCache, "Flag cache set @ %x -> %x: cache is now aliased\n", addr, cA);
                            }
                            cL->num_valid++;
                        } else {
                            DPRINTF(FlagCache, "Flag cache set @ %x -> %x: tag already exists\n", addr, cA);
                        }
                    } else {
                        DPRINTF(FlagCache, "Flag set @ %x -> %x: cache is already aliased\n", addr, cA);
                    }
                }
            // Clear flag
            } else if (write_addr == FC_CLEAR_FLAG) {
                if (get_data == 0) {
                    Addr aA = arrayAddr(addr);
                    flag_array[aA] = false;
                    DPRINTF(FlagCache, "Flag array clear @ %x -> %x\n", addr, aA);
                } else if (get_data == 1) {
                    Addr cA = cacheAddr(addr);
                    // Get cache line
                    cacheLine * cL = &cache_array[cA];
                    // Cache isn't aliased
                    if (cL->num_valid <= FC_NUM_WAYS) {
                        bool found = false;
                        // Associative lookup
                        for (unsigned i = 0; i < cL->num_valid; ++i){
                            // Shift items back
                            if (found){
                                cL->tags[i-1] = cL->tags[i];
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
                }
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
