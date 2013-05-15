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
#include "mem/fifo.hh"
#include "debug/Fifo.hh"

#include "arch/registers.hh"
#include "config/the_isa.hh"
#include "debug/MemoryAccess.hh"
//#include "mem/abstract_mem.hh"

using namespace std;

Fifo::Fifo(const Params* p) :
    AbstractMemory(p),
    lat(p->latency), lat_var(p->latency_var)
{
    for (size_t i = 0; i < p->port_port_connection_count; ++i) {
        ports.push_back(new MemoryPort(csprintf("%s-port-%d", name(), i),
                                       *this));
    }
    
    // Get fifo size
    fifo_size = p->fifo_size;
    if (fifo_size < 2) { fifo_size = 2; }
}

void
Fifo::init()
{
    for (vector<MemoryPort*>::iterator p = ports.begin(); p != ports.end();
         ++p) {
        if (!(*p)->isConnected()) {
            fatal("Fifo port %s is unconnected!\n", (*p)->name());
        } else {
            (*p)->sendRangeChange();
        }
    }

    // Initialize head/tail pointers
    head_pointer = 1;
    tail_pointer = 0;
    invalidPacket.init();
    
    fifo_array = new monitoringPacket[fifo_size];
    fifo_array[0].init();

}

Fifo::~Fifo()
{
    delete [] fifo_array;
}

Tick
Fifo::calculateLatency(PacketPtr pkt)
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
Fifo::doAtomicAccess(PacketPtr pkt)
{
    access(pkt);
    return calculateLatency(pkt);
}

void
Fifo::doFunctionalAccess(PacketPtr pkt)
{
    //functionalAccess(pkt);

    /* Based on AbstractMemory::functionalAccess */
    
    // Check that address is within bounds
    assert(pkt->getAddr() >= range.start &&
           (pkt->getAddr() + pkt->getSize() - 1) <= range.end);

    // "Local" address
    // uint8_t *hostAddr = pmemAddr + pkt->getAddr() - range.start;

    // Read request
    if (pkt->isRead()) {
        if (pmemAddr){
            //Get address we are reading
            Addr read_addr = pkt->getAddr();
            //This is the data we will send
            uint64_t send_data = 0;
            
            if (read_addr == FIFO_FULL) { send_data = full(); }
            else if (read_addr == FIFO_EMPTY) { send_data = empty(); }
            else {
                //This is the monitoring packet we will read
                monitoringPacket mp = invalidPacket;
                //if (!empty()) { mp = fifo_array[tail_pointer]; }
                
                //Skip invalid packets
                while (!empty() && !mp.valid) {
                    if (fifo_array[tail_pointer].valid){
                        mp = fifo_array[tail_pointer];
                    }else{
                        // Update tail_pointer
                        tail_pointer++;
                        tail_pointer %= fifo_size;

                        DPRINTF(Fifo, "Skipped invalid fifo packet, now head at %d, tail is at %d\n", head_pointer, tail_pointer);
                    }
                }
                
                //Set data based on address
                if (read_addr == FIFO_VALID) { send_data = mp.valid; }
                else if (read_addr == FIFO_INSTADDR) { send_data = mp.instAddr; }
                else if (read_addr == FIFO_MEMADDR) { send_data = mp.memAddr; }
                else if (read_addr == FIFO_MEMEND) { send_data = mp.memEnd; }
                else if (read_addr == FIFO_DATA) { send_data = mp.data; }
                else if (read_addr == FIFO_STORE) { send_data = mp.store; }
                else if (read_addr == FIFO_DONE) { send_data = mp.done; }
                else if (read_addr == FIFO_RS1) { send_data = mp.rs1; }
                else if (read_addr == FIFO_RS2) { send_data = mp.rs2; }
                else if (read_addr == FIFO_RS3) { send_data = mp.rs3; }
                else if (read_addr == FIFO_RD) { send_data = mp.rd; }
                else if (read_addr == FIFO_CONTROL) { send_data = mp.control; }
                else if (read_addr == FIFO_CALL) { send_data = mp.call; }
                else if (read_addr == FIFO_RET) { send_data = mp.ret; }
                else if (read_addr == FIFO_LR) { send_data = mp.lr; }
                else if (read_addr == FIFO_NEXTPC) { send_data = mp.nextpc; }
                else if (read_addr == FIFO_LOAD) { send_data = mp.load; }                
                else if (read_addr == FIFO_INTALU) { send_data = mp.intalu; }
                else if (read_addr == FIFO_INDCTRL) { send_data = mp.indctrl; }
                else {
                  warn("Unrecognized read from fifo address %x\n", read_addr);
                }
            }
            //Send data
            pkt->setData((uint8_t *)&send_data);
        }
        pkt->makeResponse();
    // Write request
    } else if (pkt->isWrite()) {
        if (pmemAddr) {
            //pop fifo
            if (!empty() && pkt->getAddr() == FIFO_NEXT){
                // Update tail_pointer
                tail_pointer++;
                tail_pointer %= fifo_size;
                
                DPRINTF(Fifo, "Pop fifo, now head at %d, tail is at %d\n", head_pointer, tail_pointer);
            // If there is space in the fifo
            } else if (!full() && pkt->getAddr() == FIFO_ADDR) {
                // Copy from packet to memory
                pkt->writeData((uint8_t *)(fifo_array + head_pointer));
                // Update head_pointer
                head_pointer++;
                head_pointer %= fifo_size;

                DPRINTF(Fifo, "Write at head %d, tail is at %d\n", head_pointer, tail_pointer);
            } else if (empty()){
                DPRINTF(Fifo, "Empty at head %d, tail is at %d\n", head_pointer, tail_pointer);
            } else if (full()){
                DPRINTF(Fifo, "Full at head %d, tail is at %d\n", head_pointer, tail_pointer);
            }
        }
        pkt->makeResponse();
    // Not used/implemented for fifo
    /*
    } else if (pkt->isPrint()) {
        Packet::PrintReqState *prs =
            dynamic_cast<Packet::PrintReqState*>(pkt->senderState);
        assert(prs);
        // Need to call printLabels() explicitly since we're not going
        // through printObj().
        prs->printLabels();
        // Right now we just print the single byte at the specified address.
        ccprintf(prs->os, "%s%#x\n", prs->curPrefix(), *hostAddr);
    */
    } else {
        panic("AbstractMemory: unimplemented functional command %s",
              pkt->cmdString());
    }

}

SlavePort &
Fifo::getSlavePort(const std::string &if_name, int idx)
{
    if (if_name != "port") {
        return MemObject::getSlavePort(if_name, idx);
    } else {
        if (idx >= static_cast<int>(ports.size())) {
            fatal("Fifo::getSlavePort: unknown index %d\n", idx);
        }

        return *ports[idx];
    }
}

unsigned int
Fifo::drain(Event *de)
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

Fifo::MemoryPort::MemoryPort(const std::string& _name,
                                     Fifo& _memory)
    : SimpleTimingPort(_name, &_memory), memory(_memory)
{ }

AddrRangeList
Fifo::MemoryPort::getAddrRanges()
{
    AddrRangeList ranges;
    ranges.push_back(memory.getAddrRange());
    return ranges;
}

Tick
Fifo::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return memory.doAtomicAccess(pkt);
}

void
Fifo::MemoryPort::recvFunctional(PacketPtr pkt)
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
Fifo::MemoryPort::recvTimingReq(PacketPtr pkt)
{
  // If full and writing
  if (memory.full() && (pkt->cmd == MemCmd::WriteReq)) {
    // Indicate unsuccessful send
    return false;
  // Otherwise, there's space
  } else {
    // Use functional function to handle writing
    recvFunctional(pkt);
    // Indicate write accepted
    return true;
  }
}

Fifo*
FifoParams::create()
{
    return new Fifo(this);
}
