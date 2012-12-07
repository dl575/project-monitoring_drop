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
 */

#include "base/random.hh"
#include "mem/timer.hh"

#include "debug/SlackTimer.hh"

using namespace std;

Timer::Timer(const Params* p) :
    AbstractMemory(p),
    lat(p->latency), lat_var(p->latency_var)
{
    for (size_t i = 0; i < p->port_port_connection_count; ++i) {
        ports.push_back(new MemoryPort(csprintf("%s-port-%d", name(), i),
                                       *this));
    }
}

void
Timer::init()
{
    for (vector<MemoryPort*>::iterator p = ports.begin(); p != ports.end();
         ++p) {
        if (!(*p)->isConnected()) {
            fatal("Timer port %s is unconnected!\n", (*p)->name());
        } else {
            (*p)->sendRangeChange();
        }
    }

    // Initialize stored_tp
    stored_tp.init();
}

Tick
Timer::calculateLatency(PacketPtr pkt)
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
Timer::doAtomicAccess(PacketPtr pkt)
{
    access(pkt);
    return calculateLatency(pkt);
}

void
Timer::doFunctionalAccess(PacketPtr pkt)
{
    //functionalAccess(pkt);

    /* Based on AbstractMemory::functionalAccess */

    assert(pkt->getAddr() >= range.start &&
           (pkt->getAddr() + pkt->getSize() - 1) <= range.end);

//    uint8_t *hostAddr = pmemAddr + pkt->getAddr() - range.start;

    if (pkt->isRead()) {
        if (pmemAddr) {
          /*
            int current_time = curTick();
            int difference_time = current_time - stored_tp.subtaskStart;
            int slack = stored_tp.subtaskWCET - difference_time;
            //memcpy(pkt->getPtr<uint8_t>(), hostAddr, pkt->getSize());
            //memcpy(pkt->getPtr<uint8_t>(), &stored_tp, pkt->getSize());
            */
            memcpy(pkt->getPtr<uint8_t>(), &stored_tp.slack, pkt->getSize());
        }
        //TRACE_PACKET("Read");
        pkt->makeResponse();
    } else if (pkt->isWrite()) {
        if (pmemAddr) {
            //memcpy(hostAddr, pkt->getPtr<uint8_t>(), pkt->getSize());

            int write_addr = pkt->getAddr();
            // Start subtask
            if (write_addr == TIMER_START_SUBTASK) {
              // Save WCET that was written by CPU
              memcpy(&stored_tp.subtaskWCET, pkt->getPtr<uint8_t>(), pkt->getSize());
              // Save current time
              stored_tp.subtaskStart = curTick();
              DPRINTF(SlackTimer, "Written to timer: start = %d, WCET = %d\n", stored_tp.subtaskStart, stored_tp.subtaskWCET);
            // End subtask
            } else if (write_addr == TIMER_END_SUBTASK) {
              // Accumulate remaining subtask slack into total task slack 
              stored_tp.slack += stored_tp.subtaskWCET - (curTick() - stored_tp.subtaskStart);
              DPRINTF(SlackTimer, "Written to timer: end, slack = %d\n", stored_tp.slack);
            } else if (write_addr == TIMER_START_TASK) {
              // Reset all variables
              stored_tp.init();
            } else if (write_addr == TIMER_END_TASK) {
              // Do nothing for now, may need to do something later
            } else {
              warn("Unknown address written to for timer.");
            }
        }
        //TRACE_PACKET("Write");
        pkt->makeResponse();
    // Not supporting for slack timer
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
Timer::getSlavePort(const std::string &if_name, int idx)
{
    if (if_name != "port") {
        return MemObject::getSlavePort(if_name, idx);
    } else {
        if (idx >= static_cast<int>(ports.size())) {
            fatal("Timer::getSlavePort: unknown index %d\n", idx);
        }

        return *ports[idx];
    }
}

unsigned int
Timer::drain(Event *de)
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

Timer::MemoryPort::MemoryPort(const std::string& _name,
                                     Timer& _memory)
    : SimpleTimingPort(_name, &_memory), memory(_memory)
{ }

AddrRangeList
Timer::MemoryPort::getAddrRanges()
{
    AddrRangeList ranges;
    ranges.push_back(memory.getAddrRange());
    return ranges;
}

Tick
Timer::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return memory.doAtomicAccess(pkt);
}

void
Timer::MemoryPort::recvFunctional(PacketPtr pkt)
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

Timer*
TimerParams::create()
{
    return new Timer(this);
}
