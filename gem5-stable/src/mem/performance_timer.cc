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
#include "mem/performance_timer.hh"

#include "debug/SlackTimer.hh"

using namespace std;

PerformanceTimer::PerformanceTimer(const Params* p) :
    AbstractMemory(p),
    lat(p->latency), lat_var(p->latency_var),
    povr(p->percent_overhead),
    important_percent(p->important_percent),
    use_start_ticks(p->use_start_ticks)
{
    for (size_t i = 0; i < p->port_port_connection_count; ++i) {
        ports.push_back(new MemoryPort(csprintf("%s-port-%d", name(), i),
                                       *this));
    }
    
    start_ticks = p->start_cycles * p->start_cycles_clock;
    
    if (p->slack_lo <= p->slack_hi){
        slack_lo = p->slack_lo * p->start_cycles_clock;
        slack_hi = p->slack_hi * p->start_cycles_clock;
    } else {
        warn("Low slack value higher than high slack value. Switching...");
        slack_hi = p->slack_lo * p->start_cycles_clock;
        slack_lo = p->slack_hi * p->start_cycles_clock;
    }
    
    important_policy = p->important_policy;
    important_slack = p->important_slack * p->start_cycles_clock;

    srand(p->seed);
    
    // printf("Random init: %d, %d\n", rand(), p->seed);
    // DPRINTF(SlackTimer, "Performance timer initialized with probabilistic range {%lld, %lld}\n", slack_lo, slack_hi);
}

void
PerformanceTimer::init()
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
    drop_thres = 0;
    drops = 0;
    not_drops = 0;
}

Tick
PerformanceTimer::calculateLatency(PacketPtr pkt)
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
PerformanceTimer::doAtomicAccess(PacketPtr pkt)
{
    access(pkt);
    return calculateLatency(pkt);
}

long long int
PerformanceTimer::effectiveSlack(){
    return stored_tp.slack + (curTick() - stored_tp.taskStart)*povr;
}

long long int
PerformanceTimer::effectiveImportantSlack(){
    return stored_tp.slack + (curTick() - stored_tp.taskStart)*important_percent;
}

void
PerformanceTimer::doFunctionalAccess(PacketPtr pkt)
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
            
            long long int slack;
            long long int impt_slack = 0;
            if (stored_tp.intask && !stored_tp.isDecrement){
                slack = effectiveSlack();
                if (important_policy == 2)
                    impt_slack = effectiveImportantSlack();
            } else if (stored_tp.isDecrement){
                slack = effectiveSlack() - (curTick() - stored_tp.decrementStart);
                if (slack > effectiveSlack()){ panic("Timer Underflow."); }

                if (important_policy == 2) {
                    impt_slack = effectiveImportantSlack() - (curTick() - stored_tp.decrementStart);
                    if (impt_slack > effectiveImportantSlack())
                        panic("Timer Underflow.");
                }
            } else if (!stored_tp.intask && curTick() < stored_tp.WCET_end){
                slack = stored_tp.slack - (curTick() - stored_tp.decrementStart);
                if (slack > stored_tp.slack){ panic("Timer Underflow."); }

                if (important_policy == 2) {
                    impt_slack = stored_tp.important_slack - (curTick() - stored_tp.decrementStart);
                    if (impt_slack > stored_tp.important_slack)
                        panic("Timer Underflow.");
                }
            } else {
                slack = LLONG_MAX;
                impt_slack = LLONG_MAX;
            }
            
            Addr read_addr = pkt->getAddr();
            uint64_t send_data = 0;
            
            if (read_addr == TIMER_READ_SLACK){
                send_data = slack;
            } else if (read_addr == TIMER_READ_DROP) {
                long long int adjusted_slack = slack - drop_thres;
                int drop_status;
                if (adjusted_slack < slack_lo){
                    drop_status = 0; // Drop
                } else if (adjusted_slack >= slack_hi){
                    drop_status = 1; // Not Drop
                } else {
                    // We assume a linear probability model between slack_hi and slack_lo.
                    // Note (slack_hi != slack_lo) in this part of the code (due to above conditions).
                    // At slack_hi not_drop_rate = 1
                    // At slack_lo not_drop_rate = 0
                    double not_drop_rate = (double)(adjusted_slack - slack_lo)/(double)(slack_hi - slack_lo);
                    // Get a random number
                    double random_number = (double)rand()/RAND_MAX; 
                    // If we get any random number under the not_drop_rate, we don't drop
                    drop_status = (random_number <= not_drop_rate);
                    
                    DPRINTF(SlackTimer, "Prob computation: slack = %lld, rate = %f, number = %f, result = %d\n", adjusted_slack, not_drop_rate, random_number, drop_status);
                }
                send_data = drop_status;
                if (stored_tp.intask || curTick() < stored_tp.WCET_end){
                    if (drop_status) { not_drops++; }
                    else { drops++; }
                }
            } else if (read_addr == TIMER_READ_DROP_IMPORTANT) {
                if (important_policy == 0) {
                    // always forward
                    int drop_status = 1;
                    send_data = drop_status;
                    not_drops++;
                } else if (important_policy == 1) {
                    // forward/drop based on slack
                    int drop_status = (slack + important_slack >= 0);
                    send_data = drop_status;
                    if (stored_tp.intask || curTick() < stored_tp.WCET_end){
                        if (drop_status) { not_drops++; }
                        else { drops++; }
                    }
                } else if (important_policy == 2) {
                    long long int adjusted_slack = impt_slack - drop_thres;
                    int drop_status = (adjusted_slack >= 0);
                    if (stored_tp.intask || curTick() < stored_tp.WCET_end) {
                        if (drop_status) { not_drops++; }
                        else { drops++; }
                    }
                }
            } else if (read_addr == TIMER_DROPS){
                send_data = drops;
            } else if (read_addr == TIMER_NOT_DROPS){
                send_data = not_drops;
            } else if (read_addr == TIMER_TASK_PACKET){
                send_data = (stored_tp.intask || curTick() < stored_tp.WCET_end);
            }
            
            pkt->setData((uint8_t *)&send_data);
        }
        //TRACE_PACKET("Read");
        pkt->makeResponse();
    } else if (pkt->isWrite()) {
        if (pmemAddr) {
            //memcpy(hostAddr, pkt->getPtr<uint8_t>(), pkt->getSize());
            
            Addr write_addr = pkt->getAddr();
            // Read data value
            uint64_t get_data = 0;
            pkt->writeData((uint8_t *)&get_data);
            
            if (write_addr == TIMER_START_TASK) {
              // Reset all variables
              stored_tp.init();
              // Setup variables
              stored_tp.intask = true;
              stored_tp.taskStart = curTick();
              if (use_start_ticks){
                // Use param based initial slack
                stored_tp.slack = start_ticks;
                if (important_policy == 2)
                    stored_tp.important_slack = start_ticks;
              } else {
                // Use optionally passed value as initial slack
                stored_tp.slack = get_data;
                if (important_policy == 2)
                    stored_tp.important_slack = get_data;
              }
              DPRINTF(SlackTimer, "Written to timer: task start, slack = %d\n", effectiveSlack());
            } else if (write_addr == TIMER_END_TASK) {
              stored_tp.intask = false;
              stored_tp.decrementStart = curTick();
              long long int additional_time = get_data;
              stored_tp.slack = effectiveSlack();
              if (important_policy == 2)
                  stored_tp.important_slack = effectiveImportantSlack();
              long long int wait_time = stored_tp.slack + additional_time;
              stored_tp.WCET_end = curTick() + wait_time; // Actual deadline
              DPRINTF(SlackTimer, "Written to timer: task end, %d(slack) + %d(add) = %d\n", stored_tp.slack, additional_time, wait_time);
            } else if (write_addr == TIMER_SET_THRES) {
              drop_thres = get_data;
              DPRINTF(SlackTimer, "Written to timer: drop threshold = %d\n", drop_thres);
            } else if (write_addr == TIMER_START_DECREMENT) {
              if (stored_tp.intask){
                  stored_tp.isDecrement = true;
                  stored_tp.decrementStart = curTick();
                  
                  DPRINTF(SlackTimer, "Written to timer: decrement start = %d, slack = %d\n", stored_tp.decrementStart, effectiveSlack());
              }
            } else if (write_addr == TIMER_END_DECREMENT) {
              if (stored_tp.intask){
                  stored_tp.isDecrement = false;
                  //Include povr to offset increase in slack over the delay time
                  Tick delay_time = (curTick() - stored_tp.decrementStart)*(1+povr);
                  long long int slack = stored_tp.slack - delay_time;
                  if (slack > stored_tp.slack){
                    panic("Timer Underflow.");
                  }
                  stored_tp.slack = slack;
                  
                  DPRINTF(SlackTimer, "Written to timer: decrement end, slack = %d\n", effectiveSlack());

                  if (important_policy == 2) {
                    delay_time = (curTick() - stored_tp.decrementStart)*(1 + important_percent);
                    slack = stored_tp.important_slack - delay_time;
                    if (slack > stored_tp.important_slack) {
                      panic("Timer Underflow.");
                    }
                    stored_tp.important_slack = slack;
                  }
              }
            } else {
              warn("Unknown address written to for timer: %x.", write_addr);
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
PerformanceTimer::getSlavePort(const std::string &if_name, int idx)
{
    if (if_name != "port") {
        return MemObject::getSlavePort(if_name, idx);
    } else {
        if (idx >= static_cast<int>(ports.size())) {
            fatal("PerformanceTimer::getSlavePort: unknown index %d\n", idx);
        }

        return *ports[idx];
    }
}

unsigned int
PerformanceTimer::drain(Event *de)
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

void
PerformanceTimer::resume()
{
  AbstractMemory::resume();

  // Reset slack on resume (after fast-forward)
  if (use_start_ticks) {
    stored_tp.slack = start_ticks;
    // Mark this as task start time so slack is calculated correctly. Slack is
    // calculated based on the curTick and the task start time.
    stored_tp.taskStart = curTick();
  } else {
    panic("No headstart slack specified\n");
  }
  DPRINTF(SlackTimer, "Resuming simulation, slack = %d\n", effectiveSlack());

}

PerformanceTimer::MemoryPort::MemoryPort(const std::string& _name,
                                     PerformanceTimer& _memory)
    : SimpleTimingPort(_name, &_memory), memory(_memory)
{ }

AddrRangeList
PerformanceTimer::MemoryPort::getAddrRanges()
{
    AddrRangeList ranges;
    ranges.push_back(memory.getAddrRange());
    return ranges;
}

Tick
PerformanceTimer::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return memory.doAtomicAccess(pkt);
}

void
PerformanceTimer::MemoryPort::recvFunctional(PacketPtr pkt)
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

PerformanceTimer*
PerformanceTimerParams::create()
{
    return new PerformanceTimer(this);
}
