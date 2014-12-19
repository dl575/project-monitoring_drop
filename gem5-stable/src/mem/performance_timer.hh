/*
 * Copyright (c) 2012 ARM Limited
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
 *          Andreas Hansson
 */

/**
 * @file
 * SimpleMemory declaration
 */

#ifndef __PERFORMANCE_TIMER_HH__
#define __PERFORMANCE_TIMER_HH__

#include "mem/abstract_mem.hh"
#include "mem/tport.hh"
#include "mem/timer.hh"
#include "params/PerformanceTimer.hh"

/**
 * The simple memory is a basic multi-ported memory with an infinite
 * throughput and a fixed latency, potentially with a variance added
 * to it. It uses a SimpleTimingPort to implement the timing accesses.
 */
class PerformanceTimer : public AbstractMemory
{

  private:

    // Packet that is written to timer
    class timerPacket {
      public:
        // Start time of task
        Tick taskStart;
        // Accumulated slack
        long long int slack;
        // Accumulated slack for important instructions
        long long int important_slack;
        // currently executing a task
        bool intask;
        // currently decrementing slack
        bool isDecrement;
        // decrement start time
        Tick decrementStart;
        // WCET end time
        Tick WCET_end;

        // Reset all variables
        void init() {
          taskStart = 0;
          slack = 0;
          important_slack = 0;
          intask = false;
          isDecrement = false;
          decrementStart = 0;
          WCET_end = 0;
        }
    };
  
    class MemoryPort : public SimpleTimingPort
    {
        PerformanceTimer& memory;

      public:

        MemoryPort(const std::string& _name, PerformanceTimer& _memory);

      protected:

        virtual Tick recvAtomic(PacketPtr pkt);

        virtual void recvFunctional(PacketPtr pkt);

        virtual AddrRangeList getAddrRanges();

    };

    virtual void regStats();
    double effectiveOverhead();
    inline long long int effectiveSlack();
    inline long long int effectiveImportantSlack();
    inline long long int slackAllocated();
    inline Tick taskExecutionTime();
    inline Tick nonStallTime();
    void updateSlackSubtrahend();
    double actualSlowdown();
    double actualOverhead();
    double getAdjustedSlackMultiplier();
    double adjustSlackMultiplier();
    
    std::vector<MemoryPort*> ports;

    Tick lat;
    Tick lat_var;

    timerPacket stored_tp;
    // Drop threshold for monitor
    long long int drop_thres;
    // Drop Statistics
    unsigned drops, not_drops;
    // Overhead
    double povr;
    // Probabilty to drop events
    float drop_probability;
    // Start timer at initialization rather than at TIMER_START_TASK call
    bool init_intask;
    // number of ticks in a clock cycle
    Tick clock;

    // Policy for forwarding important instructions
    enum ImportantPolicy {
      // always forward important instructions
      ALWAYS,
      // initial slack as a fixed number of cycles
      SLACK,
      // slack as percent of total cycles
      PERCENT,
      // unified slack tracking for both types of instructions
      UNIFIED,
      // number of policies
      NUM_POLICIES
    };
    enum ImportantPolicy important_policy;

    // Additional slack for important instructions
    long long int important_slack;
    // Additional slack for important instructions as percent of total cycles
    double important_percent;
    // increment slack on important instructions only
    bool increment_important_only;
    // read slack multiplier from file
    bool read_slack_multiplier;
    // Interval for adjusting slack multiplier (in cycles)
    long long int slack_multiplier_interval;
    // whether the last instruction that requested slack is important
    bool last_important;
    // slack multiplier
    double slack_multiplier;
    // slack subtrahend
    long long int slack_subtrahend;
    // last non-stall time
    long long int last_non_stall_time;
    // last allocated slack
    long long int last_slack_allocated;
    // last time slack multiplier is updated
    Tick slack_multiplier_last_update;
    // last time slack subtrahend was updated
    Tick slack_subtrahend_last_update;
    // directory to store persistence data
    std::string persistence_dir;

    // statistics
    // allocated slack
    long long int _cumulative_delay_time;
    Stats::Scalar cumulative_delay_time;
    Stats::Scalar slack_allocated;
    Stats::Scalar task_execution_time;
    Stats::Formula non_stall_time;
    Stats::Formula actual_slowdown;
    Stats::Formula actual_overhead;
    
    // Start task ticks
    Tick start_ticks;
    bool use_start_ticks;

  public:

    typedef PerformanceTimerParams Params;
    PerformanceTimer(const Params *p);
    virtual ~PerformanceTimer() { }

    unsigned int drain(Event* de);

    virtual SlavePort& getSlavePort(const std::string& if_name, int idx = -1);
    virtual void init();

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

  protected:

    Tick doAtomicAccess(PacketPtr pkt);
    void doFunctionalAccess(PacketPtr pkt);
    virtual Tick calculateLatency(PacketPtr pkt);

  public:
    virtual void resume();
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

};

#endif //__PERFORMANCE_TIMER_HH__
