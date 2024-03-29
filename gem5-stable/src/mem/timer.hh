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

#ifndef __TIMER_HH__
#define __TIMER_HH__

#include "mem/abstract_mem.hh"
#include "mem/tport.hh"
#include "params/Timer.hh"
#include "mem/peripheral_addr.hh"

// read registers
#define TIMER_READ_SLACK       (TIMER_ADDR + 0x00)
#define TIMER_READ_DROP        (TIMER_ADDR + 0x04)
#define TIMER_READ_DROP_IMPORTANT (TIMER_ADDR + 0x18)
// hidden read registers (should not be used by the program)
#define TIMER_DROPS            (TIMER_ADDR + 0x100)
#define TIMER_NOT_DROPS        (TIMER_ADDR + 0x104)
#define TIMER_TASK_PACKET      (TIMER_ADDR + 0x108)
#define TIMER_ADJUSTED_SLACK_MULTIPLIER (TIMER_ADDR + 0x10C)

// write registers
#define TIMER_START_TASK       (TIMER_ADDR + 0x00)
#define TIMER_END_TASK         (TIMER_ADDR + 0x04)
#define TIMER_START_SUBTASK    (TIMER_ADDR + 0x08)
#define TIMER_END_SUBTASK      (TIMER_ADDR + 0x0c)
#define TIMER_ENDSTART_SUBTASK (TIMER_ADDR + 0x10)
#define TIMER_SET_THRES        (TIMER_ADDR + 0x14)
// hidden write registers (should not be used by the program)
#define TIMER_START_DECREMENT  (TIMER_ADDR + 0x100)
#define TIMER_END_DECREMENT    (TIMER_ADDR + 0x104)

/**
 * The simple memory is a basic multi-ported memory with an infinite
 * throughput and a fixed latency, potentially with a variance added
 * to it. It uses a SimpleTimingPort to implement the timing accesses.
 */
class Timer : public AbstractMemory
{

  private:

    // Packet that is written to timer
    class timerPacket {
      public:
        // Start time of subtask
        Tick subtaskStart;
        // WCET of current subtask
        long long int subtaskWCET;
        // Accumulated slack
        long long int slack;
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
          intask = false;
          isDecrement = false;
          subtaskStart = 0;
          subtaskWCET = 0;
          slack = 0;
          decrementStart = 0;
          WCET_end = 0;
        }
    };
  
    class MemoryPort : public SimpleTimingPort
    {
        Timer& memory;

      public:

        MemoryPort(const std::string& _name, Timer& _memory);

      protected:

        virtual Tick recvAtomic(PacketPtr pkt);

        virtual void recvFunctional(PacketPtr pkt);

        virtual AddrRangeList getAddrRanges();

    };

    std::vector<MemoryPort*> ports;

    Tick lat;
    Tick lat_var;

    timerPacket stored_tp;
    // Drop threshold for monitor
    long long int drop_thres;
    // Drop Statistics
    unsigned drops, not_drops;

  public:

    typedef TimerParams Params;
    Timer(const Params *p);
    virtual ~Timer() { }

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

};

#endif //__TIMER_HH__
