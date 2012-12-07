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
 * Fifo declaration
 */

#ifndef __FIFO_HH__
#define __FIFO_HH__

#include "mem/abstract_mem.hh"
#include "mem/tport.hh"
#include "params/Fifo.hh"

// Number of entries in FIFO
#define FIFO_SIZE 8
// Space between fifo entries
#define FIFO_ENTRY_SIZE 0x100
// FIFO device address
#define FIFO_ADDR 0x30000000

/**
 * The simple memory is a basic multi-ported memory with an infinite
 * throughput and a fixed latency, potentially with a variance added
 * to it. It uses a SimpleTimingPort to implement the timing accesses.
 */
class Fifo : public AbstractMemory
{

  private:

    class MemoryPort : public SimpleTimingPort
    {
        Fifo& memory;

      public:

        MemoryPort(const std::string& _name, Fifo& _memory);

      protected:

        virtual Tick recvAtomic(PacketPtr pkt);

        virtual void recvFunctional(PacketPtr pkt);
        virtual bool recvTimingReq(PacketPtr pkt);

        virtual AddrRangeList getAddrRanges();

    };

    std::vector<MemoryPort*> ports;

    Tick lat;
    Tick lat_var;

  public:

    typedef FifoParams Params;
    Fifo(const Params *p);
    virtual ~Fifo() { }

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

  private:
    // Fifo head/tail pointers
    int head_pointer;
    int tail_pointer;

  public:
    bool empty() {
      return (head_pointer == tail_pointer);
    }
    bool full() {
      return (((head_pointer + 1) % FIFO_SIZE) == tail_pointer);
    }

};

#endif //__FIFO_HH__
