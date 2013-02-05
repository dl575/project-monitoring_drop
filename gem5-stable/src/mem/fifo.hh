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
#define FIFO_SIZE 16
// Space between fifo entries
#define FIFO_ENTRY_SIZE 0x100
// FIFO device address
#define FIFO_ADDR 0x30000000
#define FIFO_ADDR_START FIFO_ADDR
#define FIFO_ADDR_END   FIFO_ADDR + 0x0000ffff

// Addresses used for parts of monitoring packet
#define FIFO_VALID         FIFO_ADDR          // valid packet
#define FIFO_INSTADDR      (FIFO_ADDR + 0x04) // program counter
#define FIFO_MEMADDR       (FIFO_ADDR + 0x08) // memory address
#define FIFO_MEMEND        (FIFO_ADDR + 0x0c) // memory end range
#define FIFO_DATA          (FIFO_ADDR + 0x10) // load/store data
#define FIFO_STORE         (FIFO_ADDR + 0x14) // store flag
#define FIFO_DONE          (FIFO_ADDR + 0x18) // main core done
#define FIFO_NUMSRCREGS    (FIFO_ADDR + 0x1c) // number of source registers
#define FIFO_SRCREGS_START (FIFO_ADDR + 0x20) // first source register
#define FIFO_SRCREGS_END   (FIFO_ADDR + 0x8c) //last source register

// Fifo registers
#define FIFO_REG_START (FIFO_ADDR + 0x1000) 
#define FIFO_FULL      (FIFO_REG_START)       // returns 1 if fifo full
#define FIFO_EMPTY     (FIFO_REG_START + 0x4) // returns 1 if fifo empty

// Fifo operations
#define FIFO_OP_RANGE_START FIFO_ADDR                    //Start of op range
#define FIFO_START_CUSTOM   (FIFO_OP_RANGE_START + 0x04) //Start a custom packet
#define FIFO_END_CUSTOM     (FIFO_OP_RANGE_START + 0x08) //End the custom packet and send
#define FIFO_NEXT           (FIFO_OP_RANGE_START + 0x0c) //Pop the fifo
#define FIFO_OP_RANGE_END   (FIFO_OP_RANGE_START + 0x10) //End of op range

// Monitoring packet that is stored as each fifo entry
class monitoringPacket {
  public:
    bool valid;           // Valid packet, 0 if fifo is empty
    Addr instAddr;        // program counter
    Addr memAddr;         // address of memory access
	Addr memEnd;          // range of memory address
    uint64_t data;        // data for memory access
    bool store;           // true if store instruction, false if load
    bool done;            // indicates that the main core program has finished
    uint8_t numsrcregs;   // indicates the number of source registers used for this instruction
    uint8_t srcregs[27];  // the actual source registers for the instruction

    // Clear all variables
    void init() {
      valid = false;
      instAddr = 0;
      memAddr = 0;
	  memEnd = 0;
      data = 0;
      store = false;
      done = false;
      numsrcregs = 0;
      for (unsigned i = 0; i < 27; ++i){
        srcregs[i] = 0;
      }
    }
};

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
    
    // Fifo Array
    monitoringPacket fifo_array[FIFO_SIZE];
    
    // invalid packet
    monitoringPacket invalidPacket;

  public:
    bool empty() {
      return (head_pointer == tail_pointer);
    }
    bool full() {
      return (((head_pointer + 1) % FIFO_SIZE) == tail_pointer);
    }

};

#endif //__FIFO_HH__
