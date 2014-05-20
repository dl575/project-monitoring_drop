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
#include "mem/peripheral_addr.hh"
#include "sim/process.hh"

// Addresses used for parts of monitoring packet
#define FIFO_VALID         FIFO_ADDR                // valid packet
#define FIFO_INSTADDR      (FIFO_ADDR + 0x04)       // program counter
#define FIFO_MEMADDR       (FIFO_ADDR + 0x08)       // memory address
#define FIFO_MEMEND        (FIFO_ADDR + 0x0c)       // memory end range
#define FIFO_DATA          (FIFO_ADDR + 0x10)       // load/store data
#define FIFO_STORE         (FIFO_ADDR + 0x14)       // store flag
#define FIFO_DONE          (FIFO_ADDR + 0x18)       // main core done
#define FIFO_RS1           (FIFO_ADDR + 0x1c)       // rs register field
#define FIFO_RS2           (FIFO_ADDR + 0x20)       // rs register field
#define FIFO_RS3           (FIFO_ADDR + 0x24)       // rs register field
#define FIFO_RD            (FIFO_ADDR + 0x28)       // rd register field
#define FIFO_CONTROL       (FIFO_ADDR + 0x2c)       // control flag
#define FIFO_CALL          (FIFO_ADDR + 0x30)       // call flag
#define FIFO_RET           (FIFO_ADDR + 0x34)       // return flag
#define FIFO_LR            (FIFO_ADDR + 0x38)       // link register
#define FIFO_NEXTPC        (FIFO_ADDR + 0x3c)       // next instruction address
#define FIFO_LOAD          (FIFO_ADDR + 0x40)       // load flag
#define FIFO_INTALU        (FIFO_ADDR + 0x44)       // integer ALU instruction flag
#define FIFO_INDCTRL       (FIFO_ADDR + 0x48)       // indirect control instruction flag
#define FIFO_VIRTADDR      (FIFO_ADDR + 0x4c)       // virtual memory address
#define FIFO_PHYSADDR      (FIFO_ADDR + 0x50)       // physical memory address
#define FIFO_MEMSIZE       (FIFO_ADDR + 0x54)       // size of memory access
#define FIFO_OPCODE        (FIFO_ADDR + 0x58)       // alu opcode
#define FIFO_SETTAG        (FIFO_ADDR + 0x5c)       // manually set the tag

// Fifo registers
#define FIFO_REG_START (FIFO_ADDR + 0x1000) 
#define FIFO_FULL      (FIFO_REG_START)       // returns 1 if fifo full
#define FIFO_EMPTY     (FIFO_REG_START + 0x4) // returns 1 if fifo empty
#define FIFO_PACKET    (FIFO_REG_START + 0x8) // returns full fifo packet

// Fifo operations
#define FIFO_OP_RANGE_START FIFO_ADDR                    //Start of op range
#define FIFO_START_CUSTOM   (FIFO_OP_RANGE_START + 0x04) //Start a custom packet
#define FIFO_END_CUSTOM     (FIFO_OP_RANGE_START + 0x08) //End the custom packet and send
#define FIFO_NEXT           (FIFO_OP_RANGE_START + 0x0c) //Pop the fifo
#define FIFO_CUSTOM_SIZE    (FIFO_OP_RANGE_START + 0x10) //Custom data to be sent
#define FIFO_CUSTOM_DATA    (FIFO_OP_RANGE_START + 0x14) //Custom data to be sent
#define FIFO_OP_RANGE_END   (FIFO_OP_RANGE_START + 0x18) //End of op range

// Monitoring packet that is stored as each fifo entry
class monitoringPacket {
  public:
    bool valid;           // Valid packet, 0 if fifo is empty
    Addr instAddr;        // program counter
    Addr memAddr;         // address of memory access
    Addr memEnd;          // range of memory address
    Addr virtAddr;        // virtual address of memory access
    Addr physAddr;        // physical address of memory access
    size_t size;          // size of memory access
    uint64_t data;        // data for memory access
    bool store;           // true if store instruction
    bool load;            // true if load instruction
    bool done;            // indicates that the main core program has finished
    uint8_t rs1;          // rs register field
    uint8_t rs2;          // rs register field
    uint8_t rs3;          // rs register field
    uint8_t rd;           // rd register field
    bool control;         // true if control instruction
    bool call;            // true if call instruction
    bool ret;             // true if return instruction
    bool intalu;          // integer ALU instruction
    bool indctrl;         // indirect control instruction
    uint64_t lr;          // link register
    uint64_t nextpc;      // next program counter
    uint8_t opcode;       // opcode for integer ALU instructions
    bool settag;          // manually set tag
    char inst_dis[32];    // Disassembled string of instruction (for debugging)
    // Data for handling syscall read
    Addr syscallReadBufPtr;
    int syscallReadNbytes;

    // Clear all variables
    void init() {
      valid = false;
      instAddr = 0;
      memAddr = 0;
      memEnd = 0;
      virtAddr = 0;
      physAddr = 0;
      size = 0;
      data = 0;
      store = false;
      load = false;
      done = false;
      rs1 = 0;
      rs2 = 0;
      rs3 = 0;
      rd = 0;
      control = false;
      call = false;
      ret = false;
      intalu = false;
      indctrl = false;
      lr = 0;
      nextpc = 0;
      opcode = 0;
      settag = false;
      memset(inst_dis, '\0', 32);
      syscallReadBufPtr = 0;
      syscallReadNbytes = 0;
    }

    // Print out full monitoring packet information
    void print() {
      printf("monitoringPacket {\n");
      printf("  valid = %d\n", valid);
      printf("  instAddr = 0x%x\n", (int)instAddr);
      printf("  memAddr = 0x%x\n", (int)memAddr);
      printf("  memEnd = 0x%x\n", (int)memEnd);
      printf("  virtAddr = 0x%x\n", (int)virtAddr);
      printf("  physAddr = 0x%x\n", (int)physAddr);
      printf("  size = %d\n", (int)size);
      printf("  data = %d\n", (int)data);
      printf("  store = %d\n", store);
      printf("  load = %d\n", load);
      printf("  done = %d\n", done);
      printf("  rs1 = %d\n", rs1);
      printf("  rs2 = %d\n", rs2);
      printf("  rs3 = %d\n", rs3);
      printf("  rd = %d\n", rd);
      printf("  control = %d\n", control);
      printf("  call = %d\n", call);
      printf("  ret = %d\n", ret);
      printf("  intalu = %d\n", intalu);
      printf("  indctrl = %d\n", indctrl);
      printf("  lr = 0x%x\n", (int)lr);
      printf("  nextpc = 0x%x\n", (int)nextpc);
      printf("  opcode = %d\n", opcode);
      printf("  settag = %d\n", settag);
      printf("  syscallReadBufPtr = %llx\n", syscallReadBufPtr);
      printf("  syscallReadNbytes = %d\n", syscallReadNbytes);
      printf("}\n");
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
    virtual ~Fifo();

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
    
    // Fifo size
    unsigned fifo_size;
    
    // Fifo Array
    monitoringPacket * fifo_array;
    
    // invalid packet
    monitoringPacket invalidPacket;

  public:
    bool empty() {
      return (head_pointer == tail_pointer);
    }
    bool full() {
      return (((head_pointer + 1) % fifo_size) == tail_pointer);
    }

};

#endif //__FIFO_HH__
