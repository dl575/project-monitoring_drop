/*
 * Copyright (c) 2012 ARM Limited
 * Copyright (c) 2013 Cornell University
 * All rights reserved.
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 */

#ifndef __MONITOR_SIMPLE_ATOMIC_HH__
#define __MONITOR_SIMPLE_ATOMIC_HH__

#include "cpu/simple/base.hh"
#include "params/AtomicSimpleMonitor.hh"

#include "mem/fifo.hh"

/* Atomic Monitor Core
 * Derived from BaseSimpleCPU, accepts instructions forwarded from main core,
 * performs monitoring operations and sends result back to main core on certain
 * events.
 */
class AtomicSimpleMonitor : public BaseSimpleCPU
{
  public:

    AtomicSimpleMonitor(AtomicSimpleMonitorParams *params);
    virtual ~AtomicSimpleMonitor();

    virtual void init();

    // monitoring extensions
    enum MonitoringExtension {
      // no monitoring
      MONITOR_NONE,
      // uninitialized memory check
      MONITOR_UMC,
      // dynamic information flow tracking
      MONITOR_DIFT,
      // boundary checking
      MONITOR_BC,
      // soft error checking
      MONITOR_SEC,
      // hard bound
      MONITOR_HB,
      // dift with 32-bit tags
      MONITOR_MULTIDIFT,
      // link-register (return address) check
      MONITOR_LRC,
      // dift with flag array storing taint
      MONITOR_DIFTRF,
      // number of monitoring extensions
      NUM_MONITORING_EXTENSIONS
    };
    
    // monitoring extension
    enum MonitoringExtension monitorExt;
  
  private:

    struct TickEvent : public Event
    {
        AtomicSimpleMonitor *cpu;

        TickEvent(AtomicSimpleMonitor *c);
        void process();
        const char *description() const;
    };

    TickEvent tickEvent;

    const int width;
    bool locked;
    const bool simulate_data_stalls;
    const bool simulate_inst_stalls;

    // main simulation loop (one cycle)
    void tick();

    /**
     * An AtomicMonitorPort overrides the default behaviour of the
     * recvAtomic and ignores the packet instead of panicking.
     */
    class AtomicMonitorPort : public CpuPort
    {

      public:

        AtomicMonitorPort(const std::string &_name, BaseCPU* _cpu)
            : CpuPort(_name, _cpu)
        { }

      protected:

        virtual Tick recvAtomicSnoop(PacketPtr pkt)
        {
            // Snooping a coherence request, just return
            return 0;
        }

        /**
         * Ignore timing snoop.
         */
        virtual void recvTimingSnoopReq(PacketPtr pkt) { }        

    };

    // FIXME: a monitor does not have a instruction cache, remove this later
    AtomicMonitorPort icachePort;
    AtomicMonitorPort dcachePort;

    // Port for sending back information to CPU
    CpuPort monitorPort;

    bool fastmem;
    Request ifetch_req;
    Request data_read_req;
    Request data_write_req;
    Request monitor_req;

    bool dcache_access;
    Tick dcache_latency;
    // Number of cycles for full monitoring
    unsigned full_wcet;
    // Number of additional cycles for full monitoring
    unsigned full_delay;

  protected:

    /** Return a reference to the data port. */
    virtual CpuPort &getDataPort() { return dcachePort; }

    /** Return a reference to the instruction port. */
    virtual CpuPort &getInstPort() { return icachePort; }

  public:

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
    virtual void resume();

    void switchOut();
    void takeOverFrom(BaseCPU *oldCPU);

    virtual void activateContext(ThreadID thread_num, int delay);
    virtual void suspendContext(ThreadID thread_num);

    Fault readMem(Addr addr, uint8_t *data, unsigned size, unsigned flags);
    Fault readMemFunctional(Addr addr, uint8_t *data, unsigned size, unsigned flags);

    Fault writeMem(uint8_t *data, unsigned size,
                   Addr addr, unsigned flags, uint64_t *res);
    Fault writeMemFunctional(uint8_t *data, unsigned size,
                   Addr addr, unsigned flags, uint64_t *res);

    /**
     * Print state of address in memory system via PrintReq (for
     * debugging).
     */
    void printAddr(Addr a);

    MasterPort & getMasterPort(const std::string &if_name, int idx);

  public:
    /**
     * tag type definitions 
     */
    // generic tag type
    typedef uint8_t Tag;
    // monitor-specific tag type
    typedef bool    UMCTag;
    typedef bool    DIFTTag;
    typedef uint8_t BCTag;
    typedef uint64_t HBTag;

    // statistics
    virtual void regStats();
    virtual void resetStats();
    void setTagProxy(Addr addr, int nbytes, uint8_t tag);

  protected:
    // status registers
    bool enabled;
    bool initialized;

    // variables for setting and clearing tags
    uint64_t setTagData;

    // Fifo
    bool recvFifoPacket();
    
    // tag memory operations
    Tag readTag(Addr addr);
    Tag readTagFunctional(Addr addr);
    bool readBitTag(Addr addr);
    bool readBitTagFunctional(Addr addr);
    uint32_t readWordTag(Addr addr);
    uint64_t readDWordTag(Addr addr);
    uint64_t readDWordTagFunctional(Addr addr);

    void writeTag(Addr addr, Tag tag);
    void writeTagFunctional(Addr addr, Tag tag);
    void writeBitTag(Addr addr, bool tag);
    void writeBitTagFunctional(Addr addr, bool tag);
    void writeWordTag(Addr addr, uint32_t tag);
    void writeDWordTag(Addr addr, uint64_t tag);
    void writeDWordTagFunctional(Addr addr, uint64_t tag);
    void handlePageTableFault(Addr addr);

    // Functions for performing monitoring operations
    void processMonitoringPacket();
    void UMCExecute();
    void DIFTExecute();
    void DIFTRFExecute();
    void MultiDIFTExecute();
    void BCExecute();
    void SECExecute();
    void HBExecute();
    void LRCExecute();

    void preExecute();
    void postExecute();
    void finishMonitoring();
    void showMonitoringStats();
    void showDIFTStats();
    void revalidateRegTag(int idx);
    void invalidateRegTag(int idx);
    void revalidateMemTag(Addr addr);
    void invalidateMemTag(Addr addr);
    void setFlagCacheAddr(Addr addr);

    template <typename T>
    bool isMemoryCompactable(Addr addr, int blocksize, T tag);

    // tag convertions
    inline BCTag toMemTag(BCTag t) {
        return t & 0xf;
    }

    inline BCTag toPtrTag(BCTag t) {
        return t >> 4;
    }

    inline BCTag mergeMemPtrTags(BCTag m, BCTag p) {
        return (m & 0xf) | ((p & 0xf) << 4);
    }

    inline uint32_t toBaseTag(HBTag t) {
        return (uint32_t)(t & 0xffffffff);
    }

    inline uint32_t toBoundTag(HBTag t) {
        return (uint32_t)(t >> 32);
    }

    // ALU opcode decoding
    ALUOpCode decodeALUOpcode(uint8_t opcode);

    // statistics
    // number of instructions processed by monitor
    Stats::Scalar numMonitorInsts;
    Stats::Scalar numIntegerInsts;
    Stats::Scalar numLoadInsts;
    Stats::Scalar numStoreInsts;
    Stats::Scalar numIndirectCtrlInsts;
    Stats::Scalar numCallInsts;
    Stats::Scalar numReturnInsts;
    // UMC statistics
    Stats::Scalar numUMCErrors;
    // DIFT statistics
    // number of tainted instructions
    Stats::Scalar numTaintedIntegerInsts;
    Stats::Scalar numTaintedLoadInsts;
    Stats::Scalar numTaintedStoreInsts;
    Stats::Scalar numTaintedIndirectCtrlInsts;
    Stats::Formula numTaintedInsts;
    // BC statistics
    Stats::Scalar numBCLoadErrors;
    Stats::Scalar numBCStoreErrors;
    Stats::Formula numBCErrors;
    Stats::Scalar numSyscallTags;
};

extern SimpleThread* monitor_thread;

#endif // __MONITOR_SIMPLE_ATOMIC_HH__
