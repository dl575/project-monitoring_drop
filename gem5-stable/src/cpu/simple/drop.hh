/*
 * Copyright (c) 2012 ARM Limited
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

#ifndef __CPU_SIMPLE_DROP_HH__
#define __CPU_SIMPLE_DROP_HH__

#include "cpu/simple/base.hh"
#include "params/DropSimpleCPU.hh"

#define DROP_CLEAR_ARRAY 0
#define DROP_CLEAR_CACHE 1

class DropSimpleCPU : public BaseSimpleCPU
{
  public:

    DropSimpleCPU(DropSimpleCPUParams *params);
    virtual ~DropSimpleCPU();

    virtual void init();

  private:

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
      // number of monitoring extensions
      NUM_MONITORING_EXTENSIONS
    };
    
    // monitoring extension
    enum MonitoringExtension monitorExt;

    struct TickEvent : public Event
    {
        DropSimpleCPU *cpu;

        TickEvent(DropSimpleCPU *c);
        void process();
        const char *description() const;
    };

    TickEvent tickEvent;

    const int width;
    bool locked;
    const bool simulate_data_stalls;

    // main simulation loop (one cycle)
    void tick();

    /**
     * An DropCPUPort overrides the default behaviour of the
     * recvAtomic and ignores the packet instead of panicking.
     */
    class DropCPUPort : public CpuPort
    {

      public:

        DropCPUPort(const std::string &_name, BaseCPU* _cpu)
            : CpuPort(_name, _cpu)
        { }

      protected:

        virtual void recvTimingSnoopReq(PacketPtr pkt) { }
      
        virtual Tick recvAtomicSnoop(PacketPtr pkt)
        {
            // Snooping a coherence request, just return
            return 0;
        }

    };

    DropCPUPort icachePort;
    DropCPUPort dcachePort;

    class MonitorPort : public SimpleTimingPort
    {
        DropSimpleCPU *cpu;

      public:

        MonitorPort(const std::string& _name, DropSimpleCPU *_cpu);

      protected:

        virtual Tick recvAtomic(PacketPtr pkt);

        virtual void recvFunctional(PacketPtr pkt);
        virtual bool recvTimingReq(PacketPtr pkt);
        
        virtual AddrRangeList getAddrRanges();
    };

    // Port directly to monitor
    MonitorPort monitorPort;

    bool fastmem;
    Request ifetch_req;

    bool dcache_access;
    Tick dcache_latency;

  protected:

    /** Return a reference to the data port. */
    virtual CpuPort &getDataPort() { return dcachePort; }

    /** Return a reference to the instruction port. */
    virtual CpuPort &getInstPort() { return icachePort; }

  public:
  
    // Read from flag cache into data
    Fault readFromFlagCache(Addr addr, uint8_t * data, unsigned size, unsigned flags);
    // Write to flag cache
    Fault writeToFlagCache(Addr addr, uint8_t * data, unsigned size, unsigned flags);

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
    virtual void resume();

    void switchOut();
    void takeOverFrom(BaseCPU *oldCPU);

    virtual void activateContext(ThreadID thread_num, int delay);
    virtual void suspendContext(ThreadID thread_num);
    
    MasterPort & getMasterPort(const std::string &if_name, int idx);
    SlavePort & getSlavePort(const std::string &if_name, int idx);

    Fault readMem(Addr addr, uint8_t *data, unsigned size, unsigned flags);

    Fault writeMem(uint8_t *data, unsigned size,
                   Addr addr, unsigned flags, uint64_t *res);

    /**
     * Print state of address in memory system via PrintReq (for
     * debugging).
     */
    void printAddr(Addr a);
    
    // statistics
    virtual void regStats();
    Stats::Scalar numCacheLoads;
    Stats::Scalar numCacheStores;
    Stats::Scalar numRegLoads;
    Stats::Scalar numRegStores;

    bool backtrack_write_table;
    bool backtrack_read_table;
    std::string backtrack_table_dir;
    void writeBacktrackTable();

  private:

    // function to handle end_task command
    void endTask() {}
    // functional memory operations
    Fault readMemFunctional(Addr addr, uint8_t *data, unsigned size, unsigned flags);
    Fault writeMemFunctional(uint8_t *data, unsigned size, Addr addr, unsigned flags, uint64_t *res);
    // prevent page fault errors
    void pageAllocate(Addr addr);
    // get correct flags for writing
    unsigned getCacheFlags(size_t size);
    // forward the fifo packet to monitoring core
    bool forwardFifoPacket();
    
    bool forward_fifo_enabled;
    // Port for monitoring fifo
    CpuPort forwardFifoPort;
    // Full monitoring ticks
    Tick full_ticks;

  protected:
    virtual bool backtrack();
    bool backtrack_hb();
    bool backtrack_dift();
    bool backtrack_umc();
    void backtrack_inst_indctrl(monitoringPacket &mpkt);
    void backtrack_inst_load(monitoringPacket &mpkt);
    void backtrack_inst_store(monitoringPacket &mpkt);
    void backtrack_inst_intalu(monitoringPacket &mpkt);

};

#endif // __CPU_SIMPLE_DROP_HH__
