/*
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

#ifndef __MONITOR_SIMPLE_TIMING_HH__
#define __MONITOR_SIMPLE_TIMING_HH__

#include "cpu/simple/base.hh"
#include "cpu/translation.hh"
#include "params/TimingSimpleMonitor.hh"

class TimingSimpleMonitor : public BaseSimpleCPU
{
  public:

    TimingSimpleMonitor(TimingSimpleMonitorParams * params);
    virtual ~TimingSimpleMonitor();

    virtual void init();

  public:
    Event *drainEvent;

  private:

    /*
     * If an access needs to be broken into fragments, currently at most two,
     * the the following two classes are used as the sender state of the
     * packets so the CPU can keep track of everything. In the main packet
     * sender state, there's an array with a spot for each fragment. If a
     * fragment has already been accepted by the CPU, aka isn't waiting for
     * a retry, it's pointer is NULL. After each fragment has successfully
     * been processed, the "outstanding" counter is decremented. Once the
     * count is zero, the entire larger access is complete.
     */
    class SplitMainSenderState : public Packet::SenderState
    {
      public:
        int outstanding;
        PacketPtr fragments[2];

        int
        getPendingFragment()
        {
            if (fragments[0]) {
                return 0;
            } else if (fragments[1]) {
                return 1;
            } else {
                return -1;
            }
        }
    };

    class SplitFragmentSenderState : public Packet::SenderState
    {
      public:
        SplitFragmentSenderState(PacketPtr _bigPkt, int _index) :
            bigPkt(_bigPkt), index(_index)
        {}
        PacketPtr bigPkt;
        int index;

        void
        clearFromParent()
        {
            SplitMainSenderState * main_send_state =
                dynamic_cast<SplitMainSenderState *>(bigPkt->senderState);
            main_send_state->fragments[index] = NULL;
        }
    };

    class FetchTranslation : public BaseTLB::Translation
    {
      protected:
        TimingSimpleMonitor *cpu;

      public:
        FetchTranslation(TimingSimpleMonitor *_cpu)
            : cpu(_cpu)
        {}

        void
        markDelayed()
        {
            assert(cpu->_status == Running);
            cpu->_status = ITBWaitResponse;
        }

        void
        finish(Fault fault, RequestPtr req, ThreadContext *tc,
               BaseTLB::Mode mode)
        {
            cpu->sendFetch(fault, req, tc);
        }
    };
    FetchTranslation fetchTranslation;

    void sendData(RequestPtr req, uint8_t *data, uint64_t *res, bool read);
    void sendSplitData(RequestPtr req1, RequestPtr req2, RequestPtr req,
                       uint8_t *data, bool read);

    void translationFault(Fault fault);

    void buildPacket(PacketPtr &pkt, RequestPtr req, bool read);
    void buildSplitPacket(PacketPtr &pkt1, PacketPtr &pkt2,
            RequestPtr req1, RequestPtr req2, RequestPtr req,
            uint8_t *data, bool read);

    bool handleReadPacket(PacketPtr pkt);
    // This function always implicitly uses dcache_pkt.
    bool handleWritePacket();

    /**
     * A TimingMonitorPort overrides the default behaviour of the
     * recvTiming and recvRetry and implements events for the
     * scheduling of handling of incoming packets in the following
     * cycle.
     */
    class TimingMonitorPort : public CpuPort
    {
      public:

        TimingMonitorPort(const std::string& _name, TimingSimpleMonitor* _cpu)
            : CpuPort(_name, _cpu), cpu(_cpu), retryEvent(this)
        { }

      protected:

        /**
         * Snooping a coherence request, do nothing.
         */
        virtual void recvTimingSnoopReq(PacketPtr pkt) { }
        
        virtual Tick recvAtomicSnoop(PacketPtr pkt)
        {
            // Snooping a coherence request, just return
            return 0;
        }

        TimingSimpleMonitor* cpu;

        struct TickEvent : public Event
        {
            PacketPtr pkt;
            TimingSimpleMonitor *cpu;

            TickEvent(TimingSimpleMonitor *_cpu) : pkt(NULL), cpu(_cpu) {}
            const char *description() const { return "Timing Monitor tick"; }
            void schedule(PacketPtr _pkt, Tick t);
        };

        EventWrapper<Port, &Port::sendRetry> retryEvent;
    };

    class IcachePort : public TimingMonitorPort
    {
      public:

        IcachePort(TimingSimpleMonitor *_cpu)
            : TimingMonitorPort(_cpu->name() + "-iport", _cpu),
              tickEvent(_cpu)
        { }

      protected:

        virtual bool recvTimingResp(PacketPtr pkt);

        virtual void recvRetry();

        struct ITickEvent : public TickEvent
        {

            ITickEvent(TimingSimpleMonitor *_cpu)
                : TickEvent(_cpu) {}
            void process();
            const char *description() const { return "Timing CPU icache tick"; }
        };

        ITickEvent tickEvent;

    };

    class DcachePort : public TimingMonitorPort
    {
      public:

        DcachePort(TimingSimpleMonitor *_cpu)
            : TimingMonitorPort(_cpu->name() + "-dport", _cpu), tickEvent(_cpu)
        { }

      protected:

        virtual bool recvTimingResp(PacketPtr pkt);

        virtual void recvRetry();

        struct DTickEvent : public TickEvent
        {
            DTickEvent(TimingSimpleMonitor *_cpu)
                : TickEvent(_cpu) {}
            void process();
            const char *description() const { return "Timing Monitor dcache tick"; }
        };

        DTickEvent tickEvent;

    };

    IcachePort icachePort;
    DcachePort dcachePort;

    // Port for sending back information to CPU
    CpuPort monitorPort;

    PacketPtr ifetch_pkt;
    PacketPtr dcache_pkt;

    Tick previousTick;

  protected:

     /** Return a reference to the data port. */
    virtual CpuPort &getDataPort() { return dcachePort; }

    /** Return a reference to the instruction port. */
    virtual CpuPort &getInstPort() { return icachePort; }

  public:

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    virtual unsigned int drain(Event *drain_event);
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

    void fetch();
    void sendFetch(Fault fault, RequestPtr req, ThreadContext *tc);
    void completeIfetch(PacketPtr );
    void completeDataAccess(PacketPtr pkt);
    void advanceInst(Fault fault);

    /**
     * Print state of address in memory system via PrintReq (for
     * debugging).
     */
    void printAddr(Addr a);

    /**
     * Finish a DTB translation.
     * @param state The DTB translation state.
     */
    void finishTranslation(WholeTranslationState *state);

  private:

    typedef EventWrapper<TimingSimpleMonitor, &TimingSimpleMonitor::fetch> FetchEvent;
    FetchEvent fetchEvent;

    struct IprEvent : Event {
        Packet *pkt;
        TimingSimpleMonitor *cpu;
        IprEvent(Packet *_pkt, TimingSimpleMonitor *_cpu, Tick t);
        virtual void process();
        virtual const char *description() const;
    };

    void completeDrain();

  private:

    // Fifo Event
    void handleFifoEvent();
    typedef EventWrapper<TimingSimpleMonitor, &TimingSimpleMonitor::handleFifoEvent> FifoEvent;
    FifoEvent fifoEvent;

    // Timer event to stall at end of task
    void handleEndTaskEvent();
    typedef EventWrapper<TimingSimpleMonitor, &TimingSimpleMonitor::handleEndTaskEvent> EndTaskEvent;
    EndTaskEvent endTaskEvent;

    // Requests for reading/writing to fifo/timer
    //Request data_read_req;
    //Request data_write_req;

    // function to handle end_task command
    void endTask();

    // Additional functionality to stall due to fifo
    void stallFromFifo();

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

    MasterPort & getMasterPort(const std::string &if_name, int idx);

  protected:
    // tag memory operations
    Tag readTag(Addr addr);
    Tag readTagFunctional(Addr addr);
    bool readBitTag(Addr addr);
    bool readBitTagFunctional(Addr addr);
    uint64_t readDWordTag(Addr addr);
    uint64_t readDWordTagFunctional(Addr addr);

    void writeTag(Addr addr, Tag tag);
    void writeTagFunctional(Addr addr, Tag tag);
    void writeBitTag(Addr addr, bool tag);
    void writeBitTagFunctional(Addr addr, bool tag);
    void writeDWordTag(Addr addr, uint64_t tag);
    void writeDWordTagFunctional(Addr addr, uint64_t tag);
    void handlePageTableFault(Addr addr);
    Addr faultAddr;

    void revalidateRegTag(int idx);
    void revalidateMemTag(Addr addr);

    // status registers
    bool enabled;
    bool initialized;

    // variables for setting and clearing tags
    uint64_t setTagData;

    void processMonitoringPacket();
    void UMCExecute();
    void DIFTExecute();
    void BCExecute();
    void SECExecute();
    void HBExecute();
    void preExecuteMonitor();
    void postExecute();
    void finishMonitoring();

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


  private:
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

    bool locked;
    Request monitor_req;
 
};

#endif // __MONITOR_SIMPLE_TIMING_HH__
