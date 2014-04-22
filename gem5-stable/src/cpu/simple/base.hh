/*
 * Copyright (c) 2011 ARM Limited
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
 *          Dave Greene
 *          Nathan Binkert
 */

#ifndef __CPU_SIMPLE_BASE_HH__
#define __CPU_SIMPLE_BASE_HH__

#include "base/statistics.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/pc_event.hh"
#include "cpu/simple_thread.hh"
#include "cpu/static_inst.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/eventq.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

#include "debug/Fifo.hh"
#include "debug/FifoStall.hh"
#include "debug/Monitoring.hh"
#include "debug/SlackTimer.hh"
#include "debug/Task.hh"
#include "debug/Invalidation.hh"

#include "mem/fifo.hh"
#include "mem/timer.hh"
#include "mem/flag_cache.hh"

// forward declarations
class Checkpoint;
class Process;
class Processor;
class ThreadContext;

namespace TheISA
{
    class DTB;
    class ITB;
}

namespace Trace {
    class InstRecord;
}

struct BaseSimpleCPUParams;

class BaseSimpleCPU : public BaseCPU
{

  protected:
    typedef TheISA::MiscReg MiscReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;

  protected:
    Trace::InstRecord *traceData;

    inline void checkPcEventQueue() {
        Addr oldpc, pc = thread->instAddr();
        do {
            oldpc = pc;
            system->pcEventQueue.service(tc);
            pc = thread->instAddr();
        } while (oldpc != pc);
    }

  public:

    virtual void init();

    void wakeup();

    void zero_fill_64(Addr addr) {
      static int warned = 0;
      if (!warned) {
        warn ("WH64 is not implemented");
        warned = 1;
      }
    };

  public:
    BaseSimpleCPU(BaseSimpleCPUParams *params);
    virtual ~BaseSimpleCPU();

  public:
    /** SimpleThread object, provides all the architectural state. */
    SimpleThread *thread;

    /** ThreadContext object, provides an interface for external
     * objects to modify this thread's state.
     */
    ThreadContext *tc;

    CheckerCPU *checker;

    /* ALU Opcodes */
    enum ALUOpCode {
        ALUNone,
        ALUAdd,
        ALUSub,
        ALUMov,
        ALUAnd
    };

  protected:

    enum Status {
        Idle,
        Running,
        Faulting,
        ITBWaitResponse,
        IcacheRetry,
        IcacheWaitResponse,
        IcacheWaitSwitch,
        DTBWaitResponse,
        DcacheRetry,
        DcacheWaitResponse,
        DcacheWaitSwitch,
        SwitchedOut,
        FifoStall
    };

    Status _status;

  public:

    Addr dbg_vtophys(Addr addr);

    bool interval_stats;

    // current instruction
    TheISA::MachInst inst;

    StaticInstPtr curStaticInst;
    StaticInstPtr curMacroStaticInst;

    //This is the offset from the current pc that fetch should be performed at
    Addr fetchOffset;
    //This flag says to stay at the current pc. This is useful for
    //instructions which go beyond MachInst boundaries.
    bool stayAtPC;

    void checkForInterrupts();
    void setupFetchRequest(Request *req);
    void preExecute();
    void postExecute();
    void advancePC(Fault fault);
    MasterPort & getMasterPort(const std::string &if_name, int idx);

    virtual void deallocateContext(ThreadID thread_num);
    virtual void haltContext(ThreadID thread_num);

    // statistics
    virtual void regStats();
    virtual void resetStats();

    // number of simulated instructions
    Counter numInst;
    Counter startNumInst;
    Stats::Scalar numInsts;
    Counter numOp;
    Counter startNumOp;
    Stats::Scalar numOps;

    void countInst()
    {
        if (!curStaticInst->isMicroop() || curStaticInst->isLastMicroop()) {
            numInst++;
            numInsts++;
        }
        numOp++;
        numOps++;

        system->totalNumInsts++;
        thread->funcExeInst++;
    }

    virtual Counter totalInsts() const
    {
        return numInst - startNumInst;
    }

    virtual Counter totalOps() const
    {
        return numOp - startNumOp;
    }

    //number of integer alu accesses
    Stats::Scalar numIntAluAccesses;

    //number of float alu accesses
    Stats::Scalar numFpAluAccesses;

    //number of function calls/returns
    Stats::Scalar numCallsReturns;

    //conditional control instructions;
    Stats::Scalar numCondCtrlInsts;

    //number of int instructions
    Stats::Scalar numIntInsts;

    //number of float instructions
    Stats::Scalar numFpInsts;

    //number of integer register file accesses
    Stats::Scalar numIntRegReads;
    Stats::Scalar numIntRegWrites;

    //number of float register file accesses
    Stats::Scalar numFpRegReads;
    Stats::Scalar numFpRegWrites;

    // number of simulated memory references
    Stats::Scalar numMemRefs;
    Stats::Scalar numLoadInsts;
    Stats::Scalar numStoreInsts;

    // number of idle cycles
    Stats::Formula numIdleCycles;

    // number of busy cycles
    Stats::Formula numBusyCycles;

    // number of simulated loads
    Counter numLoad;
    Counter startNumLoad;

    // number of idle cycles
    Stats::Average notIdleFraction;
    Stats::Formula idleFraction;

    // number of cycles stalled for I-cache responses
    Stats::Scalar icacheStallCycles;
    Counter lastIcacheStall;

    // number of cycles stalled for I-cache retries
    Stats::Scalar icacheRetryCycles;
    Counter lastIcacheRetry;

    // number of cycles stalled for D-cache responses
    Stats::Scalar dcacheStallCycles;
    Counter lastDcacheStall;

    // number of cycles stalled for D-cache retries
    Stats::Scalar dcacheRetryCycles;
    Counter lastDcacheRetry;
    
    // Number of monitored micro ops
    Stats::Scalar numMonOps;
    // drop statistics
    Stats::Vector dropstats;
    Stats::Vector filterstats;
    Stats::Vector coveragedropstats;
    Stats::Vector fullstats;

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    // These functions are only used in CPU models that split
    // effective address computation from the actual memory access.
    void setEA(Addr EA) { panic("BaseSimpleCPU::setEA() not implemented\n"); }
    Addr getEA()        { panic("BaseSimpleCPU::getEA() not implemented\n");
        M5_DUMMY_RETURN}

    // The register accessor methods provide the index of the
    // instruction's operand (e.g., 0 or 1), not the architectural
    // register index, to simplify the implementation of register
    // renaming.  We find the architectural register index by indexing
    // into the instruction's own operand index table.  Note that a
    // raw pointer to the StaticInst is provided instead of a
    // ref-counted StaticInstPtr to redice overhead.  This is fine as
    // long as these methods don't copy the pointer into any long-term
    // storage (which is pretty hard to imagine they would have reason
    // to do).

    uint64_t readIntRegOperand(const StaticInst *si, int idx)
    {
        numIntRegReads++;
        return thread->readIntReg(si->srcRegIdx(idx));
    }

    FloatReg readFloatRegOperand(const StaticInst *si, int idx)
    {
        numFpRegReads++;
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return thread->readFloatReg(reg_idx);
    }

    FloatRegBits readFloatRegOperandBits(const StaticInst *si, int idx)
    {
        numFpRegReads++;
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return thread->readFloatRegBits(reg_idx);
    }

    void setIntRegOperand(const StaticInst *si, int idx, uint64_t val)
    {
        numIntRegWrites++;
        thread->setIntReg(si->destRegIdx(idx), val);
    }

    void setFloatRegOperand(const StaticInst *si, int idx, FloatReg val)
    {
        numFpRegWrites++;
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        thread->setFloatReg(reg_idx, val);
    }

    void setFloatRegOperandBits(const StaticInst *si, int idx,
                                FloatRegBits val)
    {
        numFpRegWrites++;
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        thread->setFloatRegBits(reg_idx, val);
    }

    bool readPredicate() { return thread->readPredicate(); }
    void setPredicate(bool val)
    {
        thread->setPredicate(val);
        if (traceData) {
            traceData->setPredicate(val);
        }
    }
    TheISA::PCState pcState() { return thread->pcState(); }
    void pcState(const TheISA::PCState &val) { thread->pcState(val); }
    Addr instAddr() { return thread->instAddr(); }
    Addr nextInstAddr() { return thread->nextInstAddr(); }
    MicroPC microPC() { return thread->microPC(); }

    MiscReg readMiscRegNoEffect(int misc_reg)
    {
        return thread->readMiscRegNoEffect(misc_reg);
    }

    MiscReg readMiscReg(int misc_reg)
    {
        numIntRegReads++;
        return thread->readMiscReg(misc_reg);
    }

    void setMiscReg(int misc_reg, const MiscReg &val)
    {
        numIntRegWrites++;
        return thread->setMiscReg(misc_reg, val);
    }

    MiscReg readMiscRegOperand(const StaticInst *si, int idx)
    {
        numIntRegReads++;
        int reg_idx = si->srcRegIdx(idx) - TheISA::Ctrl_Base_DepTag;
        return thread->readMiscReg(reg_idx);
    }

    void setMiscRegOperand(
            const StaticInst *si, int idx, const MiscReg &val)
    {
        numIntRegWrites++;
        int reg_idx = si->destRegIdx(idx) - TheISA::Ctrl_Base_DepTag;
        return thread->setMiscReg(reg_idx, val);
    }

    void demapPage(Addr vaddr, uint64_t asn)
    {
        thread->demapPage(vaddr, asn);
    }

    void demapInstPage(Addr vaddr, uint64_t asn)
    {
        thread->demapInstPage(vaddr, asn);
    }

    void demapDataPage(Addr vaddr, uint64_t asn)
    {
        thread->demapDataPage(vaddr, asn);
    }

    unsigned readStCondFailures() {
        return thread->readStCondFailures();
    }

    void setStCondFailures(unsigned sc_failures) {
        thread->setStCondFailures(sc_failures);
    }

     MiscReg readRegOtherThread(int regIdx, ThreadID tid = InvalidThreadID)
     {
        panic("Simple CPU models do not support multithreaded "
              "register access.\n");
     }

     void setRegOtherThread(int regIdx, const MiscReg &val,
                            ThreadID tid = InvalidThreadID)
     {
        panic("Simple CPU models do not support multithreaded "
              "register access.\n");
     }

    //Fault CacheOp(uint8_t Op, Addr EA);

    Fault hwrei() { return thread->hwrei(); }
    bool simPalCheck(int palFunc) { return thread->simPalCheck(palFunc); }

    void
    syscall(int64_t callnum)
    {
        if (FullSystem)
            panic("Syscall emulation isn't available in FS mode.\n");

        thread->syscall(callnum);
    }

    bool misspeculating() { return thread->misspeculating(); }
    ThreadContext *tcBase() { return tc; }

    // Whether monitoring is enabled
    bool monitoring_enabled;
    // Whether fifo port is enabled
    // If monitoring_enabled is false, this still allows a processor to read
    // from the fifo port without automatically monitoring
    bool fifo_enabled;
    // Enable timer for slack tracking
    bool timer_enabled;
    // Enable flag cache
    bool flagcache_enabled;
    
    enum instType { 
        inst_undef,       // Unknown instruction
        inst_load,        // Load instruction
        inst_store,       // Store instruction
        inst_call,        // Call instruction
        inst_ret,         // Return instruction
        inst_intalu,      // Integer ALU instruction
        inst_indctrl,     // Indirect control instruction
        num_inst_types    // Number of instruction types
    };

  private:
    
    template <unsigned add_size = 0> 
    class InvalidationTable
    {
      private:
        static const unsigned size = num_inst_types + add_size;
        std::string table_name;
        
      public:
        std::string sel1[size];
        std::string sel2[size];
        std::string aluop[size];
        int constant[size];
        std::string action[size];
        bool initialized;
        
        InvalidationTable(std::string name)
            : sel1(), sel2(), aluop(),
              constant(), action(),
              initialized(false)
            { table_name = name; }
        bool initTable(const char * file_name);
        void printTable();
    };
    
    class FilterPtrTable
    {
      private:
        std::string table_name;
        
      public:
        unsigned table[num_inst_types][4];
        
        FilterPtrTable(std::string name)
            : table(), initialized(false)
            { table_name = name; }
        bool initialized;
        bool initTable(const char * file_name);
        void printTable();
    };
    
    // Convert the type of instruction to string
    std::string instTypeToString(int inst_type);
    // Get the type of instruction from a string
    static instType parseInstType(const char * inst_type);
    // Perform an ALU operation
    Addr performOp(std::string &op, Addr a, Addr b);
    // Select the a value for the ALU operation
    Addr selectValue(std::string &select, Addr c);
    // Set the flagcache address based on invalidation table entry
    template <unsigned size>
    Fault setFlagCacheAddrFromTable(InvalidationTable <size> & it, unsigned idx);
    // Perform invalidation in the flag cache
    Fault performInvalidation(unsigned idx, instType itp);
    // emulate filtering but don't perform
    bool emulate_filtering;

  protected:
    // Port for monitoring fifo
    CpuPort fifoPort;
    // Port for accessing timer
    CpuPort timerPort;
    // Port for accessing flag cache
    CpuPort fcPort;
    // Invalidation table object
    static const int it_add_size = 4;
    InvalidationTable <it_add_size> invtab;
    // Filtering tables
    InvalidationTable <> filtertab1;
    InvalidationTable <> filtertab2;
    FilterPtrTable fptab;

    // hard wcet deadline
    bool hard_wcet;
    // Stall because need to write to fifo but fifo is full
    bool fifoStall;
    // Stall due to having extra slack in timer
    bool timerStalled;
    // Allows for stalling when fifo is empty
    bool fifoEmpty;
    // Amount of time spent stalled
    int fifoStallTicks;
    // Check flags
    bool check_load;
    bool check_store;
    bool check_indctrl;
    // Coverage controls
    double target_coverage;
    double packet_drop_rate;
    unsigned check_frequency;
    unsigned total_checks;
    unsigned full_packets;
    unsigned all_packets;
    // Perform dropping only on set tag operations
    bool source_dropping;

    // Data structure for handling fifo event
    class fifoEventDetails {
      public:
        Addr memAddr;
        Addr instVirtAddr;
        Addr instPhysAddr;
        Addr dataVirtAddr;
        Addr dataPhysAddr;
        unsigned dataSize;
        uint64_t data;

        void clear() {
          memAddr = 0;
          instVirtAddr = 0;
          instPhysAddr = 0;
          dataVirtAddr = 0;
          dataPhysAddr = 0;
          dataSize = 0;
          data = 0;
        }
    };
    // Data structure for saving informatino for monitoring
    fifoEventDetails fed;

    // Monitoring filter - decides which instructions to monitor
    class monitorFilter {
      public:
        bool load;    // Load instruction
        bool store;   // Store instruction
        bool call;    // Function call instruction
        bool ret;     // Return instruction
        bool intalu;  // Integer ALU instruction - includes below int*
        bool intmov; // Integer move
        bool intadd; // Integer add
        bool intsub; // Integer subtract
        bool intand; // Integer AND (logical)
        bool intmul; // Integer multiply
        bool indctrl; // Indirect control instruction

        // Constructor sets all flags to false 
        monitorFilter() {
          load = false;
          store = false;
          call = false;
          ret = false;
          intalu = false;
            intmov = false;
            intadd = false;
            intsub = false;
            intand = false;
            intmul = false;
          indctrl = false;
        }
    };
    monitorFilter mf;

    // Fifo monitoring packet
    monitoringPacket mp;


#ifdef DEBUG
    // Start time of task
    Tick start_task;
    Addr task_addr;
    // Start time of a subtask
    Tick start_subtask;
    Addr subtask_addr;
    // Count number of packets
    unsigned num_packets;
    unsigned last_packets;
    // Count cycles due to FifoStall
    unsigned num_stalls;
    unsigned last_stalls;
#endif

    // Monitoring is performed when this flag is set to true.
    // This is used to skip monitoring for postExecute of
    // translationFault(). If this stalls, then the actual
    // execution of the instruction after handling the
    // translation is skipped.
    bool perf_mon;
    // Function that performs monitoring operation during postExecute
    void performMonitoring();
    // Requests for reading/writing to fifo/timer
    Request data_read_req;
    Request data_write_req;
    virtual void endTask() {
      panic("endTask not implemented\n");
    }

    Fault ReExecFault;
    
    // Get the flagcache address based on invalidation table entry
    template <unsigned size>
    Addr getInvalidationAddr(InvalidationTable <size> & it, unsigned idx);
    // Get the type of instruction from fifo entry
    instType readFifoInstType();
    
    // Read from fifo into data
    Fault readFromFifo(Addr addr, uint8_t * data, unsigned size, unsigned flags);
    // Read from slack timer into data
    Fault readFromTimer(Addr addr, uint8_t * data, unsigned size, unsigned flags);
    // Notify the timer that an instruction is invalidated
    void notifyTimerInvalidation(bool important);
    // Read from flag cache into data
    virtual Fault readFromFlagCache(Addr addr, uint8_t * data, unsigned size, unsigned flags);
    // Write to fifo
    Fault writeToFifo(Addr addr, uint8_t * data, unsigned size, unsigned flags);
    // Write to timer
    Fault writeToTimer(Addr addr, uint8_t * data, unsigned size, unsigned flags);
    // Write to flag cache
    virtual Fault writeToFlagCache(Addr addr, uint8_t * data, unsigned size, unsigned flags);

    // Checks whether the head packet has the done flag asserted
    bool isFifoDone();
    // Checks whether fifo is empty
    bool isFifoEmpty();
    // Send monitoring packet to fifo
    bool sendFifoPacket();

    // Additional functionality to stall due to fifo
    virtual void stallFromFifo() {}

    // backtrack
    bool _backtrack;
    bool _important;
    virtual bool backtrack();

    // Flag for whether to print out ID #s of checks that are monitored in full
    bool print_checkid;
    // Keep track of and print out coverage of static instructions
    bool print_static_coverage;
    // Save IDs of checks that are monitored in full by using bit vectors
    int checkid_base;
    uint64_t checkid_vec;

    Stats::Scalar numImportantInsts;

    // backup valid flags for DIFT_RF
#define NUM_REGS 16
    bool invalid_flags[NUM_REGS];

    std::list<int> pc_checked; // List of all static instructions that raised monitoring events
    std::list<int> pc_checked_full; // List of static instructions that were checked in full

};

#endif // __CPU_SIMPLE_BASE_HH__
