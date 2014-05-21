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

#include "arch/locked_mem.hh"
#include "arch/mmapped_ipr.hh"
#include "arch/utility.hh"
#include "base/bigint.hh"
#include "base/callback.hh"
#include "config/the_isa.hh"
#include "cpu/simple/drop.hh"
#include "cpu/exetrace.hh"
#include "debug/ExecFaulting.hh"
#include "debug/SimpleCPU.hh"
#include "debug/DropSimpleCPU.hh"
#include "debug/Backtrack.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/physical.hh"
#include "params/DropSimpleCPU.hh"
#include "sim/faults.hh"
#include "sim/system.hh"
#include "sim/full_system.hh"
#include "sim/sim_events.hh"

#include "mem/fifo.hh"
#include "mem/timer.hh"
#include "mem/flag_cache.hh"

#include "debug/FlagCache.hh"

#include <iomanip>
#include <sstream>

using namespace std;
using namespace TheISA;

DropSimpleCPU::TickEvent::TickEvent(DropSimpleCPU *c)
    : Event(CPU_Tick_Pri), cpu(c)
{
}


void
DropSimpleCPU::TickEvent::process()
{
    cpu->tick();
}

const char *
DropSimpleCPU::TickEvent::description() const
{
    return "DropSimpleCPU tick";
}

void
DropSimpleCPU::init()
{
    BaseSimpleCPU::init();

    // Initialise the ThreadContext's memory proxies
    tcBase()->initMemProxies(tcBase());

    if (FullSystem && !params()->defer_registration) {
        ThreadID size = threadContexts.size();
        for (ThreadID i = 0; i < size; ++i) {
            ThreadContext *tc = threadContexts[i];
            // initialize CPU, including PC
            TheISA::initCPU(tc, tc->contextId());
        }
    }

    // Atomic doesn't do MT right now, so contextId == threadId
    ifetch_req.setThreadContext(_cpuId, 0); // Add thread ID if we add MT
    data_read_req.setThreadContext(_cpuId, 0); // Add thread ID here too
    data_write_req.setThreadContext(_cpuId, 0); // Add thread ID here too
    
    // Write threshold to timer
    if (timer_enabled){
        // Create request
        Request *timer_write_req = &data_write_req;
        // set physical address
        timer_write_req->setPhys(TIMER_SET_THRES, sizeof(full_ticks), ArmISA::TLB::AllowUnaligned, dataMasterId());
        // Create write packet
        MemCmd cmd = MemCmd::WriteReq;
        PacketPtr timerpkt = new Packet(timer_write_req, cmd);
        // Set data
        timerpkt->dataStatic(&full_ticks);

        // Send read request packet on timer port
        timerPort.sendFunctional(timerpkt);

        // Clean up
        delete timerpkt;
    }

    // initialize structures to supporting backtracking
    rptb.init();
    mptb.init();
    if (!(_backtrack && backtrack_read_table)) {
        if (ipt_impl == TABLE)  // table-based
            ipt.init();
        else if (ipt_impl == BLOOM_FILTER) // Bloom filter-based
            ipt_bloom.init();
    } else {
        ifstream is;
        // read instruction priority table
        is.open(backtrack_table_dir + "/ipt.table", std::ios::in | std::ios::binary);
        if (is.good()) {
            if (ipt_impl == TABLE)
                ipt.unserialize(is);
            else if (ipt_impl == BLOOM_FILTER)
                ipt_bloom.unserialize(is);
        } else
            warn("instruction priority table could not be opened.\n");
        is.close();
    }

    // initialize structures for dependence analysis
    odt.init();
    imt.init();
    if (read_check_sets) {
        ifstream is;
        is.open(backtrack_table_dir + "/checksets.txt", std::ios::in);
        if (is.good()) {
            std::string line;
            while (std::getline(is, line)) {
                std::stringstream lineStream(line);
                std::vector<std::string> fields;
                std::string field;
                while (std::getline(lineStream, field, ',')) {
                    fields.push_back(field);
                }
                if (fields.size() <= 1)
                    continue;
                std::vector<std::string>::iterator i = fields.begin();
                std::string inst_addr_str = *i++;
                std::stringstream ss;
                unsigned inst_addr;
                ss << std::hex << inst_addr_str;
                ss >> inst_addr;
                for (; i != fields.end(); ++i) {
                    std::string check_addr_str = *i;
                    std::stringstream sss;
                    sss << std::hex << check_addr_str;
                    unsigned check_addr;
                    sss >> check_addr;
                    addToCheckSet(inst_addr, check_addr);
                }
            }
        } else {
            warn("check sets table could not be opened.\n");
        }
    }
    if (read_optimal_dropping) {
        ifstream is;
        is.open(backtrack_table_dir + "/odp.txt", std::ios::in);
        if (is.good()) {
            std::string line;
            while (std::getline(is, line)) {
                std::stringstream ss;
                unsigned inst_addr;
                ss << std::hex << line;
                ss >> inst_addr;
                markOptimalDroppingPoint(inst_addr);
            }
        } else {
            warn("Optimal dropping table could not be opened.\n");
        }
    }
}

DropSimpleCPU::DropSimpleCPU(DropSimpleCPUParams *p)
    : BaseSimpleCPU(p), tickEvent(this), width(p->width), locked(false),
      simulate_data_stalls(p->simulate_data_stalls),
      icachePort(name() + "-iport", this), dcachePort(name() + "-iport", this),
      monitorPort(name() + "-iport", this),
      fastmem(p->fastmem), forward_fifo_enabled(p->forward_fifo_enabled),
      forwardFifoPort(name() + "-iport", this),
      full_ticks(p->full_clock),
      rptb(),
      mptb(p->mpt_size, 30-floorLog2(p->mpt_size), 2),
      ipt_tagged(p->ipt_tagged),
      ipt(p->ipt_tagged, p->ipt_size/p->ipt_entry_size, p->ipt_entry_size, 30-floorLog2(p->ipt_size), 2),
      ipt_bloom(p->ipt_size, p->ipt_false_positive_rate),
      compute_check_sets(p->compute_check_sets),
      max_check_set_size(p->max_check_set_size),
      read_check_sets(p->read_check_sets),
      compute_optimal_dropping(p->compute_optimal_dropping),
      read_optimal_dropping(p->read_optimal_dropping),
      imt(p->ipt_tagged, p->ipt_size/p->ipt_entry_size, p->ipt_entry_size, 30-floorLog2(p->ipt_size), 2),
      odt(p->ipt_tagged, p->ipt_size/p->ipt_entry_size, p->ipt_entry_size, 30-floorLog2(p->ipt_size), 2)
{
    _status = Idle;    

    // monitoring extension type
    switch(p->monitor_type) {
        case MONITOR_NONE: monitorExt = MONITOR_NONE; break;
        case MONITOR_UMC: monitorExt = MONITOR_UMC; break;
        case MONITOR_DIFT: monitorExt = MONITOR_DIFT; break;
        case MONITOR_BC: monitorExt = MONITOR_BC; break;
        case MONITOR_SEC: monitorExt = MONITOR_SEC; break;
        case MONITOR_HB: monitorExt = MONITOR_HB; break;
        // Multi-bit DIFT has same flow behavior as DIFT
        case MONITOR_MULTIDIFT: monitorExt = MONITOR_DIFT; break;
        case MONITOR_LRC: monitorExt = MONITOR_LRC; break;
        default: panic("Invalid monitor type\n");
    }

    // IPT implementation
    switch(p->ipt_impl) {
        case TABLE: ipt_impl = TABLE; break;
        case BLOOM_FILTER: ipt_impl = BLOOM_FILTER; break;
        default: panic("Invalid IPT implementation\n");
    }

    backtrack_write_table = p->backtrack_write_table;
    backtrack_read_table = p->backtrack_read_table;
    backtrack_table_dir = p->backtrack_table_dir;

    // set up callback at simulation exit to write out backtrack tables
    if (p->backtrack && p->backtrack_write_table) {
        Callback *cb = new MakeCallback<DropSimpleCPU, &DropSimpleCPU::writeBacktrackTable>(this);
        registerExitCallback(cb);
    }

    // callback to output check sets data
    if (compute_check_sets) {
        Callback *cb = new MakeCallback<DropSimpleCPU, &DropSimpleCPU::writeCheckSets>(this);
        registerExitCallback(cb);
    }

    // callback to output optimal dropping points
    if (compute_optimal_dropping) {
        Callback *cb = new MakeCallback<DropSimpleCPU, &DropSimpleCPU::writeODT>(this);
        registerExitCallback(cb);
    }

    // Callback to print out checked static instructions
    if (print_static_coverage) {
      Callback *cb = new MakeCallback<DropSimpleCPU, &DropSimpleCPU::writeCheckedPC>(this);
      registerExitCallback(cb);
    }

}


DropSimpleCPU::~DropSimpleCPU()
{
    if (tickEvent.scheduled()) {
        deschedule(tickEvent);
    }
}

void
DropSimpleCPU::regStats()
{
    using namespace Stats;

    BaseSimpleCPU::regStats();

    numCacheLoads
        .name(name() + ".flagCacheLoads")
        .desc("Number of loads from flag cache")
        ;
    
    numRegLoads
        .name(name() + ".flagRegLoads")
        .desc("Number of loads from flag register")
        ;
    
    numCacheStores
        .name(name() + ".flagCacheStores")
        .desc("Number of stores to flag cache")
        ;
    
    numRegStores
        .name(name() + ".flagRegStores")
        .desc("Number of stores to flag register")
        ;
    
    numIITWrites
        .name(name() + ".numIITWrites")
        .desc("Number of writes to instruction importance table")
        ;

    numCheckPC
      .name(name() + ".numCheckPC")
      .desc("Number of static instructions which cause a check monitoring operation.")
      ;

    numCheckPCFull
      .name(name() + ".numCheckPCFull")
      .desc("Number of static instrucstions which cause a check that are monitored at least once.")
      ;

    staticCoverage
      .name(name() + ".staticCoverage")
      .desc("Static coverage = numCheckPC/numCheckPCFull.")
      ;
    
}

void
DropSimpleCPU::serialize(ostream &os)
{
    SimObject::State so_state = SimObject::getState();
    SERIALIZE_ENUM(so_state);
    SERIALIZE_SCALAR(locked);
    BaseSimpleCPU::serialize(os);
    nameOut(os, csprintf("%s.tickEvent", name()));
    tickEvent.serialize(os);
}

void
DropSimpleCPU::unserialize(Checkpoint *cp, const string &section)
{
    SimObject::State so_state;
    UNSERIALIZE_ENUM(so_state);
    UNSERIALIZE_SCALAR(locked);
    BaseSimpleCPU::unserialize(cp, section);
    tickEvent.unserialize(cp, csprintf("%s.tickEvent", section));
}

void
DropSimpleCPU::resume()
{
    if (_status == Idle || _status == SwitchedOut)
        return;

    DPRINTF(SimpleCPU, "Resume\n");
    assert(system->getMemoryMode() == Enums::atomic);

    changeState(SimObject::Running);
    if (thread->status() == ThreadContext::Active) {
        if (!tickEvent.scheduled())
            schedule(tickEvent, nextCycle());
    }
    system->totalNumInsts = 0;
}

void
DropSimpleCPU::switchOut()
{
    assert(_status == Running || _status == Idle);
    _status = SwitchedOut;

    tickEvent.squash();
}


void
DropSimpleCPU::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    assert(!tickEvent.scheduled());

    // if any of this CPU's ThreadContexts are active, mark the CPU as
    // running and schedule its tick event.
    ThreadID size = threadContexts.size();
    for (ThreadID i = 0; i < size; ++i) {
        ThreadContext *tc = threadContexts[i];
        if (tc->status() == ThreadContext::Active && _status != Running) {
            _status = Running;
            schedule(tickEvent, nextCycle());
            break;
        }
    }
    if (_status != Running) {
        _status = Idle;
    }
    assert(threadContexts.size() == 1);
    ifetch_req.setThreadContext(_cpuId, 0); // Add thread ID if we add MT
    data_read_req.setThreadContext(_cpuId, 0); // Add thread ID here too
    data_write_req.setThreadContext(_cpuId, 0); // Add thread ID here too
}


void
DropSimpleCPU::activateContext(ThreadID thread_num, int delay)
{
    DPRINTF(SimpleCPU, "ActivateContext %d (%d cycles)\n", thread_num, delay);

    assert(thread_num == 0);
    assert(thread);

    assert(_status == Idle);
    assert(!tickEvent.scheduled());

    notIdleFraction++;
    numCycles += tickToCycles(thread->lastActivate - thread->lastSuspend);

    //Make sure ticks are still on multiples of cycles
    schedule(tickEvent, nextCycle(curTick() + ticks(delay)));
    _status = Running;
}


void
DropSimpleCPU::suspendContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "SuspendContext %d\n", thread_num);

    assert(thread_num == 0);
    assert(thread);

    if (_status == Idle)
        return;

    assert(_status == Running);

    // tick event may not be scheduled if this gets called from inside
    // an instruction's execution, e.g. "quiesce"
    if (tickEvent.scheduled())
        deschedule(tickEvent);

    notIdleFraction--;
    _status = Idle;
}

//
// write backtrack table
//
void
DropSimpleCPU::writeBacktrackTable()
{
    ofstream os;
    // write out instruction priority table
    os.open(backtrack_table_dir + "/ipt.table", std::ios::out | std::ios::binary);
    if (os.good()) {
        if (ipt_impl == TABLE)
            ipt.serialize(os);
        else if (ipt_impl == BLOOM_FILTER)
            ipt_bloom.serialize(os);
    } else
        warn("instruction priority table could not be opened.\n");
    os.close();
    
    // write out memory producer tracking table
    os.open(backtrack_table_dir + "/mptb.table", std::ios::out | std::ios::binary);
    if (os.good())
        mptb.serialize(os);
    else
        warn("memory producer tracking table could not be opened.\n");
    os.close();
    
    // write out register producer tracking table
    os.open(backtrack_table_dir + "/rptb.table", std::ios::out | std::ios::binary);
    if (os.good())
        rptb.serialize(os);
    else
        warn("register producer tracking table could not be opened.\n");
    os.close();

    // write out slack multiplier
    os.open(backtrack_table_dir + "/slack_multiplier", std::ios::out);
    if (os.good()) {
        double slack_multiplier;
        // Create request
        Request *timer_read_request = &data_read_req;
        // set physical address
        timer_read_request->setPhys(TIMER_ADJUSTED_SLACK_MULTIPLIER, sizeof(slack_multiplier), ArmISA::TLB::AllowUnaligned, dataMasterId());
        // Create read packet
        MemCmd cmd = MemCmd::ReadReq;
        PacketPtr timerpkt = new Packet(timer_read_request, cmd);
        // Set data
        timerpkt->dataStatic(&slack_multiplier);
        // Send read request packet on timer port
        timerPort.sendFunctional(timerpkt);
        // Clean up
        delete timerpkt;

        os << slack_multiplier << std::endl;
    } else
        warn("slack multiplier file could not be opened.\n");
    os.close();
}

/**
 * Write out check sets
 * File format is: instAddr,checkAddr0,checkAddr1,...
 */
void
DropSimpleCPU::writeCheckSets()
{
    ofstream os;
    // write out check sets
    os.open(backtrack_table_dir + "/checksets.txt", std::ios::out);
    if (os.good()) {
        unsigned numEntries = imt.getNumEntries();
        unsigned entrySize = imt.getEntrySize();
        unsigned step = 1 << imt.getInstShiftAmt();
        for (unsigned i = 0; i < numEntries * entrySize; i+=step) {
            os << std::hex << i;
            InstructionMetadata *metadata = imt.lookup(i);
            if (metadata != NULL) {
                std::set<Addr>::iterator it;
                for (it = metadata->checks->begin(); it != metadata->checks->end(); ++it) {
                    os << "," << *it;
                }
            }
            os << std::endl;
        }
    } else
        warn("check sets table could not be opened.\n");
    os.close();
}

/**
 * Write out optimal dropping points
 * File format is: instAddr:[01]
 */
void
DropSimpleCPU::writeODT()
{
    ofstream os;
    // write out optimal dropping points
    os.open(backtrack_table_dir + "/optimal_dropping.txt", std::ios::out);
    if (os.good()) {
        unsigned numEntries = odt.getNumEntries();
        unsigned entrySize = odt.getEntrySize();
        unsigned step = 1 << odt.getInstShiftAmt();
        for (unsigned i = 0; i < numEntries * entrySize; i+=step) {
            if (odt.lookup(i)) {
                os << std::hex << i << std::endl;
            }
        }
    } else
        warn("optimal dropping table could not be opened.\n");
    os.close();
}

/*
 *  On simulation exit, print out all PCs with check monitoring operations and
 *  all of these PCs where at least one dynamic instance was monitored in full.
 */
void
DropSimpleCPU::writeCheckedPC()
{
  // Print out all check monitoring operation PCs
  printf("All PCs:\n");
  std::list<int>::iterator it;
  for (it = pc_checked.begin(); it != pc_checked.end(); ++it) {
    printf("%x, ", *it);
  }
  printf("\n");

  // Print out check monitoring operation PCs that were monitored at least once
  printf("Full monitored PCs:\n");
  for (it = pc_checked_full.begin(); it != pc_checked_full.end(); ++it) {
    printf("%x, ", *it);
  }
  printf("\n");

  // Calculate static coverage statistics
  numCheckPC = (int)pc_checked.size();
  numCheckPCFull = (int)pc_checked_full.size();
  float coverage = (float)pc_checked_full.size()/(int)pc_checked.size();
  staticCoverage = coverage;
}

Fault
DropSimpleCPU::readMem(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags)
{

    if (traceData) {
        traceData->setAddr(addr);
    }

    //The block size of our peer.
    unsigned blockSize = dcachePort.peerBlockSize();
    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, blockSize);

    if (secondAddr > addr)
        size = secondAddr - addr;

    dcache_latency = 0;

    // use the CPU's statically allocated read request and packet objects
    Request *req = &data_read_req;

    while (1) {
        req->setVirt(0, addr, size, flags, dataMasterId(), thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, tc, BaseTLB::Read);

        // Now do the access.
        if (fault == NoFault && !req->getFlags().isSet(Request::NO_ACCESS)) {
            Packet pkt = Packet(req,
                                req->isLLSC() ? MemCmd::LoadLockedReq :
                                MemCmd::ReadReq);
            pkt.dataStatic(data);

            if (req->isMmappedIpr())
                dcache_latency += TheISA::handleIprRead(thread->getTC(), &pkt);
            else {
                if (fastmem && system->isMemAddr(pkt.getAddr()))
                    system->getPhysMem().access(&pkt);
                else
                    dcache_latency += dcachePort.sendAtomic(&pkt);
            }
            dcache_access = true;

            assert(!pkt.isError());

            if (req->isLLSC()) {
                TheISA::handleLockedRead(thread, req);
            }
        }

        //If there's a fault, return it
        if (fault != NoFault) {
            if (req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        //If we don't need to access a second cache line, stop now.
        if (secondAddr <= addr)
        {
            if (req->isLocked() && fault == NoFault) {
                assert(!locked);
                locked = true;
            }
            return fault;
        }

        /*
         * Set up for accessing the second cache line.
         */

        //Move the pointer we're reading into to the correct location.
        data += size;
        //Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        //And access the right address.
        addr = secondAddr;
    }
}

Fault
DropSimpleCPU::readMemFunctional(Addr addr, uint8_t * data,
                                       unsigned size, unsigned flags)
{
    // use the CPU's statically allocated read request and packet objects
    Request *req = &data_read_req;

    //The block size of our peer.
    unsigned blockSize = dcachePort.peerBlockSize();
    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, blockSize);

    if (secondAddr > addr)
        size = secondAddr - addr;

    while (1) {
        req->setVirt(0, addr, size, flags, dataMasterId(), thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, tc, BaseTLB::Read);

        // Now do the access.
        if (fault == NoFault && !req->getFlags().isSet(Request::NO_ACCESS)) {
            Packet pkt = Packet(req,
                                req->isLLSC() ? MemCmd::LoadLockedReq :
                                MemCmd::ReadReq);
            pkt.dataStatic(data);


            if (fastmem && system->isMemAddr(pkt.getAddr()))
                system->getPhysMem().access(&pkt);
            else
                dcachePort.sendFunctional(&pkt);

            assert(!pkt.isError());

            if (req->isLLSC()) {
                TheISA::handleLockedRead(thread, req);
            }
        }

        //If there's a fault, return it
        if (fault != NoFault) {
            if (req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        //If we don't need to access a second cache line, stop now.
        if (secondAddr <= addr)
        {
            if (req->isLocked() && fault == NoFault) {
                assert(!locked);
                locked = true;
            }
            return fault;
        }

        /*
         * Set up for accessing the second cache line.
         */

        //Move the pointer we're reading into to the correct location.
        data += size;
        //Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        //And access the right address.
        addr = secondAddr;
    }
}

Fault
DropSimpleCPU::writeMem(uint8_t *data, unsigned size,
                          Addr addr, unsigned flags, uint64_t *res)
{

    if (traceData) {
        traceData->setAddr(addr);
    }
    
    dcache_latency = 0;

    // use the CPU's statically allocated write request and packet objects
    Request *req = &data_write_req;

    //The block size of our peer.
    unsigned blockSize = dcachePort.peerBlockSize();
    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, blockSize);

    if(secondAddr > addr)
        size = secondAddr - addr;

    while(1) {
        req->setVirt(0, addr, size, flags, dataMasterId(), thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, tc, BaseTLB::Write);

        // Now do the access.
        if (fault == NoFault) {
            MemCmd cmd = MemCmd::WriteReq; // default
            bool do_access = true;  // flag to suppress cache access

            if (req->isLLSC()) {
                cmd = MemCmd::StoreCondReq;
                do_access = TheISA::handleLockedWrite(thread, req);
            } else if (req->isSwap()) {
                cmd = MemCmd::SwapReq;
                if (req->isCondSwap()) {
                    assert(res);
                    req->setExtraData(*res);
                }
            }

            if (do_access && !req->getFlags().isSet(Request::NO_ACCESS)) {
                Packet pkt = Packet(req, cmd);
                pkt.dataStatic(data);

                if (req->isMmappedIpr()) {
                    dcache_latency +=
                        TheISA::handleIprWrite(thread->getTC(), &pkt);
                } else {
                    if (fastmem && system->isMemAddr(pkt.getAddr()))
                        system->getPhysMem().access(&pkt);
                    else
                        dcache_latency += dcachePort.sendAtomic(&pkt);
                }
                dcache_access = true;
                assert(!pkt.isError());

                if (req->isSwap()) {
                    assert(res);
                    memcpy(res, pkt.getPtr<uint8_t>(), fullSize);
                }
            }

            if (res && !req->isSwap()) {
                *res = req->getExtraData();
            }
        }

        //If there's a fault or we don't need to access a second cache line,
        //stop now.
        if (fault != NoFault || secondAddr <= addr)
        {
            if (req->isLocked() && fault == NoFault) {
                assert(locked);
                locked = false;
            }
            if (fault != NoFault && req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        /*
         * Set up for accessing the second cache line.
         */

        //Move the pointer we're reading into to the correct location.
        data += size;
        //Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        //And access the right address.
        addr = secondAddr;
    }
}

Fault
DropSimpleCPU::writeMemFunctional(uint8_t *data, unsigned size,
                                        Addr addr, unsigned flags, uint64_t *res)
{
    // use the CPU's statically allocated write request and packet objects
    Request *req = &data_write_req;

    //The block size of our peer.
    unsigned blockSize = dcachePort.peerBlockSize();
    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, blockSize);

    if(secondAddr > addr)
        size = secondAddr - addr;

    while(1) {
        req->setVirt(0, addr, size, flags, dataMasterId(), thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, tc, BaseTLB::Write);

        // Now do the access.
        if (fault == NoFault) {
            MemCmd cmd = MemCmd::WriteReq; // default
            bool do_access = true;  // flag to suppress cache access

            if (req->isLLSC()) {
                cmd = MemCmd::StoreCondReq;
                do_access = TheISA::handleLockedWrite(thread, req);
            } else if (req->isSwap()) {
                cmd = MemCmd::SwapReq;
                if (req->isCondSwap()) {
                    assert(res);
                    req->setExtraData(*res);
                }
            }

            if (do_access && !req->getFlags().isSet(Request::NO_ACCESS)) {
                Packet pkt = Packet(req, cmd);
                pkt.dataStatic(data);

                if (fastmem && system->isMemAddr(pkt.getAddr()))
                    system->getPhysMem().access(&pkt);
                else
                    dcachePort.sendFunctional(&pkt);

                assert(!pkt.isError());

                if (req->isSwap()) {
                    assert(res);
                    memcpy(res, pkt.getPtr<uint8_t>(), fullSize);
                }
            }

            if (res && !req->isSwap()) {
                *res = req->getExtraData();
            }
        }

        //If there's a fault or we don't need to access a second cache line,
        //stop now.
        if (fault != NoFault || secondAddr <= addr)
        {
            if (req->isLocked() && fault == NoFault) {
                assert(locked);
                locked = false;
            }
            if (fault != NoFault && req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        /*
         * Set up for accessing the second cache line.
         */

        //Move the pointer we're reading into to the correct location.
        data += size;
        //Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        //And access the right address.
        addr = secondAddr;
    }
}

void
DropSimpleCPU::pageAllocate(Addr addr)
{
    Process *p = tc->getProcessPtr();
    // check if page exists
    if (!p->pTable->translate(addr)){
        // allocate page
        Addr page_addr = roundDown(addr, VMPageSize);
        p->allocateMonMem(page_addr, VMPageSize);
        // clear page
        uint8_t zero  = 0;
        SETranslatingPortProxy &tp = tc->getMemProxy();
        tp.memsetBlob(page_addr, zero, VMPageSize);
    }
}

unsigned
DropSimpleCPU::getCacheFlags(size_t size)
{
    unsigned cache_flags;
    switch (size){
        case 1: cache_flags = ArmISA::TLB::AlignByte | TheISA::TLB::MustBeOne; break;
        case 2: cache_flags = ArmISA::TLB::AlignHalfWord | TheISA::TLB::MustBeOne; break;
        case 4: cache_flags = ArmISA::TLB::AlignWord | TheISA::TLB::MustBeOne; break;
        default: panic("Unsupported read size.\n");
    }
    return cache_flags;
}

Fault
DropSimpleCPU::readFromFlagCache(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags)
{    
    Fault fault = NoFault;
    
    // On flag cache read
    if (addr == FC_GET_FLAG_C){
        DPRINTF(FlagCache, "Reading memory-backed cache\n");
        uint8_t invalid_bits = 0;
        // Get read address
        Addr fc_addr = 0;
        BaseSimpleCPU::readFromFlagCache(FC_GET_ADDR, (uint8_t *)&fc_addr, sizeof(fc_addr), ArmISA::TLB::AllowUnaligned);
        // Get aligned address
        Addr cache_addr = roundDown(fc_addr, 8*sizeof(invalid_bits));
        // Allocate page if it doesn't exist
        pageAllocate(cache_addr);
        // Read byte address (with latency)
        unsigned cache_flags = getCacheFlags(sizeof(invalid_bits));
        fault = readMem(cache_addr, &invalid_bits, sizeof(invalid_bits), cache_flags);
        // Bit mask read
        invalid_bits &= (1 << (fc_addr&(8*sizeof(invalid_bits)-1)));
        // Convert to flag
        uint64_t value = (invalid_bits != 0);
        // Copy back data
        memcpy(data, &value, size);
        DPRINTF(FlagCache, "Flag cache read @ %x -> %x: %x -> %d\n", fc_addr, cache_addr, invalid_bits, value);
        numCacheLoads++;
    } else {
        fault = BaseSimpleCPU::readFromFlagCache(addr, data, size, flags);
        if (addr == FC_GET_FLAG_A) {
            numRegLoads++;
        }
    }
    
    return fault;
}

Fault
DropSimpleCPU::writeToFlagCache(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags)
{
    Fault fault;
    uint64_t get_data = 0;
    memcpy(&get_data, data, size);
    
    // On flag cache write
    if ((addr == FC_SET_FLAG || addr == FC_CLEAR_FLAG) && get_data == 1){
        DPRINTF(FlagCache, "Setting memory-backed cache\n");
        uint8_t invalid_bits = 0;
        // Get write address
        Addr fc_addr = 0;
        BaseSimpleCPU::readFromFlagCache(FC_GET_ADDR, (uint8_t *)&fc_addr, sizeof(fc_addr), ArmISA::TLB::AllowUnaligned);
        // Get aligned address
        Addr cache_addr = roundDown(fc_addr, 8*sizeof(invalid_bits));
        // Allocate page if it doesn't exist
        pageAllocate(cache_addr);
        // Read byte address (with latency)
        unsigned cache_flags = getCacheFlags(sizeof(invalid_bits));
        fault = readMem(cache_addr, &invalid_bits, sizeof(invalid_bits), cache_flags);
        if (fault != NoFault) { return fault; }
        // Bit mask write
        uint8_t invalid_bits_new = 0;
        uint8_t mask = (1 << (fc_addr&7));
        if (addr == FC_SET_FLAG) {
            invalid_bits_new = invalid_bits | mask;
        } else if (addr == FC_CLEAR_FLAG) {
            invalid_bits_new = invalid_bits & ~mask;
        }
        // Write back byte (functional)
        fault = writeMemFunctional(&invalid_bits_new, sizeof(invalid_bits_new), cache_addr, cache_flags, NULL);
        
        if (addr == FC_SET_FLAG) {
            DPRINTF(FlagCache, "Flag cache set @ %x -> %x: invalid bits are %x -> %x\n", fc_addr, cache_addr, invalid_bits, invalid_bits_new);
        } else if (addr == FC_CLEAR_FLAG) {
            DPRINTF(FlagCache, "Flag cache clear @ %x -> %x: invalid bits are %x -> %x\n", fc_addr, cache_addr, invalid_bits, invalid_bits_new);
        }
        
        numCacheStores++;
    } else {
        fault = BaseSimpleCPU::writeToFlagCache(addr, data, size, flags);
        
        if ((addr == FC_SET_FLAG || addr == FC_CLEAR_FLAG) && get_data == 0){
            numRegStores++;
        }
    }

    return fault;
}

// Send monitoring packet to monitoring core
bool 
DropSimpleCPU::forwardFifoPacket() {
  // Create request
  Request *req = &data_write_req;
  // set physical address
  req->setPhys((Addr)FIFO_ADDR, sizeof(mp), ArmISA::TLB::AllowUnaligned, dataMasterId());

  // Create write packet
  PacketPtr fifopkt = new Packet(req, MemCmd::WriteReq);
  // Set data
  fifopkt->dataStatic(&mp);
  // Send request
  bool success = forwardFifoPort.sendTimingReq(fifopkt);
  // Clean up
  delete fifopkt;
  if (success){
    #ifdef DEBUG
      DPRINTF(Fifo, "Forwarded packet with values: %x, %x, %x, %x, %x, %x, %x\n", mp.valid, mp.instAddr, mp.memAddr, mp.memEnd, mp.data, mp.store, mp.done);
    #endif
  }
  
  return success;
}

bool DropSimpleCPU::getInstructionPriority(Addr addr)
{
    if (ipt_impl == TABLE) {
        if (ipt_tagged) {
            if (ipt.valid(addr))
                return ipt.lookup(addr);
            else
                return false;
        } else {
            return ipt.lookup(addr);
        }
    } else if (ipt_impl == BLOOM_FILTER) {
        return ipt_bloom.lookup(addr);
    } else {
        panic("Invalid IPT implementation!\n");
    }
}

void DropSimpleCPU::setInstructionPriority(Addr addr, const bool priority)
{
    if (ipt_impl == TABLE) {
        ipt.update(addr, priority);
        numIITWrites++;
    } else if (ipt_impl == BLOOM_FILTER) {
        // skip update if already in bloom filter
        if (!ipt_bloom.lookup(addr))
            ipt_bloom.update(addr, true);
        numIITWrites++;
    } else {
        panic("Invalid IPT implementation!\n");
    }
}

/**
 * Backtrack to identify important instructions
 * @return Whether the current instruction is important or not
 */
bool
DropSimpleCPU::backtrack()
{
    // execute monitoring operations
    if (monitorExt == MONITOR_UMC)
        return backtrack_umc();
    else if (monitorExt == MONITOR_DIFT)
        return backtrack_dift();
    else if (monitorExt == MONITOR_HB)
        return backtrack_hb();
    else
        return false;
}

bool
DropSimpleCPU::backtrack_hb()
{
    monitoringPacket mpkt;
    bool important = false;
    readFromFifo(FIFO_PACKET, (uint8_t *)&mpkt, sizeof(mpkt), ArmISA::TLB::AllowUnaligned);
    important = getInstructionPriority(mpkt.instAddr);

    // determine instruction type
    if (mpkt.load) {
        backtrack_inst_load(mpkt);

        // update producer table
        Addr rd = mpkt.rd;
        rptb.update(rd, mpkt.instAddr, 0);

        // update instruction priority table
        setInstructionPriority(mpkt.instAddr, true);

    } else if (mpkt.store) {
        backtrack_inst_store(mpkt);

        // update producer table
        Addr addr = mpkt.memAddr;
        mptb.update(addr, mpkt.instAddr);

        // update instruction priority table
        setInstructionPriority(mpkt.instAddr, true);

    } else if (mpkt.intalu) {
        if (important) {
            backtrack_inst_intalu(mpkt);
        }

        // update producer table
        Addr addr = mpkt.rd;
        rptb.update(addr, mpkt.instAddr, 0);

    }

    return important;
}

bool
DropSimpleCPU::backtrack_umc()
{
    monitoringPacket mpkt;
    bool important = false;
    readFromFifo(FIFO_PACKET, (uint8_t *)&mpkt, sizeof(mpkt), ArmISA::TLB::AllowUnaligned);
    important = getInstructionPriority(mpkt.instAddr);

    // determine instruction type
    if (mpkt.load) {
        backtrack_inst_load(mpkt);

        // update producer table
        Addr rd = mpkt.rd;
        rptb.update(rd, mpkt.instAddr, 0);

        // update instruction priority table
        setInstructionPriority(mpkt.instAddr, true);

    } else if (mpkt.store) {
        // update producer table
        Addr addr = mpkt.memAddr;
        mptb.update(addr, mpkt.instAddr);
    }

    return important;
}

bool
DropSimpleCPU::backtrack_dift()
{
    monitoringPacket mpkt;
    bool important = false;
    readFromFifo(FIFO_PACKET, (uint8_t *)&mpkt, sizeof(mpkt), ArmISA::TLB::AllowUnaligned);
    important = getInstructionPriority(mpkt.instAddr);

    // determine instruction type
    if (mpkt.indctrl) {
        backtrack_inst_indctrl(mpkt);

        // update instruction priority table
        setInstructionPriority(mpkt.instAddr, true);

    } else if (mpkt.load) {
        if (important) {
            backtrack_inst_load(mpkt);
        }

        // update producer table
        Addr rd = mpkt.rd;
        rptb.update(rd, mpkt.instAddr, 0);

    } else if (mpkt.store) {
        if (important) {
            backtrack_inst_store(mpkt);
        }

        // update producer table
        Addr addr = mpkt.memAddr;
        mptb.update(addr, mpkt.instAddr);

    } else if (mpkt.intalu) {
        if (important) {
            backtrack_inst_intalu(mpkt);
        }

        // update producer table
        Addr addr = mpkt.rd;
        rptb.update(addr, mpkt.instAddr, 0);

    }

    return important;
}

void DropSimpleCPU::backtrack_inst_indctrl(monitoringPacket &mpkt)
{
    // indirect control transfer instruction with incvalidated data initiates backtracking
    Addr addr = mpkt.rs1;

#ifdef DEBUG
    uint8_t flag;
    writeToFlagCache(FC_SET_ADDR, (uint8_t *)&addr, sizeof(Addr), ArmISA::TLB::AllowUnaligned);
    readFromFlagCache(FC_GET_FLAG_A, (uint8_t *)&flag, sizeof(flag), ArmISA::TLB::AllowUnaligned);
    if (flag) {
        // invalidated
        DPRINTF(Backtrack, "indirect control transfer instruction invalidated\n");
    }
#endif

    if (!TheISA::isISAReg(addr))
        return;
    // find out the producer of rs1
    if (rptb.valid(addr)) {
        Addr producer1 = rptb.lookup1(addr);
        if (producer1 != 0) {
            // mark producer as important
            setInstructionPriority(producer1, true);
        }
    } else {
        DPRINTF(Backtrack, "warning: cannot find producer of r%d, instAddr=0x%x\n", addr, mpkt.instAddr);
    }
}

void DropSimpleCPU::backtrack_inst_load(monitoringPacket &mpkt)
{
    Addr addr = mpkt.memAddr;
    // find out producer of memory address
    if (mptb.valid(addr)) {
        Addr producer = mptb.lookup(addr);
        if (producer != 0) {
            setInstructionPriority(producer, true);
            DPRINTF(Backtrack, "mark producer(0x%x)=0x%x as important, instAddr=0x%x\n", addr, producer, mpkt.instAddr);
        }
    } else {
        DPRINTF(Backtrack, "warning: cannot find producer of 0x%x, instAddr=0x%x\n", addr, mpkt.instAddr);
    }
}

void DropSimpleCPU::backtrack_inst_store(monitoringPacket &mpkt)
{
    Addr src = mpkt.rs2;

    if (!TheISA::isISAReg(src))
        return;
    // find out the producer of rs2
    if (rptb.valid(src)) {
        Addr producer1 = rptb.lookup1(src);
        if (producer1 != 0) {
            // mark producer as important
            setInstructionPriority(producer1, true);
            DPRINTF(Backtrack, "mark producer(r%d)=0x%x as important, instAddr=0x%x\n", src, producer1, mpkt.instAddr);
        }
    } else {
        DPRINTF(Backtrack, "warning: cannot find producer of r%d, instAddr=0x%x\n", src, mpkt.instAddr);
    }
}

void DropSimpleCPU::backtrack_inst_intalu(monitoringPacket &mpkt)
{
    // find producers of sources
    Addr rs1 = mpkt.rs1;
    Addr rs2 = mpkt.rs2;

    if (!TheISA::isISAReg(rs1))
        return;
    // find out the producer of rs1
    if (rptb.valid(rs1)) {
        Addr producer1 = rptb.lookup1(rs1);
        if (producer1 != 0) {
            // mark producer as important
            setInstructionPriority(producer1, true);
            DPRINTF(Backtrack, "mark producer(r%d)=0x%x as important, instAddr=0x%x\n", rs1, producer1, mpkt.instAddr);
        }
    } else {
        DPRINTF(Backtrack, "warning: cannot find producer of r%d, instAddr=0x%x\n", rs1, mpkt.instAddr);
    }

    if (!TheISA::isISAReg(rs2))
        return;
    // find out the producer of rs2
    if (rptb.valid(rs2)) {
        Addr producer1 = rptb.lookup1(rs2);
        if (producer1 != 0) {
            // mark producer as important
            setInstructionPriority(producer1, true);
            DPRINTF(Backtrack, "mark producer(r%d)=0x%x as important, instAddr=0x%x\n", rs2, producer1, mpkt.instAddr);
        }
    } else {
        DPRINTF(Backtrack, "warning: cannot find producer of r%d, instAddr=0x%x\n", rs2, mpkt.instAddr);
    }
}

/**
 * selective dropping
 */
void
DropSimpleCPU::adjustDropThreshold()
{
    int64_t actual_overhead_status;
    // Create request
    Request *timer_read_request = &data_read_req;
    // set physical address
    timer_read_request->setPhys(TIMER_ACTUAL_OVERHEAD_STATUS, sizeof(actual_overhead_status), ArmISA::TLB::AllowUnaligned, dataMasterId());
    // Create read packet
    MemCmd cmd = MemCmd::ReadReq;
    PacketPtr timerpkt = new Packet(timer_read_request, cmd);
    // Set data
    timerpkt->dataStatic(&actual_overhead_status);
    // Send read request packet on timer port
    timerPort.sendFunctional(timerpkt);
    // Clean up
    delete timerpkt;

    if (actual_overhead_status > 0)
        drop_threshold++;
    else if (actual_overhead_status < 0)
        drop_threshold--;
}

bool
DropSimpleCPU::addToCheckSet(Addr inst_addr, Addr check_addr)
{
    InstructionMetadata *metadata = imt.lookup(inst_addr);
    // lazy initialization
    if (metadata == NULL) {
        metadata = new InstructionMetadata();
        metadata->checks = new std::set<Addr>();
        imt.update(inst_addr, metadata);
    }
    if ((max_check_set_size > 0) && (metadata->checks->size() >= max_check_set_size)) {
        // we have reached maximum check set size, do nothing
        return false;
    } else {
        std::pair<std::set<Addr>::iterator,bool> ret;
        ret = metadata->checks->insert(check_addr);
        return ret.second;
    }
}

bool
DropSimpleCPU::mergeCheckSets(Addr inst_addr_producer, Addr inst_addr_consumer)
{
    InstructionMetadata *consumer_metadata = imt.lookup(inst_addr_consumer);
    if (consumer_metadata == NULL)
        return false;
    // handle maximum check set size
    if (max_check_set_size > 0) {
        InstructionMetadata *producer_metadata = imt.lookup(inst_addr_producer);
        if ((producer_metadata != NULL) && (producer_metadata->checks->size() >= max_check_set_size))
            return false;
    }

    std::set<Addr>::iterator it;
    bool changed = false;
    for (it = consumer_metadata->checks->begin(); it != consumer_metadata->checks->end(); ++it) {
        changed = addToCheckSet(inst_addr_producer, *it) || changed;
    }
    return changed;
}

/**
 * Backtrace to compute instruction metadata
 */
void
DropSimpleCPU::backtrace_metadata()
{
    if (!compute_check_sets)
        return;

    if (monitorExt == MONITOR_UMC)
        backtrace_metadata_umc();
    else if (monitorExt == MONITOR_DIFT)
        backtrace_metadata_dift();
    else if (monitorExt == MONITOR_HB)
        backtrace_metadata_hb();
}

void
DropSimpleCPU::backtrace_metadata_hb()
{
    monitoringPacket mpkt;
    readFromFifo(FIFO_PACKET, (uint8_t *)&mpkt, sizeof(mpkt), ArmISA::TLB::AllowUnaligned);

    // determine instruction type
    if (mpkt.load) {
        // LOAD instructions are checks for HB
        addToCheckSet(mpkt.instAddr, mpkt.instAddr);
        // do backtracing
        backtrace_metadata_inst_load(mpkt);

        // update producer table
        Addr rd = mpkt.rd;
        rptb.update(rd, mpkt.instAddr, 0);
    } else if (mpkt.store) {
        // STORE instructions are checks for HB
        addToCheckSet(mpkt.instAddr, mpkt.instAddr);
        // do backtracing
        backtrace_metadata_inst_store(mpkt);

        // update producer table
        Addr addr = mpkt.memAddr;
        mptb.update(addr, mpkt.instAddr);
    } else if (mpkt.intalu) {
        backtrace_metadata_inst_intalu(mpkt);

        // update producer table
        Addr addr = mpkt.rd;
        rptb.update(addr, mpkt.instAddr, 0);
    }
}

void
DropSimpleCPU::backtrace_metadata_dift()
{
    monitoringPacket mpkt;
    readFromFifo(FIFO_PACKET, (uint8_t *)&mpkt, sizeof(mpkt), ArmISA::TLB::AllowUnaligned);

    // determine instruction type
    if (mpkt.indctrl) {
        // INDCTRL instructions are checks for DIFT
        addToCheckSet(mpkt.instAddr, mpkt.instAddr);
        // do backtracing
        backtrace_metadata_inst_indctrl(mpkt);
    } else if (mpkt.load) {
        backtrace_metadata_inst_load(mpkt);

        // update producer table
        Addr rd = mpkt.rd;
        rptb.update(rd, mpkt.instAddr, 0);
    } else if (mpkt.store) {
        backtrace_metadata_inst_store(mpkt);

        // update producer table
        Addr addr = mpkt.memAddr;
        mptb.update(addr, mpkt.instAddr);
    } else if (mpkt.intalu) {
        backtrace_metadata_inst_intalu(mpkt);

        // update producer table
        Addr addr = mpkt.rd;
        rptb.update(addr, mpkt.instAddr, 0);
    }
}

void
DropSimpleCPU::backtrace_metadata_umc()
{
    monitoringPacket mpkt;
    readFromFifo(FIFO_PACKET, (uint8_t *)&mpkt, sizeof(mpkt), ArmISA::TLB::AllowUnaligned);

    // determine instruction type
    if (mpkt.load) {
        // LOAD instructions are checks for UMC
        addToCheckSet(mpkt.instAddr, mpkt.instAddr);
        // do backtracing
        backtrace_metadata_inst_load(mpkt);

        // update producer table
        Addr rd = mpkt.rd;
        rptb.update(rd, mpkt.instAddr, 0);
    } else if (mpkt.store) {
        // update producer table
        Addr addr = mpkt.memAddr;
        mptb.update(addr, mpkt.instAddr);
    }
}

void
DropSimpleCPU::backtrace_metadata_inst_indctrl(monitoringPacket &mpkt)
{
    // indirect control transfer does not write any registers
    Addr addr = mpkt.rs1;
    if (!TheISA::isISAReg(addr))
        return;
    // find out the producer of rs1
    if (rptb.valid(addr)) {
        Addr producer = rptb.lookup1(addr);
        if (producer != 0) {
            // add instruction to producer's check set
            addToCheckSet(producer, mpkt.instAddr);
        }
        if (compute_optimal_dropping && isOptimalDroppingPoint(mpkt.instAddr, producer))
            markOptimalDroppingPoint(mpkt.instAddr);
    } else {
        DPRINTF(Backtrack, "warning: cannot find producer of r%d, instAddr=0x%x\n", addr, mpkt.instAddr);
    }
}

void
DropSimpleCPU::backtrace_metadata_inst_load(monitoringPacket &mpkt)
{
    Addr addr = mpkt.memAddr;
    // find out producer of memory address
    if (mptb.valid(addr)) {
        Addr producer = mptb.lookup(addr);
        if (producer != 0) {
            mergeCheckSets(producer, mpkt.instAddr);
        }
        if (compute_optimal_dropping && isOptimalDroppingPoint(mpkt.instAddr, producer))
            markOptimalDroppingPoint(mpkt.instAddr);
    } else {
        DPRINTF(Backtrack, "warning: cannot find producer of 0x%x, instAddr=0x%x\n", addr, mpkt.instAddr);
    }
}

void
DropSimpleCPU::backtrace_metadata_inst_store(monitoringPacket &mpkt)
{
    Addr src = mpkt.rs2;

    if (!TheISA::isISAReg(src))
        return;
    // find out the producer of rs2
    if (rptb.valid(src)) {
        Addr producer = rptb.lookup1(src);
        if (producer != 0) {
            mergeCheckSets(producer, mpkt.instAddr);
        }
        if (compute_optimal_dropping && isOptimalDroppingPoint(mpkt.instAddr, producer))
            markOptimalDroppingPoint(mpkt.instAddr);
    } else {
        DPRINTF(Backtrack, "warning: cannot find producer of r%d, instAddr=0x%x\n", src, mpkt.instAddr);
    }
}

void
DropSimpleCPU::backtrace_metadata_inst_intalu(monitoringPacket &mpkt)
{
    // find producers of sources
    Addr rs1 = mpkt.rs1;
    Addr rs2 = mpkt.rs2;
    bool optimal_1 = false, optimal_2 = false;

    if (!TheISA::isISAReg(rs1))
        return;
    // find out the producer of rs1
    if (rptb.valid(rs1)) {
        Addr producer1 = rptb.lookup1(rs1);
        if (producer1 != 0) {
            mergeCheckSets(producer1, mpkt.instAddr);
        }
        optimal_1 = compute_optimal_dropping && isOptimalDroppingPoint(mpkt.instAddr, producer1);
    } else {
        DPRINTF(Backtrack, "warning: cannot find producer of r%d, instAddr=0x%x\n", rs1, mpkt.instAddr);
    }

    if (!TheISA::isISAReg(rs2))
        return;
    // find out the producer of rs2
    if (rptb.valid(rs2)) {
        Addr producer2 = rptb.lookup1(rs2);
        if (producer2 != 0) {
            mergeCheckSets(producer2, mpkt.instAddr);
        }
        optimal_2 = compute_optimal_dropping && isOptimalDroppingPoint(mpkt.instAddr, producer2);
    } else {
        DPRINTF(Backtrack, "warning: cannot find producer of r%d, instAddr=0x%x\n", rs2, mpkt.instAddr);
    }
    // only mark optimal dropping points when both producers are supersets
    if (optimal_1 && optimal_2)
        markOptimalDroppingPoint(mpkt.instAddr);
}

bool
DropSimpleCPU::isOptimalDroppingPoint(Addr inst_addr, Addr producer_addr)
{
    InstructionMetadata *inst_metadata = imt.lookup(inst_addr);
    InstructionMetadata *producer_metadata = imt.lookup(producer_addr);
    if (producer_metadata == NULL)
        return false;
    std::set<Addr> *producer_check_set = producer_metadata->checks;
    if (inst_metadata == NULL)
        return producer_check_set->size() > 0 ? true : false;
    std::set<Addr> *inst_check_set = inst_metadata->checks;
    // decide whether producer_check_set is a strict superset of inst_check_set
    if (producer_check_set->size() <= inst_check_set->size())
        return false;
    // handle huge sets
    if ((max_check_set_size > 0) && (producer_check_set->size() == max_check_set_size)) {
        // if the producer has a huge set, use a heuristic to determine optimality
        return max_check_set_size > inst_check_set->size() * 2 ? true : false;
    }
    std::set<Addr>::iterator it;
    for (it = inst_check_set->begin(); it != inst_check_set->end(); ++it) {
        if (producer_check_set->find(*it) == producer_check_set->end())
            return false;
    }
    return true;
}

void
DropSimpleCPU::markOptimalDroppingPoint(Addr inst_addr)
{
    odt.update(inst_addr, true);
}

bool
DropSimpleCPU::inOptimalDroppingTable()
{
    monitoringPacket mpkt;
    readFromFifo(FIFO_PACKET, (uint8_t *)&mpkt, sizeof(mpkt), ArmISA::TLB::AllowUnaligned);
    return odt.lookup(mpkt.instAddr);
}

void
DropSimpleCPU::tick()
{
    DPRINTF(SimpleCPU, "Tick\n");
    DPRINTF(DropSimpleCPU, "Tick\n");

    Tick latency = 0;

    numCycles++;

    dcache_latency = 0;
    dcache_access = false;
    
    Tick stall_ticks = 0;
    
    bool forward_successful = true;

    // reset instruction importance
    // _important = false;
    
    // If we have a valid monitoring packet, then send it along
    if (mp.valid){
        if (forward_fifo_enabled) {
            // Send packet to monitoring core
            forward_successful = forwardFifoPacket();
        }
        // If succesful, then clear the monitoring packet
        if (forward_successful){
            mp.init();
        }
    }
    
    if (forward_successful){
        bool drop = false;
        Fault fault = ReExecFault;
        // If invalidation is enabled
        if (timer_enabled){
            // Read from timer. Timer automatically performs filtering/drop if needed.
            fault = readFromTimer(TIMER_READ_DROP, (uint8_t *)&drop, sizeof(drop), ArmISA::TLB::AllowUnaligned);
        // Not invalidating, but FIFO still exists
        } else if (fifo_enabled){
            // Pop an entry off the incoming fifo from main core
            bool pop = true;
            fault = writeToFifo(FIFO_NEXT, (uint8_t *)&pop, sizeof(pop), ArmISA::TLB::AllowUnaligned);
        }
        // If no faults (no drop/filter), read a monitoring packet to send to
        // the monitoring core
        if (fault == NoFault){
            DPRINTF(Invalidation, "Perform full monitoring.\n");
            if (fifo_enabled){
                // Read the monitoring packet
                readFromFifo(FIFO_PACKET, (uint8_t *)&mp, sizeof(mp), ArmISA::TLB::AllowUnaligned);
            }
        } else {
            fault->invoke(tc, curStaticInst);
        }
    }
    
    if (simulate_data_stalls && dcache_access)
        stall_ticks += dcache_latency;

    if (stall_ticks) {
        Tick stall_cycles = stall_ticks / ticks(1);
        Tick aligned_stall_ticks = ticks(stall_cycles);

        if (aligned_stall_ticks < stall_ticks)
            aligned_stall_ticks += ticks(1);

        latency += aligned_stall_ticks;
    }
    
    // instruction takes at least one cycle
    if (latency < ticks(1))
        latency = ticks(1);

    if (_status != Idle) {
        schedule(tickEvent, roundDown(curTick() + latency + 1, 2) - 1);
    }
        
}

void
DropSimpleCPU::printAddr(Addr a)
{
    dcachePort.printAddr(a);
}

MasterPort &
DropSimpleCPU::getMasterPort(const std::string &if_name, int idx)
{
  if (if_name == "forward_fifo_port") {
    return forwardFifoPort;
  } else {
    return BaseSimpleCPU::getMasterPort(if_name, idx);
  }
}

SlavePort &
DropSimpleCPU::getSlavePort(const std::string &if_name, int idx)
{
  if (if_name == "monitor_port") {
    return monitorPort;
  } else {
    return BaseCPU::getSlavePort(if_name, idx);
  }
}

DropSimpleCPU::MonitorPort::MonitorPort(const std::string& _name,
                                        DropSimpleCPU *_cpu)
    : SimpleTimingPort(_name, _cpu), cpu(_cpu)
{ }

AddrRangeList
DropSimpleCPU::MonitorPort::getAddrRanges()
{
    AddrRangeList ranges;
    return ranges;
}

Tick
DropSimpleCPU::MonitorPort::recvAtomic(PacketPtr pkt)
{
    recvFunctional(pkt);
    // FIXME: zero latency
    return 0;
}

void
DropSimpleCPU::MonitorPort::recvFunctional(PacketPtr pkt)
{
    if (!queue.checkFunctional(pkt)) {
        if (pkt->cmd == MemCmd::WriteReq) {
            uint8_t type;
            bool set = false; // set flag if true, clear if false
            switch (pkt->getAddr()){
                case DROP_CLEAR_ARRAY: type = 0; break;
                case DROP_CLEAR_CACHE: type = 1; break;
                case DROP_FC_SET_ADDR: type = 2; break;
                case DROP_SET_ARRAY:   type = 0; set = true; break;
                case DROP_SET_CACHE:   type = 1; set = true; break;
                default: panic ("Unimplemented port request");
            }
            if (cpu->flagcache_enabled){
                cpu->writeToFlagCache(FC_SET_ADDR, pkt->getPtr<uint8_t>(), pkt->getSize(), ArmISA::TLB::AllowUnaligned);
                if (type == 0 || type == 1) {
                  if (set) {
                    cpu->writeToFlagCache(FC_SET_FLAG, &type, sizeof(type), ArmISA::TLB::AllowUnaligned);
                  } else {
                    cpu->writeToFlagCache(FC_CLEAR_FLAG, &type, sizeof(type), ArmISA::TLB::AllowUnaligned);
                  }
                }
            }
        } else {
            panic ("Unimplemented port request");
        }
    }
}

bool
DropSimpleCPU::MonitorPort::recvTimingReq(PacketPtr pkt)
{
    // Use functional function to handle writing
    recvFunctional(pkt);
    // Indicate write accepted
    return true;
}



////////////////////////////////////////////////////////////////////////
//
//  DropSimpleCPU Simulation Object
//
DropSimpleCPU *
DropSimpleCPUParams::create()
{
    numThreads = 1;
    if (!FullSystem && workload.size() != 1)
        panic("only one workload allowed");
    return new DropSimpleCPU(this);
}
