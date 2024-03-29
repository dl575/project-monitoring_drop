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
    // initialize structures for optimal dropping
    odt.init();
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
      read_optimal_dropping(p->read_optimal_dropping),
      odt(p->ipt_tagged, p->ipt_size/p->ipt_entry_size, p->ipt_entry_size, 30-floorLog2(p->ipt_size), 2)
{
    _status = Idle;    

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
        uint8_t fc_flags = 0;
        // Get read address
        Addr fc_addr = 0;
        BaseSimpleCPU::readFromFlagCache(FC_GET_ADDR, (uint8_t *)&fc_addr, sizeof(fc_addr), ArmISA::TLB::AllowUnaligned);
        // Get aligned address
        // We take the address and right shift it by at most 3 spaces.
        Addr cache_addr = fc_addr >> (3-FC_LOGBITS);
        // Allocate page if it doesn't exist
        pageAllocate(cache_addr);
        // Read data at address (with latency)
        unsigned cache_flags = getCacheFlags(sizeof(fc_flags));
        fault = readMem(cache_addr, &fc_flags, sizeof(fc_flags), cache_flags);
        // Find position address for the fc_flags, i.e. where in the fc_flags the relevant bits are
        // Basically, we take the last (3-FC_LOGBITS) bits
        // and we stride it using FC_NUMBITS
        Addr position_addr = (fc_addr&((1<<(3-FC_LOGBITS))-1))*FC_NUMBITS;
        // Now calculate the flag
        // We use the maximum flag value to mask into the data at the position address and shift back
        uint64_t value = ((fc_flags & (FC_MAXVAL << position_addr)) >> position_addr);
        // Copy back data
        memcpy(data, &value, size);
        DPRINTF(FlagCache, "Flag cache read @ %x -> {%x,%x}: %x -> %d\n", fc_addr, cache_addr, position_addr, fc_flags, value);
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
        uint8_t fc_flags = 0;
        // Get read address
        Addr fc_addr = 0;
        BaseSimpleCPU::readFromFlagCache(FC_GET_ADDR, (uint8_t *)&fc_addr, sizeof(fc_addr), ArmISA::TLB::AllowUnaligned);
        // Get aligned address
        // We take the address and right shift it by at most 3 spaces. 
        Addr cache_addr = fc_addr >> (3-FC_LOGBITS);
        // Allocate page if it doesn't exist
        pageAllocate(cache_addr);
        // Read data at address (with latency)
        unsigned cache_flags = getCacheFlags(sizeof(fc_flags));
        fault = readMem(cache_addr, &fc_flags, sizeof(fc_flags), cache_flags);
        if (fault != NoFault) { return fault; }
        // Find position address for the fc_flags, i.e. where in the fc_flags the relevant bits are
        // Basically, we take the last (3-FC_LOGBITS) bits
        // and we stride it using FC_NUMBITS
        Addr position_addr = (fc_addr&((1<<(3-FC_LOGBITS))-1))*FC_NUMBITS;
        // Bit mask write
        uint8_t fc_flags_new = 0;
        uint8_t mask = (FC_MAXVAL << position_addr);
        if (addr == FC_SET_FLAG) {
            fc_flags_new = fc_flags | mask;
        } else if (addr == FC_CLEAR_FLAG) {
            fc_flags_new = fc_flags & ~mask;
        }
        // Write back byte (functional)
        fault = writeMemFunctional(&fc_flags_new, sizeof(fc_flags_new), cache_addr, cache_flags, NULL);
        
        if (addr == FC_SET_FLAG) {
            warn_once("Using FC_SET_FLAG is depricated. Use the new FlagCache interface.\n");
            DPRINTF(FlagCache, "Flag cache set @ %x -> {%x,%x}: flag bits are %x -> %x\n", fc_addr, cache_addr, position_addr, fc_flags, fc_flags_new);
        } else if (addr == FC_CLEAR_FLAG) {
            warn_once("Using FC_CLEAR_FLAG is depricated. Use the new FlagCache interface.\n");
            DPRINTF(FlagCache, "Flag cache clear @ %x -> {%x,%x}: flag bits are %x -> %x\n", fc_addr, cache_addr, position_addr, fc_flags, fc_flags_new);
        }
        numCacheStores++;
    // Write a value to flag cache
    } else if (addr == FC_SET_CACHE) {
        DPRINTF(FlagCache, "Setting memory-backed cache\n");
        uint8_t fc_flags = 0;
        // Get read address
        Addr fc_addr = 0;
        BaseSimpleCPU::readFromFlagCache(FC_GET_ADDR, (uint8_t *)&fc_addr, sizeof(fc_addr), ArmISA::TLB::AllowUnaligned);
        // Get aligned address
        // We take the address and right shift it by at most 3 spaces. 
        Addr cache_addr = fc_addr >> (3-FC_LOGBITS);
        // Allocate page if it doesn't exist
        pageAllocate(cache_addr);
        // Read data at address (with latency)
        unsigned cache_flags = getCacheFlags(sizeof(fc_flags));
        fault = readMem(cache_addr, &fc_flags, sizeof(fc_flags), cache_flags);
        if (fault != NoFault) { return fault; }
        // Find position address for the fc_flags, i.e. where in the fc_flags the relevant bits are
        // Basically, we take the last (3-FC_LOGBITS) bits
        // and we stride it using FC_NUMBITS
        Addr position_addr = (fc_addr&((1<<(3-FC_LOGBITS))-1))*FC_NUMBITS;
        // Bit mask write
        // Make sure data does not exceed maximum value
        uint8_t write_data = (get_data > FC_MAXVAL)? FC_MAXVAL : get_data;
        // Create mask to clear out existing data
        uint8_t clear_mask = ~(FC_MAXVAL << position_addr);
        // Clear existing data and OR new data shifted to correct position
        uint8_t fc_flags_new = (fc_flags & clear_mask) | (write_data << position_addr);
        // Write back byte (functional)
        fault = writeMemFunctional(&fc_flags_new, sizeof(fc_flags_new), cache_addr, cache_flags, NULL);
        DPRINTF(FlagCache, "Flag cache set value %d @ %x -> {%x,%x}: flag bits are %x -> %x\n", write_data, fc_addr, cache_addr, position_addr, fc_flags, fc_flags_new);
        numCacheStores++;
    // Write to flag array (register file)
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

/*
 * Return whether the forwardFifo (drop core to monitor core) is full.
 */
bool
DropSimpleCPU::forwardFifoPortFull()
{
  // Create new request
  Request *req = &data_read_req;
  // Full flag
  bool full;
  // set physical address
  req->setPhys((Addr)FIFO_FULL, sizeof(full), ArmISA::TLB::AllowUnaligned, dataMasterId());
  // Create read packet
  PacketPtr fifopkt = new Packet(req, MemCmd::ReadReq);
  // Set data
  fifopkt->dataStatic(&full);
  // Send request
  forwardFifoPort.sendFunctional(fifopkt);
  // Clean up
  delete fifopkt;

  return full;
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
        if (important) {
            backtrack_inst_store(mpkt);
        }

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
    Addr src = mpkt.rs1;

    if (!TheISA::isISAReg(src))
        return;
    // find out the producer of rs1
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
    // settag operations are considered optimal dropping points
    bool retval = mpkt.settag ? true : odt.lookup(mpkt.instAddr);
    return retval;
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
    
    // If we're not currently trying to send a FIFO packet and FIFO is not already full
    if (forward_successful && !forwardFifoPortFull()){
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
            switch (pkt->getAddr()){
                case DROP_CLEAR_ARRAY:
                    cpu->writeToFlagCache(FC_SET_ADDR, pkt->getPtr<uint8_t>(), pkt->getSize(), ArmISA::TLB::AllowUnaligned);
                    type = FC_ARRAY;
                    cpu->writeToFlagCache(FC_CLEAR_FLAG, &type, sizeof(type), ArmISA::TLB::AllowUnaligned);
                    break;
                case DROP_CLEAR_CACHE: 
                    cpu->writeToFlagCache(FC_SET_ADDR, pkt->getPtr<uint8_t>(), pkt->getSize(), ArmISA::TLB::AllowUnaligned);
                    type = FC_CACHE;
                    cpu->writeToFlagCache(FC_CLEAR_FLAG, &type, sizeof(type), ArmISA::TLB::AllowUnaligned);
                    break;
                case DROP_SET_ARRAY:
                    cpu->writeToFlagCache(FC_SET_ADDR, pkt->getPtr<uint8_t>(), pkt->getSize(), ArmISA::TLB::AllowUnaligned);
                    type = FC_ARRAY;
                    cpu->writeToFlagCache(FC_SET_FLAG, &type, sizeof(type), ArmISA::TLB::AllowUnaligned);
                    break;
                case DROP_SET_CACHE:
                    cpu->writeToFlagCache(FC_SET_ADDR, pkt->getPtr<uint8_t>(), pkt->getSize(), ArmISA::TLB::AllowUnaligned);
                    type = FC_CACHE;
                    cpu->writeToFlagCache(FC_CLEAR_FLAG, &type, sizeof(type), ArmISA::TLB::AllowUnaligned);
                    break;
                case DROP_FC_SET_ADDR:
                    cpu->writeToFlagCache(FC_SET_ADDR, pkt->getPtr<uint8_t>(), pkt->getSize(), ArmISA::TLB::AllowUnaligned);
                    break;
                case DROP_SET_ARRAY_VALUE:
                    cpu->writeToFlagCache(FC_SET_ARRAY, pkt->getPtr<uint8_t>(), pkt->getSize(), ArmISA::TLB::AllowUnaligned);
                    break;
                case DROP_SET_CACHE_VALUE:
                    cpu->writeToFlagCache(FC_SET_CACHE, pkt->getPtr<uint8_t>(), pkt->getSize(), ArmISA::TLB::AllowUnaligned);
                    break;
                default: panic ("Unimplemented port request");
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
