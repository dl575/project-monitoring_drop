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

    if (_backtrack && backtrack_read_table) {
        ifstream is;
        // read invalidation priority table
        is.open(backtrack_table_dir + "/ipt.table", std::ios::in | std::ios::binary);
        if (is.good())
            ipt.unserialize(is);
        else
            warn("invalidation priority table could not be opened.\n");
        is.close();
    }
}

DropSimpleCPU::DropSimpleCPU(DropSimpleCPUParams *p)
    : BaseSimpleCPU(p), tickEvent(this), width(p->width), locked(false),
      simulate_data_stalls(p->simulate_data_stalls),
      icachePort(name() + "-iport", this), dcachePort(name() + "-iport", this),
      monitorPort(name() + "-iport", this),
      fastmem(p->fastmem), forward_fifo_enabled(p->forward_fifo_enabled),
      forwardFifoPort(name() + "-iport", this),
      full_ticks(p->full_clock)
{
    _status = Idle;    

    // set up instruction-count-based event to write out backtrack tables
    // if (p->backtrack && p->backtrack_write_table) {
    //     if (p->max_insts_any_thread != 0) {
    //         const char *cause = "writing out backtrack tables max instruction count";
    //         for (ThreadID tid = 0; tid < numThreads; ++tid) {
    //             Event *event = new BacktrackTableWriteEvent(cause, p->backtrack_table_dir, &ipt, &mptb, &rptb);
    //             comInstEventQueue[tid]->schedule(event, p->max_insts_any_thread-1);
    //         }
    //     }    
    // }

    backtrack_write_table = p->backtrack_write_table;
    backtrack_read_table = p->backtrack_read_table;
    backtrack_table_dir = p->backtrack_table_dir;

    // set up callback at simulation exit to write out backtrack tables
    if (p->backtrack && p->backtrack_write_table) {
        Callback *cb = new MakeCallback<DropSimpleCPU, &DropSimpleCPU::writeBacktrackTable>(this);
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
    // write out invalidation priority table
    os.open(backtrack_table_dir + "/ipt.table", std::ios::out | std::ios::binary);
    if (os.good())
        ipt.serialize(os);
    else
        warn("invalidation priority table could not be opened.\n");
    os.close();
    
    // write out memory producer tracking table
    os.open(backtrack_table_dir + "/mptb.table", std::ios::out | std::ios::binary);
    if (os.good())
        mptb.serialize(os);
    else
        warn("invalidation priority table could not be opened.\n");
    os.close();
    
    // write out register producer tracking table
    os.open(backtrack_table_dir + "/rptb.table", std::ios::out | std::ios::binary);
    if (os.good())
        rptb.serialize(os);
    else
        warn("invalidation priority table could not be opened.\n");
    os.close();
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
            switch (pkt->getAddr()){
                case DROP_CLEAR_ARRAY: type = 0; break;
                case DROP_CLEAR_CACHE: type = 1; break;
                default: panic ("Unimplemented port request");
            }
            if (cpu->flagcache_enabled){
                cpu->writeToFlagCache(FC_SET_ADDR, pkt->getPtr<uint8_t>(), pkt->getSize(), ArmISA::TLB::AllowUnaligned);
                cpu->writeToFlagCache(FC_CLEAR_FLAG, &type, sizeof(type), ArmISA::TLB::AllowUnaligned);
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
