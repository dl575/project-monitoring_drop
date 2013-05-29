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

#include "arch/locked_mem.hh"
#include "arch/mmapped_ipr.hh"
#include "arch/utility.hh"
#include "base/bigint.hh"
#include "base/chunk_generator.hh"
#include "config/the_isa.hh"
#include "cpu/simple/atomic_monitor.hh"
#include "cpu/exetrace.hh"
#include "debug/ExecFaulting.hh"
#include "debug/SimpleCPU.hh"
#include "debug/Monitor.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/physical.hh"
#include "params/AtomicSimpleMonitor.hh"
#include "sim/faults.hh"
#include "sim/system.hh"
#include "sim/full_system.hh"

using namespace std;
using namespace TheISA;

SimpleThread* monitor_thread;

AtomicSimpleMonitor::TickEvent::TickEvent(AtomicSimpleMonitor *c)
    : Event(CPU_Tick_Pri), cpu(c)
{
}


void
AtomicSimpleMonitor::TickEvent::process()
{
    cpu->tick();
}

const char *
AtomicSimpleMonitor::TickEvent::description() const
{
    return "AtomicSimpleMonitor tick";
}

void
AtomicSimpleMonitor::init()
{
    BaseCPU::init();

    // Initialise the ThreadContext's memory proxies
    tcBase()->initMemProxies(tcBase());

    if (FullSystem && !params()->defer_registration) {
        ThreadID size = threadContexts.size();
        for (ThreadID i = 0; i < size; ++i) {
            ThreadContext *tc = threadContexts[i];
            // initialize CPU, including PC
            // architectural registers are also cleared
            TheISA::initCPU(tc, tc->contextId());
        }
    }

    // Atomic doesn't do MT right now, so contextId == threadId
    ifetch_req.setThreadContext(_cpuId, 0); // Add thread ID if we add MT
    data_read_req.setThreadContext(_cpuId, 0); // Add thread ID here too
    data_write_req.setThreadContext(_cpuId, 0); // Add thread ID here too

    // enable the monitor
    enabled = true;
    // monitor not initialized yet
    initialized = false;

    // Initialize monitoring packet
    mp.init();

    // copy thread pointer
    monitor_thread = thread;
}

AtomicSimpleMonitor::AtomicSimpleMonitor(AtomicSimpleMonitorParams *p)
    : BaseSimpleCPU(p), tickEvent(this), width(p->width), locked(false),
      simulate_data_stalls(p->simulate_data_stalls),
      simulate_inst_stalls(p->simulate_inst_stalls),
      icachePort(name() + "-iport", this), dcachePort(name() + "-iport", this),
      monitorPort(name() + "-iport", this),
      fastmem(p->fastmem)
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
        default: panic("Invalid monitor type\n");
    }
    
}


AtomicSimpleMonitor::~AtomicSimpleMonitor()
{
    if (tickEvent.scheduled()) {
        deschedule(tickEvent);
    }
}

void
AtomicSimpleMonitor::serialize(ostream &os)
{
    SimObject::State so_state = SimObject::getState();
    SERIALIZE_ENUM(so_state);
    SERIALIZE_SCALAR(locked);
    BaseSimpleCPU::serialize(os);
    nameOut(os, csprintf("%s.tickEvent", name()));
    tickEvent.serialize(os);
}

void
AtomicSimpleMonitor::unserialize(Checkpoint *cp, const string &section)
{
    SimObject::State so_state;
    UNSERIALIZE_ENUM(so_state);
    UNSERIALIZE_SCALAR(locked);
    BaseSimpleCPU::unserialize(cp, section);
    tickEvent.unserialize(cp, csprintf("%s.tickEvent", section));
}

void
AtomicSimpleMonitor::resume()
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
AtomicSimpleMonitor::switchOut()
{
    assert(_status == Running || _status == Idle);
    _status = SwitchedOut;

    tickEvent.squash();
}


void
AtomicSimpleMonitor::takeOverFrom(BaseCPU *oldCPU)
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
AtomicSimpleMonitor::activateContext(ThreadID thread_num, int delay)
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

    // since we store tags using ISA registers
    // clear them before activating context
    thread->clearArchRegs();
}


void
AtomicSimpleMonitor::suspendContext(ThreadID thread_num)
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

void
AtomicSimpleMonitor::regStats()
{
    using namespace Stats;
    BaseSimpleCPU::regStats();
    numMonitorInsts
        .name(name() + ".numMonitorInsts")
        .desc("Number of instructions processed by monitor")
        ;
    numIntegerInsts
        .name(name() + ".numIntegerInsts")
        .desc("Number of integer instructions processed by monitor")
        ;
    numLoadInsts
        .name(name() + ".numLoadInsts")
        .desc("Number of load instructions processed by monitor")
        ;
    numStoreInsts
        .name(name() + ".numStoreInsts")
        .desc("Number of store instructions processed by monitor")
        ;
    numIndirectCtrlInsts
        .name(name() + ".numIndirectCtrlInsts")
        .desc("Number of indirect control instructions processed by monitor")
        ;
    // numTaintedInsts
        // .name(name() + ".numTaintedInsts")
        // .desc("Number of tainted instructions")
        // ;
    numTaintedIntegerInsts
        .name(name() + ".numTaintedIntegerInsts")
        .desc("Number of tainted integer instructions")
        ;
    numTaintedLoadInsts
        .name(name() + ".numTaintedLoadInsts")
        .desc("Number of tainted load instructions")
        ;
    numTaintedStoreInsts
        .name(name() + ".numTaintedStoreInsts")
        .desc("Number of tainted store instructions")
        ;
    numTaintedIndirectCtrlInsts
        .name(name() + ".numTaintedIndirectCtrlInsts")
        .desc("Number of tainted indirect control instructions")
        ;
    numUMCErrors
        .name(name() + ".numUMCErrors")
        .desc("Number of UMC errors")
        ;
    numBCLoadErrors
        .name(name() + ".numBCLoadErrors")
        .desc("Number of BC Load errors")
        ;
    numBCStoreErrors
        .name(name() + ".numBCStoreErrors")
        .desc("Number of BC Store errors")
        ;
    // numBCErrors
        // .name(name() + ".numBCErrors")
        // .desc("Number of BC errors")
        // ;
}

void
AtomicSimpleMonitor::resetStats()
{
    numMonitorInsts = 0;
    numIntegerInsts = 0;
    numLoadInsts = 0;
    numStoreInsts = 0;
    numIndirectCtrlInsts = 0;
    numTaintedIntegerInsts = 0;
    numTaintedLoadInsts = 0;
    numTaintedStoreInsts = 0;
    numTaintedIndirectCtrlInsts = 0;
    numUMCErrors = 0;
    numBCLoadErrors = 0;
    numBCStoreErrors = 0;
}

Fault
AtomicSimpleMonitor::readMem(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags)
{
    // use the CPU's statically allocated read request and packet objects
    Request *req = &data_read_req;

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
AtomicSimpleMonitor::readMemFunctional(Addr addr, uint8_t * data,
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
AtomicSimpleMonitor::writeMem(uint8_t *data, unsigned size,
                          Addr addr, unsigned flags, uint64_t *res)
{
    // use the CPU's statically allocated write request and packet objects
    Request *req = &data_write_req;

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

    if(secondAddr > addr)
        size = secondAddr - addr;

    dcache_latency = 0;

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
AtomicSimpleMonitor::writeMemFunctional(uint8_t *data, unsigned size,
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
AtomicSimpleMonitor::handlePageTableFault(Addr addr)
{
    Process *p = tc->getProcessPtr();
    Addr page_base_addr = roundDown(addr, VMPageSize);
    p->allocateMem(page_base_addr, VMPageSize);
    // clear page
    SETranslatingPortProxy &tp = tc->getMemProxy();
    tp.memsetBlob(page_base_addr, 0, VMPageSize);
}

/*
 * Read a tag from memory/cache
 */
AtomicSimpleMonitor::Tag
AtomicSimpleMonitor::readTag(Addr addr) {
    // assert(addr < TAG_MEM_SIZE_BITS);
    
    Tag tag = 0;
    Fault fault;

    fault = readMem(addr, &tag, sizeof(Tag), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(addr);
        // retry
        // fault = readMem(addr_h, &octet, sizeof(uint8_t), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
        // assert (fault == NoFault);
        // the new page should always be clear
    }

    return tag;
}

AtomicSimpleMonitor::Tag
AtomicSimpleMonitor::readTagFunctional(Addr addr) {
    // assert(addr < TAG_MEM_SIZE_BITS);
    
    Tag tag = 0;
    Fault fault;

    fault = readMemFunctional(addr, &tag, sizeof(Tag), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(addr);
        // retry
        // fault = readMem(addr_h, &octet, sizeof(uint8_t), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
        // assert (fault == NoFault);
        // the new page should always be clear
    }

    return tag;
}

/*
 * Read a bit tag from packed tag memory
 */
bool
AtomicSimpleMonitor::readBitTag(Addr addr) {
    // assert(addr < TAG_MEM_SIZE_BITS);

    Addr addr_h = addr >> 3;
    Addr addr_l = addr & 7;

    uint8_t octet = 0;
    Fault fault;

    fault = readMem(addr_h, &octet, sizeof(uint8_t), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(addr_h);
        // retry
        // fault = readMem(addr_h, &octet, sizeof(uint8_t), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
        // assert (fault == NoFault);
        // the new page should always be clear
        return false;
    }

    return (bool)(octet & (1 << addr_l));
}

bool
AtomicSimpleMonitor::readBitTagFunctional(Addr addr) {
    // assert(addr < TAG_MEM_SIZE_BITS);

    Addr addr_h = addr >> 3;
    Addr addr_l = addr & 7;

    uint8_t octet = 0;
    Fault fault;

    fault = readMemFunctional(addr_h, &octet, sizeof(uint8_t), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(addr_h);
        // retry
        // fault = readMem(addr_h, &octet, sizeof(uint8_t), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
        // assert (fault == NoFault);
        // the new page should always be clear
        return false;
    }

    return (bool)(octet & (1 << addr_l));
}

uint64_t
AtomicSimpleMonitor::readDWordTag(Addr addr)
{
    // make sure tag address can fit into memory space
    assert(addr < 0x80000000);
    Addr tag_addr = addr << 1;
    uint64_t dword;
    Fault fault = readMem(tag_addr, (uint8_t*)&dword, sizeof(uint64_t), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(tag_addr);
        // the new page should always be clear
        return 0;
    }
    return dword;
}

uint64_t
AtomicSimpleMonitor::readDWordTagFunctional(Addr addr)
{
    // make sure tag address can fit into memory space
    assert(addr < 0x80000000);
    Addr tag_addr = addr << 1;
    uint64_t dword;
    Fault fault = readMemFunctional(tag_addr, (uint8_t*)&dword, sizeof(uint64_t), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(tag_addr);
        // the new page should always be clear
        return 0;
    }
    return dword;
}

/*
 * Write a tag to memory/cache
 */
void
AtomicSimpleMonitor::writeTag(Addr addr, Tag tag) {
    // assert(addr < TAG_MEM_SIZE_BITS);

    Fault fault;
    
    fault = writeMemFunctional(&tag, sizeof(Tag), addr, TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne, NULL);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(addr);
        // retry
        fault = writeMem(&tag, sizeof(Tag), addr, TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne, NULL);
        assert (fault == NoFault);
    }
}

void
AtomicSimpleMonitor::writeTagFunctional(Addr addr, Tag tag) {
    // assert(addr < TAG_MEM_SIZE_BITS);

    Fault fault;
    
    fault = writeMem(&tag, sizeof(Tag), addr, TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne, NULL);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(addr);
        // retry
        fault = writeMemFunctional(&tag, sizeof(Tag), addr, TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne, NULL);
        assert (fault == NoFault);
    }
}

/*
 * Write a bit tag to packed tag memory
 */
void
AtomicSimpleMonitor::writeBitTag(Addr addr, bool tag)
{
    // assert(addr < TAG_MEM_SIZE_BITS);

    Addr addr_h = addr >> 3;
    Addr addr_l = addr & 7;
    Fault fault;
    uint8_t octet = 0;
    
    fault = readMemFunctional(addr_h, &octet, sizeof(uint8_t), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(addr_h);
        // retry
        // fault = readMem(addr_h, &octet, sizeof(uint8_t), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
        // assert (fault == NoFault);
    }
    if (tag)
        octet |= 1 << addr_l;
    else
        octet &= ~(1 << addr_l);
    fault = writeMem(&octet, sizeof(uint8_t), addr_h, TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne, NULL);
    assert(fault == NoFault);
}

void
AtomicSimpleMonitor::writeBitTagFunctional(Addr addr, bool tag)
{
    // assert(addr < TAG_MEM_SIZE_BITS);

    Addr addr_h = addr >> 3;
    Addr addr_l = addr & 7;
    Fault fault;
    uint8_t octet = 0;
    
    fault = readMemFunctional(addr_h, &octet, sizeof(uint8_t), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(addr_h);
        // retry
        // fault = readMem(addr_h, &octet, sizeof(uint8_t), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
        // assert (fault == NoFault);
    }
    if (tag)
        octet |= 1 << addr_l;
    else
        octet &= ~(1 << addr_l);
    fault = writeMemFunctional(&octet, sizeof(uint8_t), addr_h, TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne, NULL);
    assert(fault == NoFault);
}

void
AtomicSimpleMonitor::writeDWordTag(Addr addr, uint64_t tag)
{
    // make sure tag address can fit into memory space
    assert(addr < 0x80000000);
    Addr tag_addr = addr << 1;
    Fault fault = writeMem((uint8_t*)&tag, sizeof(uint64_t), tag_addr, TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne, NULL);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(tag_addr);
        // retry
        fault = writeMem((uint8_t*)&tag, sizeof(uint64_t), tag_addr, TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne, NULL);
        assert (fault == NoFault);
    }
}

void
AtomicSimpleMonitor::writeDWordTagFunctional(Addr addr, uint64_t tag)
{
    // make sure tag address can fit into memory space
    assert(addr < 0x80000000);
    Addr tag_addr = addr << 1;
    Fault fault = writeMemFunctional((uint8_t*)&tag, sizeof(uint64_t), tag_addr, TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne, NULL);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(tag_addr);
        // retry
        fault = writeMemFunctional((uint8_t*)&tag, sizeof(uint64_t), tag_addr, TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne, NULL);
        assert (fault == NoFault);
    }
}

void
AtomicSimpleMonitor::setTagProxy(Addr addr, int nbytes, uint8_t tag)
{
    // TODO: add support to variable-size tags
    // set tag for memory chunk
    int i;
    for (ChunkGenerator gen(addr, nbytes, 64); !gen.done(); gen.next()) {
        if (gen.size() == 64) {
            writeDWordTagFunctional(gen.addr() >> 4, tag ? 0xffffffffffffffff : 0);
        } else {
            for (i = 0; i < gen.size(); ++i) {
                writeBitTagFunctional(gen.addr()+i, (bool)tag);
            }
        }
    }
}

void
AtomicSimpleMonitor::revalidateRegTag(int idx)
{
    // set address
    // create request
    Request *req = &monitor_req;
    req->setVirt(0, 0x30020000, 4, TheISA::TLB::AllowUnaligned, dataMasterId(), thread->pcState().instAddr());
    // create packet
    PacketPtr p = new Packet(req, MemCmd::WriteReq);
    p->dataStatic(&idx);
    // send packet
    monitorPort.sendFunctional(p);
    // clean up
    delete p;
    // set data
    // create request
    req->setVirt(0, 0x30020000, 4, TheISA::TLB::AllowUnaligned, dataMasterId(), thread->pcState().instAddr());
    // create packet
    p = new Packet(req, MemCmd::WriteReq);
    int data = 0;
    p->dataStatic(&data);
    // send packet
    monitorPort.sendFunctional(p);
    // clean up
    delete p;
}

void
AtomicSimpleMonitor::revalidateMemTag(Addr addr)
{
    // set address
    // create request
    Request *req = &monitor_req;
    req->setVirt(0, 0x30020000, 4, TheISA::TLB::AllowUnaligned, dataMasterId(), thread->pcState().instAddr());
    // create packet
    PacketPtr p = new Packet(req, MemCmd::WriteReq);
    unsigned word_addr = addr >> 2;
    p->dataStatic(&word_addr);
    // send packet
    monitorPort.sendFunctional(p);
    // clean up
    delete p;
    // set data
    // create request
    req->setVirt(0, 0x30020000, 4, TheISA::TLB::AllowUnaligned, dataMasterId(), thread->pcState().instAddr());
    // create packet
    p = new Packet(req, MemCmd::WriteReq);
    int data = 1;
    p->dataStatic(&data);
    // send packet
    monitorPort.sendFunctional(p);
    // clean up
    delete p;
}

/*
 * Pre-execute
 *  Try to read a monitoring packet from FIFO.
 */
void
AtomicSimpleMonitor::preExecute()
{
    // maintain r0=0 semantic
    thread->setIntReg(ZeroReg, 0);
    // pop FIFO entry
    uint32_t data = 1;
    Fault fault = writeToFifo(FIFO_NEXT, (uint8_t*)&data, sizeof(uint32_t), ArmISA::TLB::AllowUnaligned);
    // read a packet from FIFO
    if (fault == NoFault) {
        readFromFifo(FIFO_PACKET, (uint8_t *)&mp, sizeof(mp), ArmISA::TLB::AllowUnaligned);
    } else {
        mp.valid = false;
    }
}

/*
 * Post-execute
 * Collect monitoring statistics
 */
void
AtomicSimpleMonitor::postExecute()
{
    // TODO

}

/**
 * UMC Execution
 */
void
AtomicSimpleMonitor::UMCExecute()
{
    if (mp.load) {
        DPRINTF(Monitor, "UMC: Load instruction\n");
        numLoadInsts++;
        numMonitorInsts++;
        if (!(UMCTag)readBitTag(mp.memAddr)) {
            DPRINTF(Monitor, "UMC Error: reading uinitialized memory, VA=0x%x\n", (unsigned)mp.memAddr);
            numUMCErrors++;
        }
    } else if (mp.store && !mp.settag) {
        DPRINTF(Monitor, "UMC: Store instruction\n");
        numStoreInsts++;
        numMonitorInsts++;
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            writeBitTag(pbyte, 1);
        }
    } else if (mp.store && mp.settag) {
        DPRINTF(Monitor, "UMC: Initializing mem[0x%x:0x%x]\n", mp.memAddr, mp.memEnd);
        numMonitorInsts++;
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            writeBitTag(pbyte, 1);
        }
    } else {
        DPRINTF(Monitor, "Unknown instruction\n");
        numMonitorInsts++;
    }
}

/**
 * DIFT Execution
 */
void
AtomicSimpleMonitor::DIFTExecute()
{
    DIFTTag tresult;
    if (mp.intalu) {
        // integer ALU operation
        DPRINTF(Monitor, "DIFT: Integer ALU instruction\n");
        DIFTTag trd = thread->readIntReg(mp.rd);
        tresult = false;

        if (TheISA::isISAReg(mp.rs1))
            tresult |= (DIFTTag)thread->readIntReg(mp.rs1);
        if (TheISA::isISAReg(mp.rs2))
            tresult |= (DIFTTag)thread->readIntReg(mp.rs1);
        thread->setIntReg((int)mp.rd, (uint64_t)tresult);

        if (trd || tresult) {
            numTaintedIntegerInsts++;
        }
        numIntegerInsts++;
        numMonitorInsts++;
    } else if (mp.load) {
        DPRINTF(Monitor, "DIFT: Load instruction, addr=0x%x\n", mp.memAddr);
        uint64_t trd = thread->readIntReg(mp.rd);
        tresult = false;
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            tresult |= (DIFTTag)readBitTag(pbyte);
        }
        thread->setIntReg((int)mp.rd, (uint64_t)tresult);
        
        if (trd || tresult) {
            numTaintedLoadInsts++;
        }
        numLoadInsts++;
        numMonitorInsts++;
    } else if (mp.store && !mp.settag) {
        DPRINTF(Monitor, "DIFT: Store instruction, addr=0x%x\n", mp.memAddr);
        DIFTTag tsrc = (DIFTTag)thread->readIntReg(mp.rs1);
        DIFTTag tdest = false;

        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            tdest |= readTagFunctional(pbyte);
            writeBitTag(pbyte, tsrc);
        }

        if (tdest || tsrc) {
            numTaintedStoreInsts++;
        }
        numStoreInsts++;
        numMonitorInsts++;
    } else if (mp.store && mp.settag) {
        DPRINTF(Monitor, "DIFT: Set taint mem[0x%x:0x%x]=%d\n", mp.memAddr, mp.memEnd, mp.data);
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            writeBitTag(pbyte, (bool)mp.data);
        }
    } else if (mp.indctrl) {
        DPRINTF(Monitor, "DIFT: Indirect control transfer instruction\n");
        DIFTTag trs1 = (bool)thread->readIntReg(mp.rs1);
        if (trs1) {
            DPRINTF(Monitor, "DIFT: Fatal: indirect control transfer on tainted register, PC=0x%x\n", 
                mp.instAddr);
            numTaintedIndirectCtrlInsts++;
        }
        numIndirectCtrlInsts++;
        numMonitorInsts++;
    } else {
        DPRINTF(Monitor, "Unknown instruction\n");
        numMonitorInsts++;
    }
}

/**
 * BC Execution
 */
void
AtomicSimpleMonitor::BCExecute()
{
    if (mp.intalu) {
        // integer ALU operation
        DPRINTF(Monitor, "BC: Integer ALU instruction\n");
        ALUOpCode opcode = decodeALUOpcode(mp.opcode);
        if (opcode == ALUMov) {
            if (TheISA::isISAReg(mp.rs1)) {
                BCTag trs1 = (BCTag)thread->readIntReg(mp.rs1);
                thread->setIntReg((int)mp.rd, (uint64_t)trs1);
            } else {
                // mov immediate
                thread->setIntReg((int)mp.rd, 0);
            }
        } else if ((opcode == ALUAdd) || (opcode == ALUSub)) {
            BCTag tresult = 0;
            if (TheISA::isISAReg(mp.rs1) && TheISA::isISAReg(mp.rs2)) {
                // two register operands
                BCTag trs1 = (BCTag)thread->readIntReg(mp.rs1);
                BCTag trs2 = (BCTag)thread->readIntReg(mp.rs2);
                if (trs1 == 0 || trs2 == 0)
                    tresult = trs1 | trs2;
                else if (opcode == ALUAdd)
                    tresult = trs1 + trs2;
                else if (opcode == ALUSub)
                    tresult = trs1 - trs2;
                thread->setIntReg((int)mp.rd, (uint64_t)tresult);
            } else if (TheISA::isISAReg(mp.rs1)) {
                // register + imm operands
                BCTag trs1 = (BCTag)thread->readIntReg(mp.rs1);
                thread->setIntReg((int)mp.rd, (uint64_t)trs1);
            } else {
                // we should never reach here
                // panic("Incorrect number of ALU operands!\n");
            }
        } else {
            // other ALU operations
            thread->setIntReg((int)mp.rd, 0);
        }

        numIntegerInsts++;
        numMonitorInsts++;
    } else if (mp.load) {
        DPRINTF(Monitor, "BC: Load instruction, addr=0x%x\n", mp.memAddr);

        BCTag tmem = (BCTag)readTag(mp.memAddr);
        if (TheISA::isISAReg(mp.rs1) && !(TheISA::isISAReg(mp.rs2))) {
            BCTag trs1 = (BCTag)thread->readIntReg(mp.rs1);
            bool pass = (trs1 == 0) || (toMemTag(tmem) == trs1);
            if (!pass) {
                DPRINTF(Monitor, "BC: Out-of-bound load detected! tptr=%d, tmem=%d\n", trs1, toMemTag(tmem));
                numBCLoadErrors++;
            }

            // update register tag
            thread->setIntReg((int)mp.rd, toPtrTag(tmem));
            if (toPtrTag(tmem) != 0) {
                DPRINTF(Monitor, "BC: write pointer tag %d to r%d\n", toMemTag(tmem), (int)mp.rd);
            }
        } else {
            // should not reach here
            // if we reach here for some reason, conservatively clear dest reg tag
            thread->setIntReg((int)mp.rd, 0);
        }

        numLoadInsts++;
        numMonitorInsts++;
    } else if (mp.store && !mp.settag) {
        DPRINTF(Monitor, "BC: Store instruction, addr=0x%x\n", mp.memAddr);

        if (TheISA::isISAReg(mp.rs1) && TheISA::isISAReg(mp.rs2)) {
            BCTag tsrc = (BCTag)thread->readIntReg(mp.rs1);
            BCTag tptr = (BCTag)thread->readIntReg(mp.rs2);
            BCTag tmem = readTagFunctional(mp.memAddr);
            bool pass = (tptr == 0) || (toMemTag(tmem) == tptr);
            if (!pass) {
                DPRINTF(Monitor, "BC: Out-of-bound store detected! tptr=%d, tmem=%d\n", tptr, toMemTag(tmem));
                numBCStoreErrors++;
            }

            // update destination pointer tag
            writeTag(mp.memAddr, mergeMemPtrTags(tmem, tsrc));
        } else {
            // should not reach here
            // if we reach here for some reason, conservatively clear pointer tag
            BCTag tmem = readTagFunctional(mp.memAddr);
            writeTag(mp.memAddr, mergeMemPtrTags(tmem, 0));
        }

        numStoreInsts++;
        numMonitorInsts++;
    } else if (mp.store && mp.settag) {
        DPRINTF(Monitor, "BC: Set tag mem[0x%x:0x%x]=%d\n", mp.memAddr, mp.memEnd, mp.data);
        for (Addr pbyte = mp.memAddr; pbyte < mp.memEnd; pbyte++) {
            writeTag(pbyte, (Tag)mp.data);
        }
    } else {
        DPRINTF(Monitor, "Unknown instruction\n");
        numMonitorInsts++;
    }
}

/**
 * SEC Execution
 */
void
AtomicSimpleMonitor::SECExecute()
{

}

/**
 * HardBound Execution
 */
void
AtomicSimpleMonitor::HBExecute()
{
    if (mp.intalu) {
        // integer ALU operation
        DPRINTF(Monitor, "HardBound: Integer ALU instruction\n");
        ALUOpCode opcode = decodeALUOpcode(mp.opcode);
        if (opcode == ALUMov) {
            if (TheISA::isISAReg(mp.rs1)) {
                HBTag trs1 = (HBTag)thread->readIntReg(mp.rs1);
                thread->setIntReg((int)mp.rd, (uint64_t)trs1);
            } else {
                // mov immediate
                thread->setIntReg((int)mp.rd, 0);
            }
        } else if ((opcode == ALUAdd) || (opcode == ALUSub)) {
            HBTag tresult = 0;
            if (TheISA::isISAReg(mp.rs1) && TheISA::isISAReg(mp.rs2)) {
                HBTag trs1 = (HBTag)thread->readIntReg(mp.rs1);
                HBTag trs2 = (HBTag)thread->readIntReg(mp.rs2);
                if (toBoundTag(trs1)) {
                    tresult = trs1;
                } else {
                    tresult = trs2;
                }
                thread->setIntReg((int)mp.rd, (uint64_t)tresult);
            } else if (TheISA::isISAReg(mp.rs1)) {
                HBTag trs1 = (HBTag)thread->readIntReg(mp.rs1);
                thread->setIntReg((int)mp.rd, (uint64_t)trs1);
            } else {
                // we should never reach here
                // panic("Incorrect number of ALU operands!\n");
            }
        } else {
            // other ALU operations
            thread->setIntReg((int)mp.rd, 0);
        }

        numIntegerInsts++;
        numMonitorInsts++;
    } else if (mp.load) {
        DPRINTF(Monitor, "HardBound: Load instruction, vaddr=0x%x\n", mp.memAddr);

        if (TheISA::isISAReg(mp.rs1) && !(TheISA::isISAReg(mp.rs2))) {
            HBTag tptr = (HBTag)thread->readIntReg(mp.rs1);
            // check if access is within bound
            bool pass = (tptr == 0) || ((mp.memAddr >= toBaseTag(tptr)) 
                        && (mp.memAddr < toBoundTag(tptr)));
            if (!pass) {
                DPRINTF(Monitor, "HardBound: Out-of-bound load detected! base=0x%x, bound=0x%x, vaddr=0x%x\n", 
                        toBaseTag(tptr), toBoundTag(tptr), mp.memAddr);
                numBCLoadErrors++;
            }
            // update register tag, only when load word
            if (mp.size == 4) {
                HBTag tmem = readDWordTag(mp.physAddr);
                thread->setIntReg((int)mp.rd, tmem);
            }
        } else {
            // should not reach here
            // if we reach here for some reason, conservatively clear dest reg tag
            thread->setIntReg((int)mp.rd, 0);
        }

        numLoadInsts++;
        numMonitorInsts++;
    } else if (mp.store && !mp.settag) {
        DPRINTF(Monitor, "HardBound: Store instruction, vaddr=0x%x\n", mp.memAddr);

        if (TheISA::isISAReg(mp.rs1) && TheISA::isISAReg(mp.rs2)) {
            BCTag tsrc = (BCTag)thread->readIntReg(mp.rs1);
            BCTag tptr = (BCTag)thread->readIntReg(mp.rs2);
            bool pass = (tptr == 0) || ((mp.memAddr >= toBaseTag(tptr)) 
                        && (mp.memAddr < toBoundTag(tptr)));
            if (!pass) {
                DPRINTF(Monitor, "HardBound: Out-of-bound store detected! base=0x%x, bound=0x%x, vaddr=0x%x\n", 
                        toBaseTag(tptr), toBoundTag(tptr), mp.memAddr);
                numBCStoreErrors++;
            }
            // update destination pointer tag
            if (mp.size == 4) {
                writeDWordTag(mp.physAddr, tsrc);
            }
        } else {
            // should not reach here
            // if we reach here for some reason, conservatively clear pointer tag
            if (mp.size == 4)
                writeDWordTag(mp.physAddr, 0);
        }

        numStoreInsts++;
        numMonitorInsts++;
    } else if (mp.store && mp.settag) {
        if (mp.size == 0) {
            // set base address
            setTagData = mp.data;
        } else if (mp.size == 1) {
            // set bound address
            setTagData = setTagData | (mp.data << 32);
            DPRINTF(Monitor, "HardBound: Set tag mem[0x%x]=%lx\n", mp.memAddr, setTagData);
            writeDWordTag(mp.physAddr, setTagData);
        }
    } else {
        DPRINTF(Monitor, "Unknown instruction\n");
        numMonitorInsts++;
    }
}

/**
 * Compact tag memory
 */
template <typename T>
bool
AtomicSimpleMonitor::isMemoryCompactable(Addr addr, int blocksize, T tag)
{
    assert(isPowerOf2(blocksize));
    addr = addr & ~(blocksize-1);
    int tagsize = sizeof(T);
    for (int i = 0; i < blocksize; i+=tagsize) {
        if ((T)readTagFunctional(addr+i) != tag)
            return false;
    }
    return true;
}


/**
 * Finish monitoring
 */
void
AtomicSimpleMonitor::finishMonitoring()
{
    // print monitoring statistics
    showMonitoringStats();
    // main core has finished, exit current monitoring thread context
    tc->halt(0);
}

/**
 * Display Monitoring Statistics
 */
void
AtomicSimpleMonitor::showMonitoringStats()
{
    if (monitorExt == MONITOR_DIFT) {
        showDIFTStats();
    }
}

/**
 * Display DIFT Statistics
 */
void
AtomicSimpleMonitor::showDIFTStats()
{
    // DPRINTF(Monitor, "[DIFT] Number of monitoring instructions: %d\n", numMonitorInsts);
    // DPRINTF(Monitor, "[DIFT] Number of tainted instructions: %d\n", numTaintedInsts);
}

void
AtomicSimpleMonitor::processMonitoringPacket()
{
    assert(mp.valid);

    // execute monitoring operations
    if (monitorExt == MONITOR_NONE)
        return;
    else if (monitorExt == MONITOR_UMC)
        UMCExecute();
    else if (monitorExt == MONITOR_DIFT)
        DIFTExecute();
    else if (monitorExt == MONITOR_BC)
        BCExecute();
    else if (monitorExt == MONITOR_SEC)
        SECExecute();
    else if (monitorExt == MONITOR_HB)
        HBExecute();

    // clear monitoring packet after processing
    mp.init();
}

void
AtomicSimpleMonitor::tick()
{
    DPRINTF(SimpleCPU, "Monitor Tick\n");

    Tick latency = 0;
    Tick stall_ticks = 0;
    if (!initialized) {
        // initialize monitor on first cycle
        thread->clearArchRegs();
        initialized = true;
    } else if (enabled) {
        // fetch an entry from main core
        preExecute();

        if (mp.valid) {
            // model latency of reading a packet
            stall_ticks += ticks(1);
            // packet is valid, continue processing
            DPRINTF(Monitor, "Read a packet from main core\n");
            if (!mp.done) {
                processMonitoringPacket();
            } else {
                finishMonitoring();
            }
        }

    }

    if (simulate_data_stalls && dcache_access)
        stall_ticks += dcache_latency;

    if (stall_ticks) {
        Tick stall_cycles = stall_ticks / ticks(1);
        Tick aligned_stall_ticks = ticks(stall_cycles);

        if (aligned_stall_ticks < stall_ticks)
            aligned_stall_ticks += 1;

        latency += aligned_stall_ticks;
    }
    
    // instruction takes at least one cycle
    if (latency < ticks(1))
        latency = ticks(1);

    if (_status != Idle)
        schedule(tickEvent, roundDown(curTick()+latency, 2) + 1);
}


void
AtomicSimpleMonitor::printAddr(Addr a)
{
    dcachePort.printAddr(a);
}

AtomicSimpleMonitor::ALUOpCode
AtomicSimpleMonitor::decodeALUOpcode(uint8_t opcode)
{
    switch(opcode) {
      case 0x0d:
        return ALUMov;
      case 0x04:
        return ALUAdd;
      case 0x05:
        return ALUAdd;
      case 0x02:
        return ALUSub;
      default:
        return ALUNone;
    }
}

////////////////////////////////////////////////////////////////////////
//
//  AtomicSimpleMonitor Simulation Object
//
AtomicSimpleMonitor *
AtomicSimpleMonitorParams::create()
{
    numThreads = 1;
    if (!FullSystem && workload.size() != 1)
        panic("only one workload allowed");
    return new AtomicSimpleMonitor(this);
}

MasterPort &
AtomicSimpleMonitor::getMasterPort(const std::string &if_name, int idx)
{
  if (if_name == "monitor_port") {
    return monitorPort;
  } else {
    return BaseSimpleCPU::getMasterPort(if_name, idx);
  }
}