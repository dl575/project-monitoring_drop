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
#include "debug/MonitorSimpleCPU.hh"
#include "debug/Monitor.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/physical.hh"
#include "params/AtomicSimpleMonitor.hh"
#include "sim/faults.hh"
#include "sim/system.hh"
#include "sim/full_system.hh"

#include "cpu/simple/drop.hh"

using namespace std;
using namespace TheISA;

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
    BaseSimpleCPU::init();

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
    
    // Write threshold to timer
    if (timer_enabled){
        // Convert wcet cycles to ticks
        Tick full_ticks = full_wcet*ticks(1);
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
}

AtomicSimpleMonitor::AtomicSimpleMonitor(AtomicSimpleMonitorParams *p)
    : BaseSimpleCPU(p), tickEvent(this), width(p->width), locked(false),
      simulate_data_stalls(p->simulate_data_stalls),
      simulate_inst_stalls(p->simulate_inst_stalls),
      icachePort(name() + "-iport", this), dcachePort(name() + "-iport", this),
      monitorPort(name() + "-iport", this),
      fastmem(p->fastmem), full_wcet(p->full_wcet), full_delay(p->full_delay)
{
    _status = Idle;
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
    numCallInsts
        .name(name() + ".numCallInsts")
        .desc("Number of call instructions processed by monitor")
        ;
    numReturnInsts
        .name(name() + ".numReturnInsts")
        .desc("Number of return instructions processed by monitor")
        ;
    numTaintedInsts
        .name(name() + ".numTaintedInsts")
        .desc("Number of tainted instructions")
        ;
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
    numBCErrors
        .name(name() + ".numBCErrors")
        .desc("Number of BC errors")
        ;
    numSyscallTags
        .name(name() + ".numSyscallTags")
        .desc("Number of syscalls that lead to setting tags")
        ;
    
    numTaintedInsts = numTaintedIntegerInsts + numTaintedLoadInsts + numTaintedStoreInsts + numTaintedIndirectCtrlInsts;
    numBCErrors = numBCLoadErrors + numBCStoreErrors;
}

void
AtomicSimpleMonitor::resetStats()
{
    numMonitorInsts = 0;
    numIntegerInsts = 0;
    numLoadInsts = 0;
    numStoreInsts = 0;
    numIndirectCtrlInsts = 0;
    numCallInsts = 0;
    numReturnInsts = 0;
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
    p->allocateMonMem(page_base_addr, VMPageSize);
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
 * Read a word tag from packed tag memory
 */
uint32_t
AtomicSimpleMonitor::readWordTag(Addr addr) {
    // assert(addr < TAG_MEM_SIZE_BITS);

    Addr addr_h = addr & 0xFFFFFFFC;

    uint32_t tag = 0;
    Fault fault;

    fault = readMem(addr_h, (uint8_t *)&tag, sizeof(tag), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(addr_h);
        // retry
        // fault = readMem(addr_h, &octet, sizeof(uint8_t), TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne);
        // assert (fault == NoFault);
        // the new page should always be clear
        return 0;
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
AtomicSimpleMonitor::writeWordTag(Addr addr, uint32_t tag)
{
    // assert(addr < TAG_MEM_SIZE_BITS);

    Addr addr_h = addr & 0xFFFFFFFC;
    Fault fault;
    fault = writeMem((uint8_t *)&tag, sizeof(tag), addr_h, TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne, NULL);
    if (fault != NoFault) {
        // page table fault
        handlePageTableFault(addr_h);
        // retry
        fault = writeMemFunctional((uint8_t *)&tag, sizeof(tag), addr_h, TheISA::TLB::AllowUnaligned | TheISA::TLB::MustBeOne, NULL);
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

// Set tags from syscalls. Assumes tags are single bit.
void
AtomicSimpleMonitor::setTagProxy(Addr addr, int nbytes, uint8_t tag)
{
    // TODO: add support to variable-size tags
    // set tag for memory chunk
    int i;
    for (ChunkGenerator gen(addr, nbytes, 64); !gen.done(); gen.next()) {
        // Handle chunks of 64 
        if (gen.size() == 64) {
            // Use writeDWordTag to quickly set 64 tags bits at a time
            // Combining the addr >> 4 here with the addr << 1 in writeDWordTag
            // means the writing starts at addr >> 3. This corresponds to there
            // being 1 bit of tag per byte of memory.
            writeDWordTag(gen.addr() >> 4, tag ? 0xffffffffffffffff : 0);
            DPRINTF(Monitor, "writeDWordTag 0x%llx\n", gen.addr());
            // Revalidate corresponding flags and set FADE tags
            for (i = 0; i < gen.size(); ++i) {
                setDropMemTag(gen.addr()+i, 1, 0);
            }
        } else {
            for (i = 0; i < gen.size(); ++i) {
                writeBitTag(gen.addr()+i, (bool)tag);
                DPRINTF(Monitor, "writeBitTag 0x%llx\n", gen.addr() + i);
                // Revalidate corresopnding flags and set FADE tags
                setDropMemTag(gen.addr()+i, 1, 0);
            }
        }
    }
    numSyscallTags++;
}

void
AtomicSimpleMonitor::revalidateRegTag(int idx)
{
  // Clear flag to indicate valid
  // If invalidation is performed on this core
  if (invtab.initialized) {
    // Set register number in flag cache
    setFlagCacheAddr(idx);
    // Clear register invalidation flag
    unsigned type = FC_ARRAY;
    writeToFlagCache(FC_CLEAR_FLAG, (uint8_t *)&type, sizeof(type),
        ArmISA::TLB::AllowUnaligned);
  // Invalidation is done on another core, send message
  } else {
    // create request
    Request *req = &monitor_req;
    req->setPhys(DROP_CLEAR_ARRAY, sizeof(idx), ArmISA::TLB::AllowUnaligned, dataMasterId());
    // create packet
    PacketPtr p = new Packet(req, MemCmd::WriteReq);
    p->dataStatic(&idx);
    // send packet
    monitorPort.sendFunctional(p);
    // clean up
    delete p;
  }
}


/*
 * Set the invalidation for the passed register number.
 */
void
AtomicSimpleMonitor::invalidateRegTag(int idx)
{
    // If invalidation is performed on this core
    if (invtab.initialized) {
      // Set register number in flag cache
      setFlagCacheAddr(idx);
      // Clear register invalidation flag
      unsigned type = FC_ARRAY;
      writeToFlagCache(FC_SET_FLAG, (uint8_t *)&type, sizeof(type),
          ArmISA::TLB::AllowUnaligned);
    // Invalidation is done on another core, send message
    } else {
      // create request
      Request *req = &monitor_req;
      req->setPhys(DROP_SET_ARRAY, sizeof(idx), ArmISA::TLB::AllowUnaligned, dataMasterId());
      // create packet
      PacketPtr p = new Packet(req, MemCmd::WriteReq);
      p->dataStatic(&idx);
      // send packet
      monitorPort.sendFunctional(p);
      // clean up
      delete p;
    }
}

void
AtomicSimpleMonitor::revalidateMemTag(Addr addr)
{
  // Clear flag to indicate valid
  // If invalidation is performed on this core
  if (invtab.initialized) {
    // Set address to revalidate
    setFlagCacheAddr(addr);
    // Clear invaliation flag in cache
    unsigned type = FC_CACHE; 
    writeToFlagCache(FC_CLEAR_FLAG, (uint8_t *)&type, sizeof(type),
        ArmISA::TLB::AllowUnaligned);
  // Invalidation is done on another core, send message
  } else {
    // create request
    Request *req = &monitor_req;
    Addr word_addr = addr >> 2;
    req->setPhys(DROP_CLEAR_CACHE, sizeof(word_addr), ArmISA::TLB::AllowUnaligned, dataMasterId());
    // create packet
    PacketPtr p = new Packet(req, MemCmd::WriteReq);
    p->dataStatic(&word_addr);
    // send packet
    monitorPort.sendFunctional(p);
    // clean up
    delete p;
  }
}

void
AtomicSimpleMonitor::invalidateMemTag(Addr addr)
{
    // If invalidation is performed on this core
    if (invtab.initialized) {
      // Set address to revalidate
      setFlagCacheAddr(addr);
      // Clear invaliation flag in cache
      unsigned type = FC_CACHE; 
      writeToFlagCache(FC_SET_FLAG, (uint8_t *)&type, sizeof(type),
          ArmISA::TLB::AllowUnaligned);
    // Invalidation is done on another core, send message
    } else {
      // create request
      Request *req = &monitor_req;
      Addr word_addr = addr >> 2;
      req->setPhys(DROP_SET_CACHE, sizeof(word_addr), ArmISA::TLB::AllowUnaligned, dataMasterId());
      // create packet
      PacketPtr p = new Packet(req, MemCmd::WriteReq);
      p->dataStatic(&word_addr);
      // send packet
      monitorPort.sendFunctional(p);
      // clean up
      delete p;
    }
}

void
AtomicSimpleMonitor::setDropRegTag(int idx, unsigned fadeTag, unsigned invalidTag)
{
    if (full_monitoring)
        return;
    setFlagCacheAddr(idx << 2);
    // calculate combined tag
    unsigned combined_tag = (fadeTag << 1) | invalidTag;
    // create request
    Request *req = &monitor_req;
    req->setPhys(DROP_SET_ARRAY_VALUE, sizeof(unsigned), ArmISA::TLB::AllowUnaligned, dataMasterId());
    // create packet
    PacketPtr p = new Packet(req, MemCmd::WriteReq);
    p->dataStatic(&combined_tag);
    // send packet
    monitorPort.sendFunctional(p);
    // clean up
    delete p;
}

void
AtomicSimpleMonitor::setDropMemTag(Addr addr, unsigned fadeTag, unsigned invalidTag)
{
    if (full_monitoring)
        return;
    setFlagCacheAddr(addr);
    // calculate combined tag
    unsigned combined_tag = (fadeTag << 1) | invalidTag;
    // create request
    Request *req = &monitor_req;
    req->setPhys(DROP_SET_CACHE_VALUE, sizeof(unsigned), ArmISA::TLB::AllowUnaligned, dataMasterId());
    // create packet
    PacketPtr p = new Packet(req, MemCmd::WriteReq);
    p->dataStatic(&combined_tag);
    // send packet
    monitorPort.sendFunctional(p);
    // clean up
    delete p;
}

// Set current address in flag cache without setting/clearing flags
void
AtomicSimpleMonitor::setFlagCacheAddr(Addr addr)
{
  Addr word_addr = addr >> 2;

  // If invalidation is performed on this core
  if (invtab.initialized) {
    writeToFlagCache(FC_SET_ADDR, (uint8_t *)&word_addr, sizeof(word_addr),
        ArmISA::TLB::AllowUnaligned);
  // Invalidation is done to another core, send message
  } else {
    // create request
    Request *req = &monitor_req;
    req->setPhys(DROP_FC_SET_ADDR, sizeof(word_addr), ArmISA::TLB::AllowUnaligned, dataMasterId());
    // create packet
    PacketPtr p = new Packet(req, MemCmd::WriteReq);
    p->dataStatic(&word_addr);
    // send packet
    monitorPort.sendFunctional(p);
    // clean up
    delete p;
  }
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

    // If finished processing last packet, then reset local stored packet
    if (mp.valid) {
      mp.init();
    }

    bool drop = false;
    Fault fault = ReExecFault;
    // Invalidation enabled
    //if (timer_enabled && invtab.initialized) {
    if (invtab.initialized) {
      // Check whether to drop
      fault = readFromTimer(TIMER_READ_DROP, (uint8_t *)&drop, sizeof(drop),
          ArmISA::TLB::AllowUnaligned);
    // Not invalidating, just advance FIFO
    } else {
      // pop FIFO entry
      uint32_t data = 1;
      fault = writeToFifo(FIFO_NEXT, (uint8_t*)&data, sizeof(uint32_t), ArmISA::TLB::AllowUnaligned);
    }

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
    // Load instruction
    if (mp.opcode_custom == OPCODE_LOAD) {
        DPRINTF(Monitor, "UMC: Load instruction mem[0x%x]\n", mp.memAddr);
        numLoadInsts++;
        numMonitorInsts++;
        if (!(UMCTag)readBitTag(mp.memAddr)) {
            DPRINTF(Monitor, "UMC Error: reading uinitialized memory, VA=0x%x\n", (unsigned)mp.memAddr);
            numUMCErrors++;
        }
    // Store instruction
    } else if (mp.opcode_custom == OPCODE_STORE) {
        DPRINTF(Monitor, "UMC: Store instruction mem[0x%x:0x%x]\n", mp.memAddr, mp.memEnd);
        numStoreInsts++;
        numMonitorInsts++;
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            writeBitTag(pbyte, 1);
        }
        /*
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
            setDropMemTag(pbyte, 1, 0);
        }
        */
    // Set tag instruction
    } else if (mp.opcode_custom == OPCODE_CUSTOM_DATA) {
        DPRINTF(Monitor, "UMC: Initializing mem[0x%x:0x%x]\n", mp.memAddr, mp.memEnd);
        numMonitorInsts++;
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            writeBitTag(pbyte, 1);
        }
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
            setDropMemTag(pbyte, 1, 0);
        }
    // Read syscall
    } else if (mp.opcode_custom == OPCODE_SYSCALLREAD) {
      DPRINTF(Monitor, "UMC: Syscall read instruction\n");
      // Set tags for syscall read
      for (ChunkGenerator gen(mp.syscallReadBufPtr, mp.syscallReadNbytes, TheISA::VMPageSize); !gen.done(); gen.next()) {
        /*
        Addr paddr;
        if (mp.syscallReadP->pTable->translate(gen.addr(), paddr)) {
          setTagProxy(paddr, gen.size(), true);
        } else {
          fatal("Address Translation Error\n");
        }
        */
        setTagProxy(gen.addr(), gen.size(), true);
      }
    // Unhandled instruction type
    } else {
        warn("Unknown instruction PC = %x\n", mp.instAddr);
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
    // integer ALU operation
    if (mp.intalu) {
        DPRINTF(Monitor, "DIFT: Integer ALU instruction\n");
        DIFTTag trd = false;
        if (TheISA::isISAReg(mp.rd)) {
          trd = thread->readIntReg(mp.rd);
        }
        tresult = false;

        if (TheISA::isISAReg(mp.rs1))
            tresult |= (DIFTTag)thread->readIntReg(mp.rs1);
        if (TheISA::isISAReg(mp.rs2))
            tresult |= (DIFTTag)thread->readIntReg(mp.rs2);
        if (TheISA::isISAReg(mp.rd)) {
            thread->setIntReg((int)mp.rd, (uint64_t)tresult);
            setDropRegTag((int)mp.rd, tresult ? 1 : 0, 0);
        }

        if (trd || tresult) {
            numTaintedIntegerInsts++;
        }
        numIntegerInsts++;
        numMonitorInsts++;
    // Load instruction
    } else if (mp.load) {
        DPRINTF(Monitor, "DIFT: Load instruction, addr=0x%x\n", mp.memAddr);
        DIFTTag trd = false;
        if (TheISA::isISAReg(mp.rd)) {
            trd = thread->readIntReg(mp.rd);
        }
        tresult = false;
        tresult |= (DIFTTag)readBitTag(mp.memAddr);
        if (TheISA::isISAReg(mp.rd)) {
            thread->setIntReg((int)mp.rd, (uint64_t)tresult);
            setDropRegTag((int)mp.rd, tresult ? 1 : 0, 0);
        }
        
        if (trd || tresult) {
            numTaintedLoadInsts++;
        }
        numLoadInsts++;
        numMonitorInsts++;
    // Store instruction
    } else if (mp.store && !mp.settag) {
        DPRINTF(Monitor, "DIFT: Store instruction, addr=0x%x\n", mp.memAddr);
        DIFTTag tsrc = false;
        if (TheISA::isISAReg(mp.rs1)) {
            tsrc = (DIFTTag)thread->readIntReg(mp.rs1);
        }
        DIFTTag tdest = false;

        tdest |= readTagFunctional(mp.memAddr);
        writeBitTag(mp.memAddr, tsrc);
        setDropMemTag(mp.memAddr, tsrc ? 1 : 0, 0);

        if (tdest || tsrc) {
            numTaintedStoreInsts++;
        }
        numStoreInsts++;
        numMonitorInsts++;
    // Set tag instruction
    } else if (mp.custom) {
        DPRINTF(Monitor, "DIFT: Set taint mem[0x%x:0x%x]=%d\n", mp.memAddr, mp.memEnd, mp.data);
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            writeBitTag(pbyte, (bool)mp.data);
        }
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
            setDropMemTag(mp.memAddr, 1, 0);
        }
    // Indirect control instruction
    } else if (mp.indctrl) {
        DPRINTF(Monitor, "DIFT: Indirect control transfer instruction\n");
        DIFTTag trs1 = false;
        if (TheISA::isISAReg(mp.rs1)) {
            trs1 = (bool)thread->readIntReg(mp.rs1);
        }
        if (trs1) {
            DPRINTF(Monitor, "DIFT: Fatal: indirect control transfer on tainted register, PC=0x%x\n", 
                mp.instAddr);
            numTaintedIndirectCtrlInsts++;
        }
        numIndirectCtrlInsts++;
        numMonitorInsts++;
    // Read syscall
    } else if (mp.settag && mp.syscallReadNbytes > 0) {
      DPRINTF(Monitor, "DIFT: Syscall read instruction\n");
      // Set tags for read syscall
      for (ChunkGenerator gen(mp.syscallReadBufPtr, mp.syscallReadNbytes, TheISA::VMPageSize); !gen.done(); gen.next()) {
        /*
        Addr paddr;
        if (mp.syscallReadP->pTable->translate(gen.addr(), paddr)) {
          setTagProxy(paddr, gen.size(), true);
        } else {
          fatal("Address Translation Error\n");
        }
        */
        setTagProxy(gen.addr(), gen.size(), true);
      }
    } else {
        warn("Unknown instruction PC = %x\n", mp.instAddr);
        numMonitorInsts++;
    }
}

/**
 * DIFT Execution with register taints stored in flag array. Invalidation flags
 * are stored in the thread register file in order to collect statistics on
 * coverage.
 */
void
AtomicSimpleMonitor::DIFTRFExecute()
{
    DIFTTag tresult;
    // ALU instruction
    if (mp.intalu) {
        // integer ALU operation - handled by filter
        warn("ALU instruction reached monitor\n");
    // Load instruction
    } else if (mp.load) {
        DPRINTF(Monitor, "DIFT: Load instruction, addr=0x%x\n", mp.memAddr);
        // Resulting taint is OR of memory taints
        tresult = false;
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            tresult |= (DIFTTag)readBitTag(pbyte);
        }
        // Set taint in flag array
        if (tresult) {
          // Invalidate sets the taint
          invalidateRegTag((int)mp.rd);
        } else {
          // Revalidate clears the taint
          revalidateRegTag((int)mp.rd);
        }
        // Full monitoring of load means resulting tag is valid, clear invalidation
        //thread->setIntReg((int)mp.rd, false);
        int rd = (int)mp.rd;
        if (rd >= 0 && rd < NUM_REGS) {
          invalid_flags[rd] = false;
        }
        
        numLoadInsts++;
        numMonitorInsts++;
    // Store instruction
    } else if (mp.store && !mp.settag) {
        DPRINTF(Monitor, "DIFT: Store instruction, addr=0x%x\n", mp.memAddr);
        // Get source taint
        DIFTTag tsrc;
        setFlagCacheAddr(mp.rs1);
        readFromFlagCache(FC_GET_FLAG_A, (uint8_t *)&tsrc, sizeof(tsrc), ArmISA::TLB::AllowUnaligned);
        // Get source invalidation
        //DIFTTag tinv = (DIFTTag)thread->readIntReg(mp.rs1);
        DIFTTag tinv = false;
        int rs1 = (int)mp.rs1;
        if (rs1 >= 0 && rs1 < NUM_REGS && invalid_flags[rs1]) {
          tinv = true;
        }

        // If source is invalid 
        if (tinv) {
          // Set all target taints as invalid
          for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
              invalidateMemTag(pbyte);
          }
        // Source is valid
        } else {
          // Destination taint is source taint
          for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
              writeBitTag(pbyte, tsrc);
          }
          // Set all target taints as valid
          for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
              revalidateMemTag(pbyte);
          }
        }

        numStoreInsts++;
        numMonitorInsts++;
    // Set taint instruction
    } else if (mp.store && mp.settag) {
      DPRINTF(Monitor, "DIFT: Set taint mem not implemented for DIFT-RF\n");
      /*
        DPRINTF(Monitor, "DIFT: Set taint mem[0x%x:0x%x]=%d\n", mp.memAddr, mp.memEnd, mp.data);
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            writeBitTag(pbyte, (bool)mp.data);
        }
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
            revalidateMemTag(pbyte);
        }
        */
    // Indirect control instruction
    } else if (mp.indctrl) {
        // Filter should handle any untainted indirect branches. Thus,
        // this must be a tainted branch.
        warn("Fatal: indirect control transfer on tainted register, PC=0x%x\n", mp.instAddr);
        numIndirectCtrlInsts++;
        numMonitorInsts++;
    } else {
        warn("Unknown instruction PC = %x\n", mp.instAddr);
        numMonitorInsts++;
    }
}

/**
 * Multi-bit DIFT Execution
 */
void
AtomicSimpleMonitor::MultiDIFTExecute()
{
    DIFTTag tresult;
    if (mp.intalu) {
        // integer ALU operation
        DPRINTF(Monitor, "Multi DIFT: Integer ALU instruction\n");
        DIFTTag trd = false;
        if (TheISA::isISAReg(mp.rd)) {
            trd = thread->readIntReg(mp.rd);
        }
        tresult = false;

        if (TheISA::isISAReg(mp.rs1))
            tresult |= (DIFTTag)thread->readIntReg(mp.rs1);
        if (TheISA::isISAReg(mp.rs2))
            tresult |= (DIFTTag)thread->readIntReg(mp.rs1);
        if (TheISA::isISAReg(mp.rd)) {
            thread->setIntReg((int)mp.rd, (uint64_t)tresult);
            setDropRegTag((int)mp.rd, tresult ? 1 : 0, 0);
        }

        if (trd || tresult) {
            numTaintedIntegerInsts++;
        }
        numIntegerInsts++;
        numMonitorInsts++;
    } else if (mp.load) {
        DPRINTF(Monitor, "Multi DIFT: Load instruction, addr=0x%x\n", mp.memAddr);
        DIFTTag trd = false;
        if (TheISA::isISAReg(mp.rd)) {
            trd = thread->readIntReg(mp.rd);
        }
        tresult = false;
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            tresult |= (DIFTTag)readWordTag(pbyte);
        }
        if (TheISA::isISAReg(mp.rd)) {
            thread->setIntReg((int)mp.rd, (uint64_t)tresult);
            setDropRegTag((int)mp.rd, tresult ? 1 : 0, 0);
        }
        
        if (trd || tresult) {
            numTaintedLoadInsts++;
        }
        numLoadInsts++;
        numMonitorInsts++;
    } else if (mp.store && !mp.settag) {
        DPRINTF(Monitor, "Multi DIFT: Store instruction, addr=0x%x\n", mp.memAddr);
        DIFTTag tsrc = false;
        if (TheISA::isISAReg(mp.rs1)) {
            tsrc = (DIFTTag)thread->readIntReg(mp.rs1);
        }
        DIFTTag tdest = false;

        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            tdest |= readTagFunctional(pbyte);
            writeWordTag(pbyte, tsrc);
        }
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
            setDropMemTag(mp.memAddr, tsrc ? 1 : 0, 0);
        }

        if (tdest || tsrc) {
            numTaintedStoreInsts++;
        }
        numStoreInsts++;
        numMonitorInsts++;
    } else if (mp.custom) {
        DPRINTF(Monitor, "Multi DIFT: Set taint mem[0x%x:0x%x]=%d\n", mp.memAddr, mp.memEnd, mp.data);
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            writeWordTag(pbyte, (bool)mp.data);
        }
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
            setDropMemTag(mp.memAddr, 1, 0);
        }
    } else if (mp.indctrl) {
        DPRINTF(Monitor, "Multi DIFT: Indirect control transfer instruction\n");
        DIFTTag trs1 = false;
        if (TheISA::isISAReg(mp.rs1)) {
            trs1 = (bool)thread->readIntReg(mp.rs1);
        }
        if (trs1) {
            DPRINTF(Monitor, "Multi DIFT: Fatal: indirect control transfer on tainted register, PC=0x%x\n", 
                mp.instAddr);
            numTaintedIndirectCtrlInsts++;
        }
        numIndirectCtrlInsts++;
        numMonitorInsts++;
    // Read syscall
    } else if (mp.settag && mp.syscallReadNbytes > 0) {
      DPRINTF(Monitor, "Multi DIFT: Syscall read instruction\n");
      // Set tags for read syscall
      for (Addr addr = mp.syscallReadBufPtr; addr < mp.syscallReadBufPtr + mp.syscallReadNbytes; addr+=4) {
        writeWordTag(addr, 1);
        setDropMemTag(addr, 1, 0);
      }
    } else {
        warn("Unknown instruction PC = %x\n", mp.instAddr);
        numMonitorInsts++;
    }
}

/**
 * BC Execution
 */
void
AtomicSimpleMonitor::BCExecute()
{
  warn("BC Execute is out-of-date. Please double-check implementation.\n");
    if (mp.intalu) {
        // integer ALU operation
        DPRINTF(Monitor, "BC: Integer ALU instruction\n");
        ALUOpCode opcode = decodeALUOpcode(mp.opcode);
        if (opcode == ALUMov) {
            if (TheISA::isISAReg(mp.rs1)) {
                BCTag trs1 = (BCTag)thread->readIntReg(mp.rs1);
                if (TheISA::isISAReg(mp.rd)) {
                    thread->setIntReg((int)mp.rd, (uint64_t)trs1);
                    setDropRegTag((int)mp.rd, trs1 ? 1 : 0, 0);
                }
            } else {
                if (TheISA::isISAReg(mp.rd)) {
                    // mov immediate
                    thread->setIntReg((int)mp.rd, 0);
                    setDropRegTag((int)mp.rd, 0, 0);
                }
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
                if (TheISA::isISAReg(mp.rd)) {
                    thread->setIntReg((int)mp.rd, (uint64_t)tresult);
                    setDropRegTag((int)mp.rd, tresult ? 1 : 0, 0);
                }
            } else if (TheISA::isISAReg(mp.rs1)) {
                // register + imm operands
                BCTag trs1 = (BCTag)thread->readIntReg(mp.rs1);
                if (TheISA::isISAReg(mp.rd)) {
                    thread->setIntReg((int)mp.rd, (uint64_t)trs1);
                    setDropRegTag((int)mp.rd, trs1 ? 1 : 0, 0);
                }
            } else {
                // we should never reach here
                // panic("Incorrect number of ALU operands!\n");
            }
        } else {
            if (TheISA::isISAReg(mp.rd)) {
                // other ALU operations
                thread->setIntReg((int)mp.rd, 0);
                setDropRegTag((int)mp.rd, 0, 0);
            }
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
            if (TheISA::isISAReg(mp.rd)) {
                thread->setIntReg((int)mp.rd, toPtrTag(tmem));
                setDropRegTag((int)mp.rd, tmem ? 1 : 0, 0);
            }
            if (toPtrTag(tmem) != 0) {
                DPRINTF(Monitor, "BC: write pointer tag %d to r%d\n", toMemTag(tmem), (int)mp.rd);
            }
        } else {
            // should not reach here
            // if we reach here for some reason, conservatively clear dest reg tag
            if (TheISA::isISAReg(mp.rd)) {
                thread->setIntReg((int)mp.rd, 0);
                setDropRegTag((int)mp.rd, 0, 0);
            }
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
            setDropMemTag(mp.memAddr, mergeMemPtrTags(tmem, tsrc) ? 1 : 0, 0);
        } else {
            // should not reach here
            // if we reach here for some reason, conservatively clear pointer tag
            BCTag tmem = readTagFunctional(mp.memAddr);
            writeTag(mp.memAddr, mergeMemPtrTags(tmem, 0));
            setDropMemTag(mp.memAddr, mergeMemPtrTags(tmem, 0) ? 1 : 0, 0);
        }

        numStoreInsts++;
        numMonitorInsts++;
    } else if (mp.store && mp.settag) {
        DPRINTF(Monitor, "BC: Set tag mem[0x%x:0x%x]=%d\n", mp.memAddr, mp.memEnd, mp.data);
        for (Addr pbyte = mp.memAddr; pbyte < mp.memEnd; pbyte++) {
            writeTag(pbyte, (Tag)mp.data);
        }
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
            setDropMemTag(pbyte, 1, 0);
        }
    } else {
        warn("Unknown instruction PC = %x\n", mp.instAddr);
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
    // ALU operation
    if (mp.intalu) {
        // integer ALU operation
        DPRINTF(Monitor, "HardBound: Integer ALU instruction\n");
        ALUOpCode opcode = decodeALUOpcode(mp.opcode);
        if (opcode == ALUMov || opcode == ALUAnd) {
            if (TheISA::isISAReg(mp.rs1)) {
                HBTag trs1 = (HBTag)thread->readIntReg(mp.rs1);
                if (TheISA::isISAReg(mp.rd)) {
                    thread->setIntReg((int)mp.rd, (uint64_t)trs1);
                    DPRINTF(Monitor, "  r[%d] = %lx\n", (int)mp.rd, (uint64_t)trs1);
                    setDropRegTag((int)mp.rd, trs1 ? 1 : 0, 0);
                }
            } else {
                // mov immediate
                if (TheISA::isISAReg(mp.rd)) {
                    thread->setIntReg((int)mp.rd, 0);
                    DPRINTF(Monitor, "  r[%d] = 0\n", (int)mp.rd);
                    setDropRegTag((int)mp.rd, 0, 0);
                }
            }
        } else if ((opcode == ALUAdd) || (opcode == ALUSub) || (opcode == ALUAdduop)) {
            HBTag tresult = 0;
            if (TheISA::isISAReg(mp.rs1) && TheISA::isISAReg(mp.rs2)) {
                HBTag trs1 = (HBTag)thread->readIntReg(mp.rs1);
                HBTag trs2 = (HBTag)thread->readIntReg(mp.rs2);
                if (toBoundTag(trs1)) {
                    tresult = trs1;
                } else {
                    tresult = trs2;
                }
                if (TheISA::isISAReg(mp.rd)) {
                    thread->setIntReg((int)mp.rd, (uint64_t)tresult);
                    DPRINTF(Monitor, "  r[%d] = %lx\n", (int)mp.rd, (uint64_t)tresult);
                    setDropRegTag((int)mp.rd, tresult ? 1 : 0, 0);
                }
            } else if (TheISA::isISAReg(mp.rs1)) {
                HBTag trs1 = (HBTag)thread->readIntReg(mp.rs1);
                if (TheISA::isISAReg(mp.rd)) {
                    thread->setIntReg((int)mp.rd, (uint64_t)trs1);
                    DPRINTF(Monitor, "  r[%d] = %lx\n", (int)mp.rd, (uint64_t)trs1);
                    setDropRegTag((int)mp.rd, trs1 ? 1 : 0, 0);
                }
            } else {
                // we should never reach here
                // panic("Incorrect number of ALU operands!\n");
            }
        } else {
            // other ALU operations
            if (TheISA::isISAReg(mp.rd)) {
                thread->setIntReg((int)mp.rd, 0);
                DPRINTF(Monitor, "  r[%d] = 0\n", (int)mp.rd);
                setDropRegTag((int)mp.rd, 0, 0);
            }
        }

        numIntegerInsts++;
        numMonitorInsts++;
    // Load instruction
    } else if (mp.load) {
        DPRINTF(Monitor, "HardBound: Load instruction, vaddr=0x%x\n", mp.memAddr);

        if (TheISA::isISAReg(mp.rs1) && !(TheISA::isISAReg(mp.rs2))) {
            HBTag tptr = (HBTag)thread->readIntReg(mp.rs1);
            // check if access is within bound
            DPRINTF(Monitor, "  m[%lx < %lx < %lx]\n", toBaseTag(tptr), mp.memAddr, toBoundTag(tptr));
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
                if (TheISA::isISAReg(mp.rd)) {
                    thread->setIntReg((int)mp.rd, tmem);
                    setDropRegTag((int)mp.rd, tmem ? 1 : 0, 0);
                }
            }
        } else {
            // should not reach here
            // if we reach here for some reason, conservatively clear dest reg tag
            if (TheISA::isISAReg(mp.rd)) {
                thread->setIntReg((int)mp.rd, 0);
                setDropRegTag((int)mp.rd, 0, 0);
            }
        }

        numLoadInsts++;
        numMonitorInsts++;
    // Store instruction
    } else if (mp.store && !mp.settag) {
        DPRINTF(Monitor, "HardBound: Store instruction, vaddr=0x%x\n", mp.memAddr);

        if (TheISA::isISAReg(mp.rs1) && TheISA::isISAReg(mp.rs2)) {

            HBTag tsrc = (HBTag)thread->readIntReg(mp.rs1);
            HBTag tptr = (HBTag)thread->readIntReg(mp.rs2);
            DPRINTF(Monitor, "  m[%lx <= %lx <= %lx]\n", toBaseTag(tptr), mp.memAddr, toBoundTag(tptr));
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
            for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
                setDropMemTag(pbyte, tsrc ? 1 : 0, 0);
            }
        } else {
            // should not reach here
            // if we reach here for some reason, conservatively clear pointer tag
            if (mp.size == 4)
                writeDWordTag(mp.physAddr, 0);
            for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
                setDropMemTag(pbyte, 0, 0);
            }
        }

        numStoreInsts++;
        numMonitorInsts++;
    // SETTAG instruction
    } else if (mp.custom) {
        if (mp.size == 0) {
            // set base address
            setTagData = mp.data;
        } else if (mp.size == 1) {
            // set bound address
            setTagData = setTagData | (mp.data << 32);
            DPRINTF(Monitor, "HardBound: Set tag mem[0x%x]=%llx\n", mp.memAddr, setTagData);
            writeDWordTag(mp.physAddr, setTagData);
            setDropMemTag(mp.physAddr, 1, 0);
        }
    } else {
        warn("Unknown instruction PC = %x\n", mp.instAddr);
        numMonitorInsts++;
    }
}

/**
 * Link-Register Check (Return Address Check)
 */
void
AtomicSimpleMonitor::LRCExecute()
{
  static int lr_ptr = 0;

  // Monitoring stats
  numMonitorInsts++;

  // On call,
  if (mp.call) {
    // Save link register address
    writeWordTag(lr_ptr << 2, mp.lr);
    DPRINTF(Monitor, "LRC: push lr[%d]: %x\n", lr_ptr, mp.lr);
    // Update LR stack pointer
    lr_ptr++;
    // Clear invalidation flag (also updates address in dropping hardware)
    revalidateMemTag(lr_ptr << 2);

    // Monitoring stats
    numCallInsts++;
  // On return,
  } else if (mp.ret) {
    // Read saved link register address
    int savedpc = readWordTag((lr_ptr - 1) << 2);
    // Next PC the program is going to
    int nextpc = (int)mp.nextpc;
    // If next PC and saved LR don't match, raise error
    if (savedpc != nextpc) {
      DPRINTF(Monitor, "LRC: ERROR lr (%x) != nextpc (%x)\n", savedpc, nextpc);
    } else {
      DPRINTF(Monitor, "LRC: popped lr (%x) == nextpc (%x)\n", savedpc, nextpc);
    }
    // Update LR stack pointer
    lr_ptr--;
    // Update LR stack pointer in dropping hardware
    setFlagCacheAddr(lr_ptr << 2);

    // Monitoring stats
    numReturnInsts++;
  } else if (!mp.settag) {
    warn("Unknown instruction PC = %x\n", mp.instAddr);
  }
}

/**
 * LockSet Execution
 */
void
AtomicSimpleMonitor::LSExecute()
{
}

/**
 * InstType Execution
 */
void
AtomicSimpleMonitor::InstTypeExecute()
{
    numMonitorInsts++;
    Counter c = numMonitorInsts.value();
    // Monitoring stats
    if ((c % 1000)==0){
        DPRINTF(Monitor, "InstType: Epoch reached. Clear stats.\n");
        for (unsigned i = 0; i < OPCODE_NUM; ++i){
            thread->setIntReg(i, 0);
            setDropRegTag(i, 0, 0);
        }
    }
    InstTypeCount count = (InstTypeCount)thread->readIntReg(mp.opcode_custom);
    DPRINTF(Monitor, "InstType: Instruction opcode %x, count = %u\n", mp.opcode_custom, count+1);
    thread->setIntReg(mp.opcode_custom, count+1);
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
    if (monitorExt == MONITOR_NONE) {
        return;
    } else if (monitorExt == MONITOR_UMC) {
        UMCExecute();
    } else if (monitorExt == MONITOR_DIFT) {
        DIFTExecute();
    } else if (monitorExt == MONITOR_MULTIDIFT) {
        MultiDIFTExecute();
    } else if (monitorExt == MONITOR_BC) {
        BCExecute();
    } else if (monitorExt == MONITOR_SEC) {
        SECExecute();
    } else if (monitorExt == MONITOR_HB) {
        HBExecute();
    } else if (monitorExt == MONITOR_LRC) {
        LRCExecute();
    } else if (monitorExt == MONITOR_DIFTRF) {
        DIFTRFExecute();
    } else if (monitorExt == MONITOR_LS) {
        LSExecute();
    } else if (monitorExt == MONITOR_INSTTYPE) {
        InstTypeExecute();
    } else
        panic("Unknown monitoring extension specified\n");

    // clear monitoring packet after processing
    mp.init();
}

void
AtomicSimpleMonitor::tick()
{
    DPRINTF(SimpleCPU, "Monitor Tick\n");
    DPRINTF(MonitorSimpleCPU, "Monitor Tick\n");

    Tick latency = 0;
    Tick stall_ticks = 0;
    dcache_access = false; // assume no dcache access
    Tick single_tick = ticks(1);

    if (!initialized) {
        // initialize monitor on first cycle
        thread->clearArchRegs();
        initialized = true;
    } else if (enabled) {
        // fetch an entry from main core
        preExecute();
        
        if (mp.valid) {
            // model latency of reading a packet
            // stall_ticks += ticks(1);
            // packet is valid, continue processing
            if (!mp.done) {
                processMonitoringPacket();
            } else {
                finishMonitoring();
            }
            // since we are doing full monitoring, we need to add the full_delay cycles
            single_tick = ticks(1+full_delay);
        } 

    }

    if (simulate_data_stalls && dcache_access)
        stall_ticks += dcache_latency;

    if (stall_ticks) {
        Tick stall_cycles = stall_ticks / single_tick;
        Tick aligned_stall_ticks = single_tick * stall_cycles;

        if (aligned_stall_ticks < stall_ticks)
            aligned_stall_ticks += single_tick;

        latency += aligned_stall_ticks;
    }
    
    // instruction takes at least one cycle
    if (latency < single_tick)
        latency = single_tick;

    if (_status != Idle)
        schedule(tickEvent, roundDown(curTick() + latency + 2, 3) - 2);
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
      case 0x06:
        return ALUAdduop;
      case 0x0c:
        return ALUAdduop;
      case 0x02:
        return ALUSub;
      case 0x00:
        return ALUAnd;
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
