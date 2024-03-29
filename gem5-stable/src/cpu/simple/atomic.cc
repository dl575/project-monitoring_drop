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
#include "config/the_isa.hh"
#include "cpu/simple/atomic.hh"
#include "cpu/simple/drop.hh" // Include for flagcache defines
#include "cpu/exetrace.hh"
#include "debug/ExecFaulting.hh"
#include "debug/SimpleCPU.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/physical.hh"
#include "params/AtomicSimpleCPU.hh"
#include "sim/faults.hh"
#include "sim/system.hh"
#include "sim/full_system.hh"

#include "mem/fifo.hh"
#include "mem/timer.hh"
#include "mem/flag_cache.hh"

using namespace std;
using namespace TheISA;

SimpleThread* AtomicSimpleCPU::main_thread = NULL;

AtomicSimpleCPU::TickEvent::TickEvent(AtomicSimpleCPU *c)
    : Event(CPU_Tick_Pri), cpu(c)
{
}


void
AtomicSimpleCPU::TickEvent::process()
{
    cpu->tick();
}

const char *
AtomicSimpleCPU::TickEvent::description() const
{
    return "AtomicSimpleCPU tick";
}

void
AtomicSimpleCPU::init()
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

    // Copy thread pointer
    if (main_core)
        main_thread = thread;
}

AtomicSimpleCPU::AtomicSimpleCPU(AtomicSimpleCPUParams *p)
    : BaseSimpleCPU(p), tickEvent(this), width(p->width), locked(false),
      simulate_data_stalls(p->simulate_data_stalls),
      simulate_inst_stalls(p->simulate_inst_stalls),
      icachePort(name() + "-iport", this), dcachePort(name() + "-iport", this),
      monitorPort(name() + "-iport", this),
      main_core(p->main_core),
      fastmem(p->fastmem),
      fifoEvent(this)
{
    _status = Idle;
}


AtomicSimpleCPU::~AtomicSimpleCPU()
{
    if (tickEvent.scheduled()) {
        deschedule(tickEvent);
    }
}

void
AtomicSimpleCPU::serialize(ostream &os)
{
    SimObject::State so_state = SimObject::getState();
    SERIALIZE_ENUM(so_state);
    SERIALIZE_SCALAR(locked);
    BaseSimpleCPU::serialize(os);
    nameOut(os, csprintf("%s.tickEvent", name()));
    tickEvent.serialize(os);
}

void
AtomicSimpleCPU::unserialize(Checkpoint *cp, const string &section)
{
    SimObject::State so_state;
    UNSERIALIZE_ENUM(so_state);
    UNSERIALIZE_SCALAR(locked);
    BaseSimpleCPU::unserialize(cp, section);
    tickEvent.unserialize(cp, csprintf("%s.tickEvent", section));
}

void
AtomicSimpleCPU::resume()
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
AtomicSimpleCPU::switchOut()
{
    assert(_status == Running || _status == Idle);
    _status = SwitchedOut;

    tickEvent.squash();
}


void
AtomicSimpleCPU::takeOverFrom(BaseCPU *oldCPU)
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
AtomicSimpleCPU::activateContext(ThreadID thread_num, int delay)
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
AtomicSimpleCPU::suspendContext(ThreadID thread_num)
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


Fault
AtomicSimpleCPU::readMem(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags)
{

    if (traceData) {
        traceData->setAddr(addr);
    }
    // Save address for monitoring
    fed.memAddr = addr;
    fed.dataVirtAddr = addr;
    fed.dataSize = size;

    // Read from fifo
    if (fifo_enabled && addr >= FIFO_ADDR_START && addr <= FIFO_ADDR_END) {
      return readFromFifo(addr, data, size, flags);
    }

    // Read from timer
    if (timer_enabled && (addr == TIMER_READ_SLACK || addr == TIMER_READ_DROP)) {
      return readFromTimer(addr, data, size, flags);
    }
    
    // Read from flag cache
    if (flagcache_enabled && (addr >= FLAG_CACHE_ADDR_START && addr <= FLAG_CACHE_ADDR_END)){
        return readFromFlagCache(addr, data, size, flags);
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
            // save physical memory address for monitoring
            fed.dataPhysAddr = req->getPaddr();
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

        // Save data for monitoring
        fed.data = 0;
        memcpy(&fed.data, data, size);

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
AtomicSimpleCPU::writeMem(uint8_t *data, unsigned size,
                          Addr addr, unsigned flags, uint64_t *res)
{

    if (traceData) {
        traceData->setAddr(addr);
    }
    // Save memory address for monitoring
    fed.memAddr = addr;
    fed.dataVirtAddr = addr;
    fed.dataSize = size;
    // Save data being written
    fed.data = 0;
    memcpy(&fed.data, data, size);
    
    dcache_latency = 0;

    // Write to fifo
    // Used to handle fifo control (writing data to fifo is done 
    // automatically by monitoring)
    if (fifo_enabled && (addr >= FIFO_OP_RANGE_START && addr < FIFO_OP_RANGE_END)) {
        return writeToFifo(addr, data, size, flags);
    }
    // Timer
    if (timer_enabled && (addr >= TIMER_ADDR_START && addr <= TIMER_ADDR_END)) { 
        return writeToTimer(addr, data, size, flags);
    }
    
    // Write to flag cache
    if (flagcache_enabled && (addr >= FLAG_CACHE_ADDR_START && addr <= FLAG_CACHE_ADDR_END)) {
      // If invalidation is performed on this core
      if (invtab.initialized) {
        return writeToFlagCache(addr, data, size, flags);
      // Invalidation is done on another core, send message
      } else {
        // Create request
        Request *req = &monitor_req;
        Addr drop_addr = 0;
        switch (addr) {
            case FC_SET_ADDR: drop_addr = DROP_FC_SET_ADDR; break;
            case FC_CACHE_REVALIDATE: drop_addr = DROP_CLEAR_CACHE; break;
            case FC_ARRAY_REVALIDATE: drop_addr = DROP_CLEAR_ARRAY; break;
            case FC_CACHE_INVALIDATE: drop_addr = DROP_SET_CACHE; break;
            case FC_ARRAY_INVALIDATE: drop_addr = DROP_SET_ARRAY; break;
            case FC_SET_ARRAY: drop_addr = DROP_SET_ARRAY_VALUE; break;
            case FC_SET_CACHE: drop_addr = DROP_SET_CACHE_VALUE; break;
            default: panic("Unimplemented Flag Cache Operation %x\n", addr);
        }          
        req->setPhys(drop_addr, sizeof(fed.data), ArmISA::TLB::AllowUnaligned, dataMasterId());
        // Create packet
        PacketPtr p = new Packet(req, MemCmd::WriteReq);
        p->dataStatic(&fed.data);
        // Send packet
        monitorPort.sendFunctional(p);
        // Clean up
        delete p;

        return NoFault;
      }
    }

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
            // save physical memory address for monitoring
            fed.dataPhysAddr = req->getPaddr();

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


void
AtomicSimpleCPU::tick()
{
    DPRINTF(SimpleCPU, "Tick\n");

    Tick latency = 0;

    for (int i = 0; i < width || locked; ++i) {
        numCycles++;

        if (!curStaticInst || !curStaticInst->isDelayedCommit())
            checkForInterrupts();

        checkPcEventQueue();
        // We must have just got suspended by a PC event
        if (_status == Idle)
            return;

        Fault fault = NoFault;

        TheISA::PCState pcState = thread->pcState();

        bool needToFetch = !isRomMicroPC(pcState.microPC()) &&
                           !curMacroStaticInst;
        if (needToFetch) {
            setupFetchRequest(&ifetch_req);
            fault = thread->itb->translateAtomic(&ifetch_req, tc,
                                                 BaseTLB::Execute);
        }

        if (fault == NoFault) {
            Tick icache_latency = 0;
            Tick fifo_latency = 0;
            timer_latency = 0;
            bool icache_access = false;
            dcache_access = false; // assume no dcache access

            if (needToFetch) {
                // This is commented out because the decoder would act like
                // a tiny cache otherwise. It wouldn't be flushed when needed
                // like the I cache. It should be flushed, and when that works
                // this code should be uncommented.
                //Fetch more instruction memory if necessary
                //if(decoder.needMoreBytes())
                //{
                    icache_access = true;
                    Packet ifetch_pkt = Packet(&ifetch_req, MemCmd::ReadReq);
                    ifetch_pkt.dataStatic(&inst);

                    if (fastmem && system->isMemAddr(ifetch_pkt.getAddr()))
                        system->getPhysMem().access(&ifetch_pkt);
                    else
                        icache_latency = icachePort.sendAtomic(&ifetch_pkt);

                    assert(!ifetch_pkt.isError());

                    // ifetch_req is initialized to read the instruction directly
                    // into the CPU object's inst field.
                //}
            }

            preExecute();

            if (curStaticInst) {

                setPredicate(true);
                fault = curStaticInst->execute(this, traceData);

                // keep an instruction count
                if (fault == NoFault)
                    countInst();
                else if (traceData && !DTRACE(ExecFaulting)) {
                    delete traceData;
                    traceData = NULL;
                }

                postExecute();
               
            }

            // @todo remove me after debugging with legion done
            if (curStaticInst && (!curStaticInst->isMicroop() ||
                        curStaticInst->isFirstMicroop()))
                instCnt++;

            Tick stall_ticks = 0;
            if (simulate_inst_stalls && icache_access)
                stall_ticks += icache_latency;

            if (simulate_data_stalls && dcache_access)
                stall_ticks += dcache_latency;

            // Add fifo writing stall
            stall_ticks += fifo_latency;
            // Add timer latency for stall
            stall_ticks += timer_latency;

            if (stall_ticks) {
                Tick stall_cycles = stall_ticks / ticks(1);
                Tick aligned_stall_ticks = ticks(stall_cycles);

                if (aligned_stall_ticks < stall_ticks)
                    aligned_stall_ticks += ticks(1);
                    
                latency += aligned_stall_ticks;
            }

        }
        if(fault != NoFault || !stayAtPC)
            advancePC(fault);
    }

    // instruction takes at least one cycle
    if (latency < ticks(1))
        latency = ticks(1);

    if (_status != Idle) {
        if (fifoStall) {
            DPRINTF(Fifo, "Rescheduling...\n");
            schedule(fifoEvent, curTick() + latency);
        #ifdef DEBUG
            // Count how many packets cause a stall
            num_packets++;
        #endif
            // Store start of stall time
            fifoStallTicks = curTick();
        } else {
            schedule(tickEvent, curTick() + latency);
        }
    }
}

// function to handle end_task command
void AtomicSimpleCPU::endTask() {
  if (hard_wcet) {
    timerStalled = true;
    timer_latency = fed.data;
    DPRINTF(SlackTimer, "The CPU will be stalled for %d ticks\n", fed.data);
  } else {
    DPRINTF(SlackTimer, "hard_wcet = false. CPU is not stalled at end of task.\n");
  }
}

void AtomicSimpleCPU::handleFifoEvent() {

  // Try again
  if (sendFifoPacket()) {
    // Successful
    // Schedule tick event to resume CPU
    fifoStall = false;

    // Finish decrementing timer
    Request *req = &data_write_req;
    req->setPhys(TIMER_END_DECREMENT, sizeof(bool), ArmISA::TLB::AllowUnaligned, dataMasterId());
    // Read command
    MemCmd cmd = MemCmd::WriteReq;
    // Create packet
    PacketPtr pkt = new Packet(req, cmd);
    // Point packet to data pointer
    pkt->dataStatic(&fifoStall);

    // Send read request
    timerPort.sendFunctional(pkt);

    delete pkt;

    schedule(tickEvent, curTick() + ticks(1));
    
  #ifdef DEBUG
    unsigned stall_amt = curTick()-fifoStallTicks;
    DPRINTF(Fifo, "Success, stalled for %d\n", stall_amt/ticks(1));
    num_stalls += stall_amt;
    DPRINTF(FifoStall, "Fifo caused stall for %d ticks\n", stall_amt);
  #endif
  } else {
    // Failed
    // Retry on next cycle
    schedule(fifoEvent, curTick() + ticks(1));
  }

}

// Function to setup monitoring for read syscall
void 
AtomicSimpleCPU::monitorSyscallRead(Addr bufPtr, int nbytes)
{
  // Mark that there is a syscall read that should be sent throught FIFO
  fed.syscallRead = true;
  // Save information about the read so that the monitor can handle it
  fed.syscallReadBufPtr = bufPtr;
  fed.syscallReadNbytes = nbytes;
}

void
AtomicSimpleCPU::printAddr(Addr a)
{
    dcachePort.printAddr(a);
}


////////////////////////////////////////////////////////////////////////
//
//  AtomicSimpleCPU Simulation Object
//
AtomicSimpleCPU *
AtomicSimpleCPUParams::create()
{
    numThreads = 1;
    if (!FullSystem && workload.size() != 1)
        panic("only one workload allowed");
    return new AtomicSimpleCPU(this);
}

MasterPort & 
AtomicSimpleCPU::getMasterPort(const std::string &if_name, int idx)
{
  if (if_name == "monitor_port") {
    return monitorPort;
  } else {
    return BaseSimpleCPU::getMasterPort(if_name, idx);
  }
}
