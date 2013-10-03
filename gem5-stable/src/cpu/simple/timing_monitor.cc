/*
 * Copyright (c) 2010-2012 ARM Limited
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
 */

#include "arch/locked_mem.hh"
#include "arch/mmapped_ipr.hh"
#include "arch/utility.hh"
#include "base/bigint.hh"
#include "config/the_isa.hh"
#include "cpu/simple/timing_monitor.hh"
#include "cpu/exetrace.hh"
#include "debug/Config.hh"
#include "debug/ExecFaulting.hh"
#include "debug/SimpleCPU.hh"
#include "debug/Monitor.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/TimingSimpleMonitor.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

#include "mem/fifo.hh"
#include "mem/timer.hh"
#include "mem/flag_cache.hh"

#include "cpu/simple/drop.hh"

using namespace std;
using namespace TheISA;

void
TimingSimpleMonitor::init()
{
    BaseSimpleCPU::init();

    // Initialise the ThreadContext's memory proxies
    tcBase()->initMemProxies(tcBase());

    if (FullSystem && !params()->defer_registration) {
        for (int i = 0; i < threadContexts.size(); ++i) {
            ThreadContext *tc = threadContexts[i];
            // initialize CPU, including PC
            TheISA::initCPU(tc, _cpuId);
        }
    }

    // enable the monitor
    enabled = true;
    // monitor not initialized yet
    initialized = false;
}

void
TimingSimpleMonitor::TimingMonitorPort::TickEvent::schedule(PacketPtr _pkt, Tick t)
{
    pkt = _pkt;
    cpu->schedule(this, t);
}

TimingSimpleMonitor::TimingSimpleMonitor(TimingSimpleMonitorParams *p)
    : BaseSimpleCPU(p), fetchTranslation(this), icachePort(this),
    dcachePort(this), monitorPort(name() + "-iport", this), fetchEvent(this),
    fifoEvent(this), endTaskEvent(this)
{
    _status = Idle;

    ifetch_pkt = dcache_pkt = NULL;
    drainEvent = NULL;
    previousTick = 0;
    changeState(SimObject::Running);
    system->totalNumInsts = 0;

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


TimingSimpleMonitor::~TimingSimpleMonitor()
{
}

void
TimingSimpleMonitor::serialize(ostream &os)
{
    SimObject::State so_state = SimObject::getState();
    SERIALIZE_ENUM(so_state);
    BaseSimpleCPU::serialize(os);
}

void
TimingSimpleMonitor::unserialize(Checkpoint *cp, const string &section)
{
    SimObject::State so_state;
    UNSERIALIZE_ENUM(so_state);
    BaseSimpleCPU::unserialize(cp, section);
}

unsigned int
TimingSimpleMonitor::drain(Event *drain_event)
{
    // TimingSimpleMonitor is ready to drain if it's not waiting for
    // an access to complete.
    if (_status == Idle || _status == Running || _status == SwitchedOut) {
        changeState(SimObject::Drained);
        return 0;
    } else {
        changeState(SimObject::Draining);
        drainEvent = drain_event;
        return 1;
    }
}

void
TimingSimpleMonitor::resume()
{
    DPRINTF(SimpleCPU, "Resume\n");
    if (_status != SwitchedOut && _status != Idle) {
        assert(system->getMemoryMode() == Enums::timing);

        if (fetchEvent.scheduled())
           deschedule(fetchEvent);

        schedule(fetchEvent, nextCycle());
    }

    changeState(SimObject::Running);
}

void
TimingSimpleMonitor::switchOut()
{
    assert(_status == Running || _status == Idle);
    _status = SwitchedOut;
    numCycles += tickToCycles(curTick() - previousTick);

    // If we've been scheduled to resume but are then told to switch out,
    // we'll need to cancel it.
    if (fetchEvent.scheduled())
        deschedule(fetchEvent);
}


void
TimingSimpleMonitor::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    // if any of this CPU's ThreadContexts are active, mark the CPU as
    // running and schedule its tick event.
    for (int i = 0; i < threadContexts.size(); ++i) {
        ThreadContext *tc = threadContexts[i];
        if (tc->status() == ThreadContext::Active && _status != Running) {
            _status = Running;
            break;
        }
    }

    if (_status != Running) {
        _status = Idle;
    }
    assert(threadContexts.size() == 1);
    previousTick = curTick();
}


void
TimingSimpleMonitor::activateContext(ThreadID thread_num, int delay)
{
    DPRINTF(SimpleCPU, "ActivateContext %d (%d cycles)\n", thread_num, delay);

    assert(thread_num == 0);
    assert(thread);

    assert(_status == Idle);

    notIdleFraction++;
    _status = Running;

    // kick things off by initiating the fetch of the next instruction
    schedule(fetchEvent, nextCycle(curTick() + ticks(delay)));
}


void
TimingSimpleMonitor::suspendContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "SuspendContext %d\n", thread_num);

    assert(thread_num == 0);
    assert(thread);

    if (_status == Idle)
        return;

    assert(_status == Running);

    // just change status to Idle... if status != Running,
    // completeInst() will not initiate fetch of next instruction.

    notIdleFraction--;
    _status = Idle;
}

bool
TimingSimpleMonitor::handleReadPacket(PacketPtr pkt)
{
    RequestPtr req = pkt->req;
    if (req->isMmappedIpr()) {
        Tick delay;
        delay = TheISA::handleIprRead(thread->getTC(), pkt);
        new IprEvent(pkt, this, nextCycle(curTick() + delay));
        _status = DcacheWaitResponse;
        dcache_pkt = NULL;
    } else if (!dcachePort.sendTimingReq(pkt)) {
        _status = DcacheRetry;
        dcache_pkt = pkt;
    } else {
        _status = DcacheWaitResponse;
        // memory system takes ownership of packet
        dcache_pkt = NULL;
    }
    return dcache_pkt == NULL;
}

void
TimingSimpleMonitor::sendData(RequestPtr req, uint8_t *data, uint64_t *res,
                          bool read)
{
    PacketPtr pkt;
    buildPacket(pkt, req, read);
    pkt->dataDynamicArray<uint8_t>(data);
    if (req->getFlags().isSet(Request::NO_ACCESS)) {
        assert(!dcache_pkt);
        pkt->makeResponse();
        completeDataAccess(pkt);
    } else if (read) {
        handleReadPacket(pkt);
    } else {
        bool do_access = true;  // flag to suppress cache access

        if (req->isLLSC()) {
            do_access = TheISA::handleLockedWrite(thread, req);
        } else if (req->isCondSwap()) {
            assert(res);
            req->setExtraData(*res);
        }

        if (do_access) {
            dcache_pkt = pkt;
            handleWritePacket();
        } else {
            _status = DcacheWaitResponse;
            completeDataAccess(pkt);
        }
    }
}

void
TimingSimpleMonitor::sendSplitData(RequestPtr req1, RequestPtr req2,
                               RequestPtr req, uint8_t *data, bool read)
{
    PacketPtr pkt1, pkt2;
    buildSplitPacket(pkt1, pkt2, req1, req2, req, data, read);
    if (req->getFlags().isSet(Request::NO_ACCESS)) {
        assert(!dcache_pkt);
        pkt1->makeResponse();
        completeDataAccess(pkt1);
    } else if (read) {
        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(pkt1->senderState);
        if (handleReadPacket(pkt1)) {
            send_state->clearFromParent();
            send_state = dynamic_cast<SplitFragmentSenderState *>(
                    pkt2->senderState);
            if (handleReadPacket(pkt2)) {
                send_state->clearFromParent();
            }
        }
    } else {
        dcache_pkt = pkt1;
        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(pkt1->senderState);
        if (handleWritePacket()) {
            send_state->clearFromParent();
            dcache_pkt = pkt2;
            send_state = dynamic_cast<SplitFragmentSenderState *>(
                    pkt2->senderState);
            if (handleWritePacket()) {
                send_state->clearFromParent();
            }
        }
    }
}

void
TimingSimpleMonitor::translationFault(Fault fault)
{
    // fault may be NoFault in cases where a fault is suppressed,
    // for instance prefetches.
    numCycles += tickToCycles(curTick() - previousTick);
    previousTick = curTick();

    if (traceData) {
        // Since there was a fault, we shouldn't trace this instruction.
        delete traceData;
        traceData = NULL;
    }

    // Don't perform monitoring for the translationFault
    perf_mon = false;
    // skip postExecute entirely for monitor on tranlsation fault
    postExecute();
    // Reset perform monitoring flag
    perf_mon = true;

    if (getState() == SimObject::Draining) {
        advancePC(fault);
        completeDrain();
    } else {
        advanceInst(fault);
    }
}

void
TimingSimpleMonitor::buildPacket(PacketPtr &pkt, RequestPtr req, bool read)
{
    MemCmd cmd;
    if (read) {
        cmd = MemCmd::ReadReq;
        if (req->isLLSC())
            cmd = MemCmd::LoadLockedReq;
    } else {
        cmd = MemCmd::WriteReq;
        if (req->isLLSC()) {
            cmd = MemCmd::StoreCondReq;
        } else if (req->isSwap()) {
            cmd = MemCmd::SwapReq;
        }
    }
    pkt = new Packet(req, cmd);
}

void
TimingSimpleMonitor::buildSplitPacket(PacketPtr &pkt1, PacketPtr &pkt2,
        RequestPtr req1, RequestPtr req2, RequestPtr req,
        uint8_t *data, bool read)
{
    pkt1 = pkt2 = NULL;

    assert(!req1->isMmappedIpr() && !req2->isMmappedIpr());

    if (req->getFlags().isSet(Request::NO_ACCESS)) {
        buildPacket(pkt1, req, read);
        return;
    }

    buildPacket(pkt1, req1, read);
    buildPacket(pkt2, req2, read);

    req->setPhys(req1->getPaddr(), req->getSize(), req1->getFlags(), dataMasterId());
    PacketPtr pkt = new Packet(req, pkt1->cmd.responseCommand());

    pkt->dataDynamicArray<uint8_t>(data);
    pkt1->dataStatic<uint8_t>(data);
    pkt2->dataStatic<uint8_t>(data + req1->getSize());

    SplitMainSenderState * main_send_state = new SplitMainSenderState;
    pkt->senderState = main_send_state;
    main_send_state->fragments[0] = pkt1;
    main_send_state->fragments[1] = pkt2;
    main_send_state->outstanding = 2;
    pkt1->senderState = new SplitFragmentSenderState(pkt, 0);
    pkt2->senderState = new SplitFragmentSenderState(pkt, 1);
}

Fault
TimingSimpleMonitor::readMem(Addr addr, uint8_t *data,
                         unsigned size, unsigned flags)
{
    if (traceData) {
        traceData->setAddr(addr);
    }
    // Save address for monitoring
    fed.memAddr = addr;

    // Read from fifo
    if (fifo_enabled && addr >= FIFO_ADDR_START && addr <= FIFO_ADDR_END) {
      // Read from fifo into data
      Fault result = readFromFifo(addr, data, size, flags);

      // Create packet and request to be used in completeDataAccess
      const int asid = 0;
      const ThreadID tid = 0;
      const Addr pc = thread->instAddr();
      RequestPtr req2  = new Request(asid, addr, size,
                                    flags, dataMasterId(), pc, _cpuId, tid);
      MemCmd cmd2 = MemCmd::ReadReq;
      PacketPtr pkt2 = new Packet(req2, cmd2);
      pkt2->dataStatic(data);
      pkt2->req->setFlags(Request::NO_ACCESS);

      // Only return data if no fault
      if (result == NoFault) {
        completeDataAccess(pkt2);
      }

      return result;  
    }

    // Read from timer
    if (timer_enabled && (addr == TIMER_READ_SLACK || addr == TIMER_READ_DROP)) {
      // Read from timer into data
      Fault result = readFromTimer(addr, data, size, flags);

      // Create packet and request to be used in completeDataAccess
      const int asid = 0;
      const ThreadID tid = 0;
      const Addr pc = thread->instAddr();
      RequestPtr req2  = new Request(asid, addr, size,
                                    flags, dataMasterId(), pc, _cpuId, tid);
      MemCmd cmd2 = MemCmd::ReadReq;
      PacketPtr pkt2 = new Packet(req2, cmd2);
      pkt2->dataStatic(data);
      pkt2->req->setFlags(Request::NO_ACCESS);

      // Only return data if no fault
      if (result == NoFault){
        completeDataAccess(pkt2);
      }

      return result;
    }
    
    // Read from flag cache
    if (flagcache_enabled && (addr >= FLAG_CACHE_ADDR_START && addr <= FLAG_CACHE_ADDR_END)) {
      // Read from flag cache into data
      Fault result = readFromFlagCache(addr, data, size, flags);

      // Create packet and request to be used in completeDataAccess
      const int asid = 0;
      const ThreadID tid = 0;
      const Addr pc = thread->instAddr();
      RequestPtr req2  = new Request(asid, addr, size,
                                    flags, dataMasterId(), pc, _cpuId, tid);
      MemCmd cmd2 = MemCmd::ReadReq;
      PacketPtr pkt2 = new Packet(req2, cmd2);
      pkt2->dataStatic(data);
      pkt2->req->setFlags(Request::NO_ACCESS);

      // Only return data if no fault
      if (result == NoFault){
        completeDataAccess(pkt2);
      }

      return result;
    }

    Fault fault;
    const int asid = 0;
    const ThreadID tid = 0;
    const Addr pc = thread->instAddr();
    unsigned block_size = dcachePort.peerBlockSize();
    BaseTLB::Mode mode = BaseTLB::Read;

    RequestPtr req  = new Request(asid, addr, size,
                                  flags, dataMasterId(), pc, _cpuId, tid);

    Addr split_addr = roundDown(addr + size - 1, block_size);
    assert(split_addr <= addr || split_addr - addr < block_size);

    _status = DTBWaitResponse;
    if (split_addr > addr) {
      //FIXME
panic("split read not handled\n");
        RequestPtr req1, req2;
        assert(!req->isLLSC() && !req->isSwap());
        req->splitOnVaddr(split_addr, req1, req2);

        WholeTranslationState *state =
            new WholeTranslationState(req, req1, req2, new uint8_t[size],
                                      NULL, mode);
        DataTranslation<TimingSimpleMonitor *> *trans1 =
            new DataTranslation<TimingSimpleMonitor *>(this, state, 0);
        DataTranslation<TimingSimpleMonitor *> *trans2 =
            new DataTranslation<TimingSimpleMonitor *>(this, state, 1);

        thread->dtb->translateTiming(req1, tc, trans1, mode);
        thread->dtb->translateTiming(req2, tc, trans2, mode);
    } else {
      /*
        WholeTranslationState *state =
            new WholeTranslationState(req, new uint8_t[size], NULL, mode);
        DataTranslation<TimingSimpleMonitor *> *translation
            = new DataTranslation<TimingSimpleMonitor *>(this, state);
        thread->dtb->translateTiming(req, tc, translation, mode);
            */
        Fault fault = thread->dtb->translateAtomic(req, tc, mode);
        // return fault if found
        if (fault != NoFault) {
          return fault;
        }
        WholeTranslationState *state =
            new WholeTranslationState(req, new uint8_t[size], NULL, mode);
        DataTranslation<TimingSimpleMonitor *> *translation
            = new DataTranslation<TimingSimpleMonitor *>(this, state);
        translation->finish(fault, req, tc, mode);
    }


    return NoFault;
}


Fault
TimingSimpleMonitor::readMemFunctional(Addr addr, uint8_t * data,
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


            /*
            if (fastmem && system->isMemAddr(pkt.getAddr()))
                system->getPhysMem().access(&pkt);
            else
                dcachePort.sendFunctional(&pkt);
                */
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

bool
TimingSimpleMonitor::handleWritePacket()
{
    RequestPtr req = dcache_pkt->req;
    if (req->isMmappedIpr()) {
        Tick delay;
        delay = TheISA::handleIprWrite(thread->getTC(), dcache_pkt);
        new IprEvent(dcache_pkt, this, nextCycle(curTick() + delay));
        _status = DcacheWaitResponse;
        dcache_pkt = NULL;
    } else if (!dcachePort.sendTimingReq(dcache_pkt)) {
        _status = DcacheRetry;
    } else {
        _status = DcacheWaitResponse;
        // memory system takes ownership of packet
        dcache_pkt = NULL;
    }
    return dcache_pkt == NULL;
}

Fault
TimingSimpleMonitor::writeMem(uint8_t *data, unsigned size,
                          Addr addr, unsigned flags, uint64_t *res)
{
    uint8_t *newData = new uint8_t[size];
    memcpy(newData, data, size);

    const int asid = 0;
    const ThreadID tid = 0;
    const Addr pc = thread->instAddr();
    unsigned block_size = dcachePort.peerBlockSize();
    BaseTLB::Mode mode = BaseTLB::Write;

    if (traceData) {
        traceData->setAddr(addr);
    }
    // Save memory address for monitoring
    fed.memAddr = addr;
    // Save data being written
    fed.data = 0;
    memcpy(&fed.data, data, size);

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
        return writeToFlagCache(addr, data, size, flags);
    }

    RequestPtr req = new Request(asid, addr, size,
                                 flags, dataMasterId(), pc, _cpuId, tid);

    Addr split_addr = roundDown(addr + size - 1, block_size);
    assert(split_addr <= addr || split_addr - addr < block_size);

    _status = DTBWaitResponse;
    if (split_addr > addr) {
      //FIXME:
      panic("split write not handled\n");
        RequestPtr req1, req2;
        assert(!req->isLLSC() && !req->isSwap());
        req->splitOnVaddr(split_addr, req1, req2);

        WholeTranslationState *state =
            new WholeTranslationState(req, req1, req2, newData, res, mode);
        DataTranslation<TimingSimpleMonitor *> *trans1 =
            new DataTranslation<TimingSimpleMonitor *>(this, state, 0);
        DataTranslation<TimingSimpleMonitor *> *trans2 =
            new DataTranslation<TimingSimpleMonitor *>(this, state, 1);

        thread->dtb->translateTiming(req1, tc, trans1, mode);
        thread->dtb->translateTiming(req2, tc, trans2, mode);
    } else {
      /*
        WholeTranslationState *state =
            new WholeTranslationState(req, newData, res, mode);
        DataTranslation<TimingSimpleMonitor *> *translation =
            new DataTranslation<TimingSimpleMonitor *>(this, state);
        thread->dtb->translateTiming(req, tc, translation, mode);
        */
        Fault fault = thread->dtb->translateAtomic(req, tc, mode);
        // Return fault if found
        if (fault != NoFault) {
          return fault;
        }
        WholeTranslationState *state =
            new WholeTranslationState(req, newData, res, mode);
        DataTranslation<TimingSimpleMonitor *> *translation =
            new DataTranslation<TimingSimpleMonitor *>(this, state);
        translation->finish(fault, req, tc, mode);
    }

    // Translation faults will be returned via finishTranslation()
    return NoFault;
}

Fault
TimingSimpleMonitor::writeMemFunctional(uint8_t *data, unsigned size,
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

                /*
                if (fastmem && system->isMemAddr(pkt.getAddr()))
                    system->getPhysMem().access(&pkt);
                else
                    dcachePort.sendFunctional(&pkt);
                    */
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
TimingSimpleMonitor::finishTranslation(WholeTranslationState *state)
{
    _status = Running;

    if (state->getFault() != NoFault) {
        if (state->isPrefetch()) {
            state->setNoFault();
        }
        delete [] state->data;
        state->deleteReqs();
        translationFault(state->getFault());
    } else {
        if (!state->isSplit) {
            sendData(state->mainReq, state->data, state->res,
                     state->mode == BaseTLB::Read);
        } else {
            sendSplitData(state->sreqLow, state->sreqHigh, state->mainReq,
                          state->data, state->mode == BaseTLB::Read);
        }
    }

    delete state;
}


void
TimingSimpleMonitor::fetch()
{
    DPRINTF(SimpleCPU, "Monitor Tick\n");

    if (!initialized) {
        // initialize monitor on first cycle
        thread->clearArchRegs();
        initialized = true;
    } else if (enabled) {
        // fetch an entry from main core
      if (_status == Running) {
        preExecuteMonitor();
      }

        if (mp.valid) {
            // model latency of reading a packet
            // stall_ticks += ticks(1);
            // packet is valid, continue processing
            if (!mp.done) {
                processMonitoringPacket();
                // dcache miss, stop now and wait for dcache response
                if (_status != Running) {
                  return;
                }
            } else {
                finishMonitoring();
            }
        }

    }

    if (_status != Idle) {
      // If there was already one scheduled, kill that one
      if (fetchEvent.scheduled()) {
        deschedule(fetchEvent);
      }
        schedule(fetchEvent, curTick() + ticks(1));
    }
}

/*
 * Pre-execute
 *  Try to read a monitoring packet from FIFO.
 */
void
TimingSimpleMonitor::preExecuteMonitor()
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

void
TimingSimpleMonitor::processMonitoringPacket()
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
    if (_status == Running) {
      mp.init();
    }
}

/**
 * UMC Execution
 */
void
TimingSimpleMonitor::UMCExecute()
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
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
            revalidateMemTag(pbyte);
        }
    } else if (mp.store && mp.settag) {
        DPRINTF(Monitor, "UMC: Initializing mem[0x%x:0x%x]\n", mp.memAddr, mp.memEnd);
        numMonitorInsts++;
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte++) {
            writeBitTag(pbyte, 1);
        }
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
            revalidateMemTag(pbyte);
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
TimingSimpleMonitor::DIFTExecute()
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
        revalidateRegTag((int)mp.rd);

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
            // dcache miss, return now and wait for dcache response
            if (_status != Running) {
              return;
            }
        }
        thread->setIntReg((int)mp.rd, (uint64_t)tresult);
        revalidateRegTag((int)mp.rd);
        
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
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
            revalidateMemTag(pbyte);
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
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
            revalidateMemTag(pbyte);
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
TimingSimpleMonitor::BCExecute()
{
    if (mp.intalu) {
        // integer ALU operation
        DPRINTF(Monitor, "BC: Integer ALU instruction\n");
        ALUOpCode opcode = decodeALUOpcode(mp.opcode);
        if (opcode == ALUMov) {
            if (TheISA::isISAReg(mp.rs1)) {
                BCTag trs1 = (BCTag)thread->readIntReg(mp.rs1);
                thread->setIntReg((int)mp.rd, (uint64_t)trs1);
                revalidateRegTag((int)mp.rd);
            } else {
                // mov immediate
                thread->setIntReg((int)mp.rd, 0);
                revalidateRegTag((int)mp.rd);
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
                revalidateRegTag((int)mp.rd);
            } else if (TheISA::isISAReg(mp.rs1)) {
                // register + imm operands
                BCTag trs1 = (BCTag)thread->readIntReg(mp.rs1);
                thread->setIntReg((int)mp.rd, (uint64_t)trs1);
                revalidateRegTag((int)mp.rd);
            } else {
                // we should never reach here
                // panic("Incorrect number of ALU operands!\n");
            }
        } else {
            // other ALU operations
            thread->setIntReg((int)mp.rd, 0);
            revalidateRegTag((int)mp.rd);
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
            revalidateRegTag((int)mp.rd);
            if (toPtrTag(tmem) != 0) {
                DPRINTF(Monitor, "BC: write pointer tag %d to r%d\n", toMemTag(tmem), (int)mp.rd);
            }
        } else {
            // should not reach here
            // if we reach here for some reason, conservatively clear dest reg tag
            thread->setIntReg((int)mp.rd, 0);
            revalidateRegTag((int)mp.rd);
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
            revalidateMemTag(mp.memAddr);
        } else {
            // should not reach here
            // if we reach here for some reason, conservatively clear pointer tag
            BCTag tmem = readTagFunctional(mp.memAddr);
            writeTag(mp.memAddr, mergeMemPtrTags(tmem, 0));
            revalidateMemTag(mp.memAddr);
        }

        numStoreInsts++;
        numMonitorInsts++;
    } else if (mp.store && mp.settag) {
        DPRINTF(Monitor, "BC: Set tag mem[0x%x:0x%x]=%d\n", mp.memAddr, mp.memEnd, mp.data);
        for (Addr pbyte = mp.memAddr; pbyte < mp.memEnd; pbyte++) {
            writeTag(pbyte, (Tag)mp.data);
        }
        for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
            revalidateMemTag(pbyte);
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
TimingSimpleMonitor::SECExecute()
{

}

/**
 * HardBound Execution
 */
void
TimingSimpleMonitor::HBExecute()
{
    if (mp.intalu) {
        // integer ALU operation
        DPRINTF(Monitor, "HardBound: Integer ALU instruction\n");
        ALUOpCode opcode = decodeALUOpcode(mp.opcode);
        if (opcode == ALUMov) {
            if (TheISA::isISAReg(mp.rs1)) {
                HBTag trs1 = (HBTag)thread->readIntReg(mp.rs1);
                thread->setIntReg((int)mp.rd, (uint64_t)trs1);
                revalidateRegTag((int)mp.rd);
            } else {
                // mov immediate
                thread->setIntReg((int)mp.rd, 0);
                revalidateRegTag((int)mp.rd);
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
                revalidateRegTag((int)mp.rd);
            } else if (TheISA::isISAReg(mp.rs1)) {
                HBTag trs1 = (HBTag)thread->readIntReg(mp.rs1);
                thread->setIntReg((int)mp.rd, (uint64_t)trs1);
                revalidateRegTag((int)mp.rd);
            } else {
                // we should never reach here
                // panic("Incorrect number of ALU operands!\n");
            }
        } else {
            // other ALU operations
            thread->setIntReg((int)mp.rd, 0);
            revalidateRegTag((int)mp.rd);
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
                revalidateRegTag((int)mp.rd);
            }
        } else {
            // should not reach here
            // if we reach here for some reason, conservatively clear dest reg tag
            thread->setIntReg((int)mp.rd, 0);
            revalidateRegTag((int)mp.rd);
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
            for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
                revalidateMemTag(pbyte);
            }
        } else {
            // should not reach here
            // if we reach here for some reason, conservatively clear pointer tag
            if (mp.size == 4)
                writeDWordTag(mp.physAddr, 0);
            for (Addr pbyte = mp.memAddr; pbyte <= mp.memEnd; pbyte += 4) {
                revalidateMemTag(pbyte);
            }
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
            revalidateMemTag(mp.memAddr);
        }
    } else {
        DPRINTF(Monitor, "Unknown instruction\n");
        numMonitorInsts++;
    }
}

/**
 * Finish monitoring
 */
void
TimingSimpleMonitor::finishMonitoring()
{
    // main core has finished, exit current monitoring thread context
    tc->halt(0);
}


void
TimingSimpleMonitor::sendFetch(Fault fault, RequestPtr req, ThreadContext *tc)
{
    if (fault == NoFault) {
        DPRINTF(SimpleCPU, "Sending fetch for addr %#x(pa: %#x)\n",
                req->getVaddr(), req->getPaddr());
        ifetch_pkt = new Packet(req, MemCmd::ReadReq);
        ifetch_pkt->dataStatic(&inst);
        DPRINTF(SimpleCPU, " -- pkt addr: %#x\n", ifetch_pkt->getAddr());

        if (!icachePort.sendTimingReq(ifetch_pkt)) {
            // Need to wait for retry
            _status = IcacheRetry;
        } else {
            // Need to wait for cache to respond
            _status = IcacheWaitResponse;
            // ownership of packet transferred to memory system
            ifetch_pkt = NULL;
        }
    } else {
        DPRINTF(SimpleCPU, "Translation of addr %#x faulted\n", req->getVaddr());
        delete req;
        // fetch fault: advance directly to next instruction (fault handler)
        _status = Running;
        advanceInst(fault);
    }

    numCycles += tickToCycles(curTick() - previousTick);
    previousTick = curTick();
}


void
TimingSimpleMonitor::advanceInst(Fault fault)
{
    if (_status == Faulting)
        return;

    if (fault != NoFault) {
        advancePC(fault);
        DPRINTF(SimpleCPU, "Fault occured, scheduling fetch event\n");
        reschedule(fetchEvent, nextCycle(), true);
        _status = Faulting;
        return;
    }

    if (!stayAtPC)
        advancePC(fault);

    if (_status == Running) {
        // kick off fetch of next instruction... callback from icache
        // response will cause that instruction to be executed,
        // keeping the CPU running.
        fetch();
    }
}


void
TimingSimpleMonitor::completeIfetch(PacketPtr pkt)
{
  // We should never do ifetch for monitor
  assert(false);
    DPRINTF(SimpleCPU, "Complete ICache Fetch for addr %#x\n", pkt ?
            pkt->getAddr() : 0);

    // received a response from the icache: execute the received
    // instruction

    assert(!pkt || !pkt->isError());
    assert(_status == IcacheWaitResponse);

    _status = Running;

    numCycles += tickToCycles(curTick() - previousTick);
    previousTick = curTick();

    if (getState() == SimObject::Draining) {
        if (pkt) {
            delete pkt->req;
            delete pkt;
        }

        completeDrain();
        return;
    }

    preExecute();

    // set this so that we can check later if instruction was predicated
    setPredicate(true);

    if (curStaticInst && curStaticInst->isMemRef()) {
        // load or store: just send to dcache
        Fault fault = curStaticInst->initiateAcc(this, traceData);

        // If we're not running now the instruction will complete in a dcache
        // response callback or the instruction faulted and has started an
        // ifetch
        if (_status == Running) {
            if (fault != NoFault && traceData) {
                // If there was a fault, we shouldn't trace this instruction.
                delete traceData;
                traceData = NULL;
            }

            postExecute();

            // @todo remove me after debugging with legion done
            if (curStaticInst && (!curStaticInst->isMicroop() ||
                        curStaticInst->isFirstMicroop()))
                instCnt++;
            advanceInst(fault);
        }
    } else if (curStaticInst) {
        // non-memory instruction: execute completely now
        Fault fault = curStaticInst->execute(this, traceData);

        // keep an instruction count
        if (fault == NoFault)
            countInst();
        else if (traceData && !DTRACE(ExecFaulting)) {
            delete traceData;
            traceData = NULL;
        }

        postExecute();
        // @todo remove me after debugging with legion done
        if (curStaticInst && (!curStaticInst->isMicroop() ||
                    curStaticInst->isFirstMicroop()))
            instCnt++;
        advanceInst(fault);
    } else {
        advanceInst(NoFault);
    }

    if (pkt) {
        delete pkt->req;
        delete pkt;
    }
}

// function to handle end_task command
void TimingSimpleMonitor::endTask() {
  timerStalled = true;
  DPRINTF(SlackTimer, "The CPU will be stalled for %d ticks\n", fed.data);

  // Set state of CPU to stall
  _status = FifoStall;
  // Do not advance PC yet
  stayAtPC = true;
  // Schedule stall event
  schedule(endTaskEvent, curTick() + fed.data);
}

void 
TimingSimpleMonitor::stallFromFifo() {
  // Set state of CPU to stall
  _status = FifoStall;
  // Do not advance PC yet
  stayAtPC = true;
  // Schedule packet to be resent
  schedule(fifoEvent, curTick() + ticks(1));
  
  DPRINTF(Fifo, "Rescheduling...\n");
#ifdef DEBUG
  // Count how many packets cause a stall
  num_packets++;
#endif
  // Store start of stall time
  fifoStallTicks = curTick(); 
}

// Retry sending fifo packet
void TimingSimpleMonitor::handleFifoEvent() {

  // Try again
  if (sendFifoPacket()) {
    // Successful

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

    // unstall CPU, handle next instruction
    _status = Running;
    stayAtPC = false;
    advanceInst(NoFault);
    
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

void TimingSimpleMonitor::handleEndTaskEvent() {
  DPRINTF(SlackTimer, "Resuming execution\n");
  // Unstall CPU, handle next instruction
  _status = Running;
  stayAtPC = false;
  advanceInst(NoFault);
}


void
TimingSimpleMonitor::IcachePort::ITickEvent::process()
{
    cpu->completeIfetch(pkt);
}

bool
TimingSimpleMonitor::IcachePort::recvTimingResp(PacketPtr pkt)
{
    if (!pkt->wasNacked()) {
        DPRINTF(SimpleCPU, "Received timing response %#x\n", pkt->getAddr());
        // delay processing of returned data until next CPU clock edge
        Tick next_tick = cpu->nextCycle(curTick());

        if (next_tick == curTick())
            cpu->completeIfetch(pkt);
        else
            tickEvent.schedule(pkt, next_tick);

        return true;
    } else {
        assert(cpu->_status == IcacheWaitResponse);
        pkt->reinitNacked();
        if (!sendTimingReq(pkt)) {
            cpu->_status = IcacheRetry;
            cpu->ifetch_pkt = pkt;
        }
    }

    return true;
}

void
TimingSimpleMonitor::IcachePort::recvRetry()
{
    // we shouldn't get a retry unless we have a packet that we're
    // waiting to transmit
    assert(cpu->ifetch_pkt != NULL);
    assert(cpu->_status == IcacheRetry);
    PacketPtr tmp = cpu->ifetch_pkt;
    if (sendTimingReq(tmp)) {
        cpu->_status = IcacheWaitResponse;
        cpu->ifetch_pkt = NULL;
    }
}

void
TimingSimpleMonitor::completeDataAccess(PacketPtr pkt)
{
    // received a response from the dcache: complete the load or store
    // instruction
    assert(!pkt->isError());
    assert(_status == DcacheWaitResponse || _status == DTBWaitResponse ||
           pkt->req->getFlags().isSet(Request::NO_ACCESS));

    numCycles += tickToCycles(curTick() - previousTick);
    previousTick = curTick();

    // Save data for monitoring
    pkt->writeData((uint8_t*)&fed.data);

    if (pkt->senderState) {
        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(pkt->senderState);
        assert(send_state);
        delete pkt->req;
        delete pkt;
        PacketPtr big_pkt = send_state->bigPkt;
        delete send_state;
        
        SplitMainSenderState * main_send_state =
            dynamic_cast<SplitMainSenderState *>(big_pkt->senderState);
        assert(main_send_state);
        // Record the fact that this packet is no longer outstanding.
        assert(main_send_state->outstanding != 0);
        main_send_state->outstanding--;

        if (main_send_state->outstanding) {
            return;
        } else {
            delete main_send_state;
            big_pkt->senderState = NULL;
            pkt = big_pkt;
        }
    }

    _status = Running;

    // No curStaticInst for monitor, skip
    //Fault fault = curStaticInst->completeAcc(pkt, this, traceData);
    Fault fault = NoFault;

    // No curStaticInst for countInst in Monitor
    /*
    // keep an instruction count
    if (fault == NoFault)
        countInst();
    else if (traceData) {
        // If there was a fault, we shouldn't trace this instruction.
        delete traceData;
        traceData = NULL;
    }
    */

    // the locked flag may be cleared on the response packet, so check
    // pkt->req and not pkt to see if it was a load-locked
    if (pkt->isRead() && pkt->req->isLLSC()) {
        TheISA::handleLockedRead(thread, pkt->req);
    }

    delete pkt->req;
    delete pkt;

    // Skip post Execute for monitor
    postExecute();

    if (getState() == SimObject::Draining) {
        advancePC(fault);
        completeDrain();

        return;
    }

    advanceInst(fault);
}


void
TimingSimpleMonitor::completeDrain()
{
    DPRINTF(Config, "Done draining\n");
    changeState(SimObject::Drained);
    drainEvent->process();
}

bool
TimingSimpleMonitor::DcachePort::recvTimingResp(PacketPtr pkt)
{
    if (!pkt->wasNacked()) {
        // delay processing of returned data until next CPU clock edge
        Tick next_tick = cpu->nextCycle(curTick());

        if (next_tick == curTick()) {
            cpu->completeDataAccess(pkt);
        } else {
            if (!tickEvent.scheduled()) {
                tickEvent.schedule(pkt, next_tick);
            } else {
                // In the case of a split transaction and a cache that is
                // faster than a CPU we could get two responses before
                // next_tick expires
                if (!retryEvent.scheduled()) {
                    cpu->schedule(retryEvent, next_tick);
                }
                return false;
            }
        }

        return true;
    } else  {
        assert(cpu->_status == DcacheWaitResponse);
        pkt->reinitNacked();
        if (!sendTimingReq(pkt)) {
            cpu->_status = DcacheRetry;
            cpu->dcache_pkt = pkt;
        }
    }

    return true;
}

void
TimingSimpleMonitor::DcachePort::DTickEvent::process()
{
    cpu->completeDataAccess(pkt);
}

void
TimingSimpleMonitor::DcachePort::recvRetry()
{
    // we shouldn't get a retry unless we have a packet that we're
    // waiting to transmit
    assert(cpu->dcache_pkt != NULL);
    assert(cpu->_status == DcacheRetry);
    PacketPtr tmp = cpu->dcache_pkt;
    if (tmp->senderState) {
        // This is a packet from a split access.
        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(tmp->senderState);
        assert(send_state);
        PacketPtr big_pkt = send_state->bigPkt;
        
        SplitMainSenderState * main_send_state =
            dynamic_cast<SplitMainSenderState *>(big_pkt->senderState);
        assert(main_send_state);

        if (sendTimingReq(tmp)) {
            // If we were able to send without retrying, record that fact
            // and try sending the other fragment.
            send_state->clearFromParent();
            int other_index = main_send_state->getPendingFragment();
            if (other_index > 0) {
                tmp = main_send_state->fragments[other_index];
                cpu->dcache_pkt = tmp;
                if ((big_pkt->isRead() && cpu->handleReadPacket(tmp)) ||
                        (big_pkt->isWrite() && cpu->handleWritePacket())) {
                    main_send_state->fragments[other_index] = NULL;
                }
            } else {
                cpu->_status = DcacheWaitResponse;
                // memory system takes ownership of packet
                cpu->dcache_pkt = NULL;
            }
        }
    } else if (sendTimingReq(tmp)) {
        cpu->_status = DcacheWaitResponse;
        // memory system takes ownership of packet
        cpu->dcache_pkt = NULL;
    }
}

TimingSimpleMonitor::IprEvent::IprEvent(Packet *_pkt, TimingSimpleMonitor *_cpu,
    Tick t)
    : pkt(_pkt), cpu(_cpu)
{
    cpu->schedule(this, t);
}

void
TimingSimpleMonitor::IprEvent::process()
{
    cpu->completeDataAccess(pkt);
}

const char *
TimingSimpleMonitor::IprEvent::description() const
{
    return "Timing Simple CPU Delay IPR event";
}


void
TimingSimpleMonitor::printAddr(Addr a)
{
    dcachePort.printAddr(a);
}

MasterPort &
TimingSimpleMonitor::getMasterPort(const std::string &if_name, int idx)
{
  if (if_name == "monitor_port") {
    return monitorPort;
  } else {
    return BaseSimpleCPU::getMasterPort(if_name, idx);
  }
}

/*
 * Read a tag from memory/cache
 */
TimingSimpleMonitor::Tag
TimingSimpleMonitor::readTag(Addr addr) {
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

TimingSimpleMonitor::Tag
TimingSimpleMonitor::readTagFunctional(Addr addr) {
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
TimingSimpleMonitor::readBitTag(Addr addr) {
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
TimingSimpleMonitor::readBitTagFunctional(Addr addr) {
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
TimingSimpleMonitor::readDWordTag(Addr addr)
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
TimingSimpleMonitor::readDWordTagFunctional(Addr addr)
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
TimingSimpleMonitor::writeTag(Addr addr, Tag tag) {
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
TimingSimpleMonitor::writeTagFunctional(Addr addr, Tag tag) {
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
TimingSimpleMonitor::writeBitTag(Addr addr, bool tag)
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
TimingSimpleMonitor::writeBitTagFunctional(Addr addr, bool tag)
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
TimingSimpleMonitor::writeDWordTag(Addr addr, uint64_t tag)
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
TimingSimpleMonitor::writeDWordTagFunctional(Addr addr, uint64_t tag)
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
TimingSimpleMonitor::handlePageTableFault(Addr addr)
{
    Process *p = tc->getProcessPtr();
    Addr page_base_addr = roundDown(addr, VMPageSize);
    p->allocateMonMem(page_base_addr, VMPageSize);
    //p->allocateMem(page_base_addr, VMPageSize);
    // clear page
    SETranslatingPortProxy &tp = tc->getMemProxy();
    tp.memsetBlob(page_base_addr, 0, VMPageSize);
}

void
TimingSimpleMonitor::revalidateRegTag(int idx)
{
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

void
TimingSimpleMonitor::revalidateMemTag(Addr addr)
{
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


TimingSimpleMonitor::ALUOpCode
TimingSimpleMonitor::decodeALUOpcode(uint8_t opcode)
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

void
TimingSimpleMonitor::resetStats()
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

void
TimingSimpleMonitor::regStats()
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
    
    numTaintedInsts = numTaintedIntegerInsts + numTaintedLoadInsts + numTaintedStoreInsts + numTaintedIndirectCtrlInsts;
    numBCErrors = numBCLoadErrors + numBCStoreErrors;
}

void
TimingSimpleMonitor::postExecute()
{
}

////////////////////////////////////////////////////////////////////////
//
//  TimingSimpleMonitor Simulation Object
//
TimingSimpleMonitor *
TimingSimpleMonitorParams::create()
{
    numThreads = 1;
    if (!FullSystem && workload.size() != 1)
        panic("only one workload allowed");
    return new TimingSimpleMonitor(this);
}
