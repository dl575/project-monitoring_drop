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
#include "cpu/simple/drop_timing.hh"
#include "cpu/exetrace.hh"
#include "debug/Config.hh"
#include "debug/ExecFaulting.hh"
#include "debug/SimpleCPU.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/DropTimingSimpleCPU.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

#include "mem/fifo.hh"
#include "mem/timer.hh"
#include "mem/flag_cache.hh"

using namespace std;
using namespace TheISA;

void
DropTimingSimpleCPU::init()
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

}

void
DropTimingSimpleCPU::TimingCPUPort::TickEvent::schedule(PacketPtr _pkt, Tick t)
{
    pkt = _pkt;
    cpu->schedule(this, t);
}

DropTimingSimpleCPU::DropTimingSimpleCPU(DropTimingSimpleCPUParams *p)
    : BaseSimpleCPU(p), fetchTranslation(this), icachePort(this),
    dcachePort(this), fetchEvent(this),
    fifoEvent(this), endTaskEvent(this),
    forward_fifo_enabled(p->forward_fifo_enabled),
    forwardFifoPort(name() + "-iport", this), 
    full_ticks(p->full_clock), tickEvent(this),
    monitorPort(name() + "-iport", this)
{
    _status = Idle;

    ifetch_pkt = dcache_pkt = NULL;
    drainEvent = NULL;
    previousTick = 0;
    changeState(SimObject::Running);
    system->totalNumInsts = 0;
}


DropTimingSimpleCPU::~DropTimingSimpleCPU()
{
}

void
DropTimingSimpleCPU::serialize(ostream &os)
{
    SimObject::State so_state = SimObject::getState();
    SERIALIZE_ENUM(so_state);
    BaseSimpleCPU::serialize(os);
}

void
DropTimingSimpleCPU::unserialize(Checkpoint *cp, const string &section)
{
    SimObject::State so_state;
    UNSERIALIZE_ENUM(so_state);
    BaseSimpleCPU::unserialize(cp, section);
}

unsigned int
DropTimingSimpleCPU::drain(Event *drain_event)
{
    // DropTimingSimpleCPU is ready to drain if it's not waiting for
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
DropTimingSimpleCPU::resume()
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
DropTimingSimpleCPU::switchOut()
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
DropTimingSimpleCPU::takeOverFrom(BaseCPU *oldCPU)
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
DropTimingSimpleCPU::activateContext(ThreadID thread_num, int delay)
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
DropTimingSimpleCPU::suspendContext(ThreadID thread_num)
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
DropTimingSimpleCPU::handleReadPacket(PacketPtr pkt)
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
DropTimingSimpleCPU::sendData(RequestPtr req, uint8_t *data, uint64_t *res,
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
DropTimingSimpleCPU::sendSplitData(RequestPtr req1, RequestPtr req2,
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
DropTimingSimpleCPU::translationFault(Fault fault)
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
DropTimingSimpleCPU::buildPacket(PacketPtr &pkt, RequestPtr req, bool read)
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
DropTimingSimpleCPU::buildSplitPacket(PacketPtr &pkt1, PacketPtr &pkt2,
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
DropTimingSimpleCPU::readMem(Addr addr, uint8_t *data,
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
        RequestPtr req1, req2;
        assert(!req->isLLSC() && !req->isSwap());
        req->splitOnVaddr(split_addr, req1, req2);

        WholeTranslationState *state =
            new WholeTranslationState(req, req1, req2, new uint8_t[size],
                                      NULL, mode);
        DataTranslation<DropTimingSimpleCPU *> *trans1 =
            new DataTranslation<DropTimingSimpleCPU *>(this, state, 0);
        DataTranslation<DropTimingSimpleCPU *> *trans2 =
            new DataTranslation<DropTimingSimpleCPU *>(this, state, 1);

        thread->dtb->translateTiming(req1, tc, trans1, mode);
        thread->dtb->translateTiming(req2, tc, trans2, mode);
    } else {
        WholeTranslationState *state =
            new WholeTranslationState(req, new uint8_t[size], NULL, mode);
        DataTranslation<DropTimingSimpleCPU *> *translation
            = new DataTranslation<DropTimingSimpleCPU *>(this, state);
        thread->dtb->translateTiming(req, tc, translation, mode);
    }

    return NoFault;
}


Fault
DropTimingSimpleCPU::readMemFunctional(Addr addr, uint8_t * data,
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
DropTimingSimpleCPU::handleWritePacket()
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
DropTimingSimpleCPU::writeMem(uint8_t *data, unsigned size,
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
        RequestPtr req1, req2;
        assert(!req->isLLSC() && !req->isSwap());
        req->splitOnVaddr(split_addr, req1, req2);

        WholeTranslationState *state =
            new WholeTranslationState(req, req1, req2, newData, res, mode);
        DataTranslation<DropTimingSimpleCPU *> *trans1 =
            new DataTranslation<DropTimingSimpleCPU *>(this, state, 0);
        DataTranslation<DropTimingSimpleCPU *> *trans2 =
            new DataTranslation<DropTimingSimpleCPU *>(this, state, 1);

        thread->dtb->translateTiming(req1, tc, trans1, mode);
        thread->dtb->translateTiming(req2, tc, trans2, mode);
    } else {
        WholeTranslationState *state =
            new WholeTranslationState(req, newData, res, mode);
        DataTranslation<DropTimingSimpleCPU *> *translation =
            new DataTranslation<DropTimingSimpleCPU *>(this, state);
        thread->dtb->translateTiming(req, tc, translation, mode);
    }

    // Translation faults will be returned via finishTranslation()
    return NoFault;
}


Fault
DropTimingSimpleCPU::writeMemFunctional(uint8_t *data, unsigned size,
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
DropTimingSimpleCPU::finishTranslation(WholeTranslationState *state)
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
DropTimingSimpleCPU::fetch()
{
    DPRINTF(SimpleCPU, "Drop Tick\n");

    numCycles++;

    bool forward_successful = true;
    
    if (mp.valid){
        if (forward_fifo_enabled) {
            forward_successful = forwardFifoPacket();
        }
        if (forward_successful){
            mp.init();
        }
    }
    
    if (forward_successful){
        bool drop = false;
        Fault fault = ReExecFault;
        if (timer_enabled){
            fault = readFromTimer(TIMER_READ_DROP, (uint8_t *)&drop, sizeof(drop), ArmISA::TLB::AllowUnaligned);
        } else if (fifo_enabled){
            bool pop = true;
            fault = writeToFifo(FIFO_NEXT, (uint8_t *)&pop, sizeof(pop), ArmISA::TLB::AllowUnaligned);
        }
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
    
    if (_status != Idle) {
      // If there was already one scheduled, kill that one
      if (fetchEvent.scheduled()) {
        deschedule(fetchEvent);
      }
      schedule(fetchEvent, roundDown(curTick() + ticks(1) + 1, 2) - 1);
    }

}


void
DropTimingSimpleCPU::sendFetch(Fault fault, RequestPtr req, ThreadContext *tc)
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
DropTimingSimpleCPU::advanceInst(Fault fault)
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
DropTimingSimpleCPU::completeIfetch(PacketPtr pkt)
{
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

void 
DropTimingSimpleCPU::stallFromFifo() {
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
void DropTimingSimpleCPU::handleFifoEvent() {

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

void DropTimingSimpleCPU::handleEndTaskEvent() {
  DPRINTF(SlackTimer, "Resuming execution\n");
  // Unstall CPU, handle next instruction
  _status = Running;
  stayAtPC = false;
  advanceInst(NoFault);
}


void
DropTimingSimpleCPU::IcachePort::ITickEvent::process()
{
    cpu->completeIfetch(pkt);
}

bool
DropTimingSimpleCPU::IcachePort::recvTimingResp(PacketPtr pkt)
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
DropTimingSimpleCPU::IcachePort::recvRetry()
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
DropTimingSimpleCPU::completeDataAccess(PacketPtr pkt)
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

    // No curStaticInst for dropping hardware, skip
    //Fault fault = curStaticInst->completeAcc(pkt, this, traceData);
    Fault fault = NoFault;

    // No curStaticInst for countInst in dropping hardware
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

    postExecute();

    if (getState() == SimObject::Draining) {
        advancePC(fault);
        completeDrain();

        return;
    }

    advanceInst(fault);
}


void
DropTimingSimpleCPU::completeDrain()
{
    DPRINTF(Config, "Done draining\n");
    changeState(SimObject::Drained);
    drainEvent->process();
}


void
DropTimingSimpleCPU::pageAllocate(Addr addr)
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
DropTimingSimpleCPU::getCacheFlags(size_t size)
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


// Send monitoring packet to monitoring core
bool 
DropTimingSimpleCPU::forwardFifoPacket() {
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
DropTimingSimpleCPU::regStats()
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

MasterPort &
DropTimingSimpleCPU::getMasterPort(const std::string &if_name, int idx)
{
  if (if_name == "forward_fifo_port") {
    return forwardFifoPort;
  } else {
    return BaseSimpleCPU::getMasterPort(if_name, idx);
  }
}

SlavePort &
DropTimingSimpleCPU::getSlavePort(const std::string &if_name, int idx)
{
  if (if_name == "monitor_port") {
    return monitorPort;
  } else {
    return BaseCPU::getSlavePort(if_name, idx);
  }
}

bool
DropTimingSimpleCPU::DcachePort::recvTimingResp(PacketPtr pkt)
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
                if (!retryEvent.scheduled())
                    cpu->schedule(retryEvent, next_tick);
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
DropTimingSimpleCPU::DcachePort::DTickEvent::process()
{
    cpu->completeDataAccess(pkt);
}

void
DropTimingSimpleCPU::DcachePort::recvRetry()
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

DropTimingSimpleCPU::IprEvent::IprEvent(Packet *_pkt, DropTimingSimpleCPU *_cpu,
    Tick t)
    : pkt(_pkt), cpu(_cpu)
{
    cpu->schedule(this, t);
}

void
DropTimingSimpleCPU::IprEvent::process()
{
    cpu->completeDataAccess(pkt);
}

const char *
DropTimingSimpleCPU::IprEvent::description() const
{
    return "Timing Simple CPU Delay IPR event";
}


void
DropTimingSimpleCPU::printAddr(Addr a)
{
    dcachePort.printAddr(a);
}

DropTimingSimpleCPU::MonitorPort::MonitorPort(const std::string& _name,
                                        DropTimingSimpleCPU *_cpu)
    : SimpleTimingPort(_name, _cpu), cpu(_cpu)
{ }

AddrRangeList
DropTimingSimpleCPU::MonitorPort::getAddrRanges()
{
    AddrRangeList ranges;
    return ranges;
}

Tick
DropTimingSimpleCPU::MonitorPort::recvAtomic(PacketPtr pkt)
{
    recvFunctional(pkt);
    // FIXME: zero latency
    return 0;
}

void
DropTimingSimpleCPU::MonitorPort::recvFunctional(PacketPtr pkt)
{
    if (!queue.checkFunctional(pkt)) {
        if (pkt->cmd == MemCmd::WriteReq) {
            uint8_t type;
            switch (pkt->getAddr()){
                case DROP_CLEAR_ARRAY: type = 0; break;
                case DROP_CLEAR_CACHE: type = 1; break;
                case DROP_FC_SET_ADDR: type = 2; break;
                default: panic ("Unimplemented port request");
            }
            if (cpu->flagcache_enabled){
                cpu->writeToFlagCache(FC_SET_ADDR, pkt->getPtr<uint8_t>(), pkt->getSize(), ArmISA::TLB::AllowUnaligned);
                if (type == 0 || type == 1) {
                  cpu->writeToFlagCache(FC_CLEAR_FLAG, &type, sizeof(type), ArmISA::TLB::AllowUnaligned);
                }
            }
        } else {
            panic ("Unimplemented port request");
        }
    }
}

bool
DropTimingSimpleCPU::MonitorPort::recvTimingReq(PacketPtr pkt)
{
    // Use functional function to handle writing
    recvFunctional(pkt);
    // Indicate write accepted
    return true;
}


////////////////////////////////////////////////////////////////////////
//
//  DropTimingSimpleCPU Simulation Object
//
DropTimingSimpleCPU *
DropTimingSimpleCPUParams::create()
{
    numThreads = 1;
    if (!FullSystem && workload.size() != 1)
        panic("only one workload allowed");
    return new DropTimingSimpleCPU(this);
}
