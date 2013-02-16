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
#include "cpu/exetrace.hh"
#include "debug/ExecFaulting.hh"
#include "debug/SimpleCPU.hh"
#include "debug/Fifo.hh"
#include "debug/FifoStall.hh"
#include "debug/SlackTimer.hh"
#include "debug/Task.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/physical.hh"
#include "params/AtomicSimpleCPU.hh"
#include "sim/faults.hh"
#include "sim/system.hh"
#include "sim/full_system.hh"
#include "sim/sim_exit.hh"

#include "mem/fifo.hh"
#include "mem/timer.hh"

using namespace std;
using namespace TheISA;

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
    BaseCPU::init();

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

    // Initialize monitoring variables
    mp.init();
    fed.clear();
    fifoStall = false;
    timerStalled = false;
    fifoEmpty = false;
    drops = 0;
    not_drops = 0;
}

AtomicSimpleCPU::AtomicSimpleCPU(AtomicSimpleCPUParams *p)
    : BaseSimpleCPU(p), tickEvent(this), width(p->width), locked(false),
      simulate_data_stalls(p->simulate_data_stalls),
      simulate_inst_stalls(p->simulate_inst_stalls),
      icachePort(name() + "-iport", this), dcachePort(name() + "-iport", this),
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

    // use the CPU's statically allocated read request and packet objects
    Request *req = &data_read_req;

    if (traceData) {
        traceData->setAddr(addr);
    }
    // Save address for monitoring
    fed.memAddr = addr;

    // Read from fifo
    if (fifo_enabled && addr >= FIFO_ADDR_START && addr <= FIFO_ADDR_END) {
        // Create request at fifo location
        req->setPhys(addr, size, flags, dataMasterId());
        // Read command
        MemCmd cmd = MemCmd::ReadReq;
        // Create packet
        PacketPtr pkt = new Packet(req, cmd);
        // Point packet to monitoring packet
        pkt->dataStatic(data);

        // Send read request
        fifoPort.sendFunctional(pkt);

        // Clean up
        delete pkt;
        
        bool wasEmpty = fifoEmpty;
        
        // Checking if reading from empty fifo
        if (addr < FIFO_REG_START) {
            fifoEmpty = isFifoEmpty();
        #ifdef DEBUG
            if (wasEmpty ^ fifoEmpty){
                // Print out for debug
                DPRINTF(Fifo, "Check if Fifo empty: %d\n", fifoEmpty);
            } 
        #endif
        }
        
    #ifdef DEBUG
        if (addr < FIFO_EMPTY && !(wasEmpty && fifoEmpty)){
            unsigned read_data;
            unsigned read_size = size;
            if (sizeof(unsigned) < read_size){ read_size = sizeof(unsigned); }//Make sure we don't copy garbage data
            memcpy(&read_data, data, size);
            DPRINTF(Fifo, "read fifo @ %x = %x\n", addr, read_data);
        }
    #endif

        return (fifoEmpty)? (new ReExec()) : NoFault;   
    }

    // Read from timer
    if (timer_enabled && addr >= TIMER_ADDR_START && addr <= TIMER_ADDR_END) {
        // read data
        int read_timer = 0;
        // Create request at timer location
        req->setPhys(addr, sizeof(int), flags, dataMasterId());
        // Read command
        MemCmd cmd = MemCmd::ReadReq;
        // Create packet
        PacketPtr pkt = new Packet(req, cmd);
        // Point packet to data pointer
        pkt->dataStatic(&read_timer);

        // Send read request
        timerPort.sendFunctional(pkt);
        
        if (addr == TIMER_READ_SLACK) {
            // Print out for debug
        #ifdef DEBUG
            DPRINTF(SlackTimer, "read from timer: %d ticks, %d cycles\n", read_timer, read_timer/ticks(1));
        #endif
            read_timer /= ticks(1);
            memcpy(data, &read_timer, size);

            // Clean up
            delete pkt;

            return NoFault;
        } else if (addr == TIMER_READ_DROP) {
            // Update hardware counters
            if (read_timer == 1) { not_drops++; }
            else { drops++; }
            // Print out for debug
        #ifdef DEBUG
            DPRINTF(SlackTimer, "read from timer: drop? %d, numdrops: %d, numfull: %d\n", !read_timer, drops, not_drops);
        #endif
            memcpy(data, &read_timer, size);

            // Clean up
            delete pkt;
            
            return NoFault;
        }
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
    // Save data being written
    fed.data = 0;
    memcpy(&fed.data, data, size);
    
    dcache_latency = 0;

    // Write to fifo
    // Used to handle fifo control (writing data to fifo is done 
    // automatically by monitoring)
    if (fifo_enabled && (addr >= FIFO_OP_RANGE_START && addr < FIFO_OP_RANGE_END)) {
      if (addr == FIFO_ADDR){
        int fifo_ctrl = (int)*data;
        DPRINTF(Fifo, "Write to fifo control: %d\n", fifo_ctrl);
      
        if (fifo_ctrl == 1) {
            // Enable monitoring
            monitoring_enabled = true;
            DPRINTF(Fifo, "Enabling monitoring\n");
        } else if (fifo_ctrl == 0) {
            // Disable monitoring
            monitoring_enabled = false;
            DPRINTF(Fifo, "Disabling monitoring\n");
        } else if (fifo_ctrl == 2) {
            // Finished main core, set packet to indicate this
            mp.done = true; 
            DPRINTF(Fifo, "Main core done \n");
        } else {
            warn("Unrecognized fifo control: %d\n", fifo_ctrl);
        }
      } else if (addr == FIFO_START_CUSTOM){
        mp.valid = true;
        mp.memAddr = fed.data;
        DPRINTF(Fifo, "Starting custom packet\n");
      } else if (addr == FIFO_NEXT){
        Request* req = &data_write_req;
        unsigned flags = ArmISA::TLB::AllowUnaligned;
        //size = sizeof(read_tp);
        req->setPhys(addr, sizeof(fed.data), flags, dataMasterId());
        // Read command
        MemCmd cmd = MemCmd::WriteReq;
        // Create packet
        PacketPtr pkt = new Packet(req, cmd);
        // Point packet to data pointer
        pkt->dataStatic(&fed.data);

        // Send read request
        fifoPort.sendFunctional(pkt);

        // Clean up
        delete pkt;
        
      }
      
      return NoFault;
    }
    // Timer
    if (timer_enabled) { 
      if (addr >= TIMER_ADDR_START && addr <= TIMER_ADDR_END) {

        //if end_task we get the timer slack and will add it to the
        //additional slack we are storing into the timer.
        
        if (addr == TIMER_END_TASK){
            int slack;
            Request* req = &data_read_req;
            unsigned flags = ArmISA::TLB::AllowUnaligned;
            //size = sizeof(read_tp);
            req->setPhys(TIMER_READ_SLACK, sizeof(int), flags, dataMasterId());
            // Read command
            MemCmd cmd = MemCmd::ReadReq;
            // Create packet
            PacketPtr pkt = new Packet(req, cmd);
            // Point packet to data pointer
            pkt->dataStatic(&slack);

            // Send read request
            timerPort.sendFunctional(pkt);
            
            // Clean up
            delete pkt;
            
            int stall_length = (int)fed.data*ticks(1) + slack;
            if (stall_length < 0){
                panic("Did not meet WCET. Slack is negative: %d.\n", stall_length/ticks(1));
            }
            fed.data = stall_length;
        }
        // Get data
        int write_data = 0;
        if (size > sizeof(int)) size = sizeof(int);
        memcpy(&write_data, data, size);
        //convert from cycles to ticks
        write_data *= ticks(1);
        // Create request
        Request *timer_write_req = &data_write_req;
        //unsigned size = sizeof(write_tp);
        unsigned flags = ArmISA::TLB::AllowUnaligned;
        // set physical address
        timer_write_req->setPhys(addr, sizeof(int), flags, dataMasterId());
        // Create write packet
        MemCmd cmd = MemCmd::WriteReq;
        PacketPtr timerpkt = new Packet(timer_write_req, cmd);
        // Set data
        // write_tp.subtaskStart = curTick();
        timerpkt->dataStatic(&write_data);

        // Send read request packet on timer port
        timerPort.sendFunctional(timerpkt);

#ifdef DEBUG
        // Print out data for debugging
        DPRINTF(SlackTimer, "Write to timer [%x]: %d\n", addr, write_data);
        // Print messages with start and end task so we can find WCET
        if (addr == TIMER_START_TASK) {
          start_task = curTick();
          num_packets = 0;
          task_addr = tc->instAddr();
        } else if (addr == TIMER_END_TASK) {
          DPRINTF(Task, "Task @ %x ET = %d, Packets = %d\n", task_addr, (curTick() - start_task)/ticks(1), num_packets);
        }
        // Print execution times for subtask
        if (addr == TIMER_END_SUBTASK || addr == TIMER_ENDSTART_SUBTASK) {
          DPRINTF(Task, "Subtask @ %x ET = %d\n", subtask_addr, (curTick() - start_subtask)/ticks(1));
        }
        if (addr == TIMER_START_SUBTASK || addr == TIMER_ENDSTART_SUBTASK) {
          start_subtask = curTick();
          subtask_addr = tc->instAddr();
        }
#endif // DEBUG

        // Clean up
        delete timerpkt;

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
            Tick timer_latency = 0;
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


              /*
                if (curStaticInst->isInteger()) {
                  DPRINTF(Fifo, "# src: %d, # dest: %d\n", curStaticInst->numSrcRegs(),
                      curStaticInst->numDestRegs());
                  int i;
                  for (i = 0; i < curStaticInst->numSrcRegs(); i++) {
                    DPRINTF(Fifo, "src[%d] = %x\n", i, curStaticInst->srcRegIdx(i));
                  }
                  for (i = 0; i < curStaticInst->numDestRegs(); i++) {
                    DPRINTF(Fifo, "dest[%d] = %x\n", i, curStaticInst->destRegIdx(i));
                  }
                  // disassemble takes a pc and symbol table, not even sure what the pc is used for
                  DPRINTF(Fifo, "dis: %s\n", curStaticInst->disassemble(0, NULL));
                }
                */

                fault = curStaticInst->execute(this, traceData);

                // keep an instruction count
                if (fault == NoFault)
                    countInst();
                else if (traceData && !DTRACE(ExecFaulting)) {
                    delete traceData;
                    traceData = NULL;
                }

                postExecute();
                
                /* Check if FIFO has emptied after a timer stall */
                // Guarantees the WCET will work properly
                if (timer_enabled && fifo_enabled && timerStalled){
                    timerStalled = false;
                    
                    bool isempty = isFifoEmpty();
                    DPRINTF(SlackTimer, "Checking if FIFO has emptied: %d\n", isempty);
                    
                    if (!isempty){
                        panic("Could not finish monitoring within allotted time.\n");
                    }
                }
                
                /* Check if next FIFO packet has the done flag and
                 * exit.
                 */
                if (fifo_enabled && curStaticInst->isLoad()
                    && fed.memAddr >= FIFO_ADDR_START && fed.memAddr <= FIFO_ADDR_END
                    && !fifoEmpty && isFifoDone()) {

                    // Had some dropping event
                    if (drops || not_drops){
                        printf("Drops = %d, Non-drops = %d\n", drops, not_drops);
                    }

                    exitSimLoop("fifo done flag received");

                }
                
                /* Send FIFO Packet */
                // On store to FIFO_END_CUSTOM, we will generate 
                // a packet to be sent
                if (fifo_enabled && curStaticInst->isStore() 
                    && fed.memAddr == FIFO_END_CUSTOM){
                    
                    DPRINTF(Fifo, "Creating custom packet at %d, PC: %x\n", curTick(), tc->instAddr());
                    
                    // Monitoring packet to be sent
                    mp.instAddr = tc->instAddr();
                    mp.memEnd = fed.data;
                    mp.store = true;
                    
                    // Send packet on fifo port
                    if(sendFifoPacket()) {
                        // Successful, no need to stall
                        fifoStall = false;
                    } else {
                        // Unsuccessful, stall fifo
                        fifoStall = true;
                    }
                    
                }

                /* Monitoring */
                // Currently on loads, generate fifo event
                // Fifo and monitoring must be enabled
                // Address cannot be for fifo or timer unless it is a fifo
                // write to indicate that the main core is done.
                if (fifo_enabled && ( (mp.done && curStaticInst->isStore()) || 
                    (monitoring_enabled && 
                     (curStaticInst->isLoad() || curStaticInst->isStore()) &&
                     ((fed.memAddr < FIFO_ADDR_START) || (fed.memAddr > TIMER_ADDR_END)) )
                    ) ) {
                    
                    

                  DPRINTF(Fifo, "Monitoring event at %d, PC: %x\n", 
                      curTick(), tc->instAddr());

                  /*
                  std::ostringstream src_regs;
                  src_regs << "Src regs used ( " << MaxInstSrcRegs << "total):";
                  unsigned i;
                  for (i = 0; i < curStaticInst->numSrcRegs(); ++i){
                    src_regs << " " << i << "->" << curStaticInst->srcRegIdx(i);
                  }
                  src_regs << "\n";
                  DPRINTF(Fifo, src_regs.str().data());
                  
                  std::ostringstream dest_regs;
                  dest_regs << "Dest regs used:";
                  for (i=0; i < curStaticInst->numDestRegs(); ++i){
                    dest_regs << " " << i << "->" << curStaticInst->destRegIdx(i);
                  }
                  dest_regs << "\n";
                  DPRINTF(Fifo, dest_regs.str().data());

                  DPRINTF(Fifo, "numsrc: %d, numdest: %d\n", curStaticInst->numSrcRegs(), curStaticInst->numDestRegs());
                  DPRINTF(Fifo, "src: %d, dest: %d\n", curStaticInst->srcRegIdx(0), curStaticInst->destRegIdx(0));
                  */
                  
                  // Monitoring packet to be sent
                  mp.valid = true;
                  mp.instAddr = tc->instAddr();
                  mp.memAddr = fed.memAddr;
                  mp.memEnd = fed.memAddr;
                  mp.data = fed.data;
                  mp.numsrcregs = curStaticInst->numSrcRegs();
                  for (unsigned i = 0; i < curStaticInst->numSrcRegs(); ++i){
                    mp.srcregs[i] = curStaticInst->srcRegIdx(i);
                  }
                  if (curStaticInst->isStore()) {
                    mp.store = true;
                  } else if (curStaticInst->isLoad()) {
                    mp.store = false;
                  } else {
                    panic("Neither store nor load instruction for monitoring\n");
                  }
                  
                  // Send packet on fifo port
                  if(sendFifoPacket()) {
                    // Successful, no need to stall
                    fifoStall = false;
                  } else {
                    // Unsuccessful, stall fifo
                    fifoStall = true;
                    // Start decrementing timer
                    Request *req = &data_write_req;
                    req->setPhys(TIMER_START_DECREMENT, sizeof(bool), ArmISA::TLB::AllowUnaligned, dataMasterId());
                    // Read command
                    MemCmd cmd = MemCmd::WriteReq;
                    // Create packet
                    PacketPtr pkt = new Packet(req, cmd);
                    // Point packet to data pointer
                    pkt->dataStatic(&fifoStall);

                    // Send read request
                    timerPort.sendFunctional(pkt);

                    delete pkt;
                  }

                }
                
                /* Stall for periodicity of task */
                // Once we end a task, we should stall by the slack
                // plus the additional time we specified. This is
                // calculated in the writeMem function and stored in
                // fed.data
                if (timer_enabled && curStaticInst->isStore()
                    && fed.memAddr == TIMER_END_TASK){
                    timerStalled = true;
                    timer_latency = fed.data;
                    DPRINTF(SlackTimer, "The CPU will be stalled for %d ticks\n", fed.data);
                }
                
                // Clear fed so we don't mistakenly read the same values
                // in future instructions.
                fed.clear();
                
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
                    aligned_stall_ticks += 1;

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
            // Store start of stall time
            fifoStallTicks = curTick();
        } else {
            schedule(tickEvent, curTick() + latency);
        }
    }
}

bool AtomicSimpleCPU::isFifoDone() {
    bool isdone;
    // Create request at fifo location
    Request *req = &data_read_req;
    // Size of monitoring packet
    req->setPhys(FIFO_DONE, sizeof(bool), ArmISA::TLB::AllowUnaligned, dataMasterId());
    // Read command
    MemCmd cmd = MemCmd::ReadReq;
    // Create packet
    PacketPtr pkt = new Packet(req, cmd);
    // Point packet to data pointer
    pkt->dataStatic(&isdone);

    // Send read request
    fifoPort.sendFunctional(pkt);

    delete pkt;

    return isdone;
}

bool AtomicSimpleCPU::isFifoEmpty() {
    bool isempty;
    // Create request at fifo location
    Request *req = &data_read_req;
    // Size of monitoring packet
    req->setPhys(FIFO_EMPTY, sizeof(bool), ArmISA::TLB::AllowUnaligned, dataMasterId());
    // Read command
    MemCmd cmd = MemCmd::ReadReq;
    // Create packet
    PacketPtr pkt = new Packet(req, cmd);
    // Point packet to data pointer
    pkt->dataStatic(&isempty);

    // Send read request
    fifoPort.sendFunctional(pkt);

    delete pkt;
    
    return isempty;
}

bool AtomicSimpleCPU::sendFifoPacket() {
  // Create request
  Request *req = &data_write_req;
  // set physical address
  req->setPhys((Addr)FIFO_ADDR, sizeof(mp), ArmISA::TLB::AllowUnaligned, dataMasterId());

  // Create write packet
  PacketPtr fifopkt = new Packet(req, MemCmd::WriteReq);
  // Set data
  fifopkt->dataStatic(&mp);
  // Send request
  bool success = fifoPort.sendTimingReq(fifopkt);
  // Clean up
  delete fifopkt;
  if (success){
    #ifdef DEBUG
      DPRINTF(Fifo, "Sent Packet with values: %x, %x, %x, %x, %x, %x, %x\n", mp.valid, mp.instAddr, mp.memAddr, mp.memEnd, mp.data, mp.store, mp.done);
      num_packets++;
    #endif
    mp.init();
  }
  
  return success;
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
    
    DPRINTF(Fifo, "Success, stalled for %d\n", curTick() - fifoStallTicks);
    DPRINTF(FifoStall, "Fifo caused stall for %d ticks\n", curTick() - fifoStallTicks);
  } else {
    // Failed
    // Retry on next cycle
    schedule(fifoEvent, curTick() + ticks(1));
  }

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
