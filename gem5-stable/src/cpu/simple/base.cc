/*
 * Copyright (c) 2010-2011 ARM Limited
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

#include "arch/kernel_stats.hh"
#include "arch/stacktrace.hh"
#include "arch/tlb.hh"
#include "arch/utility.hh"
#include "arch/vtophys.hh"
#include "base/loader/symtab.hh"
#include "base/cp_annotate.hh"
#include "base/cprintf.hh"
#include "base/inifile.hh"
#include "base/misc.hh"
#include "base/pollevent.hh"
#include "base/range.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/simple/base.hh"
#include "cpu/base.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/checker/thread_context.hh"
#include "cpu/exetrace.hh"
#include "cpu/profile.hh"
#include "cpu/simple_thread.hh"
#include "cpu/smt.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/Decode.hh"
#include "debug/Fetch.hh"
#include "debug/Fifo.hh"
#include "debug/Quiesce.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "params/BaseSimpleCPU.hh"
#include "sim/byteswap.hh"
#include "sim/debug.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/sim_events.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"
#include "sim/system.hh"
#include "sim/sim_exit.hh"

#include "debug/SlackTimer.hh"
#include "debug/Task.hh"

using namespace std;
using namespace TheISA;

BaseSimpleCPU::BaseSimpleCPU(BaseSimpleCPUParams *p)
    : BaseCPU(p), traceData(NULL), thread(NULL),
    fifoPort(name() + "-iport", this),
    timerPort(name() + "-iport", this)
{
    // Store monitoring parameters
    fifo_enabled = p->fifo_enabled;
    monitoring_enabled = p->monitoring_enabled;
    timer_enabled = p->timer_enabled;

    // Monitoring filter parameters
    mf.load = p->monitoring_filter_load;
    mf.store = p->monitoring_filter_store;
    mf.call = p->monitoring_filter_call;
    mf.ret = p->monitoring_filter_ret;

    if (FullSystem)
        thread = new SimpleThread(this, 0, p->system, p->itb, p->dtb);
    else
        thread = new SimpleThread(this, /* thread_num */ 0, p->system,
                p->workload[0], p->itb, p->dtb);

    thread->setStatus(ThreadContext::Halted);

    tc = thread->getTC();

    if (p->checker) {
        BaseCPU *temp_checker = p->checker;
        checker = dynamic_cast<CheckerCPU *>(temp_checker);
        checker->setSystem(p->system);
        // Manipulate thread context
        ThreadContext *cpu_tc = tc;
        tc = new CheckerThreadContext<ThreadContext>(cpu_tc, this->checker);
    } else {
        checker = NULL;
    }

    numInst = 0;
    startNumInst = 0;
    numOp = 0;
    startNumOp = 0;
    numLoad = 0;
    startNumLoad = 0;
    lastIcacheStall = 0;
    lastDcacheStall = 0;

    threadContexts.push_back(tc);


    fetchOffset = 0;
    stayAtPC = false;
}

BaseSimpleCPU::~BaseSimpleCPU()
{
}

void
BaseSimpleCPU::deallocateContext(ThreadID thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
}


void
BaseSimpleCPU::haltContext(ThreadID thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
}


void
BaseSimpleCPU::regStats()
{
    using namespace Stats;

    BaseCPU::regStats();

    numInsts
        .name(name() + ".committedInsts")
        .desc("Number of instructions committed")
        ;

    numOps
        .name(name() + ".committedOps")
        .desc("Number of ops (including micro ops) committed")
        ;

    numIntAluAccesses
        .name(name() + ".num_int_alu_accesses")
        .desc("Number of integer alu accesses")
        ;

    numFpAluAccesses
        .name(name() + ".num_fp_alu_accesses")
        .desc("Number of float alu accesses")
        ;

    numCallsReturns
        .name(name() + ".num_func_calls")
        .desc("number of times a function call or return occured")
        ;

    numCondCtrlInsts
        .name(name() + ".num_conditional_control_insts")
        .desc("number of instructions that are conditional controls")
        ;

    numIntInsts
        .name(name() + ".num_int_insts")
        .desc("number of integer instructions")
        ;

    numFpInsts
        .name(name() + ".num_fp_insts")
        .desc("number of float instructions")
        ;

    numIntRegReads
        .name(name() + ".num_int_register_reads")
        .desc("number of times the integer registers were read")
        ;

    numIntRegWrites
        .name(name() + ".num_int_register_writes")
        .desc("number of times the integer registers were written")
        ;

    numFpRegReads
        .name(name() + ".num_fp_register_reads")
        .desc("number of times the floating registers were read")
        ;

    numFpRegWrites
        .name(name() + ".num_fp_register_writes")
        .desc("number of times the floating registers were written")
        ;

    numMemRefs
        .name(name()+".num_mem_refs")
        .desc("number of memory refs")
        ;

    numStoreInsts
        .name(name() + ".num_store_insts")
        .desc("Number of store instructions")
        ;

    numLoadInsts
        .name(name() + ".num_load_insts")
        .desc("Number of load instructions")
        ;

    notIdleFraction
        .name(name() + ".not_idle_fraction")
        .desc("Percentage of non-idle cycles")
        ;

    idleFraction
        .name(name() + ".idle_fraction")
        .desc("Percentage of idle cycles")
        ;

    numBusyCycles
        .name(name() + ".num_busy_cycles")
        .desc("Number of busy cycles")
        ;

    numIdleCycles
        .name(name()+".num_idle_cycles")
        .desc("Number of idle cycles")
        ;

    icacheStallCycles
        .name(name() + ".icache_stall_cycles")
        .desc("ICache total stall cycles")
        .prereq(icacheStallCycles)
        ;

    dcacheStallCycles
        .name(name() + ".dcache_stall_cycles")
        .desc("DCache total stall cycles")
        .prereq(dcacheStallCycles)
        ;

    icacheRetryCycles
        .name(name() + ".icache_retry_cycles")
        .desc("ICache total retry cycles")
        .prereq(icacheRetryCycles)
        ;

    dcacheRetryCycles
        .name(name() + ".dcache_retry_cycles")
        .desc("DCache total retry cycles")
        .prereq(dcacheRetryCycles)
        ;

    idleFraction = constant(1.0) - notIdleFraction;
    numIdleCycles = idleFraction * numCycles;
    numBusyCycles = (notIdleFraction)*numCycles;
}

void
BaseSimpleCPU::resetStats()
{
//    startNumInst = numInst;
     notIdleFraction = (_status != Idle);
}

void
BaseSimpleCPU::serialize(ostream &os)
{
    SERIALIZE_ENUM(_status);
    BaseCPU::serialize(os);
//    SERIALIZE_SCALAR(inst);
    nameOut(os, csprintf("%s.xc.0", name()));
    thread->serialize(os);
}

void
BaseSimpleCPU::unserialize(Checkpoint *cp, const string &section)
{
    UNSERIALIZE_ENUM(_status);
    BaseCPU::unserialize(cp, section);
//    UNSERIALIZE_SCALAR(inst);
    thread->unserialize(cp, csprintf("%s.xc.0", section));
}

void
change_thread_state(ThreadID tid, int activate, int priority)
{
}

Addr
BaseSimpleCPU::dbg_vtophys(Addr addr)
{
    return vtophys(tc, addr);
}

void
BaseSimpleCPU::wakeup()
{
    if (thread->status() != ThreadContext::Suspended)
        return;

    DPRINTF(Quiesce,"Suspended Processor awoke\n");
    thread->activate();
}

void
BaseSimpleCPU::checkForInterrupts()
{
    if (checkInterrupts(tc)) {
        Fault interrupt = interrupts->getInterrupt(tc);

        if (interrupt != NoFault) {
            fetchOffset = 0;
            interrupts->updateIntrInfo(tc);
            interrupt->invoke(tc);
            thread->decoder.reset();
        }
    }
}


void
BaseSimpleCPU::setupFetchRequest(Request *req)
{
    Addr instAddr = thread->instAddr();

    // set up memory request for instruction fetch
    DPRINTF(Fetch, "Fetch: PC:%08p\n", instAddr);

    Addr fetchPC = (instAddr & PCMask) + fetchOffset;
    req->setVirt(0, fetchPC, sizeof(MachInst), Request::INST_FETCH, instMasterId(),
            instAddr);
}


void
BaseSimpleCPU::preExecute()
{
    // maintain $r0 semantics
    thread->setIntReg(ZeroReg, 0);
#if THE_ISA == ALPHA_ISA
    thread->setFloatReg(ZeroReg, 0.0);
#endif // ALPHA_ISA

    // check for instruction-count-based events
    comInstEventQueue[0]->serviceEvents(numInst);
    system->instEventQueue.serviceEvents(system->totalNumInsts);

    // decode the instruction
    inst = gtoh(inst);

    TheISA::PCState pcState = thread->pcState();

    if (isRomMicroPC(pcState.microPC())) {
        stayAtPC = false;
        curStaticInst = microcodeRom.fetchMicroop(pcState.microPC(),
                                                  curMacroStaticInst);
    } else if (!curMacroStaticInst) {
        //We're not in the middle of a macro instruction
        StaticInstPtr instPtr = NULL;

        TheISA::Decoder *decoder = &(thread->decoder);

        //Predecode, ie bundle up an ExtMachInst
        //This should go away once the constructor can be set up properly
        decoder->setTC(thread->getTC());
        //If more fetch data is needed, pass it in.
        Addr fetchPC = (pcState.instAddr() & PCMask) + fetchOffset;
        //if(decoder->needMoreBytes())
            decoder->moreBytes(pcState, fetchPC, inst);
        //else
        //    decoder->process();

        //Decode an instruction if one is ready. Otherwise, we'll have to
        //fetch beyond the MachInst at the current pc.
        instPtr = decoder->decode(pcState);
        if (instPtr) {
            stayAtPC = false;
            thread->pcState(pcState);
        } else {
            stayAtPC = true;
            fetchOffset += sizeof(MachInst);
        }

        //If we decoded an instruction and it's microcoded, start pulling
        //out micro ops
        if (instPtr && instPtr->isMacroop()) {
            curMacroStaticInst = instPtr;
            curStaticInst = curMacroStaticInst->fetchMicroop(pcState.microPC());
        } else {
            curStaticInst = instPtr;
        }
    } else {
        //Read the next micro op from the macro op
        curStaticInst = curMacroStaticInst->fetchMicroop(pcState.microPC());
    }

    //If we decoded an instruction this "tick", record information about it.
    if (curStaticInst) {
#if TRACING_ON
        traceData = tracer->getInstRecord(curTick(), tc,
                curStaticInst, thread->pcState(), curMacroStaticInst);

        DPRINTF(Decode,"Decode: Decoded %s instruction: %#x\n",
                curStaticInst->getName(), curStaticInst->machInst);
#endif // TRACING_ON
    }
}

void
BaseSimpleCPU::postExecute()
{
    assert(curStaticInst);

    TheISA::PCState pc = tc->pcState();
    Addr instAddr = pc.instAddr();

    if (FullSystem && thread->profile) {
        bool usermode = TheISA::inUserMode(tc);
        thread->profilePC = usermode ? 1 : instAddr;
        ProfileNode *node = thread->profile->consume(tc, curStaticInst);
        if (node)
            thread->profileNode = node;
    }

    if (curStaticInst->isMemRef()) {
        numMemRefs++;
    }

    if (curStaticInst->isLoad()) {
        ++numLoad;
        comLoadEventQueue[0]->serviceEvents(numLoad);
    }

    if (CPA::available()) {
        CPA::cpa()->swAutoBegin(tc, pc.nextInstAddr());
    }

    /* Power model statistics */
    //integer alu accesses
    if (curStaticInst->isInteger()){
        numIntAluAccesses++;
        numIntInsts++;
    }

    //float alu accesses
    if (curStaticInst->isFloating()){
        numFpAluAccesses++;
        numFpInsts++;
    }
    
    //number of function calls/returns to get window accesses
    if (curStaticInst->isCall() || curStaticInst->isReturn()){
        numCallsReturns++;
    }
    
    //the number of branch predictions that will be made
    if (curStaticInst->isCondCtrl()){
        numCondCtrlInsts++;
    }
    
    //result bus acceses
    if (curStaticInst->isLoad()){
        numLoadInsts++;
    }
    
    if (curStaticInst->isStore()){
        numStoreInsts++;
    }
    /* End power model statistics */

    if (FullSystem)
        traceFunctions(instAddr);

    if (traceData) {
        traceData->dump();
        delete traceData;
        traceData = NULL;
    }

    performMonitoring();
}

void 
BaseSimpleCPU::performMonitoring() {
  
    // Check if this instruction is predicated
    bool predicate_result = readPredicate();
  
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
    if (fifo_enabled && predicate_result &&
        curStaticInst->isStore() && 
        fed.memAddr == FIFO_END_CUSTOM){
        
        DPRINTF(Fifo, "Creating custom packet at %d, PC: %x\n", curTick(), tc->instAddr());
        
        // Monitoring packet to be sent
        mp.instAddr = tc->instAddr();
        mp.memEnd = fed.data;
        mp.store = true;
        
        // Send packet on fifo port, stall if not successful
        fifoStall = !sendFifoPacket();
        // Any additional functionality needed for stall
        if (fifoStall) {
            stallFromFifo();
        }
        
    }

    /* Monitoring */
    // Currently on loads, generate fifo event
    // Fifo and monitoring must be enabled
    // Address cannot be for fifo or timer unless it is a fifo
    // write to indicate that the main core is done.
    if (fifo_enabled && ( (mp.done && curStaticInst->isStore()) || 
      (monitoring_enabled && predicate_result &&
       (
        (curStaticInst->isLoad() && mf.load) ||
        (curStaticInst->isStore() && mf.store) ||
        (curStaticInst->isCall() && mf.call) || 
        (curStaticInst->isReturn() && mf.ret)
       )
       &&
       ((fed.memAddr < FIFO_ADDR_START) || (fed.memAddr > TIMER_ADDR_END)) )
      ) ) {

        DPRINTF(Fifo, "Monitoring event at %d, PC: %x\n", 
            curTick(), tc->instAddr());

        // Monitoring packet to be sent
        mp.valid = true;
        mp.instAddr = tc->instAddr(); // current PC
        mp.memAddr = fed.memAddr; // memory access instruction
        mp.memEnd = fed.memAddr;
        mp.data = fed.data;       // memory access data
        mp.numsrcregs = curStaticInst->numSrcRegs();
        for (unsigned i = 0; i < curStaticInst->numSrcRegs(); ++i){
          mp.srcregs[i] = curStaticInst->srcRegIdx(i);
        }
        // load/store flag
        if (curStaticInst->isStore()) {
          mp.store = true;
        } else if (curStaticInst->isLoad()) {
          mp.store = false;
        }
        // Control instruction information
        mp.control = curStaticInst->isControl(); // control inst
        mp.call    = curStaticInst->isCall();    // call inst
        mp.ret     = curStaticInst->isReturn();  // return inst
        mp.lr      = tc->readIntReg(14); // Link register
        mp.nextpc  = tc->nextInstAddr(); // Next program counter

        // Send packet on fifo port, stall if not successful
        fifoStall = !sendFifoPacket();
        if (fifoStall) {
          // Any additional functionality needed for stall
          stallFromFifo();

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
        && predicate_result && fed.memAddr == TIMER_END_TASK){
        endTask();
    }

    // Clear fed so we don't mistakenly read the same values
    // in future instructions.
    fed.clear();
}

// Checks done flag in fifo
bool 
BaseSimpleCPU::isFifoDone() {
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

// Checks whether fifo is empty
bool
BaseSimpleCPU::isFifoEmpty() {
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

// Send monitoring packet to fifo
bool 
BaseSimpleCPU::sendFifoPacket() {
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
    #endif
    mp.init();
  }
  
  return success;
}

void BaseSimpleCPU::init() {

  BaseCPU::init();

  // Initialize monitoring variables
  mp.init();
  fed.clear();
  fifoStall = false;
  timerStalled = false;
  fifoEmpty = false;
  drops = 0;
  not_drops = 0;
}

void
BaseSimpleCPU::readFromTimer(Addr addr, uint8_t * data, unsigned size, unsigned flags) {
    // use the CPU's statically allocated read request and packet objects
    Request *req = &data_read_req;
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
    } else if (addr == TIMER_READ_DROP) {
        // Update hardware counters
        if (read_timer == 1) { not_drops++; }
        else { drops++; }
        // Print out for debug
    #ifdef DEBUG
        DPRINTF(SlackTimer, "read from timer: drop? %d, numdrops: %d, numfull: %d\n", !read_timer, drops, not_drops);
    #endif
        memcpy(data, &read_timer, size);
    }
    
    // Clean up
    delete pkt;
}

void
BaseSimpleCPU::readFromFifo(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags)
{
    // use the CPU's statically allocated read request and packet objects
    Request *req = &data_read_req;

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
        if (sizeof(unsigned) < read_size){ read_size = sizeof(unsigned); } //Make sure we don't copy garbage data
        memcpy(&read_data, data, size);
        DPRINTF(Fifo, "read fifo @ %x = %x\n", addr, read_data);
    }
#endif

}

void
BaseSimpleCPU::writeToFifo(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags) 
{
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
}


void
BaseSimpleCPU::writeToTimer(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags)
{

    // Get data
    int write_data = 0;
    if (size > sizeof(int)) size = sizeof(int);
    memcpy(&write_data, data, size);
  
  #ifdef DEBUG
    // Print out data for debugging
    DPRINTF(SlackTimer, "Write to timer [%x]: %d\n", addr, write_data);
    // Print messages with start and end task so we can find WCET
    if (addr == TIMER_START_TASK) {
      start_task = curTick();
      num_packets = 0;
      last_packets = 0;
      num_stalls = 0;
      last_stalls = 0;
      task_addr = tc->instAddr();
    } else if (addr == TIMER_END_TASK) {
      DPRINTF(Task, "Task @ %x ET = %d, Packets = %d\n", task_addr, (curTick() - start_task - num_stalls)/ticks(1), num_packets);
    }
    // Print execution times for subtask
    if (addr == TIMER_END_SUBTASK || addr == TIMER_ENDSTART_SUBTASK) {
      DPRINTF(Task, "Subtask @ %x ET = %d, Packets = %d\n", subtask_addr, (curTick() - start_subtask - (num_stalls - last_stalls))/ticks(1), num_packets-last_packets);
      last_packets = num_packets;
      last_stalls = num_stalls;
    }
    if (addr == TIMER_START_SUBTASK || addr == TIMER_ENDSTART_SUBTASK) {
      start_subtask = curTick();
      subtask_addr = tc->instAddr();
    }
  #endif // DEBUG
  
    //if end_task we get the timer slack and will add it to the
    //additional slack we are storing into the timer.
    
    if (addr == TIMER_END_TASK){
        int slack;
        Request* req = &data_read_req;
        flags = ArmISA::TLB::AllowUnaligned;
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

    //convert from cycles to ticks
    write_data *= ticks(1);
    // Create request
    Request *timer_write_req = &data_write_req;
    //unsigned size = sizeof(write_tp);
    flags = ArmISA::TLB::AllowUnaligned;
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

    // Clean up
    delete timerpkt;
}

void
BaseSimpleCPU::advancePC(Fault fault)
{
    //Since we're moving to a new pc, zero out the offset
    fetchOffset = 0;
    if (fault != NoFault) {
        curMacroStaticInst = StaticInst::nullStaticInstPtr;
        fault->invoke(tc, curStaticInst);
        thread->decoder.reset();
    } else {
        if (curStaticInst) {
            if (curStaticInst->isLastMicroop())
                curMacroStaticInst = StaticInst::nullStaticInstPtr;
            TheISA::PCState pcState = thread->pcState();
            TheISA::advancePC(pcState, curStaticInst);
            thread->pcState(pcState);
        }
    }
}

MasterPort &
BaseSimpleCPU::getMasterPort(const std::string &if_name, int idx)
{
  if (if_name == "fifo_port") {
    return fifoPort;
  } else if (if_name == "timer_port") {
    return timerPort;
  } else {
    return BaseCPU::getMasterPort(if_name, idx);
  }
}


/*Fault
BaseSimpleCPU::CacheOp(uint8_t Op, Addr EffAddr)
{
    // translate to physical address
    Fault fault = NoFault;
    int CacheID = Op & 0x3; // Lower 3 bits identify Cache
    int CacheOP = Op >> 2; // Upper 3 bits identify Cache Operation
    if(CacheID > 1)
      {
        warn("CacheOps not implemented for secondary/tertiary caches\n");
      }
    else
      {
        switch(CacheOP)
          { // Fill Packet Type
          case 0: warn("Invalidate Cache Op\n");
            break;
          case 1: warn("Index Load Tag Cache Op\n");
            break;
          case 2: warn("Index Store Tag Cache Op\n");
            break;
          case 4: warn("Hit Invalidate Cache Op\n");
            break;
          case 5: warn("Fill/Hit Writeback Invalidate Cache Op\n");
            break;
          case 6: warn("Hit Writeback\n");
            break;
          case 7: warn("Fetch & Lock Cache Op\n");
            break;
          default: warn("Unimplemented Cache Op\n");
          }
      }
    return fault;
}*/
