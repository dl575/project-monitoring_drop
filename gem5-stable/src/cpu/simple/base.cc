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

#include "debug/CheckId.hh"

using namespace std;
using namespace TheISA;

BaseSimpleCPU::BaseSimpleCPU(BaseSimpleCPUParams *p)
    : BaseCPU(p), traceData(NULL), thread(NULL),
    monitoring_enabled(p->monitoring_enabled), fifo_enabled(p->fifo_enabled),
    timer_enabled(p->timer_enabled), flagcache_enabled(p->flagcache_enabled),
    emulate_filtering(p->emulate_filtering),
    fifoPort(name() + "-iport", this),
    timerPort(name() + "-iport", this),
    fcPort(name() + "-iport", this),
    invtab("Invalidation Table"),
    filtertab1("Filter Table 1"),
    filtertab2("Filter Table 2"),
    fptab("Filter Pointers"),
    hard_wcet(p->hard_wcet),
    fifoStall(false), 
    timerStalled(false),
    fifoEmpty(false),
    check_load(p->check_load),
    check_store(p->check_store),
    check_indctrl(p->check_indctrl),
    target_coverage(p->target_coverage),
    check_frequency(p->check_frequency),
    total_checks(0), full_packets(1), all_packets(1),
    source_dropping(p->source_dropping),
    perf_mon(true),
    _backtrack(p->backtrack),
    optimal_dropping(p->optimal_dropping),
    print_checkid(p->print_checkid),
    print_static_coverage(p->print_static_coverage)
{

    // Monitoring filter parameters
    mf.load = p->monitoring_filter_load;
    mf.store = p->monitoring_filter_store;
    mf.call = p->monitoring_filter_call;
    mf.ret = p->monitoring_filter_ret;
    mf.intalu = p->monitoring_filter_intalu;
    mf.intand = p->monitoring_filter_intand;
    mf.intmov = p->monitoring_filter_intmov;
    mf.intadd = p->monitoring_filter_intadd;
    mf.intsub = p->monitoring_filter_intsub;
    mf.intmul = p->monitoring_filter_intmul;
    mf.indctrl = p->monitoring_filter_indctrl;
    settag_store = p->settag_store;
    
    // monitoring extension type
    switch(p->monitor_type) {
        case MONITOR_NONE: monitorExt = MONITOR_NONE; break;
        case MONITOR_UMC: monitorExt = MONITOR_UMC; break;
        case MONITOR_DIFT: monitorExt = MONITOR_DIFT; break;
        case MONITOR_BC: monitorExt = MONITOR_BC; break;
        case MONITOR_SEC: monitorExt = MONITOR_SEC; break;
        case MONITOR_HB: monitorExt = MONITOR_HB; break;
        case MONITOR_MULTIDIFT: monitorExt = MONITOR_MULTIDIFT; break;
        case MONITOR_LRC: monitorExt = MONITOR_LRC; break;
        case MONITOR_DIFTRF: monitorExt = MONITOR_DIFTRF; break;
        default: panic("Invalid monitor type\n");
    }

    // Initialize table if specified
    if (p->invalidation_file.size()){
        invtab.initTable(p->invalidation_file.data());
    }
    
    // Initialize filtering tables
    if (p->filter_file_1.size()){
        filtertab1.initTable(p->filter_file_1.data());
    }

    // Initialize filtering tables
    if (p->filter_file_2.size()){
        filtertab2.initTable(p->filter_file_2.data());
    }
    
    // Initialize filtering pointers
    if (p->filter_ptr_file.size()){
        fptab.initTable(p->filter_ptr_file.data());
    }
    
#ifdef DEBUG
    if (DTRACE(Invalidation)){
        if(invtab.initialized) invtab.printTable();
        if(filtertab1.initialized) filtertab1.printTable();
        if(filtertab2.initialized) filtertab2.printTable();
        if(fptab.initialized) fptab.printTable();
    }
#endif

    if (target_coverage > 1){
        warn("Target coverage exceeds 100%%. Setting to 100%%.\n");
        target_coverage = 1; 
    }
    if (target_coverage < 0){
        warn("Target coverage less than 0%%. Setting to 0%%.\n");
        target_coverage = 0;
    }
    
    packet_drop_rate = 1 - target_coverage;

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

    // Number of monitored/forwarded micro ops
    numMonOps
        .name(name() + ".num_mon_ops")
        .desc("Number of monitored instructions")
        ;
        
    // Drop statistics      
    dropstats
        .init(num_inst_types)
        .name(name() + ".drops")
        .desc("Number of fifo packets that were dropped")
        .flags(total)
        ;
    for (int i = 0; i < num_inst_types; i++) {
        dropstats.subname(i, instTypeToString(i));
    }
        
    fullstats
        .init(num_inst_types)
        .name(name() + ".non_drops")
        .desc("Number of fifo packets that were not dropped")
        .flags(total)
        ;
    for (int i = 0; i < num_inst_types; i++) {
        fullstats.subname(i, instTypeToString(i));
    }
    
    filterstats
        .init(num_inst_types)
        .name(name() + ".filtered")
        .desc("Number of fifo packets that were filtered")
        .flags(total)
        ;
    for (int i = 0; i < num_inst_types; i++) {
        filterstats.subname(i, instTypeToString(i));
    }

    filterdetailed
        .init(num_inst_types, FilterPtrTable::size)
        .name(name() + ".filtered_detailed")
        .desc("Number of fifo packets that were filtered (detailed)")
        .flags(total)
        ;
    for (int i = 0; i < num_inst_types; i++) {
        filterdetailed.subname(i, instTypeToString(i));
    }
    for (int i = 0; i < FilterPtrTable::size; i++) {
        bitset<2*FC_NUMBITS> ptrbits(i);
        string name = "b" + ptrbits.to_string();
        filterdetailed.ysubname(i, name);
    }

    coveragedropstats
        .init(num_inst_types)
        .name(name() + ".coverage_drops")
        .desc("Number of fifo packets that were dropped for coverage")
        .flags(total)
        ;
    for (int i = 0; i < num_inst_types; i++) {
        coveragedropstats.subname(i, instTypeToString(i));
    }

    numImportantInsts
        .name(name() + ".num_important_insts")
        .desc("Number of dynamic instructions marked as important")
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
    SERIALIZE_SCALAR(monitoring_enabled);

    SERIALIZE_ENUM(_status);
    BaseCPU::serialize(os);
//    SERIALIZE_SCALAR(inst);
    nameOut(os, csprintf("%s.xc.0", name()));
    thread->serialize(os);
}

void
BaseSimpleCPU::unserialize(Checkpoint *cp, const string &section)
{
    UNSERIALIZE_SCALAR(monitoring_enabled);

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

    if (perf_mon) {
      performMonitoring();
      // Clear fed so we don't mistakenly read the same values
      // in future instructions.
      fed.clear();
    }
}

void 
BaseSimpleCPU::performMonitoring() {
    // Handle monitoring for read syscall
    if (fed.syscallRead) {
      // Create FIFO packet
      mp.valid = true;
      mp.instAddr = tc->instAddr();
      mp.settag = true;
      mp.syscallReadBufPtr = fed.syscallReadBufPtr;
      mp.syscallReadNbytes = fed.syscallReadNbytes;
      // Send packet on fifo port, stall if not successful
      fifoStall = !sendFifoPacket();
      // Any additional functionality needed for stall
      if (fifoStall) {
        stallFromFifo();
      }
      return;
    }
    
    // Check if this instruction is predicated (true is executed, false is predicated false)
    bool predicate_result = readPredicate();
  
    /* Check if FIFO has emptied after a timer stall */
    // Guarantees the WCET will work properly
    if (timer_enabled && fifo_enabled && timerStalled){
        timerStalled = false;
        
        bool isempty = isFifoEmpty();
        DPRINTF(SlackTimer, "Checking if FIFO has emptied: %d\n", isempty);
        
        if (!isempty && hard_wcet){
            panic("Could not finish monitoring within allotted time.\n");
        }
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
        mp.size = mp.memEnd - mp.memAddr + 1;
        mp.data = 1;
        mp.settag = true;
        mp.store = false;
        mp.custom = true;
        
        // Send packet on fifo port, stall if not successful
        fifoStall = !sendFifoPacket();
        // Any additional functionality needed for stall
        if (fifoStall) {
            stallFromFifo();
        }
        return; 
    }
    // writing to FIFO_CUSTOM_DATA sends a packet to set tags
    // FIFO_END_CUSTOM is kept for backward compatibility
    if (fifo_enabled && predicate_result && curStaticInst->isStore() && 
        fed.memAddr == FIFO_CUSTOM_DATA){
        
        // Monitoring packet to be sent
        mp.instAddr = tc->instAddr();
        mp.data = fed.data;
        mp.settag = true;
        mp.store = false;
        mp.custom = true;
        DPRINTF(Fifo, "Create custom packet at %d, PC=%x, Set tag[0x%x:0x%x]=0x%x\n", 
            curTick(), tc->instAddr(), mp.memAddr, mp.memAddr+mp.size-1, mp.data);        
        
        // Send packet on fifo port, stall if not successful
        fifoStall = !sendFifoPacket();
        // Any additional functionality needed for stall
        if (fifoStall) {
            stallFromFifo();
        }
        return; 
    }

    /* Monitoring */
   
    // String representation of instruction
    std::string inst_dis = curStaticInst->disassemble(tc->instAddr(), NULL); 
    // op type
    OpClass inst_opclass = curStaticInst->opClass();
    // Determine whether the instruction writes to any ISA registers
    bool destReg = false;
    int i;
    for (i = 0; i < curStaticInst->numDestRegs(); i++) {
      if (TheISA::isISAReg(curStaticInst->destRegIdx(i))) {
        destReg = true;
      }
    }

    // Flags for echecking forwarding for specific instruction types
    bool forwardDone = mp.done && curStaticInst->isStore();
    bool forwardLoad = curStaticInst->isLoad() && mf.load;
    bool forwardStore = curStaticInst->isStore() && mf.store;
    bool forwardCall = curStaticInst->isCall() && mf.call;
    bool forwardReturn = curStaticInst->isReturn() && mf.ret;
    // integer ALU, not including control, memory barrier, thread sync, etc. instructions
    // also checks that it writes to a valid destination register
    bool forwardALU = curStaticInst->isInteger() && !curStaticInst->isControl() && ((inst_opclass == IntAluOp) || (inst_opclass == IntMultOp) || (inst_opclass = IntDivOp)) && destReg && mf.intalu;
    bool forwardMov = (inst_dis.find("mov") != string::npos) && mf.intmov && !curStaticInst->isControl();
    bool forwardAdd = (inst_dis.find("add") != string::npos) && mf.intadd;
    bool forwardSub = (inst_dis.find("sub") != string::npos) && mf.intsub;
    bool forwardAnd = (inst_dis.find("and") != string::npos) && mf.intand;
    bool forwardMul = (inst_dis.find("mul") != string::npos) && mf.intmul;
    bool forwardIndctrl = curStaticInst->isIndirectCtrl() && mf.indctrl;
    bool fifoAccess = FIFO_ADDR_START <= fed.memAddr && fed.memAddr <= FIFO_ADDR_END;
    bool timerAccess = timer_enabled && TIMER_ADDR_START <= fed.memAddr && fed.memAddr <= TIMER_ADDR_END;
    bool flagcacheAccess = flagcache_enabled && FLAG_CACHE_ADDR_START <= fed.memAddr && fed.memAddr <= FLAG_CACHE_ADDR_END;
    // For certain instruction types, depending on mf (monitoring filter)
    // Fifo and monitoring must be enabled
    // Address cannot be for fifo, timer, or flagcache.
    if (fifo_enabled && 
         (forwardDone || 
           (monitoring_enabled && predicate_result &&
             (forwardLoad || forwardStore || forwardCall || forwardReturn || 
             forwardALU || forwardMov || forwardAdd || forwardSub || forwardAnd || 
             forwardMul || forwardIndctrl) &&
             !(fifoAccess || timerAccess || flagcacheAccess)
           )
         ) 
       ) {

        DPRINTF(Fifo, "Monitoring event at %d, PC: %x\n", 
            curTick(), tc->instAddr());
        // Count number of monitored operations
        numMonOps++;
      
        // Monitoring packet to be sent
        mp.valid = true;
        mp.instAddr = tc->instAddr(); // current PC
        mp.memAddr = fed.memAddr; // memory access instruction
        mp.memEnd = fed.memAddr + fed.dataSize - 1;
        mp.data = fed.data;       // memory access data
        mp.virtAddr = fed.dataVirtAddr;
        mp.physAddr = fed.dataPhysAddr;
        mp.lr      = tc->readIntReg(14); // Link register
        mp.nextpc  = tc->nextInstAddr(); // Next program counter
        mp.size = fed.dataSize;
        // String representation of assembly instruction
        strcpy(mp.inst_dis, inst_dis.c_str());
        
        unsigned numSrc = curStaticInst->numSrcRegs();
        bool isMicroOp = inst_dis.find("uop") != string::npos;
        bool isLoad = curStaticInst->isLoad();
        bool isStore = curStaticInst->isStore();

        /////////////////////////////////////////
        // Operands
        /////////////////////////////////////////
        if (isLoad && !isMicroOp && numSrc == 5) {
            mp.rs1 = curStaticInst->srcRegIdx(0);
            mp.rs2 = 33;
        } else if (isLoad && !isMicroOp && numSrc == 6) {
            mp.rs1 = curStaticInst->srcRegIdx(0);
            mp.rs2 = curStaticInst->srcRegIdx(5);
        } else if (isLoad && isMicroOp && numSrc == 5) {
            mp.rs1 = curStaticInst->srcRegIdx(4);
            mp.rs2 = 33;
        } else if (isStore && !isMicroOp && numSrc == 6) {
            mp.rs1 = curStaticInst->srcRegIdx(0);
            mp.rs2 = curStaticInst->srcRegIdx(5);
        } else if (TheISA::isISAReg(curStaticInst->srcRegIdx(numSrc-2))) {
            mp.rs1 = curStaticInst->srcRegIdx(numSrc-2);  // rs1 register field
            mp.rs2 = curStaticInst->srcRegIdx(numSrc-1);  // rs2 register field        
        } else {
            mp.rs1 = curStaticInst->srcRegIdx(numSrc-1);  // rs1 register field
            mp.rs2 = 33;  // rs2 register field
        }
        mp.rs3 = curStaticInst->srcRegIdx(0); // rs3 register field
        // Force source registers to be valid (use zero reg if not)
        if (!(TheISA::isISAReg(mp.rs1))) {
          mp.rs1 = 33;
        }
        if (!(TheISA::isISAReg(mp.rs2))) {
          mp.rs2 = 33;
        }
        //mp.rd = (curStaticInst->numDestRegs())? curStaticInst->destRegIdx(0) : 33;  // rd register field
        // Use zero reg (33) if no valid destination register
        mp.rd = INTREG_ZERO;
        // Otherwise, use first valid destination register
        for (i = 0; mp.rd == INTREG_ZERO && i < curStaticInst->numDestRegs(); i++) {
          if (TheISA::isISAReg(curStaticInst->destRegIdx(i))) {
            mp.rd = curStaticInst->destRegIdx(i);
          }
        }

        /////////////////////////////////////////
        // Instruction Type
        /////////////////////////////////////////
        // load/store flag
        if (isStore) {
          // Mark stores as settag instructions if configured (UMC)
          if (settag_store) {
            mp.settag = true;
            mp.store = false;
          } else {
            mp.store = true;
            mp.settag = false;
          }
        }
        mp.load = curStaticInst->isLoad();
        // Control instruction information
        mp.control = curStaticInst->isControl(); // control inst
        mp.call    = curStaticInst->isCall();    // call inst
        mp.ret     = curStaticInst->isReturn();  // return inst
        mp.indctrl = curStaticInst->isIndirectCtrl(); // indirect control
        // Integer ALU instruction
        mp.intalu  = ((curStaticInst->opClass() == IntAluOp ||
              curStaticInst->opClass() == IntMultOp ||
              curStaticInst->opClass() == IntDivOp) && !curStaticInst->isIndirectCtrl()); // integer ALU inst
        // ALU opcode
        mp.opcode = (uint8_t)curStaticInst->machInst.opcode;

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
        return;
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

  /////////////////////////////////////////////////////////////
  // Check packet properties before sending
  /////////////////////////////////////////////////////////////
  // Check that registers are always either valid or the zero reg
  assert(TheISA::isISAReg(mp.rs1) || mp.rs1 == 33);
  assert(TheISA::isISAReg(mp.rs2) || mp.rs2 == 33);
  assert(TheISA::isISAReg(mp.rd) || mp.rd == 33);
  // Check that load is not loading to zero reg
  assert(!(mp.load && mp.rd == 33));
  // Check that ALU has a valid destination reg
  assert(!(mp.intalu && mp.rd == 33));
  // Check that only one instruction type is marked
  assert(mp.control + mp.intalu + mp.store + mp.load + mp.settag == 1);
  // For DIFT, syscallread is the only settag
  if (monitorExt == MONITOR_DIFT || monitorExt == MONITOR_MULTIDIFT) {
    if (mp.settag) {
      assert(mp.syscallReadNbytes);
    }
  }
  
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
  ReExecFault = new ReExec();

  int i;
  for (i = 0; i < NUM_REGS; i++) {
    invalid_flags[i] = false;
  }

  // Initialize ranges for printing out what is checked in full
  checkid_base = 0;
  checkid_vec = 0;
}

BaseSimpleCPU::instType
BaseSimpleCPU::readFifoInstType()
{
    bool is_type = false;
    readFromFifo(FIFO_SETTAG, (uint8_t*)&is_type, sizeof(is_type), ArmISA::TLB::AllowUnaligned);
    if (is_type) {
      DPRINTF(Invalidation, "Fifo entry instruction type is SETTAG\n");
      return inst_settag;
    }
    readFromFifo(FIFO_LOAD, (uint8_t *)&is_type, sizeof(is_type), ArmISA::TLB::AllowUnaligned);
    if (is_type) { 
        DPRINTF(Invalidation, "Fifo entry instruction type is LOAD\n");
        return inst_load; 
    }
    readFromFifo(FIFO_STORE, (uint8_t *)&is_type, sizeof(is_type), ArmISA::TLB::AllowUnaligned);
    if (is_type) { 
        DPRINTF(Invalidation, "Fifo entry instruction type is STORE\n");
        return inst_store; 
    }
    readFromFifo(FIFO_INDCTRL, (uint8_t *)&is_type, sizeof(is_type), ArmISA::TLB::AllowUnaligned);
    if (is_type) {
        DPRINTF(Invalidation, "Fifo entry instruction type is INDCTRL\n");
        return inst_indctrl;
    }
    readFromFifo(FIFO_CALL, (uint8_t *)&is_type, sizeof(is_type), ArmISA::TLB::AllowUnaligned);
    if (is_type) {
        DPRINTF(Invalidation, "Fifo entry instruction type is CALL\n");
        return inst_call;
    }
    readFromFifo(FIFO_RET, (uint8_t *)&is_type, sizeof(is_type), ArmISA::TLB::AllowUnaligned);
    if (is_type) {
        DPRINTF(Invalidation, "Fifo entry instruction type is RET\n");
        return inst_ret;
    }
    readFromFifo(FIFO_INTALU, (uint8_t *)&is_type, sizeof(is_type), ArmISA::TLB::AllowUnaligned);
    if (is_type) {
        DPRINTF(Invalidation, "Fifo entry instruction type is INTALU\n");
        return inst_intalu;
    }
    DPRINTF(Invalidation, "Fifo entry instruction type is unknown\n");
    return inst_undef;
}

std::string
BaseSimpleCPU::instTypeToString(int inst_type)
{
    switch (inst_type){
        case inst_load: return "LOAD";
        case inst_store: return "STORE";
        case inst_call: return "CALL";
        case inst_ret: return "RET";
        case inst_intalu: return "INTALU";
        case inst_indctrl: return "INDCTRL";
        case inst_settag: return "SETTAG";
    };
    
    return "UNDEF";
}

BaseSimpleCPU::instType
BaseSimpleCPU::parseInstType(const char * inst_type)
{
    if (!strcmp(inst_type, "LOAD")){
        return inst_load;
    }else if (!strcmp(inst_type, "STORE")){
        return inst_store;
    }else if (!strcmp(inst_type, "CALL")){
        return inst_call;
    }else if (!strcmp(inst_type, "RET")){
        return inst_ret;
    }else if (!strcmp(inst_type, "INTALU")){
        return inst_intalu;
    }else if (!strcmp(inst_type, "INDCTRL")){
        return inst_indctrl;
    }else if (!strcmp(inst_type, "SETTAG")){
        return inst_settag;
    }
    return inst_undef;
}

Addr
BaseSimpleCPU::performOp(std::string &op, Addr a, Addr b)
{
    Addr result = 0;
    if (!op.size()) { return 0; }
    if (op == "shftr"){
       result = a >> b;
    } else if (op == "shftl"){
        result = a << b;
    } else if (op == "add"){
        result = a + b;
    } else if (op == "sub"){
        result = a - b;
    } else {
        warn("Unknown ALU operation \"%s\"\n", op.data());
    }
    return result;
}

Addr
BaseSimpleCPU::selectValue(std::string &select, Addr c)
{
    Addr value = 0;
    if (!select.size()) { return 0; }
    if (select == "c") { value = c; }
    else if (select == "prev_addr") {
        readFromFlagCache(FC_GET_ADDR, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else if (select == "MEMADDR") {
        readFromFifo(FIFO_MEMADDR, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else if (select == "INSTADDR") {
        readFromFifo(FIFO_INSTADDR, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else if (select == "NEXTPC") {
        readFromFifo(FIFO_NEXTPC, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else if (select == "LR") {
        readFromFifo(FIFO_LR, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else if (select == "DATA") {
        readFromFifo(FIFO_DATA, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else if (select == "RD") {
        readFromFifo(FIFO_RD, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else if (select == "RS1") {
        readFromFifo(FIFO_RS1, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else if (select == "RS2") {
        readFromFifo(FIFO_RS2, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else if (select == "RS3") {
        readFromFifo(FIFO_RS3, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else if (select == "PHYSADDR") {
        readFromFifo(FIFO_PHYSADDR, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else {
        warn("Unknown select \"%s\"\n", select.data());
    }
    return value;
}

template <unsigned size> Addr
BaseSimpleCPU::getInvalidationAddr(InvalidationTable <size> & it, unsigned idx)
{
    Addr a = selectValue(it.sel1[idx], (Addr)it.constant[idx]);
    Addr b = selectValue(it.sel2[idx], (Addr)it.constant[idx]);
    Addr fc_addr = performOp(it.aluop[idx], a, b);
#ifdef DEBUG
    if (it.aluop[idx].size()){
        DPRINTF(Invalidation, "Calculate address %x %s %x = %x\n", a, it.aluop[idx], b, fc_addr);
    } else {
        DPRINTF(Invalidation, "Calculate address = %x\n", fc_addr);
    }
#endif
    return fc_addr;
}

template <unsigned size> Fault
BaseSimpleCPU::setFlagCacheAddrFromTable(InvalidationTable <size> & it, unsigned idx)
{
    Addr fc_addr = getInvalidationAddr(it, idx);
    // Update address in flag cache
    return writeToFlagCache(FC_SET_ADDR, (uint8_t *)&fc_addr, sizeof(fc_addr), ArmISA::TLB::AllowUnaligned);
}

Fault
BaseSimpleCPU::performInvalidation(unsigned idx, instType itp)
{   
    Fault fault = NoFault;
    // Get target address
    Addr rd = -1;
    readFromFlagCache(FC_GET_ADDR, (uint8_t *)&rd, sizeof(rd), ArmISA::TLB::AllowUnaligned);
    // Get source registers
    Addr rs1 = -1;
    Addr rs2 = -1;
    readFromFifo(FIFO_RS1, (uint8_t *)&rs1, sizeof(rs1), ArmISA::TLB::AllowUnaligned);
    readFromFifo(FIFO_RS2, (uint8_t *)&rs2, sizeof(rs2), ArmISA::TLB::AllowUnaligned);
    
    if (invtab.action[idx] == "HW_set_cache"){
        DPRINTF(Invalidation, "Setting cache\n");
        unsigned type = 1;
        fault = writeToFlagCache(FC_SET_FLAG, (uint8_t *)&type, sizeof(type), ArmISA::TLB::AllowUnaligned);
    } else if (invtab.action[idx] == "HW_clear_cache"){
        DPRINTF(Invalidation, "Clearing cache\n");
        unsigned type = 1;
        fault = writeToFlagCache(FC_CLEAR_FLAG, (uint8_t *)&type, sizeof(type), ArmISA::TLB::AllowUnaligned);
    } else if (invtab.action[idx] == "HW_set_array"){
        DPRINTF(Invalidation, "Setting array\n");
        unsigned type = 0;
        fault = writeToFlagCache(FC_SET_FLAG, (uint8_t *)&type, sizeof(type), ArmISA::TLB::AllowUnaligned);
    } else if (invtab.action[idx] == "HW_clear_array") {
        DPRINTF(Invalidation, "Clearing array\n");
        unsigned type = 0;
        fault = writeToFlagCache(FC_CLEAR_FLAG, (uint8_t *)&type, sizeof(type), ArmISA::TLB::AllowUnaligned);
    } else if (invtab.action[idx].compare(0,14,"HW_set_array_b") == 0) {
        // Get set value from action string
        unsigned value = strtol(invtab.action[idx].substr(14).c_str(), NULL, 2);
        if (value > FC_MAXVAL) value = FC_MAXVAL;
        fault = writeToFlagCache(FC_SET_ARRAY, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else if (invtab.action[idx].compare(0,14,"HW_set_array_d") == 0) {
        // Get set value from action string
        unsigned value = strtol(invtab.action[idx].substr(14).c_str(), NULL, 10);
        if (value > FC_MAXVAL) value = FC_MAXVAL;
        fault = writeToFlagCache(FC_SET_ARRAY, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else if (invtab.action[idx].compare(0,14,"HW_set_cache_b") == 0) {
        // Get set value from action string
        unsigned value = strtol(invtab.action[idx].substr(14).c_str(), NULL, 2);
        if (value > FC_MAXVAL) value = FC_MAXVAL;
        fault = writeToFlagCache(FC_SET_CACHE, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    } else if (invtab.action[idx].compare(0,14,"HW_set_cache_d") == 0) {
        // Get set value from action string
        unsigned value = strtol(invtab.action[idx].substr(14).c_str(), NULL, 10);
        if (value > FC_MAXVAL) value = FC_MAXVAL;
        fault = writeToFlagCache(FC_SET_CACHE, (uint8_t *)&value, sizeof(value), ArmISA::TLB::AllowUnaligned);
    // These commands will also set/clear the thread register file. This is used for DIFT-RF
    // to keep invalidaiton flags for coverage statistics.
    } else if (invtab.action[idx] == "HW_clear_array_set_reg") {
        // Clear array
        DPRINTF(Invalidation, "Clearing array\n");
        unsigned type = 0;
        fault = writeToFlagCache(FC_CLEAR_FLAG, (uint8_t *)&type, sizeof(type), ArmISA::TLB::AllowUnaligned);
        // Set invalidation flag in register file
        //thread->setIntReg((int)rd, (uint64_t)true);
        if (rd >= 0 && rd < NUM_REGS) {
          invalid_flags[rd] = true;
        }
    } else if (invtab.action[idx] == "HW_clear_array_clear_reg") {
        // Clear array
        DPRINTF(Invalidation, "Clearing array\n");
        unsigned type = 0;
        fault = writeToFlagCache(FC_CLEAR_FLAG, (uint8_t *)&type, sizeof(type), ArmISA::TLB::AllowUnaligned);
        // Clear invalidation flag in register file
        //thread->setIntReg((int)rd, (uint64_t)false);
        if (rd >= 0 && rd < NUM_REGS) {
          invalid_flags[rd] = false;
        }
    } else if (invtab.action[idx] == "HW_set_array_set_reg") {
        // Set array
        DPRINTF(Invalidation, "Setting array\n");
        unsigned type = 0;
        fault = writeToFlagCache(FC_SET_FLAG, (uint8_t *)&type, sizeof(type), ArmISA::TLB::AllowUnaligned);
        // Set invalidation flag in register file
        //thread->setIntReg((int)rd, (uint64_t)true);
        if (rd >= 0 && rd < NUM_REGS) {
          invalid_flags[rd] = true;
        }
    } else if (invtab.action[idx] == "HW_set_array_clear_reg") {
        // Set array
        DPRINTF(Invalidation, "Setting array\n");
        unsigned type = 0;
        fault = writeToFlagCache(FC_SET_FLAG, (uint8_t *)&type, sizeof(type), ArmISA::TLB::AllowUnaligned);
        // Clear invalidation flag in register file
        //thread->setIntReg((int)rd, (uint64_t)false);
        if (rd >= 0 && rd < NUM_REGS) {
          invalid_flags[rd] = false;
        }
    } else if (invtab.action[idx] == "HW_set_array_propagate_reg") {
        // Set array
        DPRINTF(Invalidation, "Setting array\n");
        unsigned type = 0;
        fault = writeToFlagCache(FC_SET_FLAG, (uint8_t *)&type, sizeof(type), ArmISA::TLB::AllowUnaligned);
        // Propagate invalidation flag in register file
        bool tinv = false;
        /*
        if (TheISA::isISAReg(rs1) && thread->readIntReg(rs1)) {
          tinv = true;
        }
        if (TheISA::isISAReg(rs2) && thread->readIntReg(rs2)) {
          tinv = true;
        }
        */
        if (rs1 >= 0 && rs1 < NUM_REGS && invalid_flags[rs1]) {
          tinv = true;
        }
        if (rs2 >= 0 && rs2 < NUM_REGS && invalid_flags[rs2]) {
          tinv = true;
        }
        //thread->setIntReg((int)rd, (uint64_t)tinv);
        if (rd >= 0 && rd < NUM_REGS) {
          invalid_flags[rd] = tinv;
        }
        // Adjust statistics if valid propagation
        if (!tinv) {
          filterstats[itp]--;
          fullstats[itp]++;
        }
    } else if (invtab.action[idx] == "HW_clear_array_propagate_reg") {
        // Clear array
        DPRINTF(Invalidation, "Clearing array\n");
        unsigned type = 0;
        fault = writeToFlagCache(FC_CLEAR_FLAG, (uint8_t *)&type, sizeof(type), ArmISA::TLB::AllowUnaligned);
        // Propagate invalidation flag in register file
        bool tinv = false;
        /*
        if (TheISA::isISAReg(rs1) && thread->readIntReg(rs1)) {
          tinv = true;
        }
        if (TheISA::isISAReg(rs2) && thread->readIntReg(rs2)) {
          tinv = true;
        }
        */
        if (rs1 >= 0 && rs1 < NUM_REGS && invalid_flags[rs1]) {
          tinv = true;
        }
        if (rs2 >= 0 && rs2 < NUM_REGS && invalid_flags[rs2]) {
          tinv = true;
        }
        //thread->setIntReg((int)rd, (uint64_t)tinv);
        if (rd >= 0 && rd < NUM_REGS) {
          invalid_flags[rd] = tinv;
        }
        // Adjust statistics if valid propagation
        if (!tinv) {
          filterstats[itp]--;
          fullstats[itp]++;
        }
    } else if (invtab.action[idx] == "HW_nop_check_invalid") {
      // Invalid check
      //if (TheISA::isISAReg(rs1) && thread->readIntReg(rs1)) {
      if (rs1 >= 0 && rs1 < NUM_REGS && invalid_flags[rs1]) {
        // Do nothing, filtered out
      // Valid check
      } else {
        // Filter handles it, but should count as non-drop in statistics
        filterstats[itp]--;
        fullstats[itp]++;
      }
    } else {
        DPRINTF(Invalidation, "No invalidation operation performed\n");
    }
    
    return fault;
}

Fault
BaseSimpleCPU::readFromFlagCache(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags)
{
    // use the CPU's statically allocated read request and packet objects
    Request *req = &data_read_req;
    req->setPhys(addr, size, flags, dataMasterId());
    // Read command
    MemCmd cmd = MemCmd::ReadReq;
    // Create packet
    PacketPtr pkt = new Packet(req, cmd);
    // Point packet to monitoring packet
    pkt->dataStatic(data);

    // Send read request
    fcPort.sendFunctional(pkt);

    // Clean up
    delete pkt;

    return NoFault;
}

Fault
BaseSimpleCPU::readFromTimer(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags)
{
    bool skip_drop = false;
    bool skip_filter = false;
    instType itp = inst_undef;
    bool entry_filtered = false;
    uint8_t intask = false;
    bool coverage_drop = false;
    bool ischeck = false;
    // read data
    long long int read_timer = 1;

    ///////////////////////////////////////////////////////
    // Return slack value
    ///////////////////////////////////////////////////////
    if (addr == TIMER_READ_SLACK) {
        // use the CPU's statically allocated read request and packet objects
        Request *req = &data_read_req;
    
        // Create request at timer location
        req->setPhys(addr, sizeof(read_timer), flags, dataMasterId());
        // Read command
        MemCmd cmd = MemCmd::ReadReq;
        // Create packet
        PacketPtr pkt = new Packet(req, cmd);
        // Point packet to data pointer
        pkt->dataStatic(&read_timer);

        // Send read request
        timerPort.sendFunctional(pkt);
        
        // Clean up
        delete pkt;

        // Print out for debug
    #ifdef DEBUG
        DPRINTF(SlackTimer, "read from timer: %d ticks, %d cycles\n", read_timer, read_timer/ticks(1));
    #endif
    
        read_timer /= ticks(1);
    ///////////////////////////////////////////////////////
    // Perform filtering + dropping
    ///////////////////////////////////////////////////////
    } else if (addr == TIMER_READ_DROP) {
        // Check if we are in a task
        readFromTimer(TIMER_TASK_PACKET, &intask, sizeof(intask), ArmISA::TLB::AllowUnaligned);

        // Pop the fifo entry
        bool pop = true;
        Fault result = writeToFifo(FIFO_NEXT, (uint8_t *)&pop, sizeof(pop), ArmISA::TLB::AllowUnaligned);
        if (result != NoFault) { return result; }
        // Read instruction type from fifo
        itp = readFifoInstType();
        // Test if it is a check packet
        ischeck = (check_load && (itp == inst_load))
                  ||(check_store && (itp == inst_store))
                  ||(check_indctrl && (itp == inst_indctrl));

        // Mark static instruction as having a check event 
        // (if not previsouly found already)
        if (print_static_coverage && ischeck) {
          // Read PC from FIFO
          Addr pc = -1;
          readFromFifo(FIFO_INSTADDR, (uint8_t *)&pc, sizeof(pc), ArmISA::TLB::AllowUnaligned);
          // Look for PC in list of all PCs that raise events
          std::list<int>::iterator findIter = std::find(pc_checked.begin(), pc_checked.end(), (int)pc);
          // If not found, add to list
          if (findIter == pc_checked.end()) {
            pc_checked.push_back((int)pc);
          }
        }

        
        // Reevaluate packet drop rate
        if (check_frequency && total_checks && ischeck && intask
            && !(total_checks % check_frequency)){
            double full_checks = ((check_load)? fullstats[inst_load].result() : 0) +
                                 ((check_store)? fullstats[inst_store].result() : 0) +
                                 ((check_indctrl)? fullstats[inst_indctrl].result() : 0);
            double all_checks = ((check_load)? dropstats[inst_load].result() + filterstats[inst_load].result() + coveragedropstats[inst_load].result() + fullstats[inst_load].result() : 0) +
                                ((check_store)? dropstats[inst_store].result() + filterstats[inst_store].result() + coveragedropstats[inst_store].result() + fullstats[inst_store].result() : 0) +
                                ((check_indctrl)? dropstats[inst_indctrl].result() + filterstats[inst_indctrl].result() + coveragedropstats[inst_indctrl].result() + fullstats[inst_indctrl].result() : 0);
            double actual_coverage = full_checks/all_checks;
            packet_drop_rate += (actual_coverage - target_coverage);
            if (packet_drop_rate > 1){ packet_drop_rate = 1; }
            if (packet_drop_rate < 0){ packet_drop_rate = 0; }
            all_packets = 1;
            full_packets = 1;
            DPRINTF(Invalidation, "Reevaluating coverage drop rate: Actual = %f, Target = %f, New drop rate = %f\n", actual_coverage, target_coverage, packet_drop_rate);
        }
        
        if (ischeck && intask){ total_checks++; }
        
        // FIXME: Prevents settag packets from being dropped. Does not work in real-time settings.
        readFromFifo(FIFO_SETTAG, (uint8_t *)&skip_drop, sizeof(skip_drop), ArmISA::TLB::AllowUnaligned);
        skip_filter = skip_drop;
        // For source dropping, only set tag operations should be dropped while
        // other operations should not be dropped. Filtering is always allowed
        // to occur.
        if (source_dropping) {
          skip_drop = !skip_drop; // Allow set tag to be dropped. Disallow non-set tag to be dropped.
          skip_filter = false;
        }
        
        if (_backtrack) {
            // calculate whether an instruction is important
            _important = backtrack();
            if (_important)
                numImportantInsts++;
        }

        if (optimal_dropping) {
            _important = !inOptimalDroppingTable();
            if (_important)
                numImportantInsts++;
        }

        // Perform filtering
        if (!skip_filter && flagcache_enabled && invtab.initialized && fptab.initialized 
            && (filtertab1.initialized || filtertab2.initialized))
        {
            DPRINTF(Invalidation, "Begin filtering\n");
            uint8_t select1 = 0;
            uint8_t select2 = 0;
            // Save address
            Addr saved_addr = 0;
            readFromFlagCache(FC_GET_ADDR, (uint8_t *)&saved_addr, sizeof(saved_addr), ArmISA::TLB::AllowUnaligned);
            // Read first flag from flag cache
            if (filtertab1.initialized){
                if (filtertab1.action[itp].size()){
                    setFlagCacheAddrFromTable(filtertab1, itp);
                    if (filtertab1.action[itp] == "cache"){
                        result = readFromFlagCache(FC_GET_FLAG_C, (uint8_t *)&select1, sizeof(select1), ArmISA::TLB::AllowUnaligned);
                    } else if (filtertab1.action[itp] == "array"){
                        result = readFromFlagCache(FC_GET_FLAG_A, (uint8_t *)&select1, sizeof(select1), ArmISA::TLB::AllowUnaligned);
                    }
                    if (result != NoFault){ warn("Read from flag cache failed @ %llx\n", curTick()); }
                    // Write back original address
                    writeToFlagCache(FC_SET_ADDR, (uint8_t *)&saved_addr, sizeof(saved_addr), ArmISA::TLB::AllowUnaligned);
                }
            }
            // Read second flag from flag cache
            if (filtertab2.initialized){
                if (filtertab2.action[itp].size()){
                    setFlagCacheAddrFromTable(filtertab2, itp);
                    if (filtertab2.action[itp] == "cache"){
                        result = readFromFlagCache(FC_GET_FLAG_C, (uint8_t *)&select2, sizeof(select2), ArmISA::TLB::AllowUnaligned);
                    } else if (filtertab2.action[itp] == "array"){
                        result = readFromFlagCache(FC_GET_FLAG_A, (uint8_t *)&select2, sizeof(select2), ArmISA::TLB::AllowUnaligned);
                    }
                    if (result != NoFault){ warn("Read from flag cache failed @ %llx\n", curTick()); }
                    // Write back original address
                    writeToFlagCache(FC_SET_ADDR, (uint8_t *)&saved_addr, sizeof(saved_addr), ArmISA::TLB::AllowUnaligned);
                }
            }
            uint16_t select = (select1 << FC_NUMBITS) | select2;
            // Get index from filter pointer table
            unsigned itidx = fptab.table[itp][select];
            DPRINTF(Invalidation, "Filter table select {%d,%d} -> %d @ %d points to %d\n", select1, select2, select, itp, itidx);
            // Perform filtering operation if one exists
            if (invtab.action[itidx].size()){
                filterstats[itp]++;
                filterdetailed[itp][select]++;
                DPRINTF(Invalidation, "Filtering fifo entry: num_filtered: %d\n", filterstats.total());
                // Filtering is enabled
                if (!emulate_filtering){
                    all_packets++;
                    if (invtab.action[itidx] == "SW"){
                        // Perform filtering in SW
                        int return_code = 2;
                        DPRINTF(Invalidation, "Using software with return code %d\n", return_code);
                        memcpy(data, &return_code, size);
                        return NoFault;
                    } else {
                        setFlagCacheAddrFromTable(invtab, itidx);
                        result = performInvalidation(itidx, itp);
                        DPRINTF(Invalidation, "Filtered in HW.\n");
                        if (_backtrack)
                            notifyTimerInvalidation(_important);
                        return (result != NoFault)? result : ReExecFault; // Stay in HW
                    }
                // Emulating filtering to determine what system without
                // filtering would be like
                } else {
                    // Mark that entry would have been filtered. Even if full
                    // monitoring is performed on these, they should not count
                    // towards valid monitoring.
                    entry_filtered = true;
                }
            }
            // No entry in filter table, packet not filtered
            DPRINTF(Invalidation, "Entry not filtered\n");
        }
    
        // Perform coverage packet dropping
        if (!skip_drop && intask && (((1.0-packet_drop_rate)*(double)all_packets) < (double)full_packets)){
            coveragedropstats[itp]++;
            coverage_drop = true;
            DPRINTF(Invalidation, "Coverage packet drop fifo entry: num_coverage_dropped: %d\n", coveragedropstats.total());
        }
      
        // use the CPU's statically allocated read request and packet objects
        Request *req = &data_read_req;
        
        // If the packet is droppable, read whether there is enough slack to
        // perform full monitoring into read_timer.
        // read_timer = 1 indicates enough slack, = 0 indicates drop.
        if (!skip_drop) {
            if (!((_backtrack && _important) || (optimal_dropping && _important))) {
                // Create request at timer location
                req->setPhys(addr, sizeof(read_timer), flags, dataMasterId());
                // Read command
                MemCmd cmd = MemCmd::ReadReq;
                // Create packet
                PacketPtr pkt = new Packet(req, cmd);
                // Point packet to data pointer
                pkt->dataStatic(&read_timer);

                // Send read request
                timerPort.sendFunctional(pkt);
                
                // Clean up
                delete pkt;
            } else {
                // Create request at timer location
                req->setPhys(TIMER_READ_DROP_IMPORTANT, sizeof(read_timer), flags, dataMasterId());
                // Read command
                MemCmd cmd = MemCmd::ReadReq;
                // Create packet
                PacketPtr pkt = new Packet(req, cmd);
                // Point packet to data pointer
                pkt->dataStatic(&read_timer);

                // Send read request
                timerPort.sendFunctional(pkt);
                
                // Clean up
                delete pkt;
            }
        }
        
        // If the entry should not have been marked as filtered, then it
        // contributes to either drop or non-drop statistics.
        if (!entry_filtered && !coverage_drop && intask){
            // Increment full statistics if we have enough slack
            if (read_timer){ fullstats[itp]++; }
            // Increment drop statistics if we don't have enough slack
            else { dropstats[itp]++; }
        }
        
        if (intask){
            if (read_timer && !coverage_drop) { 
                full_packets++;

                // ID checks so we can identify coverage across multiple runs
                if (ischeck) {
                    DPRINTF(CheckId, "Full check: %d\n", total_checks);
                    if (print_checkid) {
                      // If past last bit vector
                      if (total_checks / 64 > checkid_base) {
                        // Print out last vector
                        printf("%d,%llx\n", checkid_base, (long long unsigned int)checkid_vec);
                        // Update base
                        checkid_base = total_checks / 64;
                        // Reset bit vector
                        checkid_vec = 0;
                      } 
                      // Set corresponding bit
                      checkid_vec |= (((uint64_t)1) << (total_checks % 64));
                    }
                    
                    // Mark static instruction as being monitored
                    // (if not previously monitored already)
                    if (print_static_coverage) {
                      // Read PC from FIFO
                      Addr pc = -1;
                      readFromFifo(FIFO_INSTADDR, (uint8_t *)&pc, sizeof(pc), ArmISA::TLB::AllowUnaligned);
                      // Look for PC in list of full monitored PCs
                      std::list<int>::iterator findIter = std::find(pc_checked_full.begin(), pc_checked_full.end(), (int)pc);
                      // If not found, add to list
                      if (findIter == pc_checked_full.end()) {
                        pc_checked_full.push_back((int)pc);
                      }
                    }
                }
            }
            all_packets++;
        }
        
        // Not enough slack, perform hardware invalidation
        if (fifo_enabled && (!read_timer || coverage_drop) && flagcache_enabled && invtab.initialized){
            DPRINTF(Invalidation, "Starting hardware invalidation.\n");
            // Check if LUT says to go back to software
            if (invtab.action[itp].size() && (invtab.action[itp] != "SW")){
                setFlagCacheAddrFromTable(invtab, itp);
                Fault result = performInvalidation(itp, itp);
                DPRINTF(Invalidation, "Staying in HW.\n");
                return (result != NoFault)? result : ReExecFault; // Stay in HW
            }
        }
        
    #ifdef DEBUG
        DPRINTF(SlackTimer, "read from timer: drop? %d, numdrops: %d, numfull: %d\n", !read_timer, dropstats.total(), fullstats.total());
        DPRINTF(Invalidation, "Using software with return code %d\n", read_timer);
    #endif
    
    }
    
    memcpy(data, &read_timer, size);
    
    return NoFault;
}

/**
 * Notify the timer that an instruction is invalidated
 */
void
BaseSimpleCPU::notifyTimerInvalidation(bool important)
{
    // use the CPU's statically allocated read request and packet objects
    Request *req = &data_read_req;
    // read data
    long long int read_timer = 1;

    // If the packet is droppable, read whether there is enough slack to
    // perform full monitoring into read_timer.
    // read_timer = 1 indicates enough slack, = 0 indicates drop.
    Addr addr = important ? TIMER_READ_DROP_IMPORTANT : TIMER_READ_DROP;
    // Create request at timer location
    req->setPhys(addr, sizeof(read_timer), ArmISA::TLB::AllowUnaligned, dataMasterId());
    // Read command
    MemCmd cmd = MemCmd::ReadReq;
    // Create packet
    PacketPtr pkt = new Packet(req, cmd);
    // Point packet to data pointer
    pkt->dataStatic(&read_timer);

    // Send read request
    timerPort.sendFunctional(pkt);

    // Clean up
    delete pkt;
}

Fault
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
  
#ifdef DEBUG
    if (addr < FIFO_EMPTY){
        unsigned read_data = 0;
        unsigned read_size = size;
        if (sizeof(unsigned) < read_size){ read_size = sizeof(unsigned); } //Make sure we don't copy garbage data
        memcpy(&read_data, data, size);
        DPRINTF(Fifo, "read fifo @ %x = %x\n", addr, read_data);
    }
#endif

    return NoFault;
}

Fault
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

    // virtual -> physical address translation
    Request *req = &data_write_req;
    req->setVirt(0, mp.memAddr, size, flags, dataMasterId(), thread->pcState().instAddr());
    Fault fault = thread->dtb->translateAtomic(req, tc, BaseTLB::Write);
    assert(fault == NoFault);
    mp.physAddr = req->getPaddr();
  } else if (addr == FIFO_CUSTOM_SIZE) {
    mp.size = fed.data;
    mp.memEnd = mp.memAddr + mp.size - 1;
  } else if (addr == FIFO_NEXT){

    bool wasEmpty = fifoEmpty;

    // Pop fifo, since there is an entry
    if (!wasEmpty) {
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
    
    fifoEmpty = isFifoEmpty();

#ifdef DEBUG
    if (wasEmpty ^ fifoEmpty){
        // Print out for debug
        DPRINTF(Fifo, "Check if Fifo empty: %d\n", fifoEmpty);
    } 
#endif

    // Wait until fifo is no longer empty
    if (fifoEmpty) {
      return ReExecFault; 
    }
    
    /* Check if next FIFO packet has the done flag and
     * exit.
     */
    if (isFifoDone()) { exitSimLoop("fifo done flag received"); }
    
  }
  
  return NoFault;
}


Fault
BaseSimpleCPU::writeToTimer(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags)
{

    // Get data
    int write_data = 0;
    if (size > sizeof(write_data)) size = sizeof(write_data);
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
    
    long long int stall_length = 0;
    if (addr == TIMER_END_TASK){
        long long int slack;
        Request* req = &data_read_req;
        flags = ArmISA::TLB::AllowUnaligned;
        //size = sizeof(read_tp);
        req->setPhys(TIMER_READ_SLACK, sizeof(slack), flags, dataMasterId());
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
        
        stall_length = (long long int)fed.data*ticks(1) + slack;
        fed.data = stall_length;
    }

    //convert from cycles to ticks
    long long int write_data_ext = write_data*ticks(1);
    // Create request
    Request *timer_write_req = &data_write_req;
    //unsigned size = sizeof(write_tp);
    flags = ArmISA::TLB::AllowUnaligned;
    // set physical address
    timer_write_req->setPhys(addr, sizeof(write_data_ext), flags, dataMasterId());
    // Create write packet
    MemCmd cmd = MemCmd::WriteReq;
    PacketPtr timerpkt = new Packet(timer_write_req, cmd);
    // Set data
    // write_tp.subtaskStart = curTick();
    timerpkt->dataStatic(&write_data_ext);

    // Send read request packet on timer port
    timerPort.sendFunctional(timerpkt);

    // Clean up
    delete timerpkt;
    
    if (hard_wcet && addr == TIMER_END_TASK && stall_length < 0){
        panic("Did not meet WCET. Slack is negative: %d.\n", stall_length/ticks(1));
    }
    
    return NoFault;
}

Fault
BaseSimpleCPU::writeToFlagCache(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags)
{
    Request *req = &data_write_req;
    req->setPhys(addr, size, flags, dataMasterId());
    // Write command
    MemCmd cmd = MemCmd::WriteReq;
    // Create packet
    PacketPtr pkt = new Packet(req, cmd);
    // Point packet to monitoring packet
    pkt->dataStatic(data);

    // Send read request
    fcPort.sendFunctional(pkt);

    // Clean up
    delete pkt;

    return NoFault;
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
  } else if (if_name == "flagcache_port") {
    return fcPort;
  } else {
    return BaseCPU::getMasterPort(if_name, idx);
  }
}

template <unsigned add_size> bool
BaseSimpleCPU::InvalidationTable<add_size>::initTable(const char * file_name)
{
    const int MAX_CHARS_PER_LINE = 512;
    const int MAX_TOKENS_PER_LINE = 6; // Number of columns
    const char* const DELIMITER = " ";
    
    // create a file-reading object
    ifstream fin;
    fin.open(file_name); // open a file
    if (!fin.good()) {
        warn ("Invalidation table \"%s\" could not be opened.\n", file_name);
        return false; // return if file not found
    }
  
    unsigned line_number = 0;

    // read each line of the file
    while (!fin.eof())
    {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        fin.getline(buf, MAX_CHARS_PER_LINE);
        line_number++;

        // parse the line into blank-delimited tokens
        int n = 0; // a for-loop index

        // array to store memory addresses of the tokens in buf
        const char* token[MAX_TOKENS_PER_LINE] = {}; // initialize to 0

        // parse the line
        token[0] = strtok(buf, DELIMITER); // first token
        if (token[0] && strncmp(token[0], "//", 2)) // Skip if line is blank or commented out
        {
          instType inst_type = parseInstType(token[0]);
          int idx = inst_type;
          unsigned temp = add_size;
          if (temp > 0 && (inst_type == inst_undef)) { 
              idx = atoi(token[0]);
              if (idx != 0) { 
                idx = (idx % temp) + num_inst_types;
              }
          }
          if (idx != 0){
              for (n = 1; n < MAX_TOKENS_PER_LINE; n++)
              {
                token[n] = strtok(0, DELIMITER); // subsequent tokens
                if (!token[n]) break; // no more tokens
                if (!strcmp(token[n], "-")) token[n] = "";
                switch(n) {
                    case 1:  sel1[idx] = token[n]; break;
                    case 2:  sel2[idx] = token[n]; break;
                    case 3:  aluop[idx] = token[n]; break;
                    case 4:  constant[idx] = atoi(token[n]); break;
                    case 5:  action[idx] = token[n]; break;
                }
              }
          } else {
            warn("Unrecognized instruction type \"%s\" in %s:%d.\n", token[0], file_name, line_number);
          }
        }

    }
    
    initialized = true;
    
    return true;
}

template <unsigned add_size> void
BaseSimpleCPU::InvalidationTable<add_size>::printTable()
{
    printf("%s:\n", table_name.data());
    printf("%3s %10s %10s %6s %3s %15s\n", "idx", "sel1", "sel2", "aluop", "c", "action/loc.");
    for (int i = 0; i < size; ++i){
        printf("%2d: %10s %10s %6s %3d %15s\n", i, sel1[i].data(), sel2[i].data(), aluop[i].data(), constant[i], action[i].data());
    }
}

bool
BaseSimpleCPU::FilterPtrTable::initTable(const char * file_name)
{
    const int MAX_CHARS_PER_LINE = 512;
    const int MAX_TOKENS_PER_LINE = 1 + size; // Number of columns
    const char* const DELIMITER = " ";
    
    if (it_add_size < 1){
        warn ("Invalidation table contains no additional space for pointers.\n");
        return false; // return if we cannot add pointers
    }
    
    // create a file-reading object
    ifstream fin;
    fin.open(file_name); // open a file
    if (!fin.good()) {
        warn ("Invalidation table \"%s\" could not be opened.\n", file_name);
        return false; // return if file not found
    }
  
    unsigned line_number = 0;

    // read each line of the file
    while (!fin.eof())
    {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        fin.getline(buf, MAX_CHARS_PER_LINE);
        line_number++;

        // parse the line into blank-delimited tokens
        int n = 0; // a for-loop index

        // array to store memory addresses of the tokens in buf
        const char* token[MAX_TOKENS_PER_LINE] = {}; // initialize to 0

        // parse the line
        token[0] = strtok(buf, DELIMITER); // first token
        if (token[0] && strncmp(token[0], "//", 2)) // zero if line is blank
        {
          instType inst_type = parseInstType(token[0]);
          if (inst_type != inst_undef){
              for (n = 1; n < MAX_TOKENS_PER_LINE; n++)
              {
                token[n] = strtok(0, DELIMITER); // subsequent tokens
                if (!token[n]) break; // no more tokens
                if (!strcmp(token[n], "-")) token[n] = "";
                unsigned ptr = atoi(token[n]);
                if (ptr) { ptr = (ptr % it_add_size) + num_inst_types; }
                table[inst_type][n-1] = ptr;
              }
          } else {
            warn("Unrecognized instruction type \"%s\" in %s:%d.\n", token[0], file_name, line_number);
          }
        }

    }
    
    initialized = true;
    
    return true;
}



void
BaseSimpleCPU::FilterPtrTable::printTable()
{
    printf("%s:\n", table_name.data());
    char fmt_s[10];
    char fmt_d[10];
    sprintf (fmt_s, " %%%ds", 1+2*FC_NUMBITS);
    sprintf (fmt_d, " %%%dd", 1+2*FC_NUMBITS);
    printf("%3s", "idx");
    for (int i = 0; i < size; ++i){
        bitset<2*FC_NUMBITS> i_bits(i);
        printf(fmt_s, i_bits.to_string().c_str());
    }
    printf("\n");
    for (int i = 0; i < num_inst_types; ++i){
         printf("%2d:", i); 
         for (int j = 0; j < size; ++j) { printf(fmt_d, table[i][j]); }
         printf("\n");
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

/**
 * Backtrack to identify important instructions
 * @return Whether the current instruction is important or not
 */
bool
BaseSimpleCPU::backtrack()
{
    // actual backtrack implemented in sub-classes
    return false;
}

bool
BaseSimpleCPU::inOptimalDroppingTable()
{
    // actual backtrace implemented in sub-classes
    return false;
}
