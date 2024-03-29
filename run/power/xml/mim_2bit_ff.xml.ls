<?xml version="1.0" ?>
<component id="root" name="root">
  <component id="system" name="system">
    <!--McPAT will skip the components if number is set to 0 -->
    <param name="number_of_cores" value="1"/>
    <param name="number_of_L1Directories" value="0"/>
    <param name="number_of_L2Directories" value="0"/>
    <param name="number_of_L2s" value="0"/> <!-- This number means how many L2 clusters in each cluster there can be multiple banks/ports -->
    <param name="Private_L2" value="0"/><!--1 Private, 0 shared/coherent -->
    <param name="number_of_L3s" value="0"/> <!-- This number means how many L3 clusters -->
    <param name="number_of_NoCs" value="0"/>
    <param name="homogeneous_cores" value="1"/><!--1 means homo -->
    <param name="homogeneous_L2s" value="1"/>
    <param name="homogeneous_L1Directorys" value="1"/>
    <param name="homogeneous_L2Directorys" value="1"/>
    <param name="homogeneous_L3s" value="1"/>
    <param name="homogeneous_ccs" value="1"/><!--cache coherece hardware -->
    <param name="homogeneous_NoCs" value="1"/>
    <param name="core_tech_node" value="40"/><!-- nm -->
    <param name="target_core_clockrate" value="{1e-6/config.system.switch_cpus0.clock*1e12}"/><!--MHz -->
    <param name="temperature" value="300"/> <!-- Kelvin -->
    <param name="number_cache_levels" value="1"/>
    <param name="interconnect_projection_type" value="0"/><!--0: agressive wire technology; 1: conservative wire technology -->
    <param name="device_type" value="2"/><!--0: HP(High Performance Type); 1: LSTP(Low standby power) 2: LOP (Low Operating Power)  -->
    <param name="longer_channel_device" value="0"/><!-- 0 no use; 1 use when approperiate -->
		<param name="Embedded" value="1"/><!-- Embedded processor like ARM or general purpose processors?  -->
    <param name="machine_bits" value="32"/>
    <param name="virtual_address_width" value="32"/>
    <param name="physical_address_width" value="32"/>
    <param name="virtual_memory_page_size" value="4096"/>
    <stat name="runtime_sec" value="{stats.system.switch_cpus0.numCycles * config.system.switch_cpus0.clock / 1e12}"/>
    <stat name="total_cycles" value="{stats.system.switch_cpus0.numCycles}"/>
    <stat name="idle_cycles" value="{stats.system.switch_cpus0.num_idle_cycles}"/>
    <stat name="busy_cycles"  value="{stats.system.switch_cpus0.numCycles - stats.system.switch_cpus0.num_idle_cycles}"/>
    <!--This page size(B) is complete different from the page size in Main memo secction. this page size is the size of 
	     virtual memory from OS/Archi perspective; the page size in Main memo secction is the actuall physical line in a DRAM bank  -->
    <!-- *********************** cores ******************* -->
    <component id="system.core0" name="core0">
      <!-- Core property -->
      <param name="clock_rate" value="{1e-6/config.system.switch_cpus0.clock*1e12}"/>
      <!-- for cores with unknow timing, set to 0 to force off the opt flag -->
      <param name="opt_local" value="1"/>
      <param name="instruction_length" value="32"/>
      <param name="opcode_width" value="6"/>
      <param name="x86" value="0"/>
      <param name="micro_opcode_width" value="8"/>
      <param name="machine_type" value="1"/>
      <!-- inorder/OoO; 1 inorder; 0 OOO-->
      <param name="number_hardware_threads" value="1"/>
      <!-- number_instruction_fetch_ports(icache ports) is always 1 in single-thread processor,
	   it only may be more than one in SMT processors. BTB ports always equals to fetch ports since 
	   branch information in consective branch instructions in the same fetch group can be read out from BTB once.--> 
      <param name="fetch_width" value="1"/>
      <!-- fetch_width determins the size of cachelines of L1 cache block -->
      <param name="number_instruction_fetch_ports" value="1"/>
      <param name="decode_width" value="1"/>
      <!-- decode_width determins the number of ports of the 
	   renaming table (both RAM and CAM) scheme -->
      <param name="issue_width" value="1"/>
      <param name="peak_issue_width" value="1"/>
      <!-- issue_width determins the number of ports of Issue window and other logic 
	   as in the complexity effective proccessors paper; issue_width==dispatch_width -->
      <param name="commit_width" value="1"/>
      <!-- commit_width determins the number of ports of register files -->
      <param name="fp_issue_width" value="1"/>
      <param name="prediction_width" value="0"/>  <!-- ctorng: setting this to 0 (no branch prediction) -->
      <!-- number of branch instructions can be predicted simultannouesl-->
      <!-- Current version of McPAT does not distinguish int and floating point pipelines 
	   Theses parameters are reserved for future use.--> 
      <param name="pipelines_per_core" value="1,1"/>
      <!--integer_pipeline and floating_pipelines, if the floating_pipelines is 0, then the pipeline is shared-->
      <param name="pipeline_depth" value="1,0"/>
      <!-- pipeline depth of int and fp, if pipeline is shared, the second number is the average cycles of fp ops -->
      <!-- issue and exe unit-->
      <param name="ALU_per_core" value="1"/>
      <!-- contains an adder, a shifter, and a logical unit -->
      <param name="MUL_per_core" value="1"/>
      <!-- For MUL and Div -->
      <param name="FPU_per_core" value="1"/>		
      <!-- buffer between IF and ID stage -->
      <param name="instruction_buffer_size" value="1"/>
      <!-- buffer between ID and sche/exe stage -->
      <param name="decoded_stream_buffer_size" value="1"/>
      <param name="instruction_window_scheme" value="0"/><!-- 0 PHYREG based, 1 RSBASED-->
      <!-- McPAT support 2 types of OoO cores, RS based and physical reg based-->
      <!-- ctorng: OOO <param name="instruction_window_size" value="config.system.cpu0.numIQEntries"/>-->
      <!-- ctorng: OOO <param name="fp_instruction_window_size" value="config.system.cpu0.numIQEntries"/>-->
      <!-- the instruction issue Q as in Alpha 21264; The RS as in Intel P6 -->
      <!-- ctorng: OOO <param name="ROB_size" value="config.system.cpu0.numROBEntries"/>-->
      <!-- each in-flight instruction has an entry in ROB -->
      <!-- registers -->
      <!-- ctorng: OOO <param name="archi_Regs_IRF_size" value="32"/>		-->
      <!-- ctorng: OOO <param name="archi_Regs_FRF_size" value="32"/>-->
      <!--  if OoO processor, phy_reg number is needed for renaming logic, 
	    renaming logic is for both integer and floating point insts.  -->
      <!-- ctorng: OOO <param name="phy_Regs_IRF_size" value="config.system.cpu0.numPhysIntRegs"/>-->
      <!-- ctorng: OOO <param name="phy_Regs_FRF_size" value="config.system.cpu0.numPhysFloatRegs"/>-->
      <!-- rename logic -->
      <!-- ctorng: OOO <param name="rename_scheme" value="1"/>-->
      <!-- can be RAM based(0) or CAM based(1) rename scheme 
	   RAM-based scheme will have free list, status table;
	   CAM-based scheme have the valid bit in the data field of the CAM 
	   both RAM and CAM need RAM-based checkpoint table, checkpoint_depth=# of in_flight instructions;
	   Detailed RAT Implementation see TR -->
      <param name="register_windows_size" value="0"/>
      <!-- how many windows in the windowed register file, sun processors;
	   no register windowing is used when this number is 0 -->
      <!-- In OoO cores, loads and stores can be issued whether inorder(Pentium Pro) or (OoO)out-of-order(Alpha),
	   They will always try to exeute out-of-order though. -->
      <param name="LSU_order" value="inorder"/>
      <!-- ctorng: OOO <param name="store_buffer_size" value="config.system.cpu0.SQEntries"/>-->
      <!-- By default, in-order cores do not have load buffers -->
      <!-- ctorng: OOO <param name="load_buffer_size" value="config.system.cpu0.LQEntries"/>	-->
      <!-- number of ports refer to sustainable concurrent memory accesses --> 
      <param name="memory_ports" value="1"/>	
      <!-- max_allowed_in_flight_memo_instructions determins the # of ports of load and store buffer
	   as well as the ports of Dcache which is connected to LSU -->	
      <!-- dual-pumped Dcache can be used to save the extra read/write ports -->
      <!-- dlo: no branch predictor
      <param name="RAS_size" value="config.system.cpu0.branchPred.RASSize"/>						
      -->
      <!-- general stats, defines simulation periods;require total, idle, and busy cycles for senity check  -->
      <!-- please note: if target architecture is X86, then all the instrucions refer to (fused) micro-ops -->
      <stat name="total_instructions" value="{stats.system.switch_cpus0.committedInsts}"/>
      <stat name="int_instructions" value="{stats.system.switch_cpus0.num_int_insts}"/>
      <stat name="fp_instructions" value="{stats.system.switch_cpus0.num_fp_insts}"/>

      <!-- dlo: no branch predictor
      <stat name="branch_instructions" value="stats.system.cpu0.branchPred.condPredicted"/>
      <stat name="branch_mispredictions" value="stats.system.cpu0.branchPred.condIncorrect"/>
      <stat name="load_instructions" value="stats.system.cpu0.num_load_insts"/>
      <stat name="store_instructions" value="stats.system.cpu0.num_store_insts"/>
      <stat name="committed_instructions" value="stats.system.cpu0.committedInsts"/>
      <stat name="committed_int_instructions" value="stats.system.cpu0.num_int_insts"/>
      <stat name="committed_fp_instructions" value="stats.system.cpu0.num_fp_insts"/>
      -->

      <stat name="pipeline_duty_cycle" value="1"/><!-- lteq 1, runtime_ipc/peak_ipc; averaged for all cores if homogenous -->
      <!-- the following cycle stats are used for heterogeneouse cores only, 
	   please ignore them if homogeneouse cores -->
      <stat name="total_cycles" value="{stats.system.switch_cpus0.numCycles}"/>
      <stat name="idle_cycles" value="{stats.system.switch_cpus0.num_idle_cycles}"/>
      <stat name="busy_cycles" value="{stats.system.switch_cpus0.numCycles - stats.system.switch_cpus0.num_idle_cycles}"/>
      <!-- instruction buffer stats -->
      <!-- ROB stats, both RS and Phy based OoOs have ROB
	   performance simulator should capture the difference on accesses,
	   otherwise, McPAT has to guess based on number of commited instructions. -->
     <!-- ctorng: OOO <stat name="ROB_reads" value="stats.system.cpu0.rob.rob_reads"/>-->
     <!-- ctorng: OOO <stat name="ROB_writes" value="stats.system.cpu0.rob.rob_writes"/>-->
      <!-- RAT accesses -->
      <!-- ctorng: OOO <stat name="rename_reads" value="stats.system.cpu0.rename.int_rename_lookups"/> <!-lookup in renaming logic ->-->
      <!-- ctorng: OOO <stat name="rename_writes" value="int(stats.system.cpu0.rename.RenamedOperands * stats.system.cpu0.rename.int_rename_lookups / stats.system.cpu0.rename.RenameLookups)"/><!-update dest regs. renaming logic ->-->
      <!-- ctorng: OOO <stat name="fp_rename_reads" value="stats.system.cpu0.rename.fp_rename_lookups"/>-->
      <!-- ctorng: OOO <stat name="fp_rename_writes" value="int(stats.system.cpu0.rename.RenamedOperands * stats.system.cpu0.rename.fp_rename_lookups / stats.system.cpu0.rename.RenameLookups)"/>-->
      <!-- decode and rename stage use this, should be total ic - nop -->
      <!-- Inst window stats -->
      <!-- ctorng: OOO <stat name="inst_window_reads" value="stats.system.cpu0.iq.int_inst_queue_reads"/>-->
      <!-- ctorng: OOO <stat name="inst_window_writes" value="stats.system.cpu0.iq.int_inst_queue_writes"/>-->
      <!-- ctorng: OOO <stat name="inst_window_wakeup_accesses" value="stats.system.cpu0.iq.int_inst_queue_wakeup_accesses"/>-->
      <!-- ctorng: OOO <stat name="fp_inst_window_reads" value="stats.system.cpu0.iq.fp_inst_queue_reads"/>-->
      <!-- ctorng: OOO <stat name="fp_inst_window_writes" value="stats.system.cpu0.iq.fp_inst_queue_writes"/>-->
      <!-- ctorng: OOO <stat name="fp_inst_window_wakeup_accesses" value="stats.system.cpu0.iq.fp_inst_queue_wakeup_accesses"/>-->
      <!--  RF accesses -->
      <stat name="int_regfile_reads" value="{stats.system.switch_cpus0.num_int_register_reads}"/>
      <stat name="float_regfile_reads" value="{stats.system.switch_cpus0.num_fp_register_reads}"/>
      <stat name="int_regfile_writes" value="{stats.system.switch_cpus0.num_int_register_writes}"/>
      <stat name="float_regfile_writes" value="{stats.system.switch_cpus0.num_fp_register_writes}"/>
      <!-- accesses to the working reg -->
      <!-- ctorng: OOO <stat name="function_calls" value="stats.system.cpu0.commit.function_calls"/>-->
      <stat name="context_switches" value="{stats.system.cpu0.workload.num_syscalls}"/>
      <!-- Number of Windowes switches (number of function calls and returns)-->
      <!-- Alu stats by default, the processor has one FPU that includes the divider and 
	   multiplier. The fpu accesses should include accesses to multiplier and divider  -->
      <stat name="ialu_accesses" value="{stats.system.switch_cpus0.num_int_alu_accesses}"/>			
      <stat name="fpu_accesses" value="{stats.system.switch_cpus0.num_fp_alu_accesses}"/>
      <!--<stat name="mul_accesses" value="stats.system.cpu0.mult_div_unit.multiplies"/>-->
      <!-- ctorng: OOO <stat name="cdb_alu_accesses" value="0"/>-->
      <!-- ctorng: OOO <stat name="cdb_mul_accesses" value="0"/>-->
      <!-- ctorng: OOO <stat name="cdb_fpu_accesses" value="0"/>-->
      <!-- multiple cycle accesses should be counted multiple times, 
	   otherwise, McPAT can use internal counter for different floating point instructions 
	   to get final accesses. But that needs detailed info for floating point inst mix -->
      <!--  currently the performance simulator should 
	    make sure all the numbers are final numbers, 
	    including the explicit read/write accesses, 
	    and the implicite accesses such as replacements and etc.
	    Future versions of McPAT may be able to reason the implicite access
	    based on param and stats of last level cache
	    The same rule applies to all cache access stats too!  -->
      <!-- following is AF for max power computation. 
	   Do not change them, unless you understand them-->
      <stat name="IFU_duty_cycle" value="1"/>			
      <!-- ctorng: OOO <stat name="LSU_duty_cycle" value="1"/>-->
      <stat name="MemManU_I_duty_cycle" value="1"/>
      <stat name="MemManU_D_duty_cycle" value="1"/>
      <stat name="ALU_duty_cycle" value="1"/>
      <stat name="MUL_duty_cycle" value="1"/>
      <stat name="FPU_duty_cycle" value="1"/>
      <!-- ctorng: OOO <stat name="ALU_cdb_duty_cycle" value="1"/>-->
      <!-- ctorng: OOO <stat name="MUL_cdb_duty_cycle" value="0.3"/>-->
      <!-- ctorng: OOO <stat name="FPU_cdb_duty_cycle" value="1"/>-->
      <param name="number_of_BPT" value="0"/>
      <component id="system.core0.predictor" name="PBT">
	<!-- branch predictor; tournament predictor see Alpha implementation -->


  <!-- dlo rawr: no branch predictor
  <param name="local_predictor_size" value="config.system.cpu0.branchPred.localPredictorSize"/> <!-- ctorng rawr: NOT_SURE_IF_THIS_IS_RIGHT -->
  <!--
  <param name="local_predictor_entries" value="config.system.cpu0.branchPred.localHistoryTableSize"/>
	<param name="global_predictor_entries" value="config.system.cpu0.branchPred.globalPredictorSize"/>
	<param name="global_predictor_bits" value="config.system.cpu0.branchPred.globalCtrBits"/>
  <param name="chooser_predictor_entries" value="config.system.cpu0.branchPred.choicePredictorSize"/>
  <param name="chooser_predictor_bits" value="config.system.cpu0.branchPred.choiceCtrBits"/>
  -->


	<!-- These parameters can be combined like below in next version
	     <param name="load_predictor" value="10,3,1024"/>
	     <param name="global_predictor" value="4096,2"/>
	     <param name="predictor_chooser" value="4096,2"/>
	     -->
      </component>
      <component id="system.core0.itlb" name="itlb">
        <param name="number_entries" value="{config.system.switch_cpus0.itb.size}"/>
	<stat name="total_accesses" value="{stats.system.switch_cpus0.itb.accesses}"/>
	<stat name="total_hits" value="{stats.system.switch_cpus0.itb.hits}"/>
	<stat name="total_misses" value="{stats.system.switch_cpus0.itb.misses}"/>
	<stat name="conflicts" value="0"/><!-- TODO -->
	<!-- there is no write requests to itlb although writes happen to itlb after miss, 
	     which is actually a replacement -->
      </component>
      <component id="system.core0.icache" name="icache">
	<!-- there is no write requests to itlb although writes happen to it after miss, 
	     which is actually a replacement -->
	<param name="icache_config" value="{join(',',config.system.cpu0.icache.size,config.system.cpu0.icache.block_size,config.system.cpu0.icache.assoc,1,1,config.system.cpu0.icache.hit_latency,32,0)}"/>
	<!-- the parameters are capacity,block_width, associativity, bank, throughput w.r.t. core clock, latency w.r.t. core clock,output_width, cache policy,  -->
  <!-- ctorng: the "output_width" is not used anywhere in mcpat -->
	<!-- cache_policy;//0 no write or write-though with non-write allocate;1 write-back with write-allocate -->
	<param name="buffer_sizes" value="{join(',',config.system.cpu0.icache.mshrs,config.system.cpu0.icache.mshrs,config.system.cpu0.icache.mshrs,config.system.cpu0.icache.mshrs)}"/> <!-- ctorng rawr: NOT_SURE_IF_THIS_IS_RIGHT -->
	<!-- cache controller buffer sizes: miss_buffer_size(MSHR),fill_buffer_size,prefetch_buffer_size,wb_buffer_size--> 
  <stat name="read_accesses" value="{stats.system.cpu0.icache.ReadReq_accesses::total}"/>
	<stat name="read_hits" value="{stats.system.cpu0.icache.ReadReq_hits::total}"/>
	<stat name="read_misses" value="{stats.system.cpu0.icache.ReadReq_misses::total}"/>
	<stat name="conflicts" value="{stats.system.cpu0.icache.replacements}"/>				
      </component>
      <component id="system.core0.dtlb" name="dtlb">
	<param name="number_entries" value="{config.system.switch_cpus0.dtb.size}"/><!--dual threads-->
	<stat name="total_accesses" value="{stats.system.switch_cpus0.dtb.accesses}"/>
	<stat name="total_hits" value="{stats.system.switch_cpus0.dtb.hits}"/>
	<stat name="total_misses" value="{stats.system.switch_cpus0.dtb.misses}"/>
	<stat name="conflicts" value="0"/><!-- TODO -->
      </component>
      <component id="system.core0.dcache" name="dcache">
	<!-- all the buffer related are optional -->
  <param name="dcache_config" value="{join(',',config.system.cpu0.dcache.size,config.system.cpu0.dcache.block_size,config.system.cpu0.dcache.assoc,1,1,config.system.cpu0.dcache.hit_latency,32,1)}"/>
	<param name="buffer_sizes" value="{join(',',config.system.cpu0.dcache.mshrs,config.system.cpu0.dcache.mshrs,config.system.cpu0.dcache.mshrs,config.system.cpu0.dcache.mshrs)}"/> <!-- ctorng rawr: NOT_SURE_IF_THIS_IS_RIGHT -->
	<!-- cache controller buffer sizes: miss_buffer_size(MSHR),fill_buffer_size,prefetch_buffer_size,wb_buffer_size-->	
	<stat name="read_accesses" value="{stats.system.cpu0.dcache.ReadReq_accesses::total}"/>
	<stat name="write_accesses" value="{stats.system.cpu0.dcache.WriteReq_accesses::total}"/>
	<stat name="read_hits" value="{stats.system.cpu0.dcache.ReadReq_hits::total}"/>
	<stat name="write_hits" value="{stats.system.cpu0.dcache.WriteReq_hits::total}"/>
	<stat name="read_misses" value="{stats.system.cpu0.dcache.ReadReq_misses::total}"/>
	<stat name="write_misses" value="{stats.system.cpu0.dcache.WriteReq_misses::total}"/>
	<stat name="conflicts" value="{stats.system.cpu0.dcache.replacements}"/>	
      </component>

 			<component id="system.core0.metadata" name="MIM">
        <!-- Whether MIM exists/enabled -->
        <param name="MIM_exist" value="1"/>
        <!-- Whether MIT (non-memory backed invalidation table) exists -->
        <param name="MIT_exist" value="0"/>
        <!-- Whether MIC (memory backed invalidation cache) exists -->
        <param name="MIC_exist" value="1"/>
        <!-- Flag bit width -->
        <param name="flag_width" value="2"/>
        <!--			<param name="mcache_config" value="2048,64,2,1,10,10,32,1"/>-->
  <param name="mcache_config" value="{join(',',config.system.cpu5.dcache.size,config.system.cpu5.dcache.block_size,config.system.cpu5.dcache.assoc,1,1,config.system.cpu5.dcache.hit_latency,32,1)}"/>
				<!-- the parameters are capacity,block_width, associativity, bank, throughput w.r.t. core clock, latency w.r.t. core clock,output_width, cache policy,  -->
				<!-- cache_policy;//0 no write or write-though with non-write allocate;1 write-back with write-allocate -->
				<param name="buffer_sizes" value="4, 4, 4, 4"/>
				<!-- cache controller buffer sizes: miss_buffer_size(MSHR),fill_buffer_size,prefetch_buffer_size,wb_buffer_size--> 
        <param name="store_buffer_size" value="0"/>
        <param name="load_buffer_size" value="0"/>	
        <!-- RegFU -->
        <stat name="regfile_reads" value="{stats.system.switch_cpus5.flagRegLoads}"/> 
        <stat name="regfile_writes" value="{stats.system.switch_cpus5.flagRegStores}"/> 
        <!-- LoadStoreU -->
        <stat name="LSU_duty_cycle" value="{stats.system.cpu5.dcache.overall_accesses::total/stats.system.switch_cpus0.numCycles}"/>
        <stat name="read_accesses" value="{stats.system.switch_cpus5.flagCacheLoads}"/>
        <stat name="write_accesses" value="{stats.system.switch_cpus5.flagCacheStores}"/>
        <stat name="read_misses" value="{stats.system.cpu5.dcache.ReadReq_misses::total}"/>
        <stat name="write_misses" value="{stats.system.cpu5.dcache.overall_misses::total - stats.system.cpu5.dcache.ReadReq_misses::total}"/>
        <stat name="loads" value="{stats.system.switch_cpus5.flagCacheLoads}"/>
        <stat name="stores" value="{stats.system.switch_cpus5.flagCacheStores}"/>
        <!-- ConfigTable -->
        <stat name="CT_duty_cycle" value="{stats.system.switch_cpus5.drops::total / stats.system.switch_cpus0.numCycles}"/>	
        <stat name="CT_reads" value="{stats.system.switch_cpus5.drops::total}"/>	
        <!-- FunctionalUnit -->
        <stat name="ALU_duty_cycle" value="{(stats.system.switch_cpus5.drops::total + stats.system.switch_cpus5.filtered::LOAD + stats.system.switch_cpus5.filtered::STORE + stats.system.switch_cpus5.filtered::INTALU) / stats.system.switch_cpus0.numCycles}"/>	
        <stat name="alu_accesses" value="{(stats.system.switch_cpus5.drops::total + stats.system.switch_cpus5.filtered::LOAD + stats.system.switch_cpus5.filtered::STORE + stats.system.switch_cpus5.filtered::INTALU)}"/>			
      </component>

 			<component id="system.core0.metadata" name="MFM">
        <!-- FunctionalUnit -->
        <stat name="ALU_duty_cycle" value="{(stats.system.switch_cpus5.filtered::total + stats.system.switch_cpus5.drop::total + stats.system.switch_cpus5.non_drops::total) / stats.system.switch_cpus0.numCycles}"/>	
        <stat name="alu_accesses" value="{stats.system.switch_cpus5.filtered::INTALU}"/>			
        <!-- ConfigTable -->
				<stat name="CT_duty_cycle" value="{(stats.system.switch_cpus5.filtered::total + stats.system.switch_cpus5.drop::total + stats.system.switch_cpus5.non_drops::total) / stats.system.switch_cpus0.numCycles}"/>	
				<stat name="CT_reads" value="{stats.system.switch_cpus5.filtered::total + stats.system.switch_cpus5.drops::total + stats.system.switch_cpus5.non_drops::total}"/>	
        <!-- FilterLookupTable -->
        <stat name="FLT_duty_cycle" value="{stats.system.switch_cpus5.filtered::total / stats.system.switch_cpus0.numCycles}"/>	
        <stat name="FLT_reads" value="{stats.system.switch_cpus5.filtered::total}"/>	
      </component>

      <param name="number_of_BTB" value="0"/>
      <component id="system.core0.BTB" name="BTB">
	<!-- all the buffer related are optional -->
  <param name="BTB_config" value="6144,4,2,1, 1,3"/> <!--48Kbits --> <!-- ctorng rawr: NOT_SURE_IF_THIS_IS_RIGHT -->
	<!-- the parameters are capacity,block_width,associativity,bank, throughput w.r.t. core clock, latency w.r.t. core clock,-->
  <!-- dlo: no branch predictor
  <stat name="read_accesses" value="stats.system.cpu0.branchPred.BTBLookups"/> <!--See IFU code for guideline -->
  <!--<stat name="write_accesses" value="stats.system.cpu0.branchPred.BTBLookups - stats.system.cpu0.branchPred.BTBHits"/> <!-- ctorng rawr: NOT_SURE_IF_THIS_IS_RIGHT -->
      </component>
    </component>
    <!-- This block is ignored because number_of_L1Directories = 0 but it still has to be here-->
    <!-- ctorng: also ignoring -->
    <component id="system.L1Directory0" name="L1Directory0">
	<param name="Directory_type" value="0"/>
	<!--0 cam based shadowed tag. 1 directory cache -->	
	<param name="Dir_config" value="4096,2,0,1,100,100, 8"/>
	<!-- the parameters are capacity,block_width, associativity,bank, throughput w.r.t. core clock, latency w.r.t. core clock,-->
	<param name="buffer_sizes" value="8, 8, 8, 8"/>	
	<!-- all the buffer related are optional -->
	<param name="clockrate" value="3400"/>
	<param name="ports" value="1,1,1"/>
	<!-- number of r, w, and rw search ports -->
	<param name="device_type" value="0"/>
	<!-- altough there are multiple access types, 
	     Performance simulator needs to cast them into reads or writes
	     e.g. the invalidates can be considered as writes -->
	<stat name="read_accesses" value="800000"/>
	<stat name="write_accesses" value="27276"/>
	<stat name="read_misses" value="1632"/>
	<stat name="write_misses" value="183"/>
	<stat name="conflicts" value="20"/>	
    </component>
    <!-- This block is ignored because number_of_L2Directories = 0 but it still has to be here-->
    <!-- ctorng: also ignoring -->
    <component id="system.L2Directory0" name="L2Directory0">
      <param name="Directory_type" value="0"/>
      <!--0 cam based shadowed tag. 1 directory cache -->	
      <param name="Dir_config" value="512,4,0,1,1, 1"/>
      <!-- the parameters are capacity,block_width, associativity,bank, throughput w.r.t. core clock, latency w.r.t. core clock,-->
      <param name="buffer_sizes" value="16, 16, 16, 16"/>	
      <!-- all the buffer related are optional -->
      <param name="clockrate" value="1200"/>
      <param name="ports" value="1,1,1"/>
      <!-- number of r, w, and rw search ports -->
      <param name="device_type" value="0"/>
      <!-- altough there are multiple access types, 
	   Performance simulator needs to cast them into reads or writes
	   e.g. the invalidates can be considered as writes -->
      <stat name="read_accesses" value="58824"/>
      <stat name="write_accesses" value="27276"/>
      <stat name="read_misses" value="1632"/>
      <stat name="write_misses" value="183"/>
      <stat name="conflicts" value="100"/>
    </component>
    <!-- ctorng rawr: CHANGE_IF_ADDING_L2 ignoring because there is no l2 -->
    <component id="system.L20" name="L20">
      <!-- all the buffer related are optional -->
      <param name="L2_config" value="1048576,32, 8, 8, 8, 23, 32, 1"/>
      <!-- the parameters are capacity,block_width, associativity, bank, throughput w.r.t. core clock, latency w.r.t. core clock,output_width, cache policy -->
				<param name="buffer_sizes" value="16, 16, 16, 16"/>
      <!-- cache controller buffer sizes: miss_buffer_size(MSHR),fill_buffer_size,prefetch_buffer_size,wb_buffer_size-->	
			<param name="clockrate" value="2000"/>
			<param name="ports" value="1,1,1"/>
      <!-- number of r, w, and rw ports -->
      <param name="device_type" value="0"/>
      <stat name="read_accesses" value="0"/>
      <stat name="write_accesses" value="0"/>
      <stat name="read_misses" value="0"/>
      <stat name="write_misses" value="0"/>
      <stat name="conflicts" value="0"/>	
      <stat name="duty_cycle" value="1.0"/>	
    </component>
    
    <!--**********************************************************************-->
    <!-- This block is ignored because number_of_L3s = 0 but you must have it here anyway -->
    <!-- ctorng: also ignoring -->
    <component id="system.L30" name="L30">
      <param name="L3_config" value="16777216,64,16, 16, 16, 100,1"/>
      <!-- the parameters are capacity,block_width, associativity,bank, throughput w.r.t. core clock, latency w.r.t. core clock,-->
      <param name="clockrate" value="850"/>
      <param name="ports" value="1,1,1"/>
      <!-- number of r, w, and rw ports -->
      <param name="device_type" value="0"/>
      <param name="buffer_sizes" value="16, 16, 16, 16"/>
      <!-- cache controller buffer sizes: miss_buffer_size(MSHR),fill_buffer_size,prefetch_buffer_size,wb_buffer_size-->	
      <stat name="read_accesses" value="11824"/>
      <stat name="write_accesses" value="11276"/>
      <stat name="read_misses" value="1632"/>
      <stat name="write_misses" value="183"/>
      <stat name="conflicts" value="0"/>	
      <stat name="duty_cycle" value="1.0"/>	
    </component>
    <!--**********************************************************************-->
    <!-- ctorng rawr: NOT_SURE_WHAT_TO_DO_ABOUT_NOC -->
    <component id="system.NoC0" name="noc0">
      <param name="clockrate" value="1200"/>
      <param name="type" value="0"/>
      <!--0:bus, 1:NoC , for bus no matter how many nodes sharing the bus
		 at each time only one node can send req -->
      <param name="horizontal_nodes" value="1"/>
      <param name="vertical_nodes" value="1"/>
      <param name="has_global_link" value="1"/>
      <!-- 1 has global link, 0 does not have global link -->
      <param name="link_throughput" value="1"/><!--w.r.t clock -->
      <param name="link_latency" value="1"/><!--w.r.t clock -->
      <!-- througput >= latency -->
      <!-- Router architecture -->
      <param name="input_ports" value="8"/>
      <param name="output_ports" value="7"/>
      <!-- For bus the I/O ports should be 1 -->
      <param name="virtual_channel_per_port" value="2"/>
      <param name="input_buffer_entries_per_vc" value="128"/>
      <param name="flit_bits" value="40"/>
      <param name="chip_coverage" value="1"/>
      <!-- When multiple NOC present, one NOC will cover part of the whole chip. 
	   chip_coverage <=1 -->
	   <param name="link_routing_over_percentage" value="1.0"/>
	   <!-- Links can route over other components or occupy whole area.
		by default, 50% of the NoC global links routes over other 
		components -->
     <stat name="total_accesses" value="0"/> <!-- ctorng rawr: changed this to 0.... -->
	   <!-- This is the number of total accesses within the whole network not for each router -->
	   <stat name="duty_cycle" value="1"/>
    </component>		
    <!--**********************************************************************-->
    <component id="system.mem" name="mem">
      <!-- Main memory property -->
      <param name="mem_tech_node" value="40"/>
      <param name="device_clock" value="{1e-6/(config.system.physmem.latency * 1e-12)}"/><!--MHz, this is clock rate of the actual memory device, not the FSB -->
      <param name="peak_transfer_rate" value="12800"/><!--MB/S--> <!-- ctorng rawr: NOT_SURE_IF_THIS_IS_RIGHT -->
      <param name="internal_prefetch_of_DRAM_chip" value="4"/>
      <!-- 2 for DDR, 4 for DDR2, 8 for DDR3...-->
      <!-- the device clock, peak_transfer_rate, and the internal prefetch decide the DIMM property -->
      <!-- above numbers can be easily found from Wikipedia -->
      <param name="capacity_per_channel" value="4096"/> <!-- MB -->
      <!-- capacity_per_Dram_chip=capacity_per_channel/number_of_dimms/number_ranks/Dram_chips_per_rank
	   Current McPAT assumes single DIMMs are used.--> 		
      <param name="number_ranks" value="2"/>
      <param name="num_banks_of_DRAM_chip" value="8"/>			
      <param name="Block_width_of_DRAM_chip" value="64"/> <!-- B -->
      <param name="output_width_of_DRAM_chip" value="8"/>
      <!--number of Dram_chips_per_rank= 72/output_width_of_DRAM_chip-->
      <!--number of Dram_chips_per_rank= 72/output_width_of_DRAM_chip-->
      <param name="page_size_of_DRAM_chip" value="8"/> <!-- 8 or 16 -->
      <param name="burstlength_of_DRAM_chip" value="8"/>
      <stat name="memory_accesses" value="{stats.system.cpu0.icache.overall_misses::total + stats.system.cpu0.dcache.overall_misses::total}"/>
      <stat name="memory_reads" value="{stats.system.cpu0.icache.ReadReq_misses::total + stats.system.cpu0.dcache.ReadReq_misses::total}"/>
      <stat name="memory_writes" value="{stats.system.cpu0.icache.WriteReq_misses::total + stats.system.cpu0.dcache.WriteReq_misses::total}"/>
    </component>
    <component id="system.mc" name="mc">
      <!-- Memeory controllers are for DDR(2,3...) DIMMs -->
      <!-- current version of McPAT uses published values for base parameters of memory controller
	   improvments on MC will be added in later versions. -->
      <param name="type" value="0"/> <!-- 1: low power; 0 high performance -->
      <param name="mc_clock" value="800"/><!--MHz-->
      <param name="peak_transfer_rate" value="1600"/><!--MB/S-->
      <param name="block_size" value="16"/><!--B-->
      <param name="number_mcs" value="2"/>
      <!-- current McPAT only supports homogeneous memory controllers -->
      <param name="memory_channels_per_mc" value="2"/>
      <param name="number_ranks" value="2"/>
      <param name="withPHY" value="0"/>
      <!-- # of ranks of each channel-->
      <param name="req_window_size_per_channel" value="32"/>
      <param name="IO_buffer_size_per_channel" value="32"/>
      <param name="databus_width" value="32"/>
      <param name="addressbus_width" value="32"/>
      <!-- McPAT will add the control bus width to the addressbus width automatically -->
      <stat name="memory_accesses" value="{stats.system.cpu0.icache.overall_misses::total + stats.system.cpu0.dcache.overall_misses::total}"/>
      <stat name="memory_reads" value="{stats.system.cpu0.icache.ReadReq_misses::total + stats.system.cpu0.dcache.ReadReq_misses::total}"/>
      <stat name="memory_writes" value="{stats.system.cpu0.icache.WriteReq_misses::total + stats.system.cpu0.dcache.WriteReq_misses::total}"/>
      <!-- McPAT does not track individual mc, instead, it takes the total accesses and calculate 
	   the average power per MC or per channel. This is sufficent for most application. 
	   Further trackdown can be easily added in later versions. -->  			
    </component>
    <!--**********************************************************************-->
    <component id="system.niu" name="niu">
      <!-- On chip 10Gb Ethernet NIC, including XAUI Phy and MAC controller  -->
      <!-- For a minimum IP packet size of 84B at 10Gb/s, a new packet arrives every 67.2ns. 
	   the low bound of clock rate of a 10Gb MAC is 150Mhz -->
      <param name="type" value="0"/> <!-- 1: low power; 0 high performance -->
      <param name="clockrate" value="350"/>
      <param name="number_units" value="0"/> <!-- unlike PCIe and memory controllers, each Ethernet controller only have one port -->
      <stat name="duty_cycle" value="1.0"/> <!-- achievable max load lteq 1.0 -->
      <stat name="total_load_perc" value="0.7"/> <!-- ratio of total achived load to total achivable bandwidth  -->
      <!-- McPAT does not track individual nic, instead, it takes the total accesses and calculate 
	   the average power per nic or per channel. This is sufficent for most application. -->  			
    </component>
    <!--**********************************************************************-->
    <component id="system.pcie" name="pcie">
      <!-- On chip PCIe controller, including Phy-->
      <!-- For a minimum PCIe packet size of 84B at 8Gb/s per lane (PCIe 3.0), a new packet arrives every 84ns. 
	   the low bound of clock rate of a PCIe per lane logic is 120Mhz -->
      <param name="type" value="0"/> <!-- 1: low power; 0 high performance -->
      <param name="withPHY" value="1"/>
      <param name="clockrate" value="350"/>
      <param name="number_units" value="0"/>
      <param name="num_channels" value="8"/> <!-- 2 ,4 ,8 ,16 ,32 -->
      <stat name="duty_cycle" value="1.0"/> <!-- achievable max load lteq 1.0 -->
      <stat name="total_load_perc" value="0.7"/> <!-- Percentage of total achived load to total achivable bandwidth  -->
      <!-- McPAT does not track individual pcie controllers, instead, it takes the total accesses and calculate 
	   the average power per pcie controller or per channel. This is sufficent for most application. -->  			
    </component>
    <!--**********************************************************************-->
    <component id="system.flashc" name="flashc">
      <param name="number_flashcs" value="0"/>
      <param name="type" value="1"/> <!-- 1: low power; 0 high performance -->
      <param name="withPHY" value="1"/>
      <param name="peak_transfer_rate" value="200"/><!--Per controller sustainable reak rate MB/S -->
      <stat name="duty_cycle" value="1.0"/> <!-- achievable max load lteq 1.0 -->
      <stat name="total_load_perc" value="0.7"/> <!-- Percentage of total achived load to total achivable bandwidth  -->
      <!-- McPAT does not track individual flash controller, instead, it takes the total accesses and calculate 
	   the average power per fc or per channel. This is sufficent for most application -->  			
    </component>
    <!--**********************************************************************-->

  </component>
</component>
