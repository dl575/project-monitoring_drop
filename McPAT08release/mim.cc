
#include "mim.h"
  
MIM_LoadStoreU::MIM_LoadStoreU(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_,bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 LSQ(0),
 exist(exist_)
{
	  if (!exist) return;
	  int  idx, tag, data, size, line, assoc, banks;
	  bool debug= false;
	  int ldst_opcode = XML->sys.core[ithCore].opcode_width;//16;

	  clockRate = coredynp.clockRate;
	  executionTime = coredynp.executionTime;
	  cache_p = (Cache_policy)XML->sys.core[ithCore].MIM.mcache_config[7];

	  interface_ip.num_search_ports    = XML->sys.core[ithCore].memory_ports;
	  interface_ip.is_cache			   = true;
	  interface_ip.pure_cam            = false;
	  interface_ip.pure_ram            = false;
	  //Dcache
	  size                             = (int)XML->sys.core[ithCore].MIM.mcache_config[0];
	  line                             = (int)XML->sys.core[ithCore].MIM.mcache_config[1];
	  assoc                            = (int)XML->sys.core[ithCore].MIM.mcache_config[2];
	  banks                            = (int)XML->sys.core[ithCore].MIM.mcache_config[3];
	  idx    					 	   = debug?9:int(ceil(log2(size/line/assoc)));
	  tag							   = debug?51:XML->sys.physical_address_width-idx-int(ceil(log2(line))) + EXTRA_TAG_BITS;
	  interface_ip.specific_tag        = 1;
	  interface_ip.tag_w               = tag;
    /*
	  interface_ip.cache_sz            = debug?32768:(int)XML->sys.core[ithCore].dcache.dcache_config[0];
	  interface_ip.line_sz             = debug?64:(int)XML->sys.core[ithCore].dcache.dcache_config[1];
	  interface_ip.assoc               = debug?8:(int)XML->sys.core[ithCore].dcache.dcache_config[2];
	  interface_ip.nbanks              = debug?1:(int)XML->sys.core[ithCore].dcache.dcache_config[3];
    */
    interface_ip.cache_sz = size;
    interface_ip.line_sz = line;
    interface_ip.assoc = assoc;
    interface_ip.nbanks = banks;
    // Bus width in bits
	  interface_ip.out_w               = interface_ip.line_sz*8;
    // Normal access mode
	  interface_ip.access_mode         = 0;
    // Using same througput and latency as dcache
	  interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].MIM.mcache_config[4]/clockRate;
	  interface_ip.latency             = debug?3.0/clockRate:XML->sys.core[ithCore].MIM.mcache_config[5]/clockRate;
	  interface_ip.is_cache			 = true;
	  interface_ip.obj_func_dyn_energy = 0;
	  interface_ip.obj_func_dyn_power  = 0;
	  interface_ip.obj_func_leak_power = 0;
	  interface_ip.obj_func_cycle_t    = 1;
	  interface_ip.num_rw_ports    = debug?1:XML->sys.core[ithCore].memory_ports;//usually In-order has 1 and OOO has 2 at least.
	  interface_ip.num_rd_ports    = 0;
	  interface_ip.num_wr_ports    = 0;
	  interface_ip.num_se_rd_ports = 0;
	  dcache.caches = new ArrayST(&interface_ip, "Metadata Invalidation Cache", Core_device, coredynp.opt_local, coredynp.core_ty);
	  dcache.area.set_area(dcache.area.get_area()+ dcache.caches->local_result.area);
	  area.set_area(area.get_area()+ dcache.caches->local_result.area);
	  //output_data_csv(dcache.caches.local_result);

	  //dCache controllers
    // Using same buffer sizes as dcache
	  //miss buffer
	  tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
	  data							   = (XML->sys.physical_address_width) + int(ceil(log2(size/line))) + dcache.caches->l_ip.line_sz*8;
	  interface_ip.specific_tag        = 1;
	  interface_ip.tag_w               = tag;
	  interface_ip.line_sz             = int(ceil(data/8.0));//int(ceil(pow(2.0,ceil(log2(data)))/8.0));
	  interface_ip.cache_sz            = XML->sys.core[ithCore].MIM.buffer_sizes[0]*interface_ip.line_sz;
	  interface_ip.assoc               = 0;
	  interface_ip.nbanks              = 1;
	  interface_ip.out_w               = interface_ip.line_sz*8;
	  interface_ip.access_mode         = 2;
	  interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].MIM.mcache_config[4]/clockRate;
	  interface_ip.latency             = debug?1.0/clockRate:XML->sys.core[ithCore].MIM.mcache_config[5]/clockRate;
	  interface_ip.obj_func_dyn_energy = 0;
	  interface_ip.obj_func_dyn_power  = 0;
	  interface_ip.obj_func_leak_power = 0;
	  interface_ip.obj_func_cycle_t    = 1;
	  interface_ip.num_rw_ports    = debug?1:XML->sys.core[ithCore].memory_ports;;
	  interface_ip.num_rd_ports    = 0;
	  interface_ip.num_wr_ports    = 0;
	  interface_ip.num_se_rd_ports = 0;
	  dcache.missb = new ArrayST(&interface_ip, "MICMissBuffer", Core_device, coredynp.opt_local, coredynp.core_ty);
	  dcache.area.set_area(dcache.area.get_area()+ dcache.missb->local_result.area);
	  area.set_area(area.get_area()+ dcache.missb->local_result.area);
	  //output_data_csv(dcache.missb.local_result);

	  //fill buffer
	  tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
	  data							   = dcache.caches->l_ip.line_sz;
	  interface_ip.specific_tag        = 1;
	  interface_ip.tag_w               = tag;
	  interface_ip.line_sz             = data;//int(pow(2.0,ceil(log2(data))));
	  interface_ip.cache_sz            = data*XML->sys.core[ithCore].MIM.buffer_sizes[1];
	  interface_ip.assoc               = 0;
	  interface_ip.nbanks              = 1;
	  interface_ip.out_w               = interface_ip.line_sz*8;
	  interface_ip.access_mode         = 2;
	  interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].MIM.mcache_config[4]/clockRate;
	  interface_ip.latency             = debug?1.0/clockRate:XML->sys.core[ithCore].MIM.mcache_config[5]/clockRate;
	  interface_ip.obj_func_dyn_energy = 0;
	  interface_ip.obj_func_dyn_power  = 0;
	  interface_ip.obj_func_leak_power = 0;
	  interface_ip.obj_func_cycle_t    = 1;
	  interface_ip.num_rw_ports    = debug?1:XML->sys.core[ithCore].memory_ports;;
	  interface_ip.num_rd_ports    = 0;
	  interface_ip.num_wr_ports    = 0;
	  interface_ip.num_se_rd_ports = 0;
	  dcache.ifb = new ArrayST(&interface_ip, "MICFillBuffer", Core_device, coredynp.opt_local, coredynp.core_ty);
	  dcache.area.set_area(dcache.area.get_area()+ dcache.ifb->local_result.area);
	  area.set_area(area.get_area()+ dcache.ifb->local_result.area);
	  //output_data_csv(dcache.ifb.local_result);

	  //prefetch buffer
	  tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;//check with previous entries to decide wthether to merge.
	  data							   = dcache.caches->l_ip.line_sz;//separate queue to prevent from cache polution.
	  interface_ip.specific_tag        = 1;
	  interface_ip.tag_w               = tag;
	  interface_ip.line_sz             = data;//int(pow(2.0,ceil(log2(data))));
	  interface_ip.cache_sz            = XML->sys.core[ithCore].MIM.buffer_sizes[2]*interface_ip.line_sz;
	  interface_ip.assoc               = 0;
	  interface_ip.nbanks              = 1;
	  interface_ip.out_w               = interface_ip.line_sz*8;
	  interface_ip.access_mode         = 2;
	  interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].MIM.mcache_config[4]/clockRate;
	  interface_ip.latency             = debug?1.0/clockRate:XML->sys.core[ithCore].MIM.mcache_config[5]/clockRate;
	  interface_ip.obj_func_dyn_energy = 0;
	  interface_ip.obj_func_dyn_power  = 0;
	  interface_ip.obj_func_leak_power = 0;
	  interface_ip.obj_func_cycle_t    = 1;
	  interface_ip.num_rw_ports    = debug?1:XML->sys.core[ithCore].memory_ports;;
	  interface_ip.num_rd_ports    = 0;
	  interface_ip.num_wr_ports    = 0;
	  interface_ip.num_se_rd_ports = 0;
	  dcache.prefetchb = new ArrayST(&interface_ip, "dcacheprefetchBuffer", Core_device, coredynp.opt_local, coredynp.core_ty);
	  dcache.area.set_area(dcache.area.get_area()+ dcache.prefetchb->local_result.area);
	  area.set_area(area.get_area()+ dcache.prefetchb->local_result.area);
	  //output_data_csv(dcache.prefetchb.local_result);

	  //WBB

	  if (cache_p==Write_back)
	  {
		  tag							   = XML->sys.physical_address_width + EXTRA_TAG_BITS;
		  data							   = dcache.caches->l_ip.line_sz;
		  interface_ip.specific_tag        = 1;
		  interface_ip.tag_w               = tag;
		  interface_ip.line_sz             = data;
		  interface_ip.cache_sz            = XML->sys.core[ithCore].MIM.buffer_sizes[3]*interface_ip.line_sz;
		  interface_ip.assoc               = 0;
		  interface_ip.nbanks              = 1;
		  interface_ip.out_w               = interface_ip.line_sz*8;
		  interface_ip.access_mode         = 2;
		  interface_ip.throughput          = debug?1.0/clockRate:XML->sys.core[ithCore].MIM.mcache_config[4]/clockRate;
		  interface_ip.latency             = debug?1.0/clockRate:XML->sys.core[ithCore].MIM.mcache_config[5]/clockRate;
		  interface_ip.obj_func_dyn_energy = 0;
		  interface_ip.obj_func_dyn_power  = 0;
		  interface_ip.obj_func_leak_power = 0;
		  interface_ip.obj_func_cycle_t    = 1;
		  interface_ip.num_rw_ports    = XML->sys.core[ithCore].memory_ports;
		  interface_ip.num_rd_ports    = 0;
		  interface_ip.num_wr_ports    = 0;
		  interface_ip.num_se_rd_ports = 0;
		  dcache.wbb = new ArrayST(&interface_ip, "dcacheWBB", Core_device, coredynp.opt_local, coredynp.core_ty);
		  dcache.area.set_area(dcache.area.get_area()+ dcache.wbb->local_result.area);
		  area.set_area(area.get_area()+ dcache.wbb->local_result.area);
		  //output_data_csv(dcache.wbb.local_result);
	  }

	  /*
	   * LSU--in-order processors do not have separate load queue: unified lsq
	   * partitioned among threads
	   * it is actually the store queue but for inorder processors it serves as both loadQ and StoreQ
	   */
	  tag							   = ldst_opcode+XML->sys.virtual_address_width +int(ceil(log2(XML->sys.core[ithCore].number_hardware_threads))) + EXTRA_TAG_BITS;
	  data							   = XML->sys.machine_bits;
	  interface_ip.is_cache			   = true;
	  interface_ip.line_sz             = int(ceil(data/32.0))*4;
	  interface_ip.specific_tag        = 1;
	  interface_ip.tag_w               = tag;
	  interface_ip.cache_sz            = XML->sys.core[ithCore].MIM.store_buffer_size*interface_ip.line_sz*XML->sys.core[ithCore].number_hardware_threads;
	  interface_ip.assoc               = 0;
	  interface_ip.nbanks              = 1;
	  interface_ip.out_w               = interface_ip.line_sz*8;
	  interface_ip.access_mode         = 1;
	  interface_ip.throughput          = 1.0/clockRate;
	  interface_ip.latency             = 1.0/clockRate;
	  interface_ip.obj_func_dyn_energy = 0;
	  interface_ip.obj_func_dyn_power  = 0;
	  interface_ip.obj_func_leak_power = 0;
	  interface_ip.obj_func_cycle_t    = 1;
	  interface_ip.num_rw_ports        = 0;
	  interface_ip.num_rd_ports        = XML->sys.core[ithCore].memory_ports;
	  interface_ip.num_wr_ports        = XML->sys.core[ithCore].memory_ports;
	  interface_ip.num_se_rd_ports     = 0;
	  interface_ip.num_search_ports    =XML->sys.core[ithCore].memory_ports;
	  LSQ = new ArrayST(&interface_ip, "Load(Store)Queue", Core_device, coredynp.opt_local, coredynp.core_ty);
	  LSQ->area.set_area(LSQ->area.get_area()+ LSQ->local_result.area);
	  area.set_area(area.get_area()+ LSQ->local_result.area);
	  area.set_area(area.get_area()*cdb_overhead);
	  //output_data_csv(LSQ.LSQ.local_result);
	  lsq_height=LSQ->local_result.cache_ht*sqrt(cdb_overhead);/*XML->sys.core[ithCore].number_hardware_threads*/

	  if ((coredynp.core_ty==OOO) && (XML->sys.core[ithCore].MIM.load_buffer_size >0))
	  {
		  interface_ip.line_sz             = int(ceil(data/32.0))*4;
		  interface_ip.specific_tag        = 1;
		  interface_ip.tag_w               = tag;
		  interface_ip.cache_sz            = XML->sys.core[ithCore].MIM.load_buffer_size*interface_ip.line_sz*XML->sys.core[ithCore].number_hardware_threads;
		  interface_ip.assoc               = 0;
		  interface_ip.nbanks              = 1;
		  interface_ip.out_w               = interface_ip.line_sz*8;
		  interface_ip.access_mode         = 1;
		  interface_ip.throughput          = 1.0/clockRate;
		  interface_ip.latency             = 1.0/clockRate;
		  interface_ip.obj_func_dyn_energy = 0;
		  interface_ip.obj_func_dyn_power  = 0;
		  interface_ip.obj_func_leak_power = 0;
		  interface_ip.obj_func_cycle_t    = 1;
		  interface_ip.num_rw_ports        = 0;
		  interface_ip.num_rd_ports        = XML->sys.core[ithCore].memory_ports;
		  interface_ip.num_wr_ports        = XML->sys.core[ithCore].memory_ports;
		  interface_ip.num_se_rd_ports     = 0;
		  interface_ip.num_search_ports    =XML->sys.core[ithCore].memory_ports;
		  LoadQ = new ArrayST(&interface_ip, "LoadQueue", Core_device, coredynp.opt_local, coredynp.core_ty);
		  LoadQ->area.set_area(LoadQ->area.get_area()+ LoadQ->local_result.area);
		  area.set_area(area.get_area()+ LoadQ->local_result.area);
		  area.set_area(area.get_area()*cdb_overhead);
		  //output_data_csv(LoadQ.LoadQ.local_result);
		  lsq_height=(LSQ->local_result.cache_ht + LoadQ->local_result.cache_ht)*sqrt(cdb_overhead);/*XML->sys.core[ithCore].number_hardware_threads*/
	  }

}

// Based on RegFU
MIM_ConfigTable::MIM_ConfigTable(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_,bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 ConfigTable(0),
 exist(exist_)
{
  if (!exist) return;

  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;

  // Output of config table 
  int table_output_width = 
    2*32  // 2x 32-bit constants
    + coredynp.opcode_length // ALU opcode
    + 1 // 1-bit RF/Cache select
    + 1 // 1-bit write value
    + 1 // 1 bit NOP/valid (do nothing for this case)
    + 2*5 // 2x 5-bit mux selects
    + 2; // 2 bits for filtering actions
  // Table is indexed by opcode 
  int table_input_width = coredynp.opcode_length;
  int num_entries = 1 << table_input_width;

	interface_ip.is_cache            = false;
	interface_ip.pure_cam            = false;
	interface_ip.pure_ram            = true;
  // Line size in bytes
	interface_ip.line_sz             = int(ceil(table_output_width/8.0));
	interface_ip.cache_sz            = num_entries*interface_ip.line_sz;
	interface_ip.assoc               = 1;
	interface_ip.nbanks              = 1;
  // Output bus width in bits
	interface_ip.out_w               = interface_ip.line_sz*8;
	interface_ip.access_mode         = 1;
	interface_ip.throughput          = 1.0/clockRate;
	interface_ip.latency             = 1.0/clockRate;
	interface_ip.obj_func_dyn_energy = 0;
	interface_ip.obj_func_dyn_power  = 0;
	interface_ip.obj_func_leak_power = 0;
	interface_ip.obj_func_cycle_t    = 1;
	interface_ip.num_rw_ports    = 0;
	interface_ip.num_rd_ports    = 1;
	interface_ip.num_wr_ports    = 0;
	interface_ip.num_se_rd_ports = 0;

	ConfigTable = new ArrayST(&interface_ip, "MIM Config Table", Core_device, coredynp.opt_local, coredynp.core_ty);

	ConfigTable->area.set_area(ConfigTable->area.get_area() + ConfigTable->local_result.area*cdb_overhead);
	area.set_area(area.get_area()+ ConfigTable->local_result.area*cdb_overhead);
}

// Based on RegFU
MFM_ConfigTable::MFM_ConfigTable(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_,bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 ConfigTable(0),
 exist(exist_)
{
  if (!exist) return;

  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;

  // Output of config table 
  int table_output_width = 
    4*32  // 4x 32-bit constants
    + 2*coredynp.opcode_length // 2x ALU opcode
    + 4*5 // 4x 5-bit mux selects
    + 2*1 // 1-bit RF/Cache select
    + 1; // 1 bit NOP/valid (do nothing for this case)
  // Table is indexed by opcode
  int table_input_width = coredynp.opcode_length;
  int num_entries = 1 << table_input_width;

	interface_ip.is_cache            = false;
	interface_ip.pure_cam            = false;
	interface_ip.pure_ram            = true;
  // Line size in bytes
	interface_ip.line_sz             = int(ceil(table_output_width/8.0));
	interface_ip.cache_sz            = num_entries*interface_ip.line_sz;
	interface_ip.assoc               = 1;
	interface_ip.nbanks              = 1;
  // Output bus width in bits
	interface_ip.out_w               = interface_ip.line_sz*8;
	interface_ip.access_mode         = 1;
	interface_ip.throughput          = 1.0/clockRate;
	interface_ip.latency             = 1.0/clockRate;
	interface_ip.obj_func_dyn_energy = 0;
	interface_ip.obj_func_dyn_power  = 0;
	interface_ip.obj_func_leak_power = 0;
	interface_ip.obj_func_cycle_t    = 1;
	interface_ip.num_rw_ports    = 0;
	interface_ip.num_rd_ports    = 1;
	interface_ip.num_wr_ports    = 0;
	interface_ip.num_se_rd_ports = 0;

	ConfigTable = new ArrayST(&interface_ip, "MFM Config Table", Core_device, coredynp.opt_local, coredynp.core_ty);

	ConfigTable->area.set_area(ConfigTable->area.get_area() + ConfigTable->local_result.area*cdb_overhead);
	area.set_area(area.get_area()+ ConfigTable->local_result.area*cdb_overhead);
}

// Based on RegFU
MFM_FilterLookupTable::MFM_FilterLookupTable(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_,bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 ConfigTable(0),
 exist(exist_)
{
  if (!exist) return;

  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;

  // Output of config table 
  int table_output_width = 
    2; // 2 bit index into invalidation config table
  // Table is indexed by opcode and 2 1-bit invalidation flags
  int table_input_width = coredynp.opcode_length + 2;
  int num_entries = 1 << table_input_width;

	interface_ip.is_cache            = false;
	interface_ip.pure_cam            = false;
	interface_ip.pure_ram            = true;
  // Line size in bytes
	interface_ip.line_sz             = int(ceil(table_output_width/8.0));
	interface_ip.cache_sz            = num_entries*interface_ip.line_sz;
	interface_ip.assoc               = 1;
	interface_ip.nbanks              = 1;
  // Output bus width in bits
	interface_ip.out_w               = interface_ip.line_sz*8;
	interface_ip.access_mode         = 1;
	interface_ip.throughput          = 1.0/clockRate;
	interface_ip.latency             = 1.0/clockRate;
	interface_ip.obj_func_dyn_energy = 0;
	interface_ip.obj_func_dyn_power  = 0;
	interface_ip.obj_func_leak_power = 0;
	interface_ip.obj_func_cycle_t    = 1;
	interface_ip.num_rw_ports    = 0;
	interface_ip.num_rd_ports    = 1;
	interface_ip.num_wr_ports    = 0;
	interface_ip.num_se_rd_ports = 0;

	ConfigTable = new ArrayST(&interface_ip, "MFM Config Table", Core_device, coredynp.opt_local, coredynp.core_ty);

	ConfigTable->area.set_area(ConfigTable->area.get_area() + ConfigTable->local_result.area*cdb_overhead);
	area.set_area(area.get_area()+ ConfigTable->local_result.area*cdb_overhead);
}



MIM_RegFU::MIM_RegFU(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_,bool exist_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 IRF (0),
 exist(exist_)
 {
	/*
	 * processors have separate architectural register files for each thread.
	 * therefore, the bypass buses need to travel across all the register files.
	 */
	if (!exist) return;
	int  data;

	clockRate = coredynp.clockRate;
	executionTime = coredynp.executionTime;
	//**********************************IRF***************************************
	//data							 = coredynp.int_data_width;
  data = 1; // Data width
	interface_ip.is_cache			 = false;
	interface_ip.pure_cam            = false;
	interface_ip.pure_ram            = true;
	//interface_ip.line_sz             = int(ceil(data/32.0))*4;
  // Line size in bytes
  // Really only needs to be 1 bit for MIM, but using 1 byte
	interface_ip.line_sz             = 1;
  int num_IRF_entry = 32;
  // cache size in bytes
	interface_ip.cache_sz            = num_IRF_entry*interface_ip.line_sz;
	interface_ip.assoc               = 1;
	interface_ip.nbanks              = 1;
  // bus width in bits
  // using 1-bit instead of 1-byte decreases power but increases area
	interface_ip.out_w               = interface_ip.line_sz*8;
	interface_ip.access_mode         = 1;
	interface_ip.throughput          = 1.0/clockRate;
	interface_ip.latency             = 1.0/clockRate;
	interface_ip.obj_func_dyn_energy = 0;
	interface_ip.obj_func_dyn_power  = 0;
	interface_ip.obj_func_leak_power = 0;
	interface_ip.obj_func_cycle_t    = 1;
//	interface_ip.num_rw_ports    = 1;//this is the transfer port for saving/restoring states when exceptions happen.
  interface_ip.num_rw_ports = 0;
  // 2 read ports, 1 write port
	//interface_ip.num_rd_ports    = 2*coredynp.peak_issueW;
	//interface_ip.num_wr_ports    = coredynp.peak_issueW;
	interface_ip.num_rd_ports    = 2;
	interface_ip.num_wr_ports    = 1;
	interface_ip.num_se_rd_ports = 0;
	IRF = new ArrayST(&interface_ip, "Integer Register File", Core_device, coredynp.opt_local, coredynp.core_ty);
  int number_hardware_threads = 1;
  int num_pipelines = 1;
	IRF->area.set_area(IRF->area.get_area()+ IRF->local_result.area*number_hardware_threads*num_pipelines*cdb_overhead);
	area.set_area(area.get_area()+ IRF->local_result.area*number_hardware_threads*num_pipelines*cdb_overhead);
	//area.set_area(area.get_area()*cdb_overhead);
	//output_data_csv(IRF.RF.local_result);

	int_regfile_height= IRF->local_result.cache_ht*number_hardware_threads*sqrt(cdb_overhead);
 
 }


void MIM_RegFU::computeEnergy(bool is_tdp)
{
/*
 * Architecture RF and physical RF cannot be present at the same time.
 * Therefore, the RF stats can only refer to either ARF or PRF;
 * And the same stats can be used for both.
 */

  // max 1 monitoring event per cycle
  int issueW = 1;
  double ALU_duty_cycle = XML->sys.core[ithCore].MIM.ALU_duty_cycle;
  int regfile_reads = XML->sys.core[ithCore].MIM.regfile_reads;
  int regfile_writes = XML->sys.core[ithCore].MIM.regfile_writes;

	if (!exist) return;
	if (is_tdp)
    {
    	//init stats for Peak
      // FIXME: these stats are for using core parameters
    	IRF->stats_t.readAc.access = issueW*2*(ALU_duty_cycle*1.1);
    	IRF->stats_t.writeAc.access = issueW*(ALU_duty_cycle*1.1);
    	//Rule of Thumb: about 10% RF related instructions do not need to access ALUs
    	IRF->tdp_stats = IRF->stats_t;
    }
    else
    {
    	//init stats for Runtime Dynamic (RTP)
    	IRF->stats_t.readAc.access  = regfile_reads;
    	IRF->stats_t.writeAc.access  = regfile_writes;
    	IRF->rtp_stats = IRF->stats_t;
    }
	IRF->power_t.reset();
	IRF->power_t.readOp.dynamic  +=  (IRF->stats_t.readAc.access*IRF->local_result.power.readOp.dynamic
			+IRF->stats_t.writeAc.access*IRF->local_result.power.writeOp.dynamic);

	if (is_tdp)
	{
		IRF->power  =  IRF->power_t + IRF->local_result.power *coredynp.pppm_lkg_multhread;
		power	    =  power + (IRF->power);
	}
	else
	{
		IRF->rt_power  =  IRF->power_t + IRF->local_result.power *coredynp.pppm_lkg_multhread;
		rt_power	   =  rt_power + (IRF->power_t);
	}
}

void MIM_RegFU::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	if (!exist) return;
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;

	if (is_tdp)
	{	cout << indent_str << "Metadata Integer RF:" << endl;
		cout << indent_str_next << "Area = " << IRF->area.get_area()*1e-6<< " mm^2" << endl;
		cout << indent_str_next << "Peak Dynamic = " << IRF->power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Subthreshold Leakage = "
			<< (long_channel? IRF->power.readOp.longer_channel_leakage:IRF->power.readOp.leakage) <<" W" << endl;
		cout << indent_str_next << "Gate Leakage = " << IRF->power.readOp.gate_leakage << " W" << endl;
		cout << indent_str_next << "Runtime Dynamic = " << IRF->rt_power.readOp.dynamic/executionTime << " W" << endl;
		cout <<endl;
	}
	else
	{
		cout << indent_str_next << "Integer RF    Peak Dynamic = " << IRF->rt_power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Integer RF    Subthreshold Leakage = " << IRF->rt_power.readOp.leakage <<" W" << endl;
		cout << indent_str_next << "Integer RF    Gate Leakage = " << IRF->rt_power.readOp.gate_leakage << " W" << endl;
	}
}

MIM_RegFU ::~MIM_RegFU(){

	if (!exist) return;
	if(IRF) 	               {delete IRF; IRF = 0;}
}

void MIM_LoadStoreU::computeEnergy(bool is_tdp)
{
	if (!exist) return;

  double LSU_duty_cycle = XML->sys.core[ithCore].MIM.LSU_duty_cycle;

	if (is_tdp)
	    {
	    	//init stats for Peak
	    	dcache.caches->stats_t.readAc.access  = 0.67*dcache.caches->l_ip.num_rw_ports*LSU_duty_cycle;
	    	dcache.caches->stats_t.readAc.miss    = 0;
	    	dcache.caches->stats_t.readAc.hit     = dcache.caches->stats_t.readAc.access - dcache.caches->stats_t.readAc.miss;
	    	dcache.caches->stats_t.writeAc.access = 0.33*dcache.caches->l_ip.num_rw_ports*LSU_duty_cycle;
	    	dcache.caches->stats_t.writeAc.miss   = 0;
    		dcache.caches->stats_t.writeAc.hit    = dcache.caches->stats_t.writeAc.access -	dcache.caches->stats_t.writeAc.miss;
	    	dcache.caches->tdp_stats = dcache.caches->stats_t;

	    	dcache.missb->stats_t.readAc.access  = dcache.missb->l_ip.num_search_ports;
	    	dcache.missb->stats_t.writeAc.access = dcache.missb->l_ip.num_search_ports;
	    	dcache.missb->tdp_stats = dcache.missb->stats_t;

	    	dcache.ifb->stats_t.readAc.access  = dcache.ifb->l_ip.num_search_ports;
	    	dcache.ifb->stats_t.writeAc.access = dcache.ifb->l_ip.num_search_ports;
	    	dcache.ifb->tdp_stats = dcache.ifb->stats_t;

	    	dcache.prefetchb->stats_t.readAc.access  = dcache.prefetchb->l_ip.num_search_ports;
	    	dcache.prefetchb->stats_t.writeAc.access = dcache.ifb->l_ip.num_search_ports;
	    	dcache.prefetchb->tdp_stats = dcache.prefetchb->stats_t;
	    	if (cache_p==Write_back)
	    	{
	    		dcache.wbb->stats_t.readAc.access  = dcache.wbb->l_ip.num_search_ports;
	    		dcache.wbb->stats_t.writeAc.access = dcache.wbb->l_ip.num_search_ports;
	    		dcache.wbb->tdp_stats = dcache.wbb->stats_t;
	    	}

	    	LSQ->stats_t.readAc.access = LSQ->stats_t.writeAc.access = LSQ->l_ip.num_search_ports*LSU_duty_cycle;
	    	LSQ->tdp_stats = LSQ->stats_t;
	    	if ((coredynp.core_ty==OOO) && (XML->sys.core[ithCore].MIM.load_buffer_size >0))
	    	{
	    		LoadQ->stats_t.readAc.access = LoadQ->stats_t.writeAc.access = LoadQ->l_ip.num_search_ports*LSU_duty_cycle;
	    		LoadQ->tdp_stats = LoadQ->stats_t;
	    	}
	    }
	    else
	    {
	    	//init stats for Runtime Dynamic (RTP)
	    	dcache.caches->stats_t.readAc.access  = XML->sys.core[ithCore].MIM.read_accesses;
	    	dcache.caches->stats_t.readAc.miss    = XML->sys.core[ithCore].MIM.read_misses;
	    	dcache.caches->stats_t.readAc.hit     = dcache.caches->stats_t.readAc.access - dcache.caches->stats_t.readAc.miss;
	    	dcache.caches->stats_t.writeAc.access = XML->sys.core[ithCore].MIM.write_accesses;
	    	dcache.caches->stats_t.writeAc.miss   = XML->sys.core[ithCore].MIM.write_misses;
    		dcache.caches->stats_t.writeAc.hit    = dcache.caches->stats_t.writeAc.access -	dcache.caches->stats_t.writeAc.miss;
	    	dcache.caches->rtp_stats = dcache.caches->stats_t;

	    	if (cache_p==Write_back)
	    	{
	    		dcache.missb->stats_t.readAc.access  = dcache.caches->stats_t.writeAc.miss;
	    		dcache.missb->stats_t.writeAc.access = dcache.caches->stats_t.writeAc.miss;
	    		dcache.missb->rtp_stats = dcache.missb->stats_t;

	    		dcache.ifb->stats_t.readAc.access  = dcache.caches->stats_t.writeAc.miss;
	    		dcache.ifb->stats_t.writeAc.access = dcache.caches->stats_t.writeAc.miss;
	    		dcache.ifb->rtp_stats = dcache.ifb->stats_t;

	    		dcache.prefetchb->stats_t.readAc.access  = dcache.caches->stats_t.writeAc.miss;
	    		dcache.prefetchb->stats_t.writeAc.access = dcache.caches->stats_t.writeAc.miss;
	    		dcache.prefetchb->rtp_stats = dcache.prefetchb->stats_t;

	    		dcache.wbb->stats_t.readAc.access  = dcache.caches->stats_t.writeAc.miss;
	    		dcache.wbb->stats_t.writeAc.access = dcache.caches->stats_t.writeAc.miss;
	    		dcache.wbb->rtp_stats = dcache.wbb->stats_t;
	    	}
	    	else
	    	{
	    		dcache.missb->stats_t.readAc.access  = dcache.caches->stats_t.readAc.miss;
	    		dcache.missb->stats_t.writeAc.access = dcache.caches->stats_t.readAc.miss;
	    		dcache.missb->rtp_stats = dcache.missb->stats_t;

	    		dcache.ifb->stats_t.readAc.access  = dcache.caches->stats_t.readAc.miss;
	    		dcache.ifb->stats_t.writeAc.access = dcache.caches->stats_t.readAc.miss;
	    		dcache.ifb->rtp_stats = dcache.ifb->stats_t;

	    		dcache.prefetchb->stats_t.readAc.access  = dcache.caches->stats_t.readAc.miss;
	    		dcache.prefetchb->stats_t.writeAc.access = dcache.caches->stats_t.readAc.miss;
	    		dcache.prefetchb->rtp_stats = dcache.prefetchb->stats_t;
	    	}

	    	LSQ->stats_t.readAc.access  = (XML->sys.core[ithCore].MIM.loads + XML->sys.core[ithCore].MIM.stores)*2;//flush overhead considered
	    	LSQ->stats_t.writeAc.access = (XML->sys.core[ithCore].MIM.loads + XML->sys.core[ithCore].MIM.stores)*2;
	    	LSQ->rtp_stats = LSQ->stats_t;

	    	if ((coredynp.core_ty==OOO) && (XML->sys.core[ithCore].MIM.load_buffer_size >0))
	    	{
		    	LoadQ->stats_t.readAc.access  = XML->sys.core[ithCore].MIM.loads + XML->sys.core[ithCore].MIM.stores;
		    	LoadQ->stats_t.writeAc.access = XML->sys.core[ithCore].MIM.loads + XML->sys.core[ithCore].MIM.stores;
		    	LoadQ->rtp_stats = LoadQ->stats_t;
	    	}

	    }

	dcache.power_t.reset();
	LSQ->power_t.reset();
    dcache.power_t.readOp.dynamic	+= (dcache.caches->stats_t.readAc.hit*dcache.caches->local_result.power.readOp.dynamic+
    		dcache.caches->stats_t.readAc.miss*dcache.caches->local_result.power.readOp.dynamic+
    		dcache.caches->stats_t.writeAc.miss*dcache.caches->local_result.tag_array2->power.readOp.dynamic+
    		dcache.caches->stats_t.writeAc.access*dcache.caches->local_result.power.writeOp.dynamic);

    if (cache_p==Write_back)
    {//write miss will generate a write later
    	dcache.power_t.readOp.dynamic	+= dcache.caches->stats_t.writeAc.miss*dcache.caches->local_result.power.writeOp.dynamic;
    }

    dcache.power_t.readOp.dynamic	+=  dcache.missb->stats_t.readAc.access*dcache.missb->local_result.power.searchOp.dynamic +
            dcache.missb->stats_t.writeAc.access*dcache.missb->local_result.power.writeOp.dynamic;//each access to missb involves a CAM and a write
    dcache.power_t.readOp.dynamic	+=  dcache.ifb->stats_t.readAc.access*dcache.ifb->local_result.power.searchOp.dynamic +
            dcache.ifb->stats_t.writeAc.access*dcache.ifb->local_result.power.writeOp.dynamic;
    dcache.power_t.readOp.dynamic	+=  dcache.prefetchb->stats_t.readAc.access*dcache.prefetchb->local_result.power.searchOp.dynamic +
            dcache.prefetchb->stats_t.writeAc.access*dcache.prefetchb->local_result.power.writeOp.dynamic;
    if (cache_p==Write_back)
    {
    	dcache.power_t.readOp.dynamic	+=  dcache.wbb->stats_t.readAc.access*dcache.wbb->local_result.power.searchOp.dynamic
			+ dcache.wbb->stats_t.writeAc.access*dcache.wbb->local_result.power.writeOp.dynamic;
    }

    if ((coredynp.core_ty==OOO) && (XML->sys.core[ithCore].MIM.load_buffer_size >0))
    {
    	LoadQ->power_t.reset();
    	LoadQ->power_t.readOp.dynamic  +=  LoadQ->stats_t.readAc.access*(LoadQ->local_result.power.searchOp.dynamic+ LoadQ->local_result.power.readOp.dynamic)+
    	        LoadQ->stats_t.writeAc.access*LoadQ->local_result.power.writeOp.dynamic;//every memory access invloves at least two operations on LoadQ

    	LSQ->power_t.readOp.dynamic  +=  LSQ->stats_t.readAc.access*(LSQ->local_result.power.searchOp.dynamic + LSQ->local_result.power.readOp.dynamic)
    		        + LSQ->stats_t.writeAc.access*LSQ->local_result.power.writeOp.dynamic;//every memory access invloves at least two operations on LSQ

    }
    else
    {
    	LSQ->power_t.readOp.dynamic  +=  LSQ->stats_t.readAc.access*(LSQ->local_result.power.searchOp.dynamic + LSQ->local_result.power.readOp.dynamic)
    		        + LSQ->stats_t.writeAc.access*LSQ->local_result.power.writeOp.dynamic;//every memory access invloves at least two operations on LSQ

    }

    if (is_tdp)
    {
//    	dcache.power = dcache.power_t + (dcache.caches->local_result.power)*pppm_lkg +
//    			(dcache.missb->local_result.power +
//    			dcache.ifb->local_result.power +
//    			dcache.prefetchb->local_result.power +
//    			dcache.wbb->local_result.power)*pppm_Isub;
    	dcache.power = dcache.power_t + (dcache.caches->local_result.power +
    			dcache.missb->local_result.power +
    			dcache.ifb->local_result.power +
    			dcache.prefetchb->local_result.power) *pppm_lkg;
    	if (cache_p==Write_back)
    	{
    		dcache.power = dcache.power + dcache.wbb->local_result.power*pppm_lkg;
    	}

    	LSQ->power = LSQ->power_t + LSQ->local_result.power *pppm_lkg;
    	power     = power + dcache.power + LSQ->power;

    	if ((coredynp.core_ty==OOO) && (XML->sys.core[ithCore].MIM.load_buffer_size >0))
    	{
    		LoadQ->power = LoadQ->power_t + LoadQ->local_result.power *pppm_lkg;
    		power     = power + LoadQ->power;
    	}
    }
    else
    {
//    	dcache.rt_power = dcache.power_t + (dcache.caches->local_result.power +
//    			dcache.missb->local_result.power +
//    			dcache.ifb->local_result.power +
//    			dcache.prefetchb->local_result.power +
//    			dcache.wbb->local_result.power)*pppm_lkg;
    	dcache.rt_power = dcache.power_t + (dcache.caches->local_result.power +
    			dcache.missb->local_result.power +
    			dcache.ifb->local_result.power +
    			dcache.prefetchb->local_result.power )*pppm_lkg;

    	if (cache_p==Write_back)
    	{
    		dcache.rt_power = dcache.rt_power + dcache.wbb->local_result.power*pppm_lkg;
    	}

    	LSQ->rt_power = LSQ->power_t + LSQ->local_result.power *pppm_lkg;
    	rt_power     = rt_power + dcache.rt_power + LSQ->rt_power;

    	if ((coredynp.core_ty==OOO) && (XML->sys.core[ithCore].MIM.load_buffer_size >0))
    	{
    		LoadQ->rt_power = LoadQ->power_t + LoadQ->local_result.power *pppm_lkg;
    		rt_power     = rt_power + LoadQ->rt_power;
    	}
    }
}


void MIM_LoadStoreU::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	if (!exist) return;
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;


	if (is_tdp)
	{
		cout << indent_str << "Data Cache:" << endl;
		cout << indent_str_next << "Area = " << dcache.area.get_area()*1e-6<< " mm^2" << endl;
		cout << indent_str_next << "Peak Dynamic = " << dcache.power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Subthreshold Leakage = "
			<< (long_channel? dcache.power.readOp.longer_channel_leakage:dcache.power.readOp.leakage )<<" W" << endl;
		cout << indent_str_next << "Gate Leakage = " << dcache.power.readOp.gate_leakage << " W" << endl;
		cout << indent_str_next << "Runtime Dynamic = " << dcache.rt_power.readOp.dynamic/executionTime << " W" << endl;
		cout <<endl;
		if (coredynp.core_ty==Inorder)
		{
			cout << indent_str << "Load/Store Queue:" << endl;
			cout << indent_str_next << "Area = " << LSQ->area.get_area()*1e-6  << " mm^2" << endl;
			cout << indent_str_next << "Peak Dynamic = " << LSQ->power.readOp.dynamic*clockRate  << " W" << endl;
			cout << indent_str_next << "Subthreshold Leakage = "
				<< (long_channel? LSQ->power.readOp.longer_channel_leakage:LSQ->power.readOp.leakage)  << " W" << endl;
			cout << indent_str_next << "Gate Leakage = " << LSQ->power.readOp.gate_leakage  << " W" << endl;
			cout << indent_str_next << "Runtime Dynamic = " << LSQ->rt_power.readOp.dynamic/executionTime << " W" << endl;
			cout <<endl;
		}
		else

		{
			if (XML->sys.core[ithCore].MIM.load_buffer_size >0)
			{
				cout << indent_str << "LoadQ:" << endl;
				cout << indent_str_next << "Area = " << LoadQ->area.get_area() *1e-6 << " mm^2" << endl;
				cout << indent_str_next << "Peak Dynamic = " << LoadQ->power.readOp.dynamic*clockRate  << " W" << endl;
				cout << indent_str_next << "Subthreshold Leakage = "
				<< (long_channel? LoadQ->power.readOp.longer_channel_leakage:LoadQ->power.readOp.leakage)  << " W" << endl;
				cout << indent_str_next << "Gate Leakage = " << LoadQ->power.readOp.gate_leakage  << " W" << endl;
				cout << indent_str_next << "Runtime Dynamic = " << LoadQ->rt_power.readOp.dynamic/executionTime << " W" << endl;
				cout <<endl;
			}
			cout << indent_str<< "StoreQ:" << endl;
			cout << indent_str_next << "Area = " << LSQ->area.get_area()  *1e-6<< " mm^2" << endl;
			cout << indent_str_next << "Peak Dynamic = " << LSQ->power.readOp.dynamic*clockRate  << " W" << endl;
			cout << indent_str_next << "Subthreshold Leakage = "
				<< (long_channel? LSQ->power.readOp.longer_channel_leakage:LSQ->power.readOp.leakage)  << " W" << endl;
			cout << indent_str_next << "Gate Leakage = " << LSQ->power.readOp.gate_leakage  << " W" << endl;
			cout << indent_str_next << "Runtime Dynamic = " << LSQ->rt_power.readOp.dynamic/executionTime<< " W" << endl;
			cout <<endl;
		}
	}
	else
	{
		cout << indent_str_next << "Data Cache    Peak Dynamic = " << dcache.rt_power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Data Cache    Subthreshold Leakage = " << dcache.rt_power.readOp.leakage <<" W" << endl;
		cout << indent_str_next << "Data Cache    Gate Leakage = " << dcache.rt_power.readOp.gate_leakage << " W" << endl;
		if (coredynp.core_ty==Inorder)
		{
			cout << indent_str_next << "Load/Store Queue   Peak Dynamic = " << LSQ->rt_power.readOp.dynamic*clockRate  << " W" << endl;
			cout << indent_str_next << "Load/Store Queue   Subthreshold Leakage = " << LSQ->rt_power.readOp.leakage  << " W" << endl;
			cout << indent_str_next << "Load/Store Queue   Gate Leakage = " << LSQ->rt_power.readOp.gate_leakage  << " W" << endl;
		}
		else
		{
			cout << indent_str_next << "LoadQ   Peak Dynamic = " << LoadQ->rt_power.readOp.dynamic*clockRate  << " W" << endl;
			cout << indent_str_next << "LoadQ   Subthreshold Leakage = " << LoadQ->rt_power.readOp.leakage  << " W" << endl;
			cout << indent_str_next << "LoadQ   Gate Leakage = " << LoadQ->rt_power.readOp.gate_leakage  << " W" << endl;
			cout << indent_str_next << "StoreQ   Peak Dynamic = " << LSQ->rt_power.readOp.dynamic*clockRate  << " W" << endl;
			cout << indent_str_next << "StoreQ   Subthreshold Leakage = " << LSQ->rt_power.readOp.leakage  << " W" << endl;
			cout << indent_str_next << "StoreQ   Gate Leakage = " << LSQ->rt_power.readOp.gate_leakage  << " W" << endl;
		}
	}

}


void MIM_ConfigTable::computeEnergy(bool is_tdp)
{
	if (!exist) return;

  int issueW = 1;
  double CT_duty_cycle = XML->sys.core[ithCore].MIM.CT_duty_cycle;

	if (is_tdp)
    {
      // FIXME: stats are for core
    	//init stats for Peak
    	ConfigTable->stats_t.readAc.access  = issueW*2*(CT_duty_cycle*1.1);
    	ConfigTable->stats_t.writeAc.access  = issueW*(CT_duty_cycle*1.1);
    	//Rule of Thumb: about 10% RF related instructions do not need to access ALUs
    	ConfigTable->tdp_stats = ConfigTable->stats_t;
     }
    else
    {
    	//init stats for Runtime Dynamic (RTP)
    	ConfigTable->stats_t.readAc.access  = XML->sys.core[ithCore].MIM.CT_reads;
      // Never write to ConfigTable
    	ConfigTable->stats_t.writeAc.access  = 0; //XML->sys.core[ithCore].int_regfile_writes;
    	ConfigTable->rtp_stats = ConfigTable->stats_t;

    }
	ConfigTable->power_t.reset();
	ConfigTable->power_t.readOp.dynamic  +=  (ConfigTable->stats_t.readAc.access*ConfigTable->local_result.power.readOp.dynamic
			+ConfigTable->stats_t.writeAc.access*ConfigTable->local_result.power.writeOp.dynamic);

	if (is_tdp)
	{
		ConfigTable->power  =  ConfigTable->power_t + ConfigTable->local_result.power *coredynp.pppm_lkg_multhread;
		power	    =  power + ConfigTable->power;
	}
	else
	{
		ConfigTable->rt_power  =  ConfigTable->power_t + ConfigTable->local_result.power *coredynp.pppm_lkg_multhread;
		rt_power	   =  rt_power + ConfigTable->power_t;
	}
}


void MIM_ConfigTable::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	if (!exist) return;
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;

	if (is_tdp)
	{	
    cout << indent_str << "MIM Config Table:" << endl;
		cout << indent_str_next << "Area = " << ConfigTable->area.get_area()*1e-6<< " mm^2" << endl;
		cout << indent_str_next << "Peak Dynamic = " << ConfigTable->power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Subthreshold Leakage = "
			<< (long_channel? ConfigTable->power.readOp.longer_channel_leakage:ConfigTable->power.readOp.leakage) <<" W" << endl;
		cout << indent_str_next << "Gate Leakage = " << ConfigTable->power.readOp.gate_leakage << " W" << endl;
		cout << indent_str_next << "Runtime Dynamic = " << ConfigTable->rt_power.readOp.dynamic/executionTime << " W" << endl;
		cout <<endl;
	}
	else
	{
		cout << indent_str_next << "Config Table    Peak Dynamic = " << ConfigTable->rt_power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Config Table    Subthreshold Leakage = " << ConfigTable->rt_power.readOp.leakage <<" W" << endl;
		cout << indent_str_next << "Config Table    Gate Leakage = " << ConfigTable->rt_power.readOp.gate_leakage << " W" << endl;
	}
}

void MFM_ConfigTable::computeEnergy(bool is_tdp)
{
	if (!exist) return;

  int issueW = 1;
  double CT_duty_cycle = XML->sys.core[ithCore].MFM.CT_duty_cycle;

	if (is_tdp)
    {
      // FIXME: stats are for core
    	//init stats for Peak
    	ConfigTable->stats_t.readAc.access  = issueW*2*(CT_duty_cycle*1.1);
    	ConfigTable->stats_t.writeAc.access  = issueW*(CT_duty_cycle*1.1);
    	//Rule of Thumb: about 10% RF related instructions do not need to access ALUs
    	ConfigTable->tdp_stats = ConfigTable->stats_t;
     }
    else
    {
    	//init stats for Runtime Dynamic (RTP)
    	ConfigTable->stats_t.readAc.access  = XML->sys.core[ithCore].MFM.CT_reads;
      // Never write to ConfigTable
    	ConfigTable->stats_t.writeAc.access  = 0; //XML->sys.core[ithCore].int_regfile_writes;
    	ConfigTable->rtp_stats = ConfigTable->stats_t;

    }
	ConfigTable->power_t.reset();
	ConfigTable->power_t.readOp.dynamic  +=  (ConfigTable->stats_t.readAc.access*ConfigTable->local_result.power.readOp.dynamic
			+ConfigTable->stats_t.writeAc.access*ConfigTable->local_result.power.writeOp.dynamic);

	if (is_tdp)
	{
		ConfigTable->power  =  ConfigTable->power_t + ConfigTable->local_result.power *coredynp.pppm_lkg_multhread;
		power	    =  power + ConfigTable->power;
	}
	else
	{
		ConfigTable->rt_power  =  ConfigTable->power_t + ConfigTable->local_result.power *coredynp.pppm_lkg_multhread;
		rt_power	   =  rt_power + ConfigTable->power_t;
	}
}

void MFM_ConfigTable::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	if (!exist) return;
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;

	if (is_tdp)
	{	
    cout << indent_str << "MFM Config Table:" << endl;
		cout << indent_str_next << "Area = " << ConfigTable->area.get_area()*1e-6<< " mm^2" << endl;
		cout << indent_str_next << "Peak Dynamic = " << ConfigTable->power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Subthreshold Leakage = "
			<< (long_channel? ConfigTable->power.readOp.longer_channel_leakage:ConfigTable->power.readOp.leakage) <<" W" << endl;
		cout << indent_str_next << "Gate Leakage = " << ConfigTable->power.readOp.gate_leakage << " W" << endl;
		cout << indent_str_next << "Runtime Dynamic = " << ConfigTable->rt_power.readOp.dynamic/executionTime << " W" << endl;
		cout <<endl;
	}
	else
	{
		cout << indent_str_next << "Config Table    Peak Dynamic = " << ConfigTable->rt_power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Config Table    Subthreshold Leakage = " << ConfigTable->rt_power.readOp.leakage <<" W" << endl;
		cout << indent_str_next << "Config Table    Gate Leakage = " << ConfigTable->rt_power.readOp.gate_leakage << " W" << endl;
	}
}

void MFM_FilterLookupTable::computeEnergy(bool is_tdp)
{
	if (!exist) return;

  int issueW = 1;
  double CT_duty_cycle = XML->sys.core[ithCore].MFM.FLT_duty_cycle;

	if (is_tdp)
    {
      // FIXME: stats are for core
    	//init stats for Peak
    	ConfigTable->stats_t.readAc.access  = issueW*2*(CT_duty_cycle*1.1);
    	ConfigTable->stats_t.writeAc.access  = issueW*(CT_duty_cycle*1.1);
    	//Rule of Thumb: about 10% RF related instructions do not need to access ALUs
    	ConfigTable->tdp_stats = ConfigTable->stats_t;
     }
    else
    {
    	//init stats for Runtime Dynamic (RTP)
    	ConfigTable->stats_t.readAc.access  = XML->sys.core[ithCore].MFM.FLT_reads;
      // Never write to ConfigTable
    	ConfigTable->stats_t.writeAc.access  = 0; //XML->sys.core[ithCore].int_regfile_writes;
    	ConfigTable->rtp_stats = ConfigTable->stats_t;

    }
	ConfigTable->power_t.reset();
	ConfigTable->power_t.readOp.dynamic  +=  (ConfigTable->stats_t.readAc.access*ConfigTable->local_result.power.readOp.dynamic
			+ConfigTable->stats_t.writeAc.access*ConfigTable->local_result.power.writeOp.dynamic);

	if (is_tdp)
	{
		ConfigTable->power  =  ConfigTable->power_t + ConfigTable->local_result.power *coredynp.pppm_lkg_multhread;
		power	    =  power + ConfigTable->power;
	}
	else
	{
		ConfigTable->rt_power  =  ConfigTable->power_t + ConfigTable->local_result.power *coredynp.pppm_lkg_multhread;
		rt_power	   =  rt_power + ConfigTable->power_t;
	}
}

void MFM_FilterLookupTable::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	if (!exist) return;
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;

	if (is_tdp)
	{	
    cout << indent_str << "MFM Filter Lookup Table:" << endl;
		cout << indent_str_next << "Area = " << ConfigTable->area.get_area()*1e-6<< " mm^2" << endl;
		cout << indent_str_next << "Peak Dynamic = " << ConfigTable->power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Subthreshold Leakage = "
			<< (long_channel? ConfigTable->power.readOp.longer_channel_leakage:ConfigTable->power.readOp.leakage) <<" W" << endl;
		cout << indent_str_next << "Gate Leakage = " << ConfigTable->power.readOp.gate_leakage << " W" << endl;
		cout << indent_str_next << "Runtime Dynamic = " << ConfigTable->rt_power.readOp.dynamic/executionTime << " W" << endl;
		cout <<endl;
	}
	else
	{
		cout << indent_str_next << "Config Table    Peak Dynamic = " << ConfigTable->rt_power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Config Table    Subthreshold Leakage = " << ConfigTable->rt_power.readOp.leakage <<" W" << endl;
		cout << indent_str_next << "Config Table    Gate Leakage = " << ConfigTable->rt_power.readOp.gate_leakage << " W" << endl;
	}
}

MIM_LoadStoreU ::~MIM_LoadStoreU(){

	if (!exist) return;
	if(LSQ) 	               {delete LSQ; LSQ = 0;}
	}

MIM_ConfigTable ::~MIM_ConfigTable() {
  if (!exist) return;
  if (ConfigTable) {delete ConfigTable; ConfigTable = 0;}
}

MFM_ConfigTable ::~MFM_ConfigTable() {
  if (!exist) return;
  if (ConfigTable) {delete ConfigTable; ConfigTable = 0;}
}

MFM_FilterLookupTable ::~MFM_FilterLookupTable() {
  if (!exist) return;
  if (ConfigTable) {delete ConfigTable; ConfigTable = 0;}
}

MIM::MIM(ParseXML* XML_interface, int ithCore_, InputParameter *interface_ip_, const CoreDynParam & dyn_p_, bool exist_) :
  XML(XML_interface),
  ithCore(ithCore_),
  interface_ip(*interface_ip_),
  coredynp(dyn_p_),
  alu(0),
  rfu(0),
  lsu(0),
  ct(0),
  mfm_alu(0),
  mfm_ct(0),
  mfm_flt(0),
  exist(exist_)
{
  if (!exist) return;

  double fu_height = 0.0;
  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;

  // Create components

  // ALU
  // slightly hacky way to create 2 ALUs for MIM
  alu = new MIM_FunctionalUnit(XML, ithCore, &interface_ip, coredynp, ALU);
  // Register file
  rfu = new MIM_RegFU(XML, ithCore, &interface_ip, coredynp);
  // Load/store + metadata invalidation cache
  lsu = new MIM_LoadStoreU(XML, ithCore, &interface_ip, coredynp);
  // Configuration table
  ct = new MIM_ConfigTable(XML, ithCore, &interface_ip, coredynp);

  // MFM ALU
  mfm_alu = new MFM_FunctionalUnit(XML, ithCore, &interface_ip, coredynp, ALU);
  // MFM Config Table
  mfm_ct = new MFM_ConfigTable(XML, ithCore, &interface_ip, coredynp);
  // MFM Filter Lookup Table
  mfm_flt = new MFM_FilterLookupTable(XML, ithCore, &interface_ip, coredynp);

  // Add in areas of new components
  area.set_area(area.get_area() + alu->area.get_area() + rfu->area.get_area() + ct->area.get_area() + mfm_alu->area.get_area() + mfm_ct->area.get_area() + mfm_flt->area.get_area());
  if (lsu->exist)
  {
    //lsu->area.set_area(lsu->area.get_area() + pipeline_area_per_unit);
    // FIXME: ok to remove pipeline_area_per_unit?
    lsu->area.set_area(lsu->area.get_area());
    area.set_area(area.get_area() + lsu->area.get_area());
  }
  // FIXME: no idea what FU_height is
  fu_height = alu->FU_height + mfm_alu->FU_height;

  // FIXME: interconnects/bypass?
}

void MIM::computeEnergy(bool is_tdp)
{
  if (!exist) return;
  // FIXME: what is pppm?
  double pppm_t[4] = {1, 1, 1, 1};

  rfu->computeEnergy(is_tdp);
  alu->computeEnergy(is_tdp);
  lsu->computeEnergy(is_tdp);
  ct->computeEnergy(is_tdp);
  mfm_alu->computeEnergy(is_tdp);
  mfm_ct->computeEnergy(is_tdp);
  mfm_flt->computeEnergy(is_tdp);

  // FIXME: no idea what this is
  // FIXME: Should be able to remove OOO for MIM_LS
  double num_units = 4.0;
  if (coredynp.core_ty == OOO)
  {
    num_units = 5.0;
  }

  if (is_tdp)
  {
    // 2 means two source operands need to be passed for each in instruction
    // FIXME: don't know what this is doing
    set_pppm(pppm_t, 2*coredynp.ALU_cdb_duty_cycle, 2, 2, 2*coredynp.ALU_cdb_duty_cycle); 

    power = power + alu->power + rfu->power + ct->power + mfm_alu->power + mfm_ct->power + mfm_flt->power;

    if (lsu->exist)
    {
      // FIXME: don't know what this is doing
      set_pppm(pppm_t, coredynp.num_pipelines/num_units*coredynp.LSU_duty_cycle, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
      // FIXME: removing pipeline
      //lsu->power = lsu->power + corepipe->power*pppm_t;
      lsu->power = lsu->power;
      power = power + lsu->power;
    }
  }
  else
  {
    // Load-store unit
    lsu->computeEnergy(is_tdp);

    double rtp_pipeline_coe;

		if (coredynp.core_ty==OOO)
		{
			num_units = 5.0;
        	set_pppm(pppm_t, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
		}
		else
		{
			if (XML->sys.homogeneous_cores==1)
			{
				rtp_pipeline_coe = coredynp.pipeline_duty_cycle * XML->sys.total_cycles * XML->sys.number_of_cores;
			}
			else
			{
				rtp_pipeline_coe = coredynp.pipeline_duty_cycle * coredynp.total_cycles;
			}
		set_pppm(pppm_t, coredynp.num_pipelines*rtp_pipeline_coe/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units, coredynp.num_pipelines/num_units);
		}

		if (lsu->exist)
		{
			//lsu->rt_power = lsu->rt_power + corepipe->power*pppm_t;
      // FIXME: remove pipeline
			lsu->rt_power = lsu->rt_power;
			rt_power     = rt_power  + lsu->rt_power;
		}

    // ALU
    set_pppm(pppm_t, XML->sys.core[ithCore].cdb_alu_accesses, 2, 2, XML->sys.core[ithCore].cdb_alu_accesses);

    rt_power = rt_power + rfu->rt_power + alu->rt_power + ct->rt_power + mfm_alu->rt_power + mfm_ct->rt_power + mfm_flt->rt_power;
  }
}

void MIM::displayEnergy(uint32_t indent, int plevel, bool is_tdp)
{
  if (!exist) return;
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  bool long_channel = XML->sys.longer_channel_device;

  if (is_tdp)
  {
    // Register file
		cout << indent_str << "Metadata Register Files:" << endl;
		cout << indent_str_next << "Area = " << rfu->area.get_area()*1e-6<< " mm^2" << endl;
		cout << indent_str_next << "Peak Dynamic = " << rfu->power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Subthreshold Leakage = "
			<< (long_channel? rfu->power.readOp.longer_channel_leakage:rfu->power.readOp.leakage) <<" W" << endl;
		cout << indent_str_next << "Gate Leakage = " << rfu->power.readOp.gate_leakage << " W" << endl;
		cout << indent_str_next << "Runtime Dynamic = " << rfu->rt_power.readOp.dynamic/executionTime << " W" << endl;
		cout <<endl;
		if (plevel>3){
			rfu->displayEnergy(indent+4,is_tdp);
		}

    // ALU
    if (plevel > 3) {
      alu->displayEnergy(indent, is_tdp);
    }

    // Cache
    if (lsu->exist) 
    {
			cout << indent_str<< "Metadata Load Store Unit:" << endl;
			cout << indent_str_next << "Area = " << lsu->area.get_area()*1e-6  << " mm^2" << endl;
			cout << indent_str_next << "Peak Dynamic = " << lsu->power.readOp.dynamic*clockRate  << " W" << endl;
			cout << indent_str_next << "Subthreshold Leakage = "
				<< (long_channel? lsu->power.readOp.longer_channel_leakage:lsu->power.readOp.leakage ) << " W" << endl;
			//cout << indent_str_next << "Subthreshold Leakage = " << lsu->power.readOp.longer_channel_leakage  << " W" << endl;
			cout << indent_str_next << "Gate Leakage = " << lsu->power.readOp.gate_leakage  << " W" << endl;
			cout << indent_str_next << "Runtime Dynamic = " << lsu->rt_power.readOp.dynamic/executionTime << " W" << endl;
			cout <<endl;
			if (plevel >2){
				lsu->displayEnergy(indent+4,plevel,is_tdp);
			}
    }

    // Config Table
    ct->displayEnergy(indent, is_tdp);

    // MFM ALU
    if (plevel > 3) {
      mfm_alu->displayEnergy(indent, is_tdp);
    }
    // MFM Config Table
    mfm_ct->displayEnergy(indent, is_tdp);
    // MFM Filter Lookup Table
    mfm_flt->displayEnergy(indent, is_tdp);
  } 
  else 
  {
    cout << "ERROR (MIM::displayEnergy): is_tdp false not implemented\n" << endl;
    assert(false);
  }
}

MIM::~MIM() {
  if (!exist) return;
  if (alu) { delete alu; alu = 0; }
  if (rfu) { delete rfu; rfu = 0; }
  if (lsu) { delete lsu; lsu = 0; }
  if (ct)  { delete ct;  ct  = 0; }
  if (mfm_alu) { delete mfm_alu; mfm_alu = 0; }
  if (mfm_ct) { delete mfm_ct; mfm_ct = 0; }
  if (mfm_flt) { delete mfm_flt; mfm_flt = 0; }
}

MIM_FunctionalUnit::MIM_FunctionalUnit(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, enum FU_type fu_type_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 fu_type(fu_type_)
{
    double area_t;//, leakage, gate_leakage;
    double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
	clockRate = coredynp.clockRate;
	executionTime = coredynp.executionTime;

  // 1 integer ALU used by MIM
  num_fu = 1;

	//XML_interface=_XML_interface;
	uca_org_t result2;
	result2 = init_interface(&interface_ip);
	if (XML->sys.Embedded)
	{
    area_t = 280*260*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2 ALU + MUl
    leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
    gate_leakage = area_t*(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;
//			base_energy = coredynp.core_ty==Inorder? 0:89e-3; //W The base energy of ALU average numbers from Intel 4G and 773Mhz (Wattch)
//			base_energy *=(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);
    base_energy = 0;
    per_access_energy = 1.15/3/1e9/4/1.3/1.3*g_tp.peri_global.Vdd*g_tp.peri_global.Vdd*(g_ip->F_sz_nm/90.0);//(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);//0.00649*1e-9; //This is per cycle energy(nJ)
    FU_height=(6222*num_fu)*interface_ip.F_sz_um;//integer ALU

    per_access_energy *=0.5;//According to ARM data embedded processor has much lower per acc energy
	}
	else
	{
			area_t = 280*260*2*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2 ALU + MUl
			leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
			gate_leakage = area_t*(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;
			base_energy = coredynp.core_ty==Inorder? 0:89e-3; //W The base energy of ALU average numbers from Intel 4G and 773Mhz (Wattch)
			base_energy *=(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);
			per_access_energy = 1.15/1e9/4/1.3/1.3*g_tp.peri_global.Vdd*g_tp.peri_global.Vdd*(g_ip->F_sz_nm/90.0);//(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);//0.00649*1e-9; //This is per cycle energy(nJ)
			FU_height=(6222*num_fu)*interface_ip.F_sz_um;//integer ALU
		}
	//IEXEU, simple ALU and FPU
	//  double C_ALU, C_EXEU, C_FPU; //Lum Equivalent capacitance of IEXEU and FPU. Based on Intel and Sun 90nm process fabracation.
	//
	//  C_ALU	  = 0.025e-9;//F
	//  C_EXEU  = 0.05e-9; //F
	//  C_FPU	  = 0.35e-9;//F
    area.set_area(area_t*num_fu);
    leakage *= num_fu;
    gate_leakage *=num_fu;
	double macro_layout_overhead = g_tp.macro_layout_overhead;
//	if (!XML->sys.Embedded)
		area.set_area(area.get_area()*macro_layout_overhead);
}

void MIM_FunctionalUnit::computeEnergy(bool is_tdp)
{
	double pppm_t[4]    = {1,1,1,1};

	double FU_duty_cycle;

	if (is_tdp)
	{
		set_pppm(pppm_t, 2, 2, 2, 2);//2 means two source operands needs to be passed for each int instruction.
	
    stats_t.readAc.access = 1*num_fu;
    tdp_stats = stats_t;
    FU_duty_cycle = XML->sys.core[ithCore].MIM.ALU_duty_cycle;

    //power.readOp.dynamic = base_energy/clockRate + energy*stats_t.readAc.access;
    power.readOp.dynamic = per_access_energy*stats_t.readAc.access + base_energy/clockRate;
		double sckRation = g_tp.sckt_co_eff;
		power.readOp.dynamic *= sckRation*FU_duty_cycle;
		power.writeOp.dynamic *= sckRation;
		power.searchOp.dynamic *= sckRation;

    power.readOp.leakage = leakage;
    power.readOp.gate_leakage = gate_leakage;
    double long_channel_device_reduction = longer_channel_device_reduction(Core_device, coredynp.core_ty);
    power.readOp.longer_channel_leakage	= power.readOp.leakage*long_channel_device_reduction;
	}
	else
	{
    stats_t.readAc.access = XML->sys.core[ithCore].MIM.alu_accesses;
    rtp_stats = stats_t;

    //rt_power.readOp.dynamic = base_energy*executionTime + energy*stats_t.readAc.access;
    rt_power.readOp.dynamic = per_access_energy*stats_t.readAc.access + base_energy*executionTime;
		double sckRation = g_tp.sckt_co_eff;
		rt_power.readOp.dynamic *= sckRation;
		rt_power.writeOp.dynamic *= sckRation;
		rt_power.searchOp.dynamic *= sckRation;
	}
}

void MIM_FunctionalUnit::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;

//	cout << indent_str_next << "Results Broadcast Bus Area = " << bypass->area.get_area() *1e-6 << " mm^2" << endl;
	if (is_tdp)
	{
    cout << indent_str << "MIM Integer ALUs (Count: "<< num_fu <<" ):" << endl;
    cout << indent_str_next << "Area = " << area.get_area()*1e-6  << " mm^2" << endl;
    cout << indent_str_next << "Peak Dynamic = " << power.readOp.dynamic*clockRate  << " W" << endl;
//			cout << indent_str_next << "Subthreshold Leakage = " << power.readOp.leakage  << " W" << endl;
    cout << indent_str_next<< "Subthreshold Leakage = "
          << (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
    cout << indent_str_next << "Gate Leakage = " << power.readOp.gate_leakage  << " W" << endl;
    cout << indent_str_next << "Runtime Dynamic = " << rt_power.readOp.dynamic/executionTime << " W" << endl;
    cout <<endl;
	}
	else
	{
	}

}

void MIM_FunctionalUnit::leakage_feedback(double temperature)
{
  // Update the temperature and initialize the global interfaces.
  interface_ip.temp = (unsigned int)round(temperature/10.0)*10;

  uca_org_t init_result = init_interface(&interface_ip); // init_result is dummy

  // This is part of MIM_FunctionalUnit()
  double area_t, leakage, gate_leakage;
  double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();

  if (fu_type == FPU)
  {
	area_t = 4.47*1e6*(g_ip->F_sz_nm*g_ip->F_sz_nm/90.0/90.0);//this is um^2 The base number
	if (g_ip->F_sz_nm>90)
		area_t = 4.47*1e6*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2
	leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(5*g_tp.min_w_nmos_, 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
	gate_leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(5*g_tp.min_w_nmos_, 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
  }
  else if (fu_type == ALU)
  {
    area_t = 280*260*2*num_fu*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2 ALU + MUl
    leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
    gate_leakage = area_t*(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;
  }
  else if (fu_type == MUL)
  {
    area_t = 280*260*2*3*num_fu*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2 ALU + MUl
    leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
    gate_leakage = area_t*(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;
  }
  else
  {
    cout<<"Unknown Functional Unit Type"<<endl;
    exit(1);
  }

  power.readOp.leakage = leakage*num_fu;
  power.readOp.gate_leakage = gate_leakage*num_fu;
  power.readOp.longer_channel_leakage = longer_channel_device_reduction(Core_device, coredynp.core_ty);
}

MFM_FunctionalUnit::MFM_FunctionalUnit(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, enum FU_type fu_type_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 fu_type(fu_type_)
{
    double area_t;//, leakage, gate_leakage;
    double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
	clockRate = coredynp.clockRate;
	executionTime = coredynp.executionTime;

  // 2 integer ALU used by MFM
  num_fu = 2;

	//XML_interface=_XML_interface;
	uca_org_t result2;
	result2 = init_interface(&interface_ip);
	if (XML->sys.Embedded)
	{
    area_t = 280*260*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2 ALU + MUl
    leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
    gate_leakage = area_t*(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;
//			base_energy = coredynp.core_ty==Inorder? 0:89e-3; //W The base energy of ALU average numbers from Intel 4G and 773Mhz (Wattch)
//			base_energy *=(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);
    base_energy = 0;
    per_access_energy = 1.15/3/1e9/4/1.3/1.3*g_tp.peri_global.Vdd*g_tp.peri_global.Vdd*(g_ip->F_sz_nm/90.0);//(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);//0.00649*1e-9; //This is per cycle energy(nJ)
    FU_height=(6222*num_fu)*interface_ip.F_sz_um;//integer ALU

    per_access_energy *=0.5;//According to ARM data embedded processor has much lower per acc energy
	}
	else
	{
			area_t = 280*260*2*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2 ALU + MUl
			leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
			gate_leakage = area_t*(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;
			base_energy = coredynp.core_ty==Inorder? 0:89e-3; //W The base energy of ALU average numbers from Intel 4G and 773Mhz (Wattch)
			base_energy *=(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);
			per_access_energy = 1.15/1e9/4/1.3/1.3*g_tp.peri_global.Vdd*g_tp.peri_global.Vdd*(g_ip->F_sz_nm/90.0);//(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);//0.00649*1e-9; //This is per cycle energy(nJ)
			FU_height=(6222*num_fu)*interface_ip.F_sz_um;//integer ALU
		}
	//IEXEU, simple ALU and FPU
	//  double C_ALU, C_EXEU, C_FPU; //Lum Equivalent capacitance of IEXEU and FPU. Based on Intel and Sun 90nm process fabracation.
	//
	//  C_ALU	  = 0.025e-9;//F
	//  C_EXEU  = 0.05e-9; //F
	//  C_FPU	  = 0.35e-9;//F
    area.set_area(area_t*num_fu);
    leakage *= num_fu;
    gate_leakage *=num_fu;
	double macro_layout_overhead = g_tp.macro_layout_overhead;
//	if (!XML->sys.Embedded)
		area.set_area(area.get_area()*macro_layout_overhead);
}

void MFM_FunctionalUnit::computeEnergy(bool is_tdp)
{
	double pppm_t[4]    = {1,1,1,1};

	double FU_duty_cycle;

	if (is_tdp)
	{
		set_pppm(pppm_t, 2, 2, 2, 2);//2 means two source operands needs to be passed for each int instruction.
	
    stats_t.readAc.access = 1*num_fu;
    tdp_stats = stats_t;
    FU_duty_cycle = XML->sys.core[ithCore].MFM.ALU_duty_cycle;

    //power.readOp.dynamic = base_energy/clockRate + energy*stats_t.readAc.access;
    power.readOp.dynamic = per_access_energy*stats_t.readAc.access + base_energy/clockRate;
		double sckRation = g_tp.sckt_co_eff;
		power.readOp.dynamic *= sckRation*FU_duty_cycle;
		power.writeOp.dynamic *= sckRation;
		power.searchOp.dynamic *= sckRation;

    power.readOp.leakage = leakage;
    power.readOp.gate_leakage = gate_leakage;
    double long_channel_device_reduction = longer_channel_device_reduction(Core_device, coredynp.core_ty);
    power.readOp.longer_channel_leakage	= power.readOp.leakage*long_channel_device_reduction;
	}
	else
	{
    stats_t.readAc.access = XML->sys.core[ithCore].MFM.alu_accesses;
    rtp_stats = stats_t;

    //rt_power.readOp.dynamic = base_energy*executionTime + energy*stats_t.readAc.access;
    rt_power.readOp.dynamic = per_access_energy*stats_t.readAc.access + base_energy*executionTime;
		double sckRation = g_tp.sckt_co_eff;
		rt_power.readOp.dynamic *= sckRation;
		rt_power.writeOp.dynamic *= sckRation;
		rt_power.searchOp.dynamic *= sckRation;
	}
}

void MFM_FunctionalUnit::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;

//	cout << indent_str_next << "Results Broadcast Bus Area = " << bypass->area.get_area() *1e-6 << " mm^2" << endl;
	if (is_tdp)
	{
    cout << indent_str << "MFM Integer ALUs (Count: "<< num_fu <<" ):" << endl;
    cout << indent_str_next << "Area = " << area.get_area()*1e-6  << " mm^2" << endl;
    cout << indent_str_next << "Peak Dynamic = " << power.readOp.dynamic*clockRate  << " W" << endl;
//			cout << indent_str_next << "Subthreshold Leakage = " << power.readOp.leakage  << " W" << endl;
    cout << indent_str_next<< "Subthreshold Leakage = "
          << (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
    cout << indent_str_next << "Gate Leakage = " << power.readOp.gate_leakage  << " W" << endl;
    cout << indent_str_next << "Runtime Dynamic = " << rt_power.readOp.dynamic/executionTime << " W" << endl;
    cout <<endl;
	}
	else
	{
	}

}

void MFM_FunctionalUnit::leakage_feedback(double temperature)
{
  // Update the temperature and initialize the global interfaces.
  interface_ip.temp = (unsigned int)round(temperature/10.0)*10;

  uca_org_t init_result = init_interface(&interface_ip); // init_result is dummy

  // This is part of MFM_FunctionalUnit()
  double area_t, leakage, gate_leakage;
  double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();

  if (fu_type == FPU)
  {
	area_t = 4.47*1e6*(g_ip->F_sz_nm*g_ip->F_sz_nm/90.0/90.0);//this is um^2 The base number
	if (g_ip->F_sz_nm>90)
		area_t = 4.47*1e6*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2
	leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(5*g_tp.min_w_nmos_, 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
	gate_leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(5*g_tp.min_w_nmos_, 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
  }
  else if (fu_type == ALU)
  {
    area_t = 280*260*2*num_fu*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2 ALU + MUl
    leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
    gate_leakage = area_t*(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;
  }
  else if (fu_type == MUL)
  {
    area_t = 280*260*2*3*num_fu*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2 ALU + MUl
    leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
    gate_leakage = area_t*(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;
  }
  else
  {
    cout<<"Unknown Functional Unit Type"<<endl;
    exit(1);
  }

  power.readOp.leakage = leakage*num_fu;
  power.readOp.gate_leakage = gate_leakage*num_fu;
  power.readOp.longer_channel_leakage = longer_channel_device_reduction(Core_device, coredynp.core_ty);
}


