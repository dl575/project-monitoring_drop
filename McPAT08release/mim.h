#ifndef MIM_H_
#define MIM_H_

#include "XML_Parse.h"
#include "logic.h"
#include "parameter.h"
#include "array.h"
#include "interconnect.h"
#include "basic_components.h"
#include "sharedcache.h"

class MIM_RegFU :public Component {
  public:

	ParseXML *XML;
	int  ithCore;
	InputParameter interface_ip;
	CoreDynParam  coredynp;
	double clockRate,executionTime;
	double scktRatio, chip_PR_overhead, macro_PR_overhead;
	double int_regfile_height, fp_regfile_height;
	ArrayST * IRF;
	bool exist;

	MIM_RegFU(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
	~MIM_RegFU();
};


class MIM_LoadStoreU :public Component {
  public:

	ParseXML *XML;
	int  ithCore;
	InputParameter interface_ip;
	CoreDynParam  coredynp;
	enum Cache_policy cache_p;
	double clockRate,executionTime;
	double scktRatio, chip_PR_overhead, macro_PR_overhead;
	double lsq_height;
	DataCache dcache;
	ArrayST * LSQ;//it is actually the store queue but for inorder processors it serves as both loadQ and StoreQ
	ArrayST * LoadQ;
	bool exist;

	MIM_LoadStoreU(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
	~MIM_LoadStoreU();
};

class MIM_InvalidationTable : public Component {
  public:
    ParseXML *XML;
    int ithCore;
    InputParameter interface_ip;
    CoreDynParam coredynp;
    double clockRate, executionTime;
    ArrayST * InvalidationTable;
    bool exist;

	  MIM_InvalidationTable(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent=0, int plevel=100, bool is_tdp=true);
    ~MIM_InvalidationTable();
};



class MIM_ConfigTable : public Component {
  public:
    ParseXML *XML;
    int ithCore;
    InputParameter interface_ip;
    CoreDynParam coredynp;
    double clockRate, executionTime;
    ArrayST * ConfigTable;
    bool exist;

	  MIM_ConfigTable(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent=0, int plevel=100, bool is_tdp=true);
    ~MIM_ConfigTable();
};

class MFM_FilterLookupTable : public Component {
  public:
    ParseXML *XML;
    int ithCore;
    InputParameter interface_ip;
    CoreDynParam coredynp;
    double clockRate, executionTime;
    ArrayST * ConfigTable;
    bool exist;

	  MFM_FilterLookupTable(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent=0, int plevel=100, bool is_tdp=true);
    ~MFM_FilterLookupTable();
};

class MFM_ConfigTable : public Component {
  public:
    ParseXML *XML;
    int ithCore;
    InputParameter interface_ip;
    CoreDynParam coredynp;
    double clockRate, executionTime;
    ArrayST * ConfigTable;
    bool exist;

	  MFM_ConfigTable(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent=0, int plevel=100, bool is_tdp=true);
    ~MFM_ConfigTable();
};

class MIM_FunctionalUnit :public Component{
public:
	ParseXML *XML;
	int  ithCore;
	InputParameter interface_ip;
	CoreDynParam  coredynp;
	double FU_height;
	double clockRate,executionTime;
	double num_fu;
	double energy, base_energy,per_access_energy, leakage, gate_leakage;
	bool  is_default;
	enum FU_type fu_type;
	statsDef       tdp_stats;
	statsDef       rtp_stats;
	statsDef       stats_t;
	powerDef       power_t;

	MIM_FunctionalUnit(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, enum FU_type fu_type);
    void computeEnergy(bool is_tdp=true);
	void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
    void leakage_feedback(double temperature);
};

class MFM_FunctionalUnit :public Component{
public:
	ParseXML *XML;
	int  ithCore;
	InputParameter interface_ip;
	CoreDynParam  coredynp;
	double FU_height;
	double clockRate,executionTime;
	double num_fu;
	double energy, base_energy,per_access_energy, leakage, gate_leakage;
	bool  is_default;
	enum FU_type fu_type;
	statsDef       tdp_stats;
	statsDef       rtp_stats;
	statsDef       stats_t;
	powerDef       power_t;

	MFM_FunctionalUnit(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, enum FU_type fu_type);
    void computeEnergy(bool is_tdp=true);
	void displayEnergy(uint32_t indent = 0,int plevel = 100, bool is_tdp=true);
    void leakage_feedback(double temperature);
};

class IIT_BloomFilter : public Component {
  public:
    ParseXML *XML;
    int ithCore;
    InputParameter interface_ip;
    CoreDynParam coredynp;
    double clockRate, executionTime;
    ArrayST * ConfigTable;
    bool exist;

    IIT_BloomFilter(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent=0, int plevel=100, bool is_tdp=true);
    ~IIT_BloomFilter();
};

class MemoryProducerTable : public Component {
  public:
    ParseXML *XML;
    int ithCore;
    InputParameter interface_ip;
    CoreDynParam coredynp;
    double clockRate, executionTime;
    ArrayST * ConfigTable;
    bool exist;

    MemoryProducerTable(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent=0, int plevel=100, bool is_tdp=true);
    ~MemoryProducerTable();
};

class RegisterProducerTable : public Component {
  public:
    ParseXML *XML;
    int ithCore;
    InputParameter interface_ip;
    CoreDynParam coredynp;
    double clockRate, executionTime;
    ArrayST * ConfigTable;
    bool exist;

    RegisterProducerTable(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent=0, int plevel=100, bool is_tdp=true);
    ~RegisterProducerTable();
};

class MIM : public Component {
  public:
    ParseXML *XML;
    int ithCore;
    InputParameter interface_ip;
	  double clockRate, executionTime;
	  CoreDynParam coredynp;
    MIM_FunctionalUnit * alu;
    MIM_RegFU * rfu;
    MIM_LoadStoreU * lsu;
    MIM_InvalidationTable * mit;
    MIM_ConfigTable * ct;
    MFM_FunctionalUnit * mfm_alu;
    MFM_ConfigTable * mfm_ct;
    MFM_FilterLookupTable * mfm_flt;
    IIT_BloomFilter *iit;
    MemoryProducerTable *mpt;
    RegisterProducerTable *rpt;

    bool exist;

    MIM(ParseXML *XML_interface, int ithCore_, InputParameter *interface_ip_, const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0, int plevel = 100, bool is_tdp=true);
    ~MIM();
};

#endif
