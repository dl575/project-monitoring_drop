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
    MIM_ConfigTable * ct;

    bool exist;

    MIM(ParseXML *XML_interface, int ithCore_, InputParameter *interface_ip_, const CoreDynParam & dyn_p_, bool exist_=true);
    void computeEnergy(bool is_tdp=true);
    void displayEnergy(uint32_t indent = 0, int plevel = 100, bool is_tdp=true);
    ~MIM();
};



#endif
