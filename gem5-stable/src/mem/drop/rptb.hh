/*
 * Register Producer Tracking Table
 *
 * Authors: Tao Chen
 */

#ifndef __REGISTER_PTB_HH__
#define __REGISTER_PTB_HH__

#include <iostream>
#include <fstream>

#include "arch/types.hh"
#include "config/the_isa.hh"
#include "arch/registers.hh"

class RegisterPTB
{
  private:
    struct PTBEntry
    {
        PTBEntry() : valid(false), producerPC1(0), producerPC2(0)
        {}
        // whether the entry is valid
        bool valid;
        // the entry's producers
        Addr producerPC1;
        Addr producerPC2;
    };

  public:
    /**
     * Creates a memory producer tracking table
     */
    RegisterPTB();

    void init();
    void reset();

    /**
     * Looks up register in the PTB
     */
    Addr lookup1(int reg);
    Addr lookup2(int reg);

    /**
     * Checks if a register is in the PTB
     */
    bool valid(int reg);

    /**
     * Update the producer of an memory location
     */
    void update(int reg, const Addr producerPC1, const Addr producerPC2);

    /**
     * Serialize the table to an output stream
     */
    void serialize(std::ostream &os);

    /**
     * Unserialize the table from an input stream
     */
    void unserialize(std::istream &is);

  private:
    /** The actual PTB */
    PTBEntry ptb[TheISA::NumIntRegs];

    /** The number of entries in the PTB. */
    unsigned numEntries;
};

#endif // __REGISTER_PTB_HH__