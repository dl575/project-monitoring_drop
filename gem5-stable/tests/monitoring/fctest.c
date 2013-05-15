
/*
 * All variables local and initialized. Should pass with no error.
 */

#include <stdio.h>

#include "flagcache.h"

int main(int argc, char *argv[]) {
  INIT_FC;
  
  unsigned int i;
  
  // Test array
  for (i = 0; i < 32; i+=8){
    FC_SET_ADDR(i);
    unsigned int addr = FC_GET_ADDR;
    if (i != addr) { 
        printf("Could not set address: %x\n", addr);
        return 1; 
    }
    FC_ARRAY_SET;
    unsigned int flag = FC_ARRAY_GET;
    if (flag != 1){
        printf("Could not set array: %x\n", flag);
        return 1;
    }
    FC_ARRAY_CLEAR;
    flag = FC_ARRAY_GET;
    if (flag != 0){
        printf("Could not set array: %x\n", flag);
        return 1;
    }
  }
  
  // Test cache
  for (i = 0; i < 512; i+=64){
    FC_SET_ADDR(i);
    FC_CACHE_SET;
    unsigned int flag = FC_CACHE_GET;
    if (flag != 1){
        printf("Could not set cache: %x\n", flag);
        return 1;
    }
    FC_CACHE_CLEAR;
    FC_CACHE_SET;
  }
  
  for (i = 0; i < 512; i+=32){
    FC_SET_ADDR(i);
    FC_CACHE_CLEAR;
  }

  return 0;
}
