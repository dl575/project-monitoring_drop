#include "increment.h"

int sum;
static int i;

void increment_init ( void ) {
    i = 2;
}

void increment ( void ) {
   if (i){
    sum += i;
   }
}

