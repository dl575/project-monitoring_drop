
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

int main(int argc, char *argv[]) {
  register int i;

  // Monitoring packet
  struct {
    int instAddr;
    int memAddr;
    int data;
    bool store;
  } data;

  bool metadata[1024];

  for(i = 0; i < 50; i++) {
    // Read FIFO data
    memcpy(&data, (void *)(0x30000000), 3*sizeof(int) + sizeof(bool));

    printf("%x : m[%x] = %x ", data.instAddr, data.memAddr, data.data);
    // Store
    if (data.store) {
      printf("store\n");
      // Write metadata
      metadata[(data.memAddr >> 2) % 1024] = 1;
    // Load
    } else {
      printf("load\n");
      if (metadata[(data.memAddr >> 2) % 1024] == 0) {
        printf("UMC error\n");
      }
    }
  }

  return 0;
}
