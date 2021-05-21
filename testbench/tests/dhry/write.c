// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Western Digital Corporation or its affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// newlib stab replacement functions to use printf from GCC libraries in DemoTB


#include <stdint.h>

// address, snooped by DemoTB verilog
extern volatile char tohost;

int _write(int handle, char *s, int size){
    int i;
    handle = handle; // not used
    for( i=0; i<size; i++){
        tohost = s[i];
    }
    return i;
}


// function to read cpu mcycle csr for performance measurements
// simplified version
uint64_t get_mcycle(){
unsigned int mcyclel;
unsigned int mcycleh0 = 0, mcycleh1=1;
uint64_t cycles;

while(mcycleh0 != mcycleh1) {
    asm volatile ("csrr %0,mcycleh"  : "=r" (mcycleh0) );
    asm volatile ("csrr %0,mcycle"   : "=r" (mcyclel)  );
    asm volatile ("csrr %0,mcycleh"  : "=r" (mcycleh1) );
}
cycles = mcycleh1;
return (cycles << 32) | mcyclel;

}

// place initial heap to ext memory (address 0x80000000 is not used by demo TB)
char * heap = (char *) 0x80000000;

// newlib stab for _sbrk ... original uses ecall to get memory from OS.
// this one just use 'endless' ext memory
char * _sbrk (int adj){
    char * rv = heap;

   heap += adj;
   return rv;
}

// dummy newlib replacement : original uses ecall to check file status from OS
// this one does nothing, but always returns OK
int _fstat(int fd){
    return 0;
}
