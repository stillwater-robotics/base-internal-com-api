#include "stdio.h"
#include "stdint.h"

int main(int argc, char* argv[]){
    float x = 1.000;
    printf("x %f\n", x);
    uint32_t y = *((uint32_t *)((float*) &x));
    printf("y %d\n", y);
    float z = *((float*)((uint32_t *)&y));
    printf("z %f\n", z);
}