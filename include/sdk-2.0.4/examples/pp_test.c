#include "estrap.h"
#include "stdio.h"

int main(int argc, char *argv[]){
    struct cerial   *cer;
    uint16_t        status;

    // 1. open connection (like version.c does via estrap_in)
    if((cer = estrap_in(argc, argv)) == NULL){
        return 1;
    }

    // 2. read current pan
    int pos;
    cpi_ptcmd(cer, &status, OP_PAN_CURRENT_POS_GET, &pos);
    printf("Current pan = %d\n", pos);

    // 3. move pan to 0
    cpi_ptcmd(cer, &status, OP_PAN_DESIRED_POS_SET, 400); // PP SET

    // (optionally) send await (A opcode) if you want to wait until finished

    // 4. read pan again
    cpi_ptcmd(cer, &status, OP_PAN_CURRENT_POS_GET, &pos);
    printf("Pan after move = %d\n", pos);

    // 5. close connection
    estrap_out(cer);




}