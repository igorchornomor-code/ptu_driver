#include "ptu_sdk.h"
#include "estrap.h"
#include <stdlib.h>
#include <enums.h>

struct ptu_handle {
    struct cerial* cer;
};


// OPEN, CLOSE
ptu_handle* ptu_open(const char* port, int* err_out) { // open connection
    if (err_out) *err_out = 0;

    int argc = 3;
    char prog[] = "ptu_driver";
    char opt[]  = "-p";
    char *argv[] = { prog, opt, (char*)port, NULL };

    struct cerial* cer = estrap_in(argc, argv);
    if (!cer) {
        if (err_out) *err_out = -1;
        return NULL;
    }

    ptu_handle* h = malloc(sizeof *h);
    if (!h) {
        if (err_out) *err_out = -2;
        estrap_out(cer);
        return NULL;
    }

    h->cer = cer;
    return h;
}

void ptu_close(ptu_handle* h) { // close connection
    if (!h) return;
    estrap_out(h->cer);
    free(h);
}




// PP, TP
int ptu_get_pan_pos(ptu_handle* h,  int* ticks_out) { // PP GET
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_PAN_CURRENT_POS_GET, ticks_out);
}

int ptu_set_pan_abs(ptu_handle* h,  int ticks) { // PP SET
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_PAN_DESIRED_POS_SET, ticks);
}


int ptu_get_tilt_pos(ptu_handle* h, int* ticks_out) { // TP GET
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_TILT_CURRENT_POS_GET, ticks_out);
}

int ptu_set_tilt_abs(ptu_handle* h,  int ticks) { // TP SET
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_TILT_DESIRED_POS_SET, ticks);
}



// C, CI, CV
int ptu_get_control_mode(ptu_handle* h, int* mode_out) { // C
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_SPEED_CONTROL_MODE_GET, mode_out);
}

// if(cpi_ptcmd(psCer, &status, OP_SPEED_CONTROL_MODE_SET,
//                 (cpi_enum)CPI_CONTROL_PURE_VELOCITY)) {
//       estrap_die("Unable to set control mode\n");
//     }

int ptu_set_velocity_mode(ptu_handle* h) { // CV
    if (!h) return -1;
    uint16_t status;
    int n = cpi_ptcmd(h->cer, &status, OP_SPEED_CONTROL_MODE_SET, (cpi_enum)CPI_CONTROL_PURE_VELOCITY);
    return n;
}

int ptu_set_position_mode(ptu_handle* h) { // CI
    if (!h) return -1;
    uint16_t status;
    int n = cpi_ptcmd(h->cer, &status, OP_SPEED_CONTROL_MODE_SET, (cpi_enum)CPI_CONTROL_INDEPENDENT);
    return n;
}





// PO, TO
int ptu_get_pan_pos_rel(ptu_handle* h,  int* ticks_out) { // PO GET
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_PAN_DESIRED_POS_GET, ticks_out);
}

int ptu_set_pan_rel(ptu_handle* h,  int ticks) { // PO SET
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_PAN_DESIRED_POS_REL_SET, ticks);
}

int ptu_get_tilt_pos_rel(ptu_handle* h,  int* ticks_out) { // TO GET
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_TILT_DESIRED_POS_GET, ticks_out);
}

int ptu_set_tilt_rel(ptu_handle* h,  int ticks) { // TO SET
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_TILT_DESIRED_POS_REL_SET, ticks);
}


// A
int ptu_await(ptu_handle* h) { // A
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_AWAIT, NULL);
}



// /* reset type */
// typedef enum _cpi_reset_type {
//     CPI_RESET_DEFAULT, CPI_RESET_NONE, CPI_RESET_ALL, CPI_RESET_PAN,
//     CPI_RESET_TILT
// }cpi_reset_type;

// R, RD, RE, RP, RT
int ptu_reset_home(ptu_handle* h, cpi_reset_type type) { // R, RD, RE, RP, RT
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_RESET, type);
}

// W (StepMode)
// enum cpi_stepmode {
//     CPI_STEP_FULL, CPI_STEP_HALF, CPI_STEP_QUARTER, CPI_STEP_AUTO
// };
// W
int ptu_set_step_mode(ptu_handle* h, cpi_stepmode stepmode) {
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_PAN_STEP_SET, stepmode);
}


// PS, TS
int ptu_get_pan_vel(ptu_handle* h,  int* ticks_per_s_out) { // PS GET
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_PAN_DESIRED_SPEED_GET, ticks_per_s_out);
}
int ptu_set_pan_vel(ptu_handle* h,  int ticks_per_s) { // PS SET
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_PAN_DESIRED_SPEED_SET, ticks_per_s);
}
int ptu_get_tilt_vel(ptu_handle* h, int* ticks_per_s_out) { // TS GET
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_TILT_DESIRED_SPEED_GET, ticks_per_s_out);
}
int ptu_set_tilt_vel(ptu_handle* h,  int ticks_per_s) { // TS SET
    if (!h) return -1;
    uint16_t status;
    return cpi_ptcmd(h->cer, &status, OP_TILT_DESIRED_SPEED_SET, ticks_per_s);
}

