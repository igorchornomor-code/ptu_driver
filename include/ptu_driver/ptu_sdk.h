#pragma once
#include <enums.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle so C++ side doesn't care about SDK internals
struct ptu_handle;
typedef struct ptu_handle ptu_handle;


// Lifecycle
ptu_handle* ptu_open(const char* port, int* err_out);
void        ptu_close(ptu_handle* h);


// Modes (C / CI / CV)
int ptu_get_control_mode(ptu_handle* h, int* mode_out); // C
int ptu_set_position_mode(ptu_handle* h);  // CI
int ptu_set_velocity_mode(ptu_handle* h);  // CV


// Position control (R, PP, TP, PO, TO, A)
int ptu_reset_home(ptu_handle* h, cpi_reset_type type); // R

int ptu_get_pan_pos(ptu_handle* h,  int* ticks_out); // PP get
int ptu_get_tilt_pos(ptu_handle* h, int* ticks_out); // TP get

int ptu_set_pan_abs(ptu_handle* h,  int ticks); // PP set
int ptu_set_tilt_abs(ptu_handle* h, int ticks); // TP set
int ptu_set_pan_rel(ptu_handle* h,  int ticks); // PO set
int ptu_get_pan_pos_rel(ptu_handle* h,  int* ticks_out); // PO get
int ptu_set_tilt_rel(ptu_handle* h,  int ticks); // TO set
int ptu_get_tilt_pos_rel(ptu_handle* h,  int* ticks_out); // TO get
int ptu_await(ptu_handle* h); // A

// Velocity control (PS, TS)
int ptu_get_pan_vel(ptu_handle* h,  int* ticks_per_s_out); // PS GET
int ptu_set_pan_vel(ptu_handle* h,  int ticks_per_s);      // PS SET
int ptu_get_tilt_vel(ptu_handle* h, int* ticks_per_s_out); // TS GET
int ptu_set_tilt_vel(ptu_handle* h, int ticks_per_s);      // TS GET

// Configuring Step Mode W
int ptu_set_step_mode(ptu_handle* h, cpi_stepmode stepmode); // W


#ifdef __cplusplus
}
#endif