#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_3687036102391214323);
void live_err_fun(double *nom_x, double *delta_x, double *out_1252727593250109206);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3961836712919958508);
void live_H_mod_fun(double *state, double *out_6796539516515057581);
void live_f_fun(double *state, double dt, double *out_4457925187837848875);
void live_F_fun(double *state, double dt, double *out_5458000553995393310);
void live_h_4(double *state, double *unused, double *out_8713197323082769352);
void live_H_4(double *state, double *unused, double *out_5388360966612117667);
void live_h_9(double *state, double *unused, double *out_3673011700503415190);
void live_H_9(double *state, double *unused, double *out_1231193230257340184);
void live_h_10(double *state, double *unused, double *out_5246453614228271093);
void live_H_10(double *state, double *unused, double *out_8129644063251692438);
void live_h_12(double *state, double *unused, double *out_5185319623954378759);
void live_H_12(double *state, double *unused, double *out_6009459991659711334);
void live_h_35(double *state, double *unused, double *out_6471417389158011044);
void live_H_35(double *state, double *unused, double *out_8755023023984725043);
void live_h_32(double *state, double *unused, double *out_1905004663274511408);
void live_H_32(double *state, double *unused, double *out_6908062885306408980);
void live_h_13(double *state, double *unused, double *out_6640437598097171559);
void live_H_13(double *state, double *unused, double *out_4274100762822256172);
void live_h_14(double *state, double *unused, double *out_3673011700503415190);
void live_H_14(double *state, double *unused, double *out_1231193230257340184);
void live_h_33(double *state, double *unused, double *out_1819194362250859176);
void live_H_33(double *state, double *unused, double *out_6541164045085968969);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}