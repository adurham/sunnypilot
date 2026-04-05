#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_7559082103702812710);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1140165562353213950);
void pose_H_mod_fun(double *state, double *out_7826065894907355794);
void pose_f_fun(double *state, double dt, double *out_8956214327200202745);
void pose_F_fun(double *state, double dt, double *out_475163589378229554);
void pose_h_4(double *state, double *unused, double *out_1919401175009708031);
void pose_H_4(double *state, double *unused, double *out_897543215216634988);
void pose_h_10(double *state, double *unused, double *out_4006136634733641093);
void pose_H_10(double *state, double *unused, double *out_8577831172590958455);
void pose_h_13(double *state, double *unused, double *out_655481990136332419);
void pose_H_13(double *state, double *unused, double *out_2314730610115697813);
void pose_h_14(double *state, double *unused, double *out_3170072544189949915);
void pose_H_14(double *state, double *unused, double *out_3980331647512007284);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}