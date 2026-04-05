#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_9193419516305296476) {
   out_9193419516305296476[0] = delta_x[0] + nom_x[0];
   out_9193419516305296476[1] = delta_x[1] + nom_x[1];
   out_9193419516305296476[2] = delta_x[2] + nom_x[2];
   out_9193419516305296476[3] = delta_x[3] + nom_x[3];
   out_9193419516305296476[4] = delta_x[4] + nom_x[4];
   out_9193419516305296476[5] = delta_x[5] + nom_x[5];
   out_9193419516305296476[6] = delta_x[6] + nom_x[6];
   out_9193419516305296476[7] = delta_x[7] + nom_x[7];
   out_9193419516305296476[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2380864960194282114) {
   out_2380864960194282114[0] = -nom_x[0] + true_x[0];
   out_2380864960194282114[1] = -nom_x[1] + true_x[1];
   out_2380864960194282114[2] = -nom_x[2] + true_x[2];
   out_2380864960194282114[3] = -nom_x[3] + true_x[3];
   out_2380864960194282114[4] = -nom_x[4] + true_x[4];
   out_2380864960194282114[5] = -nom_x[5] + true_x[5];
   out_2380864960194282114[6] = -nom_x[6] + true_x[6];
   out_2380864960194282114[7] = -nom_x[7] + true_x[7];
   out_2380864960194282114[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6853149165695128374) {
   out_6853149165695128374[0] = 1.0;
   out_6853149165695128374[1] = 0.0;
   out_6853149165695128374[2] = 0.0;
   out_6853149165695128374[3] = 0.0;
   out_6853149165695128374[4] = 0.0;
   out_6853149165695128374[5] = 0.0;
   out_6853149165695128374[6] = 0.0;
   out_6853149165695128374[7] = 0.0;
   out_6853149165695128374[8] = 0.0;
   out_6853149165695128374[9] = 0.0;
   out_6853149165695128374[10] = 1.0;
   out_6853149165695128374[11] = 0.0;
   out_6853149165695128374[12] = 0.0;
   out_6853149165695128374[13] = 0.0;
   out_6853149165695128374[14] = 0.0;
   out_6853149165695128374[15] = 0.0;
   out_6853149165695128374[16] = 0.0;
   out_6853149165695128374[17] = 0.0;
   out_6853149165695128374[18] = 0.0;
   out_6853149165695128374[19] = 0.0;
   out_6853149165695128374[20] = 1.0;
   out_6853149165695128374[21] = 0.0;
   out_6853149165695128374[22] = 0.0;
   out_6853149165695128374[23] = 0.0;
   out_6853149165695128374[24] = 0.0;
   out_6853149165695128374[25] = 0.0;
   out_6853149165695128374[26] = 0.0;
   out_6853149165695128374[27] = 0.0;
   out_6853149165695128374[28] = 0.0;
   out_6853149165695128374[29] = 0.0;
   out_6853149165695128374[30] = 1.0;
   out_6853149165695128374[31] = 0.0;
   out_6853149165695128374[32] = 0.0;
   out_6853149165695128374[33] = 0.0;
   out_6853149165695128374[34] = 0.0;
   out_6853149165695128374[35] = 0.0;
   out_6853149165695128374[36] = 0.0;
   out_6853149165695128374[37] = 0.0;
   out_6853149165695128374[38] = 0.0;
   out_6853149165695128374[39] = 0.0;
   out_6853149165695128374[40] = 1.0;
   out_6853149165695128374[41] = 0.0;
   out_6853149165695128374[42] = 0.0;
   out_6853149165695128374[43] = 0.0;
   out_6853149165695128374[44] = 0.0;
   out_6853149165695128374[45] = 0.0;
   out_6853149165695128374[46] = 0.0;
   out_6853149165695128374[47] = 0.0;
   out_6853149165695128374[48] = 0.0;
   out_6853149165695128374[49] = 0.0;
   out_6853149165695128374[50] = 1.0;
   out_6853149165695128374[51] = 0.0;
   out_6853149165695128374[52] = 0.0;
   out_6853149165695128374[53] = 0.0;
   out_6853149165695128374[54] = 0.0;
   out_6853149165695128374[55] = 0.0;
   out_6853149165695128374[56] = 0.0;
   out_6853149165695128374[57] = 0.0;
   out_6853149165695128374[58] = 0.0;
   out_6853149165695128374[59] = 0.0;
   out_6853149165695128374[60] = 1.0;
   out_6853149165695128374[61] = 0.0;
   out_6853149165695128374[62] = 0.0;
   out_6853149165695128374[63] = 0.0;
   out_6853149165695128374[64] = 0.0;
   out_6853149165695128374[65] = 0.0;
   out_6853149165695128374[66] = 0.0;
   out_6853149165695128374[67] = 0.0;
   out_6853149165695128374[68] = 0.0;
   out_6853149165695128374[69] = 0.0;
   out_6853149165695128374[70] = 1.0;
   out_6853149165695128374[71] = 0.0;
   out_6853149165695128374[72] = 0.0;
   out_6853149165695128374[73] = 0.0;
   out_6853149165695128374[74] = 0.0;
   out_6853149165695128374[75] = 0.0;
   out_6853149165695128374[76] = 0.0;
   out_6853149165695128374[77] = 0.0;
   out_6853149165695128374[78] = 0.0;
   out_6853149165695128374[79] = 0.0;
   out_6853149165695128374[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2034959093296113521) {
   out_2034959093296113521[0] = state[0];
   out_2034959093296113521[1] = state[1];
   out_2034959093296113521[2] = state[2];
   out_2034959093296113521[3] = state[3];
   out_2034959093296113521[4] = state[4];
   out_2034959093296113521[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2034959093296113521[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2034959093296113521[7] = state[7];
   out_2034959093296113521[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8286094502398237690) {
   out_8286094502398237690[0] = 1;
   out_8286094502398237690[1] = 0;
   out_8286094502398237690[2] = 0;
   out_8286094502398237690[3] = 0;
   out_8286094502398237690[4] = 0;
   out_8286094502398237690[5] = 0;
   out_8286094502398237690[6] = 0;
   out_8286094502398237690[7] = 0;
   out_8286094502398237690[8] = 0;
   out_8286094502398237690[9] = 0;
   out_8286094502398237690[10] = 1;
   out_8286094502398237690[11] = 0;
   out_8286094502398237690[12] = 0;
   out_8286094502398237690[13] = 0;
   out_8286094502398237690[14] = 0;
   out_8286094502398237690[15] = 0;
   out_8286094502398237690[16] = 0;
   out_8286094502398237690[17] = 0;
   out_8286094502398237690[18] = 0;
   out_8286094502398237690[19] = 0;
   out_8286094502398237690[20] = 1;
   out_8286094502398237690[21] = 0;
   out_8286094502398237690[22] = 0;
   out_8286094502398237690[23] = 0;
   out_8286094502398237690[24] = 0;
   out_8286094502398237690[25] = 0;
   out_8286094502398237690[26] = 0;
   out_8286094502398237690[27] = 0;
   out_8286094502398237690[28] = 0;
   out_8286094502398237690[29] = 0;
   out_8286094502398237690[30] = 1;
   out_8286094502398237690[31] = 0;
   out_8286094502398237690[32] = 0;
   out_8286094502398237690[33] = 0;
   out_8286094502398237690[34] = 0;
   out_8286094502398237690[35] = 0;
   out_8286094502398237690[36] = 0;
   out_8286094502398237690[37] = 0;
   out_8286094502398237690[38] = 0;
   out_8286094502398237690[39] = 0;
   out_8286094502398237690[40] = 1;
   out_8286094502398237690[41] = 0;
   out_8286094502398237690[42] = 0;
   out_8286094502398237690[43] = 0;
   out_8286094502398237690[44] = 0;
   out_8286094502398237690[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8286094502398237690[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8286094502398237690[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8286094502398237690[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8286094502398237690[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8286094502398237690[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8286094502398237690[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8286094502398237690[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8286094502398237690[53] = -9.8100000000000005*dt;
   out_8286094502398237690[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8286094502398237690[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8286094502398237690[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8286094502398237690[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8286094502398237690[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8286094502398237690[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8286094502398237690[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8286094502398237690[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8286094502398237690[62] = 0;
   out_8286094502398237690[63] = 0;
   out_8286094502398237690[64] = 0;
   out_8286094502398237690[65] = 0;
   out_8286094502398237690[66] = 0;
   out_8286094502398237690[67] = 0;
   out_8286094502398237690[68] = 0;
   out_8286094502398237690[69] = 0;
   out_8286094502398237690[70] = 1;
   out_8286094502398237690[71] = 0;
   out_8286094502398237690[72] = 0;
   out_8286094502398237690[73] = 0;
   out_8286094502398237690[74] = 0;
   out_8286094502398237690[75] = 0;
   out_8286094502398237690[76] = 0;
   out_8286094502398237690[77] = 0;
   out_8286094502398237690[78] = 0;
   out_8286094502398237690[79] = 0;
   out_8286094502398237690[80] = 1;
}
void h_25(double *state, double *unused, double *out_8179286628075921867) {
   out_8179286628075921867[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7641583892171962925) {
   out_7641583892171962925[0] = 0;
   out_7641583892171962925[1] = 0;
   out_7641583892171962925[2] = 0;
   out_7641583892171962925[3] = 0;
   out_7641583892171962925[4] = 0;
   out_7641583892171962925[5] = 0;
   out_7641583892171962925[6] = 1;
   out_7641583892171962925[7] = 0;
   out_7641583892171962925[8] = 0;
}
void h_24(double *state, double *unused, double *out_6388349683662581229) {
   out_6388349683662581229[0] = state[4];
   out_6388349683662581229[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2419914822553802258) {
   out_2419914822553802258[0] = 0;
   out_2419914822553802258[1] = 0;
   out_2419914822553802258[2] = 0;
   out_2419914822553802258[3] = 0;
   out_2419914822553802258[4] = 1;
   out_2419914822553802258[5] = 0;
   out_2419914822553802258[6] = 0;
   out_2419914822553802258[7] = 0;
   out_2419914822553802258[8] = 0;
   out_2419914822553802258[9] = 0;
   out_2419914822553802258[10] = 0;
   out_2419914822553802258[11] = 0;
   out_2419914822553802258[12] = 0;
   out_2419914822553802258[13] = 0;
   out_2419914822553802258[14] = 1;
   out_2419914822553802258[15] = 0;
   out_2419914822553802258[16] = 0;
   out_2419914822553802258[17] = 0;
}
void h_30(double *state, double *unused, double *out_6630007917379607399) {
   out_6630007917379607399[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8286827223030340064) {
   out_8286827223030340064[0] = 0;
   out_8286827223030340064[1] = 0;
   out_8286827223030340064[2] = 0;
   out_8286827223030340064[3] = 0;
   out_8286827223030340064[4] = 1;
   out_8286827223030340064[5] = 0;
   out_8286827223030340064[6] = 0;
   out_8286827223030340064[7] = 0;
   out_8286827223030340064[8] = 0;
}
void h_26(double *state, double *unused, double *out_3596739044426561189) {
   out_3596739044426561189[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7500634211776788090) {
   out_7500634211776788090[0] = 0;
   out_7500634211776788090[1] = 0;
   out_7500634211776788090[2] = 0;
   out_7500634211776788090[3] = 0;
   out_7500634211776788090[4] = 0;
   out_7500634211776788090[5] = 0;
   out_7500634211776788090[6] = 0;
   out_7500634211776788090[7] = 1;
   out_7500634211776788090[8] = 0;
}
void h_27(double *state, double *unused, double *out_5265784177972784104) {
   out_5265784177972784104[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6063233151846396847) {
   out_6063233151846396847[0] = 0;
   out_6063233151846396847[1] = 0;
   out_6063233151846396847[2] = 0;
   out_6063233151846396847[3] = 1;
   out_6063233151846396847[4] = 0;
   out_6063233151846396847[5] = 0;
   out_6063233151846396847[6] = 0;
   out_6063233151846396847[7] = 0;
   out_6063233151846396847[8] = 0;
}
void h_29(double *state, double *unused, double *out_1628334649718761754) {
   out_1628334649718761754[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7776595878715947880) {
   out_7776595878715947880[0] = 0;
   out_7776595878715947880[1] = 1;
   out_7776595878715947880[2] = 0;
   out_7776595878715947880[3] = 0;
   out_7776595878715947880[4] = 0;
   out_7776595878715947880[5] = 0;
   out_7776595878715947880[6] = 0;
   out_7776595878715947880[7] = 0;
   out_7776595878715947880[8] = 0;
}
void h_28(double *state, double *unused, double *out_3452052741895908222) {
   out_3452052741895908222[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5587749177924073162) {
   out_5587749177924073162[0] = 1;
   out_5587749177924073162[1] = 0;
   out_5587749177924073162[2] = 0;
   out_5587749177924073162[3] = 0;
   out_5587749177924073162[4] = 0;
   out_5587749177924073162[5] = 0;
   out_5587749177924073162[6] = 0;
   out_5587749177924073162[7] = 0;
   out_5587749177924073162[8] = 0;
}
void h_31(double *state, double *unused, double *out_7904092565791415978) {
   out_7904092565791415978[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3273872471064555225) {
   out_3273872471064555225[0] = 0;
   out_3273872471064555225[1] = 0;
   out_3273872471064555225[2] = 0;
   out_3273872471064555225[3] = 0;
   out_3273872471064555225[4] = 0;
   out_3273872471064555225[5] = 0;
   out_3273872471064555225[6] = 0;
   out_3273872471064555225[7] = 0;
   out_3273872471064555225[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_9193419516305296476) {
  err_fun(nom_x, delta_x, out_9193419516305296476);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2380864960194282114) {
  inv_err_fun(nom_x, true_x, out_2380864960194282114);
}
void car_H_mod_fun(double *state, double *out_6853149165695128374) {
  H_mod_fun(state, out_6853149165695128374);
}
void car_f_fun(double *state, double dt, double *out_2034959093296113521) {
  f_fun(state,  dt, out_2034959093296113521);
}
void car_F_fun(double *state, double dt, double *out_8286094502398237690) {
  F_fun(state,  dt, out_8286094502398237690);
}
void car_h_25(double *state, double *unused, double *out_8179286628075921867) {
  h_25(state, unused, out_8179286628075921867);
}
void car_H_25(double *state, double *unused, double *out_7641583892171962925) {
  H_25(state, unused, out_7641583892171962925);
}
void car_h_24(double *state, double *unused, double *out_6388349683662581229) {
  h_24(state, unused, out_6388349683662581229);
}
void car_H_24(double *state, double *unused, double *out_2419914822553802258) {
  H_24(state, unused, out_2419914822553802258);
}
void car_h_30(double *state, double *unused, double *out_6630007917379607399) {
  h_30(state, unused, out_6630007917379607399);
}
void car_H_30(double *state, double *unused, double *out_8286827223030340064) {
  H_30(state, unused, out_8286827223030340064);
}
void car_h_26(double *state, double *unused, double *out_3596739044426561189) {
  h_26(state, unused, out_3596739044426561189);
}
void car_H_26(double *state, double *unused, double *out_7500634211776788090) {
  H_26(state, unused, out_7500634211776788090);
}
void car_h_27(double *state, double *unused, double *out_5265784177972784104) {
  h_27(state, unused, out_5265784177972784104);
}
void car_H_27(double *state, double *unused, double *out_6063233151846396847) {
  H_27(state, unused, out_6063233151846396847);
}
void car_h_29(double *state, double *unused, double *out_1628334649718761754) {
  h_29(state, unused, out_1628334649718761754);
}
void car_H_29(double *state, double *unused, double *out_7776595878715947880) {
  H_29(state, unused, out_7776595878715947880);
}
void car_h_28(double *state, double *unused, double *out_3452052741895908222) {
  h_28(state, unused, out_3452052741895908222);
}
void car_H_28(double *state, double *unused, double *out_5587749177924073162) {
  H_28(state, unused, out_5587749177924073162);
}
void car_h_31(double *state, double *unused, double *out_7904092565791415978) {
  h_31(state, unused, out_7904092565791415978);
}
void car_H_31(double *state, double *unused, double *out_3273872471064555225) {
  H_31(state, unused, out_3273872471064555225);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
