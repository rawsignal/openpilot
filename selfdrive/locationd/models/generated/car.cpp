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
void err_fun(double *nom_x, double *delta_x, double *out_4865063124797718752) {
   out_4865063124797718752[0] = delta_x[0] + nom_x[0];
   out_4865063124797718752[1] = delta_x[1] + nom_x[1];
   out_4865063124797718752[2] = delta_x[2] + nom_x[2];
   out_4865063124797718752[3] = delta_x[3] + nom_x[3];
   out_4865063124797718752[4] = delta_x[4] + nom_x[4];
   out_4865063124797718752[5] = delta_x[5] + nom_x[5];
   out_4865063124797718752[6] = delta_x[6] + nom_x[6];
   out_4865063124797718752[7] = delta_x[7] + nom_x[7];
   out_4865063124797718752[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_142399555576515117) {
   out_142399555576515117[0] = -nom_x[0] + true_x[0];
   out_142399555576515117[1] = -nom_x[1] + true_x[1];
   out_142399555576515117[2] = -nom_x[2] + true_x[2];
   out_142399555576515117[3] = -nom_x[3] + true_x[3];
   out_142399555576515117[4] = -nom_x[4] + true_x[4];
   out_142399555576515117[5] = -nom_x[5] + true_x[5];
   out_142399555576515117[6] = -nom_x[6] + true_x[6];
   out_142399555576515117[7] = -nom_x[7] + true_x[7];
   out_142399555576515117[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7845543819884407850) {
   out_7845543819884407850[0] = 1.0;
   out_7845543819884407850[1] = 0.0;
   out_7845543819884407850[2] = 0.0;
   out_7845543819884407850[3] = 0.0;
   out_7845543819884407850[4] = 0.0;
   out_7845543819884407850[5] = 0.0;
   out_7845543819884407850[6] = 0.0;
   out_7845543819884407850[7] = 0.0;
   out_7845543819884407850[8] = 0.0;
   out_7845543819884407850[9] = 0.0;
   out_7845543819884407850[10] = 1.0;
   out_7845543819884407850[11] = 0.0;
   out_7845543819884407850[12] = 0.0;
   out_7845543819884407850[13] = 0.0;
   out_7845543819884407850[14] = 0.0;
   out_7845543819884407850[15] = 0.0;
   out_7845543819884407850[16] = 0.0;
   out_7845543819884407850[17] = 0.0;
   out_7845543819884407850[18] = 0.0;
   out_7845543819884407850[19] = 0.0;
   out_7845543819884407850[20] = 1.0;
   out_7845543819884407850[21] = 0.0;
   out_7845543819884407850[22] = 0.0;
   out_7845543819884407850[23] = 0.0;
   out_7845543819884407850[24] = 0.0;
   out_7845543819884407850[25] = 0.0;
   out_7845543819884407850[26] = 0.0;
   out_7845543819884407850[27] = 0.0;
   out_7845543819884407850[28] = 0.0;
   out_7845543819884407850[29] = 0.0;
   out_7845543819884407850[30] = 1.0;
   out_7845543819884407850[31] = 0.0;
   out_7845543819884407850[32] = 0.0;
   out_7845543819884407850[33] = 0.0;
   out_7845543819884407850[34] = 0.0;
   out_7845543819884407850[35] = 0.0;
   out_7845543819884407850[36] = 0.0;
   out_7845543819884407850[37] = 0.0;
   out_7845543819884407850[38] = 0.0;
   out_7845543819884407850[39] = 0.0;
   out_7845543819884407850[40] = 1.0;
   out_7845543819884407850[41] = 0.0;
   out_7845543819884407850[42] = 0.0;
   out_7845543819884407850[43] = 0.0;
   out_7845543819884407850[44] = 0.0;
   out_7845543819884407850[45] = 0.0;
   out_7845543819884407850[46] = 0.0;
   out_7845543819884407850[47] = 0.0;
   out_7845543819884407850[48] = 0.0;
   out_7845543819884407850[49] = 0.0;
   out_7845543819884407850[50] = 1.0;
   out_7845543819884407850[51] = 0.0;
   out_7845543819884407850[52] = 0.0;
   out_7845543819884407850[53] = 0.0;
   out_7845543819884407850[54] = 0.0;
   out_7845543819884407850[55] = 0.0;
   out_7845543819884407850[56] = 0.0;
   out_7845543819884407850[57] = 0.0;
   out_7845543819884407850[58] = 0.0;
   out_7845543819884407850[59] = 0.0;
   out_7845543819884407850[60] = 1.0;
   out_7845543819884407850[61] = 0.0;
   out_7845543819884407850[62] = 0.0;
   out_7845543819884407850[63] = 0.0;
   out_7845543819884407850[64] = 0.0;
   out_7845543819884407850[65] = 0.0;
   out_7845543819884407850[66] = 0.0;
   out_7845543819884407850[67] = 0.0;
   out_7845543819884407850[68] = 0.0;
   out_7845543819884407850[69] = 0.0;
   out_7845543819884407850[70] = 1.0;
   out_7845543819884407850[71] = 0.0;
   out_7845543819884407850[72] = 0.0;
   out_7845543819884407850[73] = 0.0;
   out_7845543819884407850[74] = 0.0;
   out_7845543819884407850[75] = 0.0;
   out_7845543819884407850[76] = 0.0;
   out_7845543819884407850[77] = 0.0;
   out_7845543819884407850[78] = 0.0;
   out_7845543819884407850[79] = 0.0;
   out_7845543819884407850[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2675724449816003272) {
   out_2675724449816003272[0] = state[0];
   out_2675724449816003272[1] = state[1];
   out_2675724449816003272[2] = state[2];
   out_2675724449816003272[3] = state[3];
   out_2675724449816003272[4] = state[4];
   out_2675724449816003272[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2675724449816003272[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2675724449816003272[7] = state[7];
   out_2675724449816003272[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3848783217205820649) {
   out_3848783217205820649[0] = 1;
   out_3848783217205820649[1] = 0;
   out_3848783217205820649[2] = 0;
   out_3848783217205820649[3] = 0;
   out_3848783217205820649[4] = 0;
   out_3848783217205820649[5] = 0;
   out_3848783217205820649[6] = 0;
   out_3848783217205820649[7] = 0;
   out_3848783217205820649[8] = 0;
   out_3848783217205820649[9] = 0;
   out_3848783217205820649[10] = 1;
   out_3848783217205820649[11] = 0;
   out_3848783217205820649[12] = 0;
   out_3848783217205820649[13] = 0;
   out_3848783217205820649[14] = 0;
   out_3848783217205820649[15] = 0;
   out_3848783217205820649[16] = 0;
   out_3848783217205820649[17] = 0;
   out_3848783217205820649[18] = 0;
   out_3848783217205820649[19] = 0;
   out_3848783217205820649[20] = 1;
   out_3848783217205820649[21] = 0;
   out_3848783217205820649[22] = 0;
   out_3848783217205820649[23] = 0;
   out_3848783217205820649[24] = 0;
   out_3848783217205820649[25] = 0;
   out_3848783217205820649[26] = 0;
   out_3848783217205820649[27] = 0;
   out_3848783217205820649[28] = 0;
   out_3848783217205820649[29] = 0;
   out_3848783217205820649[30] = 1;
   out_3848783217205820649[31] = 0;
   out_3848783217205820649[32] = 0;
   out_3848783217205820649[33] = 0;
   out_3848783217205820649[34] = 0;
   out_3848783217205820649[35] = 0;
   out_3848783217205820649[36] = 0;
   out_3848783217205820649[37] = 0;
   out_3848783217205820649[38] = 0;
   out_3848783217205820649[39] = 0;
   out_3848783217205820649[40] = 1;
   out_3848783217205820649[41] = 0;
   out_3848783217205820649[42] = 0;
   out_3848783217205820649[43] = 0;
   out_3848783217205820649[44] = 0;
   out_3848783217205820649[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3848783217205820649[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3848783217205820649[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3848783217205820649[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3848783217205820649[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3848783217205820649[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3848783217205820649[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3848783217205820649[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3848783217205820649[53] = -9.8000000000000007*dt;
   out_3848783217205820649[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3848783217205820649[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3848783217205820649[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3848783217205820649[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3848783217205820649[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3848783217205820649[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3848783217205820649[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3848783217205820649[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3848783217205820649[62] = 0;
   out_3848783217205820649[63] = 0;
   out_3848783217205820649[64] = 0;
   out_3848783217205820649[65] = 0;
   out_3848783217205820649[66] = 0;
   out_3848783217205820649[67] = 0;
   out_3848783217205820649[68] = 0;
   out_3848783217205820649[69] = 0;
   out_3848783217205820649[70] = 1;
   out_3848783217205820649[71] = 0;
   out_3848783217205820649[72] = 0;
   out_3848783217205820649[73] = 0;
   out_3848783217205820649[74] = 0;
   out_3848783217205820649[75] = 0;
   out_3848783217205820649[76] = 0;
   out_3848783217205820649[77] = 0;
   out_3848783217205820649[78] = 0;
   out_3848783217205820649[79] = 0;
   out_3848783217205820649[80] = 1;
}
void h_25(double *state, double *unused, double *out_2109339641109411266) {
   out_2109339641109411266[0] = state[6];
}
void H_25(double *state, double *unused, double *out_9149882930076379470) {
   out_9149882930076379470[0] = 0;
   out_9149882930076379470[1] = 0;
   out_9149882930076379470[2] = 0;
   out_9149882930076379470[3] = 0;
   out_9149882930076379470[4] = 0;
   out_9149882930076379470[5] = 0;
   out_9149882930076379470[6] = 1;
   out_9149882930076379470[7] = 0;
   out_9149882930076379470[8] = 0;
}
void h_24(double *state, double *unused, double *out_1220963728804744999) {
   out_1220963728804744999[0] = state[4];
   out_1220963728804744999[1] = state[5];
}
void H_24(double *state, double *unused, double *out_764637571092593037) {
   out_764637571092593037[0] = 0;
   out_764637571092593037[1] = 0;
   out_764637571092593037[2] = 0;
   out_764637571092593037[3] = 0;
   out_764637571092593037[4] = 1;
   out_764637571092593037[5] = 0;
   out_764637571092593037[6] = 0;
   out_764637571092593037[7] = 0;
   out_764637571092593037[8] = 0;
   out_764637571092593037[9] = 0;
   out_764637571092593037[10] = 0;
   out_764637571092593037[11] = 0;
   out_764637571092593037[12] = 0;
   out_764637571092593037[13] = 0;
   out_764637571092593037[14] = 1;
   out_764637571092593037[15] = 0;
   out_764637571092593037[16] = 0;
   out_764637571092593037[17] = 0;
}
void h_30(double *state, double *unused, double *out_1752343835319090386) {
   out_1752343835319090386[0] = state[4];
}
void H_30(double *state, double *unused, double *out_9167522196489932076) {
   out_9167522196489932076[0] = 0;
   out_9167522196489932076[1] = 0;
   out_9167522196489932076[2] = 0;
   out_9167522196489932076[3] = 0;
   out_9167522196489932076[4] = 1;
   out_9167522196489932076[5] = 0;
   out_9167522196489932076[6] = 0;
   out_9167522196489932076[7] = 0;
   out_9167522196489932076[8] = 0;
}
void h_26(double *state, double *unused, double *out_7237427866659922841) {
   out_7237427866659922841[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8493028865966067566) {
   out_8493028865966067566[0] = 0;
   out_8493028865966067566[1] = 0;
   out_8493028865966067566[2] = 0;
   out_8493028865966067566[3] = 0;
   out_8493028865966067566[4] = 0;
   out_8493028865966067566[5] = 0;
   out_8493028865966067566[6] = 0;
   out_8493028865966067566[7] = 1;
   out_8493028865966067566[8] = 0;
}
void h_27(double *state, double *unused, double *out_1127255547774113154) {
   out_1127255547774113154[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6992758884689507165) {
   out_6992758884689507165[0] = 0;
   out_6992758884689507165[1] = 0;
   out_6992758884689507165[2] = 0;
   out_6992758884689507165[3] = 1;
   out_6992758884689507165[4] = 0;
   out_6992758884689507165[5] = 0;
   out_6992758884689507165[6] = 0;
   out_6992758884689507165[7] = 0;
   out_6992758884689507165[8] = 0;
}
void h_29(double *state, double *unused, double *out_6754017102979936031) {
   out_6754017102979936031[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8768990532905227356) {
   out_8768990532905227356[0] = 0;
   out_8768990532905227356[1] = 1;
   out_8768990532905227356[2] = 0;
   out_8768990532905227356[3] = 0;
   out_8768990532905227356[4] = 0;
   out_8768990532905227356[5] = 0;
   out_8768990532905227356[6] = 0;
   out_8768990532905227356[7] = 0;
   out_8768990532905227356[8] = 0;
}
void h_28(double *state, double *unused, double *out_6089635545455406386) {
   out_6089635545455406386[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6805360261339901105) {
   out_6805360261339901105[0] = 1;
   out_6805360261339901105[1] = 0;
   out_6805360261339901105[2] = 0;
   out_6805360261339901105[3] = 0;
   out_6805360261339901105[4] = 0;
   out_6805360261339901105[5] = 0;
   out_6805360261339901105[6] = 0;
   out_6805360261339901105[7] = 0;
   out_6805360261339901105[8] = 0;
}
void h_31(double *state, double *unused, double *out_8680365730020570467) {
   out_8680365730020570467[0] = state[8];
}
void H_31(double *state, double *unused, double *out_9119236968199419042) {
   out_9119236968199419042[0] = 0;
   out_9119236968199419042[1] = 0;
   out_9119236968199419042[2] = 0;
   out_9119236968199419042[3] = 0;
   out_9119236968199419042[4] = 0;
   out_9119236968199419042[5] = 0;
   out_9119236968199419042[6] = 0;
   out_9119236968199419042[7] = 0;
   out_9119236968199419042[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4865063124797718752) {
  err_fun(nom_x, delta_x, out_4865063124797718752);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_142399555576515117) {
  inv_err_fun(nom_x, true_x, out_142399555576515117);
}
void car_H_mod_fun(double *state, double *out_7845543819884407850) {
  H_mod_fun(state, out_7845543819884407850);
}
void car_f_fun(double *state, double dt, double *out_2675724449816003272) {
  f_fun(state,  dt, out_2675724449816003272);
}
void car_F_fun(double *state, double dt, double *out_3848783217205820649) {
  F_fun(state,  dt, out_3848783217205820649);
}
void car_h_25(double *state, double *unused, double *out_2109339641109411266) {
  h_25(state, unused, out_2109339641109411266);
}
void car_H_25(double *state, double *unused, double *out_9149882930076379470) {
  H_25(state, unused, out_9149882930076379470);
}
void car_h_24(double *state, double *unused, double *out_1220963728804744999) {
  h_24(state, unused, out_1220963728804744999);
}
void car_H_24(double *state, double *unused, double *out_764637571092593037) {
  H_24(state, unused, out_764637571092593037);
}
void car_h_30(double *state, double *unused, double *out_1752343835319090386) {
  h_30(state, unused, out_1752343835319090386);
}
void car_H_30(double *state, double *unused, double *out_9167522196489932076) {
  H_30(state, unused, out_9167522196489932076);
}
void car_h_26(double *state, double *unused, double *out_7237427866659922841) {
  h_26(state, unused, out_7237427866659922841);
}
void car_H_26(double *state, double *unused, double *out_8493028865966067566) {
  H_26(state, unused, out_8493028865966067566);
}
void car_h_27(double *state, double *unused, double *out_1127255547774113154) {
  h_27(state, unused, out_1127255547774113154);
}
void car_H_27(double *state, double *unused, double *out_6992758884689507165) {
  H_27(state, unused, out_6992758884689507165);
}
void car_h_29(double *state, double *unused, double *out_6754017102979936031) {
  h_29(state, unused, out_6754017102979936031);
}
void car_H_29(double *state, double *unused, double *out_8768990532905227356) {
  H_29(state, unused, out_8768990532905227356);
}
void car_h_28(double *state, double *unused, double *out_6089635545455406386) {
  h_28(state, unused, out_6089635545455406386);
}
void car_H_28(double *state, double *unused, double *out_6805360261339901105) {
  H_28(state, unused, out_6805360261339901105);
}
void car_h_31(double *state, double *unused, double *out_8680365730020570467) {
  h_31(state, unused, out_8680365730020570467);
}
void car_H_31(double *state, double *unused, double *out_9119236968199419042) {
  H_31(state, unused, out_9119236968199419042);
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
