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
void err_fun(double *nom_x, double *delta_x, double *out_6466759103940957948) {
   out_6466759103940957948[0] = delta_x[0] + nom_x[0];
   out_6466759103940957948[1] = delta_x[1] + nom_x[1];
   out_6466759103940957948[2] = delta_x[2] + nom_x[2];
   out_6466759103940957948[3] = delta_x[3] + nom_x[3];
   out_6466759103940957948[4] = delta_x[4] + nom_x[4];
   out_6466759103940957948[5] = delta_x[5] + nom_x[5];
   out_6466759103940957948[6] = delta_x[6] + nom_x[6];
   out_6466759103940957948[7] = delta_x[7] + nom_x[7];
   out_6466759103940957948[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6301621981866168795) {
   out_6301621981866168795[0] = -nom_x[0] + true_x[0];
   out_6301621981866168795[1] = -nom_x[1] + true_x[1];
   out_6301621981866168795[2] = -nom_x[2] + true_x[2];
   out_6301621981866168795[3] = -nom_x[3] + true_x[3];
   out_6301621981866168795[4] = -nom_x[4] + true_x[4];
   out_6301621981866168795[5] = -nom_x[5] + true_x[5];
   out_6301621981866168795[6] = -nom_x[6] + true_x[6];
   out_6301621981866168795[7] = -nom_x[7] + true_x[7];
   out_6301621981866168795[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5475158372291201128) {
   out_5475158372291201128[0] = 1.0;
   out_5475158372291201128[1] = 0.0;
   out_5475158372291201128[2] = 0.0;
   out_5475158372291201128[3] = 0.0;
   out_5475158372291201128[4] = 0.0;
   out_5475158372291201128[5] = 0.0;
   out_5475158372291201128[6] = 0.0;
   out_5475158372291201128[7] = 0.0;
   out_5475158372291201128[8] = 0.0;
   out_5475158372291201128[9] = 0.0;
   out_5475158372291201128[10] = 1.0;
   out_5475158372291201128[11] = 0.0;
   out_5475158372291201128[12] = 0.0;
   out_5475158372291201128[13] = 0.0;
   out_5475158372291201128[14] = 0.0;
   out_5475158372291201128[15] = 0.0;
   out_5475158372291201128[16] = 0.0;
   out_5475158372291201128[17] = 0.0;
   out_5475158372291201128[18] = 0.0;
   out_5475158372291201128[19] = 0.0;
   out_5475158372291201128[20] = 1.0;
   out_5475158372291201128[21] = 0.0;
   out_5475158372291201128[22] = 0.0;
   out_5475158372291201128[23] = 0.0;
   out_5475158372291201128[24] = 0.0;
   out_5475158372291201128[25] = 0.0;
   out_5475158372291201128[26] = 0.0;
   out_5475158372291201128[27] = 0.0;
   out_5475158372291201128[28] = 0.0;
   out_5475158372291201128[29] = 0.0;
   out_5475158372291201128[30] = 1.0;
   out_5475158372291201128[31] = 0.0;
   out_5475158372291201128[32] = 0.0;
   out_5475158372291201128[33] = 0.0;
   out_5475158372291201128[34] = 0.0;
   out_5475158372291201128[35] = 0.0;
   out_5475158372291201128[36] = 0.0;
   out_5475158372291201128[37] = 0.0;
   out_5475158372291201128[38] = 0.0;
   out_5475158372291201128[39] = 0.0;
   out_5475158372291201128[40] = 1.0;
   out_5475158372291201128[41] = 0.0;
   out_5475158372291201128[42] = 0.0;
   out_5475158372291201128[43] = 0.0;
   out_5475158372291201128[44] = 0.0;
   out_5475158372291201128[45] = 0.0;
   out_5475158372291201128[46] = 0.0;
   out_5475158372291201128[47] = 0.0;
   out_5475158372291201128[48] = 0.0;
   out_5475158372291201128[49] = 0.0;
   out_5475158372291201128[50] = 1.0;
   out_5475158372291201128[51] = 0.0;
   out_5475158372291201128[52] = 0.0;
   out_5475158372291201128[53] = 0.0;
   out_5475158372291201128[54] = 0.0;
   out_5475158372291201128[55] = 0.0;
   out_5475158372291201128[56] = 0.0;
   out_5475158372291201128[57] = 0.0;
   out_5475158372291201128[58] = 0.0;
   out_5475158372291201128[59] = 0.0;
   out_5475158372291201128[60] = 1.0;
   out_5475158372291201128[61] = 0.0;
   out_5475158372291201128[62] = 0.0;
   out_5475158372291201128[63] = 0.0;
   out_5475158372291201128[64] = 0.0;
   out_5475158372291201128[65] = 0.0;
   out_5475158372291201128[66] = 0.0;
   out_5475158372291201128[67] = 0.0;
   out_5475158372291201128[68] = 0.0;
   out_5475158372291201128[69] = 0.0;
   out_5475158372291201128[70] = 1.0;
   out_5475158372291201128[71] = 0.0;
   out_5475158372291201128[72] = 0.0;
   out_5475158372291201128[73] = 0.0;
   out_5475158372291201128[74] = 0.0;
   out_5475158372291201128[75] = 0.0;
   out_5475158372291201128[76] = 0.0;
   out_5475158372291201128[77] = 0.0;
   out_5475158372291201128[78] = 0.0;
   out_5475158372291201128[79] = 0.0;
   out_5475158372291201128[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1322705351199537539) {
   out_1322705351199537539[0] = state[0];
   out_1322705351199537539[1] = state[1];
   out_1322705351199537539[2] = state[2];
   out_1322705351199537539[3] = state[3];
   out_1322705351199537539[4] = state[4];
   out_1322705351199537539[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1322705351199537539[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1322705351199537539[7] = state[7];
   out_1322705351199537539[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3283955945887505467) {
   out_3283955945887505467[0] = 1;
   out_3283955945887505467[1] = 0;
   out_3283955945887505467[2] = 0;
   out_3283955945887505467[3] = 0;
   out_3283955945887505467[4] = 0;
   out_3283955945887505467[5] = 0;
   out_3283955945887505467[6] = 0;
   out_3283955945887505467[7] = 0;
   out_3283955945887505467[8] = 0;
   out_3283955945887505467[9] = 0;
   out_3283955945887505467[10] = 1;
   out_3283955945887505467[11] = 0;
   out_3283955945887505467[12] = 0;
   out_3283955945887505467[13] = 0;
   out_3283955945887505467[14] = 0;
   out_3283955945887505467[15] = 0;
   out_3283955945887505467[16] = 0;
   out_3283955945887505467[17] = 0;
   out_3283955945887505467[18] = 0;
   out_3283955945887505467[19] = 0;
   out_3283955945887505467[20] = 1;
   out_3283955945887505467[21] = 0;
   out_3283955945887505467[22] = 0;
   out_3283955945887505467[23] = 0;
   out_3283955945887505467[24] = 0;
   out_3283955945887505467[25] = 0;
   out_3283955945887505467[26] = 0;
   out_3283955945887505467[27] = 0;
   out_3283955945887505467[28] = 0;
   out_3283955945887505467[29] = 0;
   out_3283955945887505467[30] = 1;
   out_3283955945887505467[31] = 0;
   out_3283955945887505467[32] = 0;
   out_3283955945887505467[33] = 0;
   out_3283955945887505467[34] = 0;
   out_3283955945887505467[35] = 0;
   out_3283955945887505467[36] = 0;
   out_3283955945887505467[37] = 0;
   out_3283955945887505467[38] = 0;
   out_3283955945887505467[39] = 0;
   out_3283955945887505467[40] = 1;
   out_3283955945887505467[41] = 0;
   out_3283955945887505467[42] = 0;
   out_3283955945887505467[43] = 0;
   out_3283955945887505467[44] = 0;
   out_3283955945887505467[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3283955945887505467[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3283955945887505467[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3283955945887505467[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3283955945887505467[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3283955945887505467[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3283955945887505467[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3283955945887505467[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3283955945887505467[53] = -9.8000000000000007*dt;
   out_3283955945887505467[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3283955945887505467[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3283955945887505467[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3283955945887505467[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3283955945887505467[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3283955945887505467[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3283955945887505467[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3283955945887505467[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3283955945887505467[62] = 0;
   out_3283955945887505467[63] = 0;
   out_3283955945887505467[64] = 0;
   out_3283955945887505467[65] = 0;
   out_3283955945887505467[66] = 0;
   out_3283955945887505467[67] = 0;
   out_3283955945887505467[68] = 0;
   out_3283955945887505467[69] = 0;
   out_3283955945887505467[70] = 1;
   out_3283955945887505467[71] = 0;
   out_3283955945887505467[72] = 0;
   out_3283955945887505467[73] = 0;
   out_3283955945887505467[74] = 0;
   out_3283955945887505467[75] = 0;
   out_3283955945887505467[76] = 0;
   out_3283955945887505467[77] = 0;
   out_3283955945887505467[78] = 0;
   out_3283955945887505467[79] = 0;
   out_3283955945887505467[80] = 1;
}
void h_25(double *state, double *unused, double *out_7334748793472295430) {
   out_7334748793472295430[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3718029853621036109) {
   out_3718029853621036109[0] = 0;
   out_3718029853621036109[1] = 0;
   out_3718029853621036109[2] = 0;
   out_3718029853621036109[3] = 0;
   out_3718029853621036109[4] = 0;
   out_3718029853621036109[5] = 0;
   out_3718029853621036109[6] = 1;
   out_3718029853621036109[7] = 0;
   out_3718029853621036109[8] = 0;
}
void h_24(double *state, double *unused, double *out_8075960681212006981) {
   out_8075960681212006981[0] = state[4];
   out_8075960681212006981[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2936175604487149648) {
   out_2936175604487149648[0] = 0;
   out_2936175604487149648[1] = 0;
   out_2936175604487149648[2] = 0;
   out_2936175604487149648[3] = 0;
   out_2936175604487149648[4] = 1;
   out_2936175604487149648[5] = 0;
   out_2936175604487149648[6] = 0;
   out_2936175604487149648[7] = 0;
   out_2936175604487149648[8] = 0;
   out_2936175604487149648[9] = 0;
   out_2936175604487149648[10] = 0;
   out_2936175604487149648[11] = 0;
   out_2936175604487149648[12] = 0;
   out_2936175604487149648[13] = 0;
   out_2936175604487149648[14] = 1;
   out_2936175604487149648[15] = 0;
   out_2936175604487149648[16] = 0;
   out_2936175604487149648[17] = 0;
}
void h_30(double *state, double *unused, double *out_3348730023416583745) {
   out_3348730023416583745[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8245726183748644307) {
   out_8245726183748644307[0] = 0;
   out_8245726183748644307[1] = 0;
   out_8245726183748644307[2] = 0;
   out_8245726183748644307[3] = 0;
   out_8245726183748644307[4] = 1;
   out_8245726183748644307[5] = 0;
   out_8245726183748644307[6] = 0;
   out_8245726183748644307[7] = 0;
   out_8245726183748644307[8] = 0;
}
void h_26(double *state, double *unused, double *out_4455149410379091890) {
   out_4455149410379091890[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7459533172495092333) {
   out_7459533172495092333[0] = 0;
   out_7459533172495092333[1] = 0;
   out_7459533172495092333[2] = 0;
   out_7459533172495092333[3] = 0;
   out_7459533172495092333[4] = 0;
   out_7459533172495092333[5] = 0;
   out_7459533172495092333[6] = 0;
   out_7459533172495092333[7] = 1;
   out_7459533172495092333[8] = 0;
}
void h_27(double *state, double *unused, double *out_5626911248896423731) {
   out_5626911248896423731[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6022132112564701090) {
   out_6022132112564701090[0] = 0;
   out_6022132112564701090[1] = 0;
   out_6022132112564701090[2] = 0;
   out_6022132112564701090[3] = 1;
   out_6022132112564701090[4] = 0;
   out_6022132112564701090[5] = 0;
   out_6022132112564701090[6] = 0;
   out_6022132112564701090[7] = 0;
   out_6022132112564701090[8] = 0;
}
void h_29(double *state, double *unused, double *out_8574637239251908692) {
   out_8574637239251908692[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7735494839434252123) {
   out_7735494839434252123[0] = 0;
   out_7735494839434252123[1] = 1;
   out_7735494839434252123[2] = 0;
   out_7735494839434252123[3] = 0;
   out_7735494839434252123[4] = 0;
   out_7735494839434252123[5] = 0;
   out_7735494839434252123[6] = 0;
   out_7735494839434252123[7] = 0;
   out_7735494839434252123[8] = 0;
}
void h_28(double *state, double *unused, double *out_4269015409183433725) {
   out_4269015409183433725[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5628850217205768919) {
   out_5628850217205768919[0] = 1;
   out_5628850217205768919[1] = 0;
   out_5628850217205768919[2] = 0;
   out_5628850217205768919[3] = 0;
   out_5628850217205768919[4] = 0;
   out_5628850217205768919[5] = 0;
   out_5628850217205768919[6] = 0;
   out_5628850217205768919[7] = 0;
   out_5628850217205768919[8] = 0;
}
void h_31(double *state, double *unused, double *out_7171839259411449720) {
   out_7171839259411449720[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3687383891744075681) {
   out_3687383891744075681[0] = 0;
   out_3687383891744075681[1] = 0;
   out_3687383891744075681[2] = 0;
   out_3687383891744075681[3] = 0;
   out_3687383891744075681[4] = 0;
   out_3687383891744075681[5] = 0;
   out_3687383891744075681[6] = 0;
   out_3687383891744075681[7] = 0;
   out_3687383891744075681[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6466759103940957948) {
  err_fun(nom_x, delta_x, out_6466759103940957948);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6301621981866168795) {
  inv_err_fun(nom_x, true_x, out_6301621981866168795);
}
void car_H_mod_fun(double *state, double *out_5475158372291201128) {
  H_mod_fun(state, out_5475158372291201128);
}
void car_f_fun(double *state, double dt, double *out_1322705351199537539) {
  f_fun(state,  dt, out_1322705351199537539);
}
void car_F_fun(double *state, double dt, double *out_3283955945887505467) {
  F_fun(state,  dt, out_3283955945887505467);
}
void car_h_25(double *state, double *unused, double *out_7334748793472295430) {
  h_25(state, unused, out_7334748793472295430);
}
void car_H_25(double *state, double *unused, double *out_3718029853621036109) {
  H_25(state, unused, out_3718029853621036109);
}
void car_h_24(double *state, double *unused, double *out_8075960681212006981) {
  h_24(state, unused, out_8075960681212006981);
}
void car_H_24(double *state, double *unused, double *out_2936175604487149648) {
  H_24(state, unused, out_2936175604487149648);
}
void car_h_30(double *state, double *unused, double *out_3348730023416583745) {
  h_30(state, unused, out_3348730023416583745);
}
void car_H_30(double *state, double *unused, double *out_8245726183748644307) {
  H_30(state, unused, out_8245726183748644307);
}
void car_h_26(double *state, double *unused, double *out_4455149410379091890) {
  h_26(state, unused, out_4455149410379091890);
}
void car_H_26(double *state, double *unused, double *out_7459533172495092333) {
  H_26(state, unused, out_7459533172495092333);
}
void car_h_27(double *state, double *unused, double *out_5626911248896423731) {
  h_27(state, unused, out_5626911248896423731);
}
void car_H_27(double *state, double *unused, double *out_6022132112564701090) {
  H_27(state, unused, out_6022132112564701090);
}
void car_h_29(double *state, double *unused, double *out_8574637239251908692) {
  h_29(state, unused, out_8574637239251908692);
}
void car_H_29(double *state, double *unused, double *out_7735494839434252123) {
  H_29(state, unused, out_7735494839434252123);
}
void car_h_28(double *state, double *unused, double *out_4269015409183433725) {
  h_28(state, unused, out_4269015409183433725);
}
void car_H_28(double *state, double *unused, double *out_5628850217205768919) {
  H_28(state, unused, out_5628850217205768919);
}
void car_h_31(double *state, double *unused, double *out_7171839259411449720) {
  h_31(state, unused, out_7171839259411449720);
}
void car_H_31(double *state, double *unused, double *out_3687383891744075681) {
  H_31(state, unused, out_3687383891744075681);
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
