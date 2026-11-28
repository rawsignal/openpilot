#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

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
void err_fun(double *nom_x, double *delta_x, double *out_8105623026857581998) {
   out_8105623026857581998[0] = delta_x[0] + nom_x[0];
   out_8105623026857581998[1] = delta_x[1] + nom_x[1];
   out_8105623026857581998[2] = delta_x[2] + nom_x[2];
   out_8105623026857581998[3] = delta_x[3] + nom_x[3];
   out_8105623026857581998[4] = delta_x[4] + nom_x[4];
   out_8105623026857581998[5] = delta_x[5] + nom_x[5];
   out_8105623026857581998[6] = delta_x[6] + nom_x[6];
   out_8105623026857581998[7] = delta_x[7] + nom_x[7];
   out_8105623026857581998[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7503404914069652669) {
   out_7503404914069652669[0] = -nom_x[0] + true_x[0];
   out_7503404914069652669[1] = -nom_x[1] + true_x[1];
   out_7503404914069652669[2] = -nom_x[2] + true_x[2];
   out_7503404914069652669[3] = -nom_x[3] + true_x[3];
   out_7503404914069652669[4] = -nom_x[4] + true_x[4];
   out_7503404914069652669[5] = -nom_x[5] + true_x[5];
   out_7503404914069652669[6] = -nom_x[6] + true_x[6];
   out_7503404914069652669[7] = -nom_x[7] + true_x[7];
   out_7503404914069652669[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4783738824564572493) {
   out_4783738824564572493[0] = 1.0;
   out_4783738824564572493[1] = 0.0;
   out_4783738824564572493[2] = 0.0;
   out_4783738824564572493[3] = 0.0;
   out_4783738824564572493[4] = 0.0;
   out_4783738824564572493[5] = 0.0;
   out_4783738824564572493[6] = 0.0;
   out_4783738824564572493[7] = 0.0;
   out_4783738824564572493[8] = 0.0;
   out_4783738824564572493[9] = 0.0;
   out_4783738824564572493[10] = 1.0;
   out_4783738824564572493[11] = 0.0;
   out_4783738824564572493[12] = 0.0;
   out_4783738824564572493[13] = 0.0;
   out_4783738824564572493[14] = 0.0;
   out_4783738824564572493[15] = 0.0;
   out_4783738824564572493[16] = 0.0;
   out_4783738824564572493[17] = 0.0;
   out_4783738824564572493[18] = 0.0;
   out_4783738824564572493[19] = 0.0;
   out_4783738824564572493[20] = 1.0;
   out_4783738824564572493[21] = 0.0;
   out_4783738824564572493[22] = 0.0;
   out_4783738824564572493[23] = 0.0;
   out_4783738824564572493[24] = 0.0;
   out_4783738824564572493[25] = 0.0;
   out_4783738824564572493[26] = 0.0;
   out_4783738824564572493[27] = 0.0;
   out_4783738824564572493[28] = 0.0;
   out_4783738824564572493[29] = 0.0;
   out_4783738824564572493[30] = 1.0;
   out_4783738824564572493[31] = 0.0;
   out_4783738824564572493[32] = 0.0;
   out_4783738824564572493[33] = 0.0;
   out_4783738824564572493[34] = 0.0;
   out_4783738824564572493[35] = 0.0;
   out_4783738824564572493[36] = 0.0;
   out_4783738824564572493[37] = 0.0;
   out_4783738824564572493[38] = 0.0;
   out_4783738824564572493[39] = 0.0;
   out_4783738824564572493[40] = 1.0;
   out_4783738824564572493[41] = 0.0;
   out_4783738824564572493[42] = 0.0;
   out_4783738824564572493[43] = 0.0;
   out_4783738824564572493[44] = 0.0;
   out_4783738824564572493[45] = 0.0;
   out_4783738824564572493[46] = 0.0;
   out_4783738824564572493[47] = 0.0;
   out_4783738824564572493[48] = 0.0;
   out_4783738824564572493[49] = 0.0;
   out_4783738824564572493[50] = 1.0;
   out_4783738824564572493[51] = 0.0;
   out_4783738824564572493[52] = 0.0;
   out_4783738824564572493[53] = 0.0;
   out_4783738824564572493[54] = 0.0;
   out_4783738824564572493[55] = 0.0;
   out_4783738824564572493[56] = 0.0;
   out_4783738824564572493[57] = 0.0;
   out_4783738824564572493[58] = 0.0;
   out_4783738824564572493[59] = 0.0;
   out_4783738824564572493[60] = 1.0;
   out_4783738824564572493[61] = 0.0;
   out_4783738824564572493[62] = 0.0;
   out_4783738824564572493[63] = 0.0;
   out_4783738824564572493[64] = 0.0;
   out_4783738824564572493[65] = 0.0;
   out_4783738824564572493[66] = 0.0;
   out_4783738824564572493[67] = 0.0;
   out_4783738824564572493[68] = 0.0;
   out_4783738824564572493[69] = 0.0;
   out_4783738824564572493[70] = 1.0;
   out_4783738824564572493[71] = 0.0;
   out_4783738824564572493[72] = 0.0;
   out_4783738824564572493[73] = 0.0;
   out_4783738824564572493[74] = 0.0;
   out_4783738824564572493[75] = 0.0;
   out_4783738824564572493[76] = 0.0;
   out_4783738824564572493[77] = 0.0;
   out_4783738824564572493[78] = 0.0;
   out_4783738824564572493[79] = 0.0;
   out_4783738824564572493[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6401182990301164404) {
   out_6401182990301164404[0] = state[0];
   out_6401182990301164404[1] = state[1];
   out_6401182990301164404[2] = state[2];
   out_6401182990301164404[3] = state[3];
   out_6401182990301164404[4] = state[4];
   out_6401182990301164404[5] = dt*(stiffness_front*(-state[2] - state[3] + state[7])*state[0]/state[1] + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/state[4] + ((-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/state[4] - state[4])*state[6] - 9.8100000000000005*state[8]) + state[5];
   out_6401182990301164404[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6401182990301164404[7] = state[7];
   out_6401182990301164404[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5658938656973252260) {
   out_5658938656973252260[0] = 1;
   out_5658938656973252260[1] = 0;
   out_5658938656973252260[2] = 0;
   out_5658938656973252260[3] = 0;
   out_5658938656973252260[4] = 0;
   out_5658938656973252260[5] = 0;
   out_5658938656973252260[6] = 0;
   out_5658938656973252260[7] = 0;
   out_5658938656973252260[8] = 0;
   out_5658938656973252260[9] = 0;
   out_5658938656973252260[10] = 1;
   out_5658938656973252260[11] = 0;
   out_5658938656973252260[12] = 0;
   out_5658938656973252260[13] = 0;
   out_5658938656973252260[14] = 0;
   out_5658938656973252260[15] = 0;
   out_5658938656973252260[16] = 0;
   out_5658938656973252260[17] = 0;
   out_5658938656973252260[18] = 0;
   out_5658938656973252260[19] = 0;
   out_5658938656973252260[20] = 1;
   out_5658938656973252260[21] = 0;
   out_5658938656973252260[22] = 0;
   out_5658938656973252260[23] = 0;
   out_5658938656973252260[24] = 0;
   out_5658938656973252260[25] = 0;
   out_5658938656973252260[26] = 0;
   out_5658938656973252260[27] = 0;
   out_5658938656973252260[28] = 0;
   out_5658938656973252260[29] = 0;
   out_5658938656973252260[30] = 1;
   out_5658938656973252260[31] = 0;
   out_5658938656973252260[32] = 0;
   out_5658938656973252260[33] = 0;
   out_5658938656973252260[34] = 0;
   out_5658938656973252260[35] = 0;
   out_5658938656973252260[36] = 0;
   out_5658938656973252260[37] = 0;
   out_5658938656973252260[38] = 0;
   out_5658938656973252260[39] = 0;
   out_5658938656973252260[40] = 1;
   out_5658938656973252260[41] = 0;
   out_5658938656973252260[42] = 0;
   out_5658938656973252260[43] = 0;
   out_5658938656973252260[44] = 0;
   out_5658938656973252260[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/state[1] + (-stiffness_front - stiffness_rear)*state[5]/state[4] + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/state[4]);
   out_5658938656973252260[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/pow(state[1], 2);
   out_5658938656973252260[47] = -dt*stiffness_front*state[0]/state[1];
   out_5658938656973252260[48] = -dt*stiffness_front*state[0]/state[1];
   out_5658938656973252260[49] = dt*(-(-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/pow(state[4], 2) + (-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/pow(state[4], 2) - 1)*state[6]);
   out_5658938656973252260[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/state[4] + 1;
   out_5658938656973252260[51] = dt*((-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/state[4] - state[4]);
   out_5658938656973252260[52] = dt*stiffness_front*state[0]/state[1];
   out_5658938656973252260[53] = -9.8100000000000005*dt;
   out_5658938656973252260[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5658938656973252260[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5658938656973252260[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5658938656973252260[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5658938656973252260[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5658938656973252260[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5658938656973252260[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5658938656973252260[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5658938656973252260[62] = 0;
   out_5658938656973252260[63] = 0;
   out_5658938656973252260[64] = 0;
   out_5658938656973252260[65] = 0;
   out_5658938656973252260[66] = 0;
   out_5658938656973252260[67] = 0;
   out_5658938656973252260[68] = 0;
   out_5658938656973252260[69] = 0;
   out_5658938656973252260[70] = 1;
   out_5658938656973252260[71] = 0;
   out_5658938656973252260[72] = 0;
   out_5658938656973252260[73] = 0;
   out_5658938656973252260[74] = 0;
   out_5658938656973252260[75] = 0;
   out_5658938656973252260[76] = 0;
   out_5658938656973252260[77] = 0;
   out_5658938656973252260[78] = 0;
   out_5658938656973252260[79] = 0;
   out_5658938656973252260[80] = 1;
}
void h_25(double *state, double *unused, double *out_2658415674908657098) {
   out_2658415674908657098[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1260758174575080816) {
   out_1260758174575080816[0] = 0;
   out_1260758174575080816[1] = 0;
   out_1260758174575080816[2] = 0;
   out_1260758174575080816[3] = 0;
   out_1260758174575080816[4] = 0;
   out_1260758174575080816[5] = 0;
   out_1260758174575080816[6] = 1;
   out_1260758174575080816[7] = 0;
   out_1260758174575080816[8] = 0;
}
void h_24(double *state, double *unused, double *out_3386724067472285602) {
   out_3386724067472285602[0] = state[4];
   out_3386724067472285602[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2297167424227242320) {
   out_2297167424227242320[0] = 0;
   out_2297167424227242320[1] = 0;
   out_2297167424227242320[2] = 0;
   out_2297167424227242320[3] = 0;
   out_2297167424227242320[4] = 1;
   out_2297167424227242320[5] = 0;
   out_2297167424227242320[6] = 0;
   out_2297167424227242320[7] = 0;
   out_2297167424227242320[8] = 0;
   out_2297167424227242320[9] = 0;
   out_2297167424227242320[10] = 0;
   out_2297167424227242320[11] = 0;
   out_2297167424227242320[12] = 0;
   out_2297167424227242320[13] = 0;
   out_2297167424227242320[14] = 1;
   out_2297167424227242320[15] = 0;
   out_2297167424227242320[16] = 0;
   out_2297167424227242320[17] = 0;
}
void h_30(double *state, double *unused, double *out_3786195456112836173) {
   out_3786195456112836173[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5655932166916535939) {
   out_5655932166916535939[0] = 0;
   out_5655932166916535939[1] = 0;
   out_5655932166916535939[2] = 0;
   out_5655932166916535939[3] = 0;
   out_5655932166916535939[4] = 1;
   out_5655932166916535939[5] = 0;
   out_5655932166916535939[6] = 0;
   out_5655932166916535939[7] = 0;
   out_5655932166916535939[8] = 0;
}
void h_26(double *state, double *unused, double *out_1439711759722488774) {
   out_1439711759722488774[0] = state[7];
}
void H_26(double *state, double *unused, double *out_603904110464768912) {
   out_603904110464768912[0] = 0;
   out_603904110464768912[1] = 0;
   out_603904110464768912[2] = 0;
   out_603904110464768912[3] = 0;
   out_603904110464768912[4] = 0;
   out_603904110464768912[5] = 0;
   out_603904110464768912[6] = 0;
   out_603904110464768912[7] = 1;
   out_603904110464768912[8] = 0;
}
void h_27(double *state, double *unused, double *out_2343257592752188547) {
   out_2343257592752188547[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3564860433518745797) {
   out_3564860433518745797[0] = 0;
   out_3564860433518745797[1] = 0;
   out_3564860433518745797[2] = 0;
   out_3564860433518745797[3] = 1;
   out_3564860433518745797[4] = 0;
   out_3564860433518745797[5] = 0;
   out_3564860433518745797[6] = 0;
   out_3564860433518745797[7] = 0;
   out_3564860433518745797[8] = 0;
}
void h_29(double *state, double *unused, double *out_2618451655036694436) {
   out_2618451655036694436[0] = state[1];
}
void H_29(double *state, double *unused, double *out_879865777403928702) {
   out_879865777403928702[0] = 0;
   out_879865777403928702[1] = 1;
   out_879865777403928702[2] = 0;
   out_879865777403928702[3] = 0;
   out_879865777403928702[4] = 0;
   out_879865777403928702[5] = 0;
   out_879865777403928702[6] = 0;
   out_879865777403928702[7] = 0;
   out_879865777403928702[8] = 0;
}
void h_28(double *state, double *unused, double *out_4435271162866397930) {
   out_4435271162866397930[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1083764494161397549) {
   out_1083764494161397549[0] = 1;
   out_1083764494161397549[1] = 0;
   out_1083764494161397549[2] = 0;
   out_1083764494161397549[3] = 0;
   out_1083764494161397549[4] = 0;
   out_1083764494161397549[5] = 0;
   out_1083764494161397549[6] = 0;
   out_1083764494161397549[7] = 0;
   out_1083764494161397549[8] = 0;
}
void h_31(double *state, double *unused, double *out_6295865203162679448) {
   out_6295865203162679448[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1230112212698120388) {
   out_1230112212698120388[0] = 0;
   out_1230112212698120388[1] = 0;
   out_1230112212698120388[2] = 0;
   out_1230112212698120388[3] = 0;
   out_1230112212698120388[4] = 0;
   out_1230112212698120388[5] = 0;
   out_1230112212698120388[6] = 0;
   out_1230112212698120388[7] = 0;
   out_1230112212698120388[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8105623026857581998) {
  err_fun(nom_x, delta_x, out_8105623026857581998);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7503404914069652669) {
  inv_err_fun(nom_x, true_x, out_7503404914069652669);
}
void car_H_mod_fun(double *state, double *out_4783738824564572493) {
  H_mod_fun(state, out_4783738824564572493);
}
void car_f_fun(double *state, double dt, double *out_6401182990301164404) {
  f_fun(state,  dt, out_6401182990301164404);
}
void car_F_fun(double *state, double dt, double *out_5658938656973252260) {
  F_fun(state,  dt, out_5658938656973252260);
}
void car_h_25(double *state, double *unused, double *out_2658415674908657098) {
  h_25(state, unused, out_2658415674908657098);
}
void car_H_25(double *state, double *unused, double *out_1260758174575080816) {
  H_25(state, unused, out_1260758174575080816);
}
void car_h_24(double *state, double *unused, double *out_3386724067472285602) {
  h_24(state, unused, out_3386724067472285602);
}
void car_H_24(double *state, double *unused, double *out_2297167424227242320) {
  H_24(state, unused, out_2297167424227242320);
}
void car_h_30(double *state, double *unused, double *out_3786195456112836173) {
  h_30(state, unused, out_3786195456112836173);
}
void car_H_30(double *state, double *unused, double *out_5655932166916535939) {
  H_30(state, unused, out_5655932166916535939);
}
void car_h_26(double *state, double *unused, double *out_1439711759722488774) {
  h_26(state, unused, out_1439711759722488774);
}
void car_H_26(double *state, double *unused, double *out_603904110464768912) {
  H_26(state, unused, out_603904110464768912);
}
void car_h_27(double *state, double *unused, double *out_2343257592752188547) {
  h_27(state, unused, out_2343257592752188547);
}
void car_H_27(double *state, double *unused, double *out_3564860433518745797) {
  H_27(state, unused, out_3564860433518745797);
}
void car_h_29(double *state, double *unused, double *out_2618451655036694436) {
  h_29(state, unused, out_2618451655036694436);
}
void car_H_29(double *state, double *unused, double *out_879865777403928702) {
  H_29(state, unused, out_879865777403928702);
}
void car_h_28(double *state, double *unused, double *out_4435271162866397930) {
  h_28(state, unused, out_4435271162866397930);
}
void car_H_28(double *state, double *unused, double *out_1083764494161397549) {
  H_28(state, unused, out_1083764494161397549);
}
void car_h_31(double *state, double *unused, double *out_6295865203162679448) {
  h_31(state, unused, out_6295865203162679448);
}
void car_H_31(double *state, double *unused, double *out_1230112212698120388) {
  H_31(state, unused, out_1230112212698120388);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
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
