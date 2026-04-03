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
void err_fun(double *nom_x, double *delta_x, double *out_4680331361908892937) {
   out_4680331361908892937[0] = delta_x[0] + nom_x[0];
   out_4680331361908892937[1] = delta_x[1] + nom_x[1];
   out_4680331361908892937[2] = delta_x[2] + nom_x[2];
   out_4680331361908892937[3] = delta_x[3] + nom_x[3];
   out_4680331361908892937[4] = delta_x[4] + nom_x[4];
   out_4680331361908892937[5] = delta_x[5] + nom_x[5];
   out_4680331361908892937[6] = delta_x[6] + nom_x[6];
   out_4680331361908892937[7] = delta_x[7] + nom_x[7];
   out_4680331361908892937[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3950569831835191548) {
   out_3950569831835191548[0] = -nom_x[0] + true_x[0];
   out_3950569831835191548[1] = -nom_x[1] + true_x[1];
   out_3950569831835191548[2] = -nom_x[2] + true_x[2];
   out_3950569831835191548[3] = -nom_x[3] + true_x[3];
   out_3950569831835191548[4] = -nom_x[4] + true_x[4];
   out_3950569831835191548[5] = -nom_x[5] + true_x[5];
   out_3950569831835191548[6] = -nom_x[6] + true_x[6];
   out_3950569831835191548[7] = -nom_x[7] + true_x[7];
   out_3950569831835191548[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1066722927762349330) {
   out_1066722927762349330[0] = 1.0;
   out_1066722927762349330[1] = 0.0;
   out_1066722927762349330[2] = 0.0;
   out_1066722927762349330[3] = 0.0;
   out_1066722927762349330[4] = 0.0;
   out_1066722927762349330[5] = 0.0;
   out_1066722927762349330[6] = 0.0;
   out_1066722927762349330[7] = 0.0;
   out_1066722927762349330[8] = 0.0;
   out_1066722927762349330[9] = 0.0;
   out_1066722927762349330[10] = 1.0;
   out_1066722927762349330[11] = 0.0;
   out_1066722927762349330[12] = 0.0;
   out_1066722927762349330[13] = 0.0;
   out_1066722927762349330[14] = 0.0;
   out_1066722927762349330[15] = 0.0;
   out_1066722927762349330[16] = 0.0;
   out_1066722927762349330[17] = 0.0;
   out_1066722927762349330[18] = 0.0;
   out_1066722927762349330[19] = 0.0;
   out_1066722927762349330[20] = 1.0;
   out_1066722927762349330[21] = 0.0;
   out_1066722927762349330[22] = 0.0;
   out_1066722927762349330[23] = 0.0;
   out_1066722927762349330[24] = 0.0;
   out_1066722927762349330[25] = 0.0;
   out_1066722927762349330[26] = 0.0;
   out_1066722927762349330[27] = 0.0;
   out_1066722927762349330[28] = 0.0;
   out_1066722927762349330[29] = 0.0;
   out_1066722927762349330[30] = 1.0;
   out_1066722927762349330[31] = 0.0;
   out_1066722927762349330[32] = 0.0;
   out_1066722927762349330[33] = 0.0;
   out_1066722927762349330[34] = 0.0;
   out_1066722927762349330[35] = 0.0;
   out_1066722927762349330[36] = 0.0;
   out_1066722927762349330[37] = 0.0;
   out_1066722927762349330[38] = 0.0;
   out_1066722927762349330[39] = 0.0;
   out_1066722927762349330[40] = 1.0;
   out_1066722927762349330[41] = 0.0;
   out_1066722927762349330[42] = 0.0;
   out_1066722927762349330[43] = 0.0;
   out_1066722927762349330[44] = 0.0;
   out_1066722927762349330[45] = 0.0;
   out_1066722927762349330[46] = 0.0;
   out_1066722927762349330[47] = 0.0;
   out_1066722927762349330[48] = 0.0;
   out_1066722927762349330[49] = 0.0;
   out_1066722927762349330[50] = 1.0;
   out_1066722927762349330[51] = 0.0;
   out_1066722927762349330[52] = 0.0;
   out_1066722927762349330[53] = 0.0;
   out_1066722927762349330[54] = 0.0;
   out_1066722927762349330[55] = 0.0;
   out_1066722927762349330[56] = 0.0;
   out_1066722927762349330[57] = 0.0;
   out_1066722927762349330[58] = 0.0;
   out_1066722927762349330[59] = 0.0;
   out_1066722927762349330[60] = 1.0;
   out_1066722927762349330[61] = 0.0;
   out_1066722927762349330[62] = 0.0;
   out_1066722927762349330[63] = 0.0;
   out_1066722927762349330[64] = 0.0;
   out_1066722927762349330[65] = 0.0;
   out_1066722927762349330[66] = 0.0;
   out_1066722927762349330[67] = 0.0;
   out_1066722927762349330[68] = 0.0;
   out_1066722927762349330[69] = 0.0;
   out_1066722927762349330[70] = 1.0;
   out_1066722927762349330[71] = 0.0;
   out_1066722927762349330[72] = 0.0;
   out_1066722927762349330[73] = 0.0;
   out_1066722927762349330[74] = 0.0;
   out_1066722927762349330[75] = 0.0;
   out_1066722927762349330[76] = 0.0;
   out_1066722927762349330[77] = 0.0;
   out_1066722927762349330[78] = 0.0;
   out_1066722927762349330[79] = 0.0;
   out_1066722927762349330[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1631803966804731977) {
   out_1631803966804731977[0] = state[0];
   out_1631803966804731977[1] = state[1];
   out_1631803966804731977[2] = state[2];
   out_1631803966804731977[3] = state[3];
   out_1631803966804731977[4] = state[4];
   out_1631803966804731977[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1631803966804731977[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1631803966804731977[7] = state[7];
   out_1631803966804731977[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8461957512847748442) {
   out_8461957512847748442[0] = 1;
   out_8461957512847748442[1] = 0;
   out_8461957512847748442[2] = 0;
   out_8461957512847748442[3] = 0;
   out_8461957512847748442[4] = 0;
   out_8461957512847748442[5] = 0;
   out_8461957512847748442[6] = 0;
   out_8461957512847748442[7] = 0;
   out_8461957512847748442[8] = 0;
   out_8461957512847748442[9] = 0;
   out_8461957512847748442[10] = 1;
   out_8461957512847748442[11] = 0;
   out_8461957512847748442[12] = 0;
   out_8461957512847748442[13] = 0;
   out_8461957512847748442[14] = 0;
   out_8461957512847748442[15] = 0;
   out_8461957512847748442[16] = 0;
   out_8461957512847748442[17] = 0;
   out_8461957512847748442[18] = 0;
   out_8461957512847748442[19] = 0;
   out_8461957512847748442[20] = 1;
   out_8461957512847748442[21] = 0;
   out_8461957512847748442[22] = 0;
   out_8461957512847748442[23] = 0;
   out_8461957512847748442[24] = 0;
   out_8461957512847748442[25] = 0;
   out_8461957512847748442[26] = 0;
   out_8461957512847748442[27] = 0;
   out_8461957512847748442[28] = 0;
   out_8461957512847748442[29] = 0;
   out_8461957512847748442[30] = 1;
   out_8461957512847748442[31] = 0;
   out_8461957512847748442[32] = 0;
   out_8461957512847748442[33] = 0;
   out_8461957512847748442[34] = 0;
   out_8461957512847748442[35] = 0;
   out_8461957512847748442[36] = 0;
   out_8461957512847748442[37] = 0;
   out_8461957512847748442[38] = 0;
   out_8461957512847748442[39] = 0;
   out_8461957512847748442[40] = 1;
   out_8461957512847748442[41] = 0;
   out_8461957512847748442[42] = 0;
   out_8461957512847748442[43] = 0;
   out_8461957512847748442[44] = 0;
   out_8461957512847748442[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8461957512847748442[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8461957512847748442[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8461957512847748442[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8461957512847748442[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8461957512847748442[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8461957512847748442[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8461957512847748442[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8461957512847748442[53] = -9.8100000000000005*dt;
   out_8461957512847748442[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8461957512847748442[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8461957512847748442[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8461957512847748442[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8461957512847748442[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8461957512847748442[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8461957512847748442[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8461957512847748442[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8461957512847748442[62] = 0;
   out_8461957512847748442[63] = 0;
   out_8461957512847748442[64] = 0;
   out_8461957512847748442[65] = 0;
   out_8461957512847748442[66] = 0;
   out_8461957512847748442[67] = 0;
   out_8461957512847748442[68] = 0;
   out_8461957512847748442[69] = 0;
   out_8461957512847748442[70] = 1;
   out_8461957512847748442[71] = 0;
   out_8461957512847748442[72] = 0;
   out_8461957512847748442[73] = 0;
   out_8461957512847748442[74] = 0;
   out_8461957512847748442[75] = 0;
   out_8461957512847748442[76] = 0;
   out_8461957512847748442[77] = 0;
   out_8461957512847748442[78] = 0;
   out_8461957512847748442[79] = 0;
   out_8461957512847748442[80] = 1;
}
void h_25(double *state, double *unused, double *out_1273494739935375238) {
   out_1273494739935375238[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8988060960736209135) {
   out_8988060960736209135[0] = 0;
   out_8988060960736209135[1] = 0;
   out_8988060960736209135[2] = 0;
   out_8988060960736209135[3] = 0;
   out_8988060960736209135[4] = 0;
   out_8988060960736209135[5] = 0;
   out_8988060960736209135[6] = 1;
   out_8988060960736209135[7] = 0;
   out_8988060960736209135[8] = 0;
}
void h_24(double *state, double *unused, double *out_3673979862628150281) {
   out_3673979862628150281[0] = state[4];
   out_3673979862628150281[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5900757514171019345) {
   out_5900757514171019345[0] = 0;
   out_5900757514171019345[1] = 0;
   out_5900757514171019345[2] = 0;
   out_5900757514171019345[3] = 0;
   out_5900757514171019345[4] = 1;
   out_5900757514171019345[5] = 0;
   out_5900757514171019345[6] = 0;
   out_5900757514171019345[7] = 0;
   out_5900757514171019345[8] = 0;
   out_5900757514171019345[9] = 0;
   out_5900757514171019345[10] = 0;
   out_5900757514171019345[11] = 0;
   out_5900757514171019345[12] = 0;
   out_5900757514171019345[13] = 0;
   out_5900757514171019345[14] = 1;
   out_5900757514171019345[15] = 0;
   out_5900757514171019345[16] = 0;
   out_5900757514171019345[17] = 0;
}
void h_30(double *state, double *unused, double *out_998300677650869349) {
   out_998300677650869349[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6940350154466093854) {
   out_6940350154466093854[0] = 0;
   out_6940350154466093854[1] = 0;
   out_6940350154466093854[2] = 0;
   out_6940350154466093854[3] = 0;
   out_6940350154466093854[4] = 1;
   out_6940350154466093854[5] = 0;
   out_6940350154466093854[6] = 0;
   out_6940350154466093854[7] = 0;
   out_6940350154466093854[8] = 0;
}
void h_26(double *state, double *unused, double *out_6257843039442582360) {
   out_6257843039442582360[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5246557641862152911) {
   out_5246557641862152911[0] = 0;
   out_5246557641862152911[1] = 0;
   out_5246557641862152911[2] = 0;
   out_5246557641862152911[3] = 0;
   out_5246557641862152911[4] = 0;
   out_5246557641862152911[5] = 0;
   out_5246557641862152911[6] = 0;
   out_5246557641862152911[7] = 1;
   out_5246557641862152911[8] = 0;
}
void h_27(double *state, double *unused, double *out_670178855258897721) {
   out_670178855258897721[0] = state[3];
}
void H_27(double *state, double *unused, double *out_9115113466266518765) {
   out_9115113466266518765[0] = 0;
   out_9115113466266518765[1] = 0;
   out_9115113466266518765[2] = 0;
   out_9115113466266518765[3] = 1;
   out_9115113466266518765[4] = 0;
   out_9115113466266518765[5] = 0;
   out_9115113466266518765[6] = 0;
   out_9115113466266518765[7] = 0;
   out_9115113466266518765[8] = 0;
}
void h_29(double *state, double *unused, double *out_4003372590009976296) {
   out_4003372590009976296[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6430118810151701670) {
   out_6430118810151701670[0] = 0;
   out_6430118810151701670[1] = 1;
   out_6430118810151701670[2] = 0;
   out_6430118810151701670[3] = 0;
   out_6430118810151701670[4] = 0;
   out_6430118810151701670[5] = 0;
   out_6430118810151701670[6] = 0;
   out_6430118810151701670[7] = 0;
   out_6430118810151701670[8] = 0;
}
void h_28(double *state, double *unused, double *out_3274584176067595372) {
   out_3274584176067595372[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6934226246488319372) {
   out_6934226246488319372[0] = 1;
   out_6934226246488319372[1] = 0;
   out_6934226246488319372[2] = 0;
   out_6934226246488319372[3] = 0;
   out_6934226246488319372[4] = 0;
   out_6934226246488319372[5] = 0;
   out_6934226246488319372[6] = 0;
   out_6934226246488319372[7] = 0;
   out_6934226246488319372[8] = 0;
}
void h_31(double *state, double *unused, double *out_4910944268189397588) {
   out_4910944268189397588[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4620349539628801435) {
   out_4620349539628801435[0] = 0;
   out_4620349539628801435[1] = 0;
   out_4620349539628801435[2] = 0;
   out_4620349539628801435[3] = 0;
   out_4620349539628801435[4] = 0;
   out_4620349539628801435[5] = 0;
   out_4620349539628801435[6] = 0;
   out_4620349539628801435[7] = 0;
   out_4620349539628801435[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4680331361908892937) {
  err_fun(nom_x, delta_x, out_4680331361908892937);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3950569831835191548) {
  inv_err_fun(nom_x, true_x, out_3950569831835191548);
}
void car_H_mod_fun(double *state, double *out_1066722927762349330) {
  H_mod_fun(state, out_1066722927762349330);
}
void car_f_fun(double *state, double dt, double *out_1631803966804731977) {
  f_fun(state,  dt, out_1631803966804731977);
}
void car_F_fun(double *state, double dt, double *out_8461957512847748442) {
  F_fun(state,  dt, out_8461957512847748442);
}
void car_h_25(double *state, double *unused, double *out_1273494739935375238) {
  h_25(state, unused, out_1273494739935375238);
}
void car_H_25(double *state, double *unused, double *out_8988060960736209135) {
  H_25(state, unused, out_8988060960736209135);
}
void car_h_24(double *state, double *unused, double *out_3673979862628150281) {
  h_24(state, unused, out_3673979862628150281);
}
void car_H_24(double *state, double *unused, double *out_5900757514171019345) {
  H_24(state, unused, out_5900757514171019345);
}
void car_h_30(double *state, double *unused, double *out_998300677650869349) {
  h_30(state, unused, out_998300677650869349);
}
void car_H_30(double *state, double *unused, double *out_6940350154466093854) {
  H_30(state, unused, out_6940350154466093854);
}
void car_h_26(double *state, double *unused, double *out_6257843039442582360) {
  h_26(state, unused, out_6257843039442582360);
}
void car_H_26(double *state, double *unused, double *out_5246557641862152911) {
  H_26(state, unused, out_5246557641862152911);
}
void car_h_27(double *state, double *unused, double *out_670178855258897721) {
  h_27(state, unused, out_670178855258897721);
}
void car_H_27(double *state, double *unused, double *out_9115113466266518765) {
  H_27(state, unused, out_9115113466266518765);
}
void car_h_29(double *state, double *unused, double *out_4003372590009976296) {
  h_29(state, unused, out_4003372590009976296);
}
void car_H_29(double *state, double *unused, double *out_6430118810151701670) {
  H_29(state, unused, out_6430118810151701670);
}
void car_h_28(double *state, double *unused, double *out_3274584176067595372) {
  h_28(state, unused, out_3274584176067595372);
}
void car_H_28(double *state, double *unused, double *out_6934226246488319372) {
  H_28(state, unused, out_6934226246488319372);
}
void car_h_31(double *state, double *unused, double *out_4910944268189397588) {
  h_31(state, unused, out_4910944268189397588);
}
void car_H_31(double *state, double *unused, double *out_4620349539628801435) {
  H_31(state, unused, out_4620349539628801435);
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
