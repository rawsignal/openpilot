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
void err_fun(double *nom_x, double *delta_x, double *out_7581984428922518495) {
   out_7581984428922518495[0] = delta_x[0] + nom_x[0];
   out_7581984428922518495[1] = delta_x[1] + nom_x[1];
   out_7581984428922518495[2] = delta_x[2] + nom_x[2];
   out_7581984428922518495[3] = delta_x[3] + nom_x[3];
   out_7581984428922518495[4] = delta_x[4] + nom_x[4];
   out_7581984428922518495[5] = delta_x[5] + nom_x[5];
   out_7581984428922518495[6] = delta_x[6] + nom_x[6];
   out_7581984428922518495[7] = delta_x[7] + nom_x[7];
   out_7581984428922518495[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7458254433530772005) {
   out_7458254433530772005[0] = -nom_x[0] + true_x[0];
   out_7458254433530772005[1] = -nom_x[1] + true_x[1];
   out_7458254433530772005[2] = -nom_x[2] + true_x[2];
   out_7458254433530772005[3] = -nom_x[3] + true_x[3];
   out_7458254433530772005[4] = -nom_x[4] + true_x[4];
   out_7458254433530772005[5] = -nom_x[5] + true_x[5];
   out_7458254433530772005[6] = -nom_x[6] + true_x[6];
   out_7458254433530772005[7] = -nom_x[7] + true_x[7];
   out_7458254433530772005[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4805202653021023192) {
   out_4805202653021023192[0] = 1.0;
   out_4805202653021023192[1] = 0.0;
   out_4805202653021023192[2] = 0.0;
   out_4805202653021023192[3] = 0.0;
   out_4805202653021023192[4] = 0.0;
   out_4805202653021023192[5] = 0.0;
   out_4805202653021023192[6] = 0.0;
   out_4805202653021023192[7] = 0.0;
   out_4805202653021023192[8] = 0.0;
   out_4805202653021023192[9] = 0.0;
   out_4805202653021023192[10] = 1.0;
   out_4805202653021023192[11] = 0.0;
   out_4805202653021023192[12] = 0.0;
   out_4805202653021023192[13] = 0.0;
   out_4805202653021023192[14] = 0.0;
   out_4805202653021023192[15] = 0.0;
   out_4805202653021023192[16] = 0.0;
   out_4805202653021023192[17] = 0.0;
   out_4805202653021023192[18] = 0.0;
   out_4805202653021023192[19] = 0.0;
   out_4805202653021023192[20] = 1.0;
   out_4805202653021023192[21] = 0.0;
   out_4805202653021023192[22] = 0.0;
   out_4805202653021023192[23] = 0.0;
   out_4805202653021023192[24] = 0.0;
   out_4805202653021023192[25] = 0.0;
   out_4805202653021023192[26] = 0.0;
   out_4805202653021023192[27] = 0.0;
   out_4805202653021023192[28] = 0.0;
   out_4805202653021023192[29] = 0.0;
   out_4805202653021023192[30] = 1.0;
   out_4805202653021023192[31] = 0.0;
   out_4805202653021023192[32] = 0.0;
   out_4805202653021023192[33] = 0.0;
   out_4805202653021023192[34] = 0.0;
   out_4805202653021023192[35] = 0.0;
   out_4805202653021023192[36] = 0.0;
   out_4805202653021023192[37] = 0.0;
   out_4805202653021023192[38] = 0.0;
   out_4805202653021023192[39] = 0.0;
   out_4805202653021023192[40] = 1.0;
   out_4805202653021023192[41] = 0.0;
   out_4805202653021023192[42] = 0.0;
   out_4805202653021023192[43] = 0.0;
   out_4805202653021023192[44] = 0.0;
   out_4805202653021023192[45] = 0.0;
   out_4805202653021023192[46] = 0.0;
   out_4805202653021023192[47] = 0.0;
   out_4805202653021023192[48] = 0.0;
   out_4805202653021023192[49] = 0.0;
   out_4805202653021023192[50] = 1.0;
   out_4805202653021023192[51] = 0.0;
   out_4805202653021023192[52] = 0.0;
   out_4805202653021023192[53] = 0.0;
   out_4805202653021023192[54] = 0.0;
   out_4805202653021023192[55] = 0.0;
   out_4805202653021023192[56] = 0.0;
   out_4805202653021023192[57] = 0.0;
   out_4805202653021023192[58] = 0.0;
   out_4805202653021023192[59] = 0.0;
   out_4805202653021023192[60] = 1.0;
   out_4805202653021023192[61] = 0.0;
   out_4805202653021023192[62] = 0.0;
   out_4805202653021023192[63] = 0.0;
   out_4805202653021023192[64] = 0.0;
   out_4805202653021023192[65] = 0.0;
   out_4805202653021023192[66] = 0.0;
   out_4805202653021023192[67] = 0.0;
   out_4805202653021023192[68] = 0.0;
   out_4805202653021023192[69] = 0.0;
   out_4805202653021023192[70] = 1.0;
   out_4805202653021023192[71] = 0.0;
   out_4805202653021023192[72] = 0.0;
   out_4805202653021023192[73] = 0.0;
   out_4805202653021023192[74] = 0.0;
   out_4805202653021023192[75] = 0.0;
   out_4805202653021023192[76] = 0.0;
   out_4805202653021023192[77] = 0.0;
   out_4805202653021023192[78] = 0.0;
   out_4805202653021023192[79] = 0.0;
   out_4805202653021023192[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_603278022302126708) {
   out_603278022302126708[0] = state[0];
   out_603278022302126708[1] = state[1];
   out_603278022302126708[2] = state[2];
   out_603278022302126708[3] = state[3];
   out_603278022302126708[4] = state[4];
   out_603278022302126708[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_603278022302126708[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_603278022302126708[7] = state[7];
   out_603278022302126708[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2142672846904298435) {
   out_2142672846904298435[0] = 1;
   out_2142672846904298435[1] = 0;
   out_2142672846904298435[2] = 0;
   out_2142672846904298435[3] = 0;
   out_2142672846904298435[4] = 0;
   out_2142672846904298435[5] = 0;
   out_2142672846904298435[6] = 0;
   out_2142672846904298435[7] = 0;
   out_2142672846904298435[8] = 0;
   out_2142672846904298435[9] = 0;
   out_2142672846904298435[10] = 1;
   out_2142672846904298435[11] = 0;
   out_2142672846904298435[12] = 0;
   out_2142672846904298435[13] = 0;
   out_2142672846904298435[14] = 0;
   out_2142672846904298435[15] = 0;
   out_2142672846904298435[16] = 0;
   out_2142672846904298435[17] = 0;
   out_2142672846904298435[18] = 0;
   out_2142672846904298435[19] = 0;
   out_2142672846904298435[20] = 1;
   out_2142672846904298435[21] = 0;
   out_2142672846904298435[22] = 0;
   out_2142672846904298435[23] = 0;
   out_2142672846904298435[24] = 0;
   out_2142672846904298435[25] = 0;
   out_2142672846904298435[26] = 0;
   out_2142672846904298435[27] = 0;
   out_2142672846904298435[28] = 0;
   out_2142672846904298435[29] = 0;
   out_2142672846904298435[30] = 1;
   out_2142672846904298435[31] = 0;
   out_2142672846904298435[32] = 0;
   out_2142672846904298435[33] = 0;
   out_2142672846904298435[34] = 0;
   out_2142672846904298435[35] = 0;
   out_2142672846904298435[36] = 0;
   out_2142672846904298435[37] = 0;
   out_2142672846904298435[38] = 0;
   out_2142672846904298435[39] = 0;
   out_2142672846904298435[40] = 1;
   out_2142672846904298435[41] = 0;
   out_2142672846904298435[42] = 0;
   out_2142672846904298435[43] = 0;
   out_2142672846904298435[44] = 0;
   out_2142672846904298435[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2142672846904298435[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2142672846904298435[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2142672846904298435[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2142672846904298435[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2142672846904298435[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2142672846904298435[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2142672846904298435[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2142672846904298435[53] = -9.8100000000000005*dt;
   out_2142672846904298435[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2142672846904298435[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2142672846904298435[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2142672846904298435[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2142672846904298435[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2142672846904298435[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2142672846904298435[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2142672846904298435[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2142672846904298435[62] = 0;
   out_2142672846904298435[63] = 0;
   out_2142672846904298435[64] = 0;
   out_2142672846904298435[65] = 0;
   out_2142672846904298435[66] = 0;
   out_2142672846904298435[67] = 0;
   out_2142672846904298435[68] = 0;
   out_2142672846904298435[69] = 0;
   out_2142672846904298435[70] = 1;
   out_2142672846904298435[71] = 0;
   out_2142672846904298435[72] = 0;
   out_2142672846904298435[73] = 0;
   out_2142672846904298435[74] = 0;
   out_2142672846904298435[75] = 0;
   out_2142672846904298435[76] = 0;
   out_2142672846904298435[77] = 0;
   out_2142672846904298435[78] = 0;
   out_2142672846904298435[79] = 0;
   out_2142672846904298435[80] = 1;
}
void h_25(double *state, double *unused, double *out_47232026210041201) {
   out_47232026210041201[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1691820306869006759) {
   out_1691820306869006759[0] = 0;
   out_1691820306869006759[1] = 0;
   out_1691820306869006759[2] = 0;
   out_1691820306869006759[3] = 0;
   out_1691820306869006759[4] = 0;
   out_1691820306869006759[5] = 0;
   out_1691820306869006759[6] = 1;
   out_1691820306869006759[7] = 0;
   out_1691820306869006759[8] = 0;
}
void h_24(double *state, double *unused, double *out_8495854549207496621) {
   out_8495854549207496621[0] = state[4];
   out_8495854549207496621[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6560635171896713611) {
   out_6560635171896713611[0] = 0;
   out_6560635171896713611[1] = 0;
   out_6560635171896713611[2] = 0;
   out_6560635171896713611[3] = 0;
   out_6560635171896713611[4] = 1;
   out_6560635171896713611[5] = 0;
   out_6560635171896713611[6] = 0;
   out_6560635171896713611[7] = 0;
   out_6560635171896713611[8] = 0;
   out_6560635171896713611[9] = 0;
   out_6560635171896713611[10] = 0;
   out_6560635171896713611[11] = 0;
   out_6560635171896713611[12] = 0;
   out_6560635171896713611[13] = 0;
   out_6560635171896713611[14] = 1;
   out_6560635171896713611[15] = 0;
   out_6560635171896713611[16] = 0;
   out_6560635171896713611[17] = 0;
}
void h_30(double *state, double *unused, double *out_9184949310893892383) {
   out_9184949310893892383[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6219516636996614957) {
   out_6219516636996614957[0] = 0;
   out_6219516636996614957[1] = 0;
   out_6219516636996614957[2] = 0;
   out_6219516636996614957[3] = 0;
   out_6219516636996614957[4] = 1;
   out_6219516636996614957[5] = 0;
   out_6219516636996614957[6] = 0;
   out_6219516636996614957[7] = 0;
   out_6219516636996614957[8] = 0;
}
void h_26(double *state, double *unused, double *out_6996661668192902125) {
   out_6996661668192902125[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5433323625743062983) {
   out_5433323625743062983[0] = 0;
   out_5433323625743062983[1] = 0;
   out_5433323625743062983[2] = 0;
   out_5433323625743062983[3] = 0;
   out_5433323625743062983[4] = 0;
   out_5433323625743062983[5] = 0;
   out_5433323625743062983[6] = 0;
   out_5433323625743062983[7] = 1;
   out_5433323625743062983[8] = 0;
}
void h_27(double *state, double *unused, double *out_3084018077654373886) {
   out_3084018077654373886[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3995922565812671740) {
   out_3995922565812671740[0] = 0;
   out_3995922565812671740[1] = 0;
   out_3995922565812671740[2] = 0;
   out_3995922565812671740[3] = 1;
   out_3995922565812671740[4] = 0;
   out_3995922565812671740[5] = 0;
   out_3995922565812671740[6] = 0;
   out_3995922565812671740[7] = 0;
   out_3995922565812671740[8] = 0;
}
void h_29(double *state, double *unused, double *out_8591890769704826794) {
   out_8591890769704826794[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5709285292682222773) {
   out_5709285292682222773[0] = 0;
   out_5709285292682222773[1] = 1;
   out_5709285292682222773[2] = 0;
   out_5709285292682222773[3] = 0;
   out_5709285292682222773[4] = 0;
   out_5709285292682222773[5] = 0;
   out_5709285292682222773[6] = 0;
   out_5709285292682222773[7] = 0;
   out_5709285292682222773[8] = 0;
}
void h_28(double *state, double *unused, double *out_2480702192977896369) {
   out_2480702192977896369[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7655059763957798269) {
   out_7655059763957798269[0] = 1;
   out_7655059763957798269[1] = 0;
   out_7655059763957798269[2] = 0;
   out_7655059763957798269[3] = 0;
   out_7655059763957798269[4] = 0;
   out_7655059763957798269[5] = 0;
   out_7655059763957798269[6] = 0;
   out_7655059763957798269[7] = 0;
   out_7655059763957798269[8] = 0;
}
void h_31(double *state, double *unused, double *out_322426088494547090) {
   out_322426088494547090[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6059531727976414459) {
   out_6059531727976414459[0] = 0;
   out_6059531727976414459[1] = 0;
   out_6059531727976414459[2] = 0;
   out_6059531727976414459[3] = 0;
   out_6059531727976414459[4] = 0;
   out_6059531727976414459[5] = 0;
   out_6059531727976414459[6] = 0;
   out_6059531727976414459[7] = 0;
   out_6059531727976414459[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7581984428922518495) {
  err_fun(nom_x, delta_x, out_7581984428922518495);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7458254433530772005) {
  inv_err_fun(nom_x, true_x, out_7458254433530772005);
}
void car_H_mod_fun(double *state, double *out_4805202653021023192) {
  H_mod_fun(state, out_4805202653021023192);
}
void car_f_fun(double *state, double dt, double *out_603278022302126708) {
  f_fun(state,  dt, out_603278022302126708);
}
void car_F_fun(double *state, double dt, double *out_2142672846904298435) {
  F_fun(state,  dt, out_2142672846904298435);
}
void car_h_25(double *state, double *unused, double *out_47232026210041201) {
  h_25(state, unused, out_47232026210041201);
}
void car_H_25(double *state, double *unused, double *out_1691820306869006759) {
  H_25(state, unused, out_1691820306869006759);
}
void car_h_24(double *state, double *unused, double *out_8495854549207496621) {
  h_24(state, unused, out_8495854549207496621);
}
void car_H_24(double *state, double *unused, double *out_6560635171896713611) {
  H_24(state, unused, out_6560635171896713611);
}
void car_h_30(double *state, double *unused, double *out_9184949310893892383) {
  h_30(state, unused, out_9184949310893892383);
}
void car_H_30(double *state, double *unused, double *out_6219516636996614957) {
  H_30(state, unused, out_6219516636996614957);
}
void car_h_26(double *state, double *unused, double *out_6996661668192902125) {
  h_26(state, unused, out_6996661668192902125);
}
void car_H_26(double *state, double *unused, double *out_5433323625743062983) {
  H_26(state, unused, out_5433323625743062983);
}
void car_h_27(double *state, double *unused, double *out_3084018077654373886) {
  h_27(state, unused, out_3084018077654373886);
}
void car_H_27(double *state, double *unused, double *out_3995922565812671740) {
  H_27(state, unused, out_3995922565812671740);
}
void car_h_29(double *state, double *unused, double *out_8591890769704826794) {
  h_29(state, unused, out_8591890769704826794);
}
void car_H_29(double *state, double *unused, double *out_5709285292682222773) {
  H_29(state, unused, out_5709285292682222773);
}
void car_h_28(double *state, double *unused, double *out_2480702192977896369) {
  h_28(state, unused, out_2480702192977896369);
}
void car_H_28(double *state, double *unused, double *out_7655059763957798269) {
  H_28(state, unused, out_7655059763957798269);
}
void car_h_31(double *state, double *unused, double *out_322426088494547090) {
  h_31(state, unused, out_322426088494547090);
}
void car_H_31(double *state, double *unused, double *out_6059531727976414459) {
  H_31(state, unused, out_6059531727976414459);
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
