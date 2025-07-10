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
void err_fun(double *nom_x, double *delta_x, double *out_8939131331752933554) {
   out_8939131331752933554[0] = delta_x[0] + nom_x[0];
   out_8939131331752933554[1] = delta_x[1] + nom_x[1];
   out_8939131331752933554[2] = delta_x[2] + nom_x[2];
   out_8939131331752933554[3] = delta_x[3] + nom_x[3];
   out_8939131331752933554[4] = delta_x[4] + nom_x[4];
   out_8939131331752933554[5] = delta_x[5] + nom_x[5];
   out_8939131331752933554[6] = delta_x[6] + nom_x[6];
   out_8939131331752933554[7] = delta_x[7] + nom_x[7];
   out_8939131331752933554[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4134102941266011107) {
   out_4134102941266011107[0] = -nom_x[0] + true_x[0];
   out_4134102941266011107[1] = -nom_x[1] + true_x[1];
   out_4134102941266011107[2] = -nom_x[2] + true_x[2];
   out_4134102941266011107[3] = -nom_x[3] + true_x[3];
   out_4134102941266011107[4] = -nom_x[4] + true_x[4];
   out_4134102941266011107[5] = -nom_x[5] + true_x[5];
   out_4134102941266011107[6] = -nom_x[6] + true_x[6];
   out_4134102941266011107[7] = -nom_x[7] + true_x[7];
   out_4134102941266011107[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8366129814821909934) {
   out_8366129814821909934[0] = 1.0;
   out_8366129814821909934[1] = 0.0;
   out_8366129814821909934[2] = 0.0;
   out_8366129814821909934[3] = 0.0;
   out_8366129814821909934[4] = 0.0;
   out_8366129814821909934[5] = 0.0;
   out_8366129814821909934[6] = 0.0;
   out_8366129814821909934[7] = 0.0;
   out_8366129814821909934[8] = 0.0;
   out_8366129814821909934[9] = 0.0;
   out_8366129814821909934[10] = 1.0;
   out_8366129814821909934[11] = 0.0;
   out_8366129814821909934[12] = 0.0;
   out_8366129814821909934[13] = 0.0;
   out_8366129814821909934[14] = 0.0;
   out_8366129814821909934[15] = 0.0;
   out_8366129814821909934[16] = 0.0;
   out_8366129814821909934[17] = 0.0;
   out_8366129814821909934[18] = 0.0;
   out_8366129814821909934[19] = 0.0;
   out_8366129814821909934[20] = 1.0;
   out_8366129814821909934[21] = 0.0;
   out_8366129814821909934[22] = 0.0;
   out_8366129814821909934[23] = 0.0;
   out_8366129814821909934[24] = 0.0;
   out_8366129814821909934[25] = 0.0;
   out_8366129814821909934[26] = 0.0;
   out_8366129814821909934[27] = 0.0;
   out_8366129814821909934[28] = 0.0;
   out_8366129814821909934[29] = 0.0;
   out_8366129814821909934[30] = 1.0;
   out_8366129814821909934[31] = 0.0;
   out_8366129814821909934[32] = 0.0;
   out_8366129814821909934[33] = 0.0;
   out_8366129814821909934[34] = 0.0;
   out_8366129814821909934[35] = 0.0;
   out_8366129814821909934[36] = 0.0;
   out_8366129814821909934[37] = 0.0;
   out_8366129814821909934[38] = 0.0;
   out_8366129814821909934[39] = 0.0;
   out_8366129814821909934[40] = 1.0;
   out_8366129814821909934[41] = 0.0;
   out_8366129814821909934[42] = 0.0;
   out_8366129814821909934[43] = 0.0;
   out_8366129814821909934[44] = 0.0;
   out_8366129814821909934[45] = 0.0;
   out_8366129814821909934[46] = 0.0;
   out_8366129814821909934[47] = 0.0;
   out_8366129814821909934[48] = 0.0;
   out_8366129814821909934[49] = 0.0;
   out_8366129814821909934[50] = 1.0;
   out_8366129814821909934[51] = 0.0;
   out_8366129814821909934[52] = 0.0;
   out_8366129814821909934[53] = 0.0;
   out_8366129814821909934[54] = 0.0;
   out_8366129814821909934[55] = 0.0;
   out_8366129814821909934[56] = 0.0;
   out_8366129814821909934[57] = 0.0;
   out_8366129814821909934[58] = 0.0;
   out_8366129814821909934[59] = 0.0;
   out_8366129814821909934[60] = 1.0;
   out_8366129814821909934[61] = 0.0;
   out_8366129814821909934[62] = 0.0;
   out_8366129814821909934[63] = 0.0;
   out_8366129814821909934[64] = 0.0;
   out_8366129814821909934[65] = 0.0;
   out_8366129814821909934[66] = 0.0;
   out_8366129814821909934[67] = 0.0;
   out_8366129814821909934[68] = 0.0;
   out_8366129814821909934[69] = 0.0;
   out_8366129814821909934[70] = 1.0;
   out_8366129814821909934[71] = 0.0;
   out_8366129814821909934[72] = 0.0;
   out_8366129814821909934[73] = 0.0;
   out_8366129814821909934[74] = 0.0;
   out_8366129814821909934[75] = 0.0;
   out_8366129814821909934[76] = 0.0;
   out_8366129814821909934[77] = 0.0;
   out_8366129814821909934[78] = 0.0;
   out_8366129814821909934[79] = 0.0;
   out_8366129814821909934[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3833305672384494666) {
   out_3833305672384494666[0] = state[0];
   out_3833305672384494666[1] = state[1];
   out_3833305672384494666[2] = state[2];
   out_3833305672384494666[3] = state[3];
   out_3833305672384494666[4] = state[4];
   out_3833305672384494666[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3833305672384494666[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3833305672384494666[7] = state[7];
   out_3833305672384494666[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2882009860667348310) {
   out_2882009860667348310[0] = 1;
   out_2882009860667348310[1] = 0;
   out_2882009860667348310[2] = 0;
   out_2882009860667348310[3] = 0;
   out_2882009860667348310[4] = 0;
   out_2882009860667348310[5] = 0;
   out_2882009860667348310[6] = 0;
   out_2882009860667348310[7] = 0;
   out_2882009860667348310[8] = 0;
   out_2882009860667348310[9] = 0;
   out_2882009860667348310[10] = 1;
   out_2882009860667348310[11] = 0;
   out_2882009860667348310[12] = 0;
   out_2882009860667348310[13] = 0;
   out_2882009860667348310[14] = 0;
   out_2882009860667348310[15] = 0;
   out_2882009860667348310[16] = 0;
   out_2882009860667348310[17] = 0;
   out_2882009860667348310[18] = 0;
   out_2882009860667348310[19] = 0;
   out_2882009860667348310[20] = 1;
   out_2882009860667348310[21] = 0;
   out_2882009860667348310[22] = 0;
   out_2882009860667348310[23] = 0;
   out_2882009860667348310[24] = 0;
   out_2882009860667348310[25] = 0;
   out_2882009860667348310[26] = 0;
   out_2882009860667348310[27] = 0;
   out_2882009860667348310[28] = 0;
   out_2882009860667348310[29] = 0;
   out_2882009860667348310[30] = 1;
   out_2882009860667348310[31] = 0;
   out_2882009860667348310[32] = 0;
   out_2882009860667348310[33] = 0;
   out_2882009860667348310[34] = 0;
   out_2882009860667348310[35] = 0;
   out_2882009860667348310[36] = 0;
   out_2882009860667348310[37] = 0;
   out_2882009860667348310[38] = 0;
   out_2882009860667348310[39] = 0;
   out_2882009860667348310[40] = 1;
   out_2882009860667348310[41] = 0;
   out_2882009860667348310[42] = 0;
   out_2882009860667348310[43] = 0;
   out_2882009860667348310[44] = 0;
   out_2882009860667348310[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2882009860667348310[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2882009860667348310[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2882009860667348310[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2882009860667348310[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2882009860667348310[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2882009860667348310[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2882009860667348310[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2882009860667348310[53] = -9.8000000000000007*dt;
   out_2882009860667348310[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2882009860667348310[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2882009860667348310[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2882009860667348310[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2882009860667348310[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2882009860667348310[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2882009860667348310[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2882009860667348310[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2882009860667348310[62] = 0;
   out_2882009860667348310[63] = 0;
   out_2882009860667348310[64] = 0;
   out_2882009860667348310[65] = 0;
   out_2882009860667348310[66] = 0;
   out_2882009860667348310[67] = 0;
   out_2882009860667348310[68] = 0;
   out_2882009860667348310[69] = 0;
   out_2882009860667348310[70] = 1;
   out_2882009860667348310[71] = 0;
   out_2882009860667348310[72] = 0;
   out_2882009860667348310[73] = 0;
   out_2882009860667348310[74] = 0;
   out_2882009860667348310[75] = 0;
   out_2882009860667348310[76] = 0;
   out_2882009860667348310[77] = 0;
   out_2882009860667348310[78] = 0;
   out_2882009860667348310[79] = 0;
   out_2882009860667348310[80] = 1;
}
void h_25(double *state, double *unused, double *out_3318328831680801391) {
   out_3318328831680801391[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6557633608898150005) {
   out_6557633608898150005[0] = 0;
   out_6557633608898150005[1] = 0;
   out_6557633608898150005[2] = 0;
   out_6557633608898150005[3] = 0;
   out_6557633608898150005[4] = 0;
   out_6557633608898150005[5] = 0;
   out_6557633608898150005[6] = 1;
   out_6557633608898150005[7] = 0;
   out_6557633608898150005[8] = 0;
}
void h_24(double *state, double *unused, double *out_2312509804207382212) {
   out_2312509804207382212[0] = state[4];
   out_2312509804207382212[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7142601137844892577) {
   out_7142601137844892577[0] = 0;
   out_7142601137844892577[1] = 0;
   out_7142601137844892577[2] = 0;
   out_7142601137844892577[3] = 0;
   out_7142601137844892577[4] = 1;
   out_7142601137844892577[5] = 0;
   out_7142601137844892577[6] = 0;
   out_7142601137844892577[7] = 0;
   out_7142601137844892577[8] = 0;
   out_7142601137844892577[9] = 0;
   out_7142601137844892577[10] = 0;
   out_7142601137844892577[11] = 0;
   out_7142601137844892577[12] = 0;
   out_7142601137844892577[13] = 0;
   out_7142601137844892577[14] = 1;
   out_7142601137844892577[15] = 0;
   out_7142601137844892577[16] = 0;
   out_7142601137844892577[17] = 0;
}
void h_30(double *state, double *unused, double *out_3593522893965307280) {
   out_3593522893965307280[0] = state[4];
}
void H_30(double *state, double *unused, double *out_359056732593466750) {
   out_359056732593466750[0] = 0;
   out_359056732593466750[1] = 0;
   out_359056732593466750[2] = 0;
   out_359056732593466750[3] = 0;
   out_359056732593466750[4] = 1;
   out_359056732593466750[5] = 0;
   out_359056732593466750[6] = 0;
   out_359056732593466750[7] = 0;
   out_359056732593466750[8] = 0;
}
void h_26(double *state, double *unused, double *out_2049206977115649555) {
   out_2049206977115649555[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8147607145937345387) {
   out_8147607145937345387[0] = 0;
   out_8147607145937345387[1] = 0;
   out_8147607145937345387[2] = 0;
   out_8147607145937345387[3] = 0;
   out_8147607145937345387[4] = 0;
   out_8147607145937345387[5] = 0;
   out_8147607145937345387[6] = 0;
   out_8147607145937345387[7] = 1;
   out_8147607145937345387[8] = 0;
}
void h_27(double *state, double *unused, double *out_3080712685733047755) {
   out_3080712685733047755[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1815706579206958161) {
   out_1815706579206958161[0] = 0;
   out_1815706579206958161[1] = 0;
   out_1815706579206958161[2] = 0;
   out_1815706579206958161[3] = 1;
   out_1815706579206958161[4] = 0;
   out_1815706579206958161[5] = 0;
   out_1815706579206958161[6] = 0;
   out_1815706579206958161[7] = 0;
   out_1815706579206958161[8] = 0;
}
void h_29(double *state, double *unused, double *out_556736842520974595) {
   out_556736842520974595[0] = state[1];
}
void H_29(double *state, double *unused, double *out_869288076907858934) {
   out_869288076907858934[0] = 0;
   out_869288076907858934[1] = 1;
   out_869288076907858934[2] = 0;
   out_869288076907858934[3] = 0;
   out_869288076907858934[4] = 0;
   out_869288076907858934[5] = 0;
   out_869288076907858934[6] = 0;
   out_869288076907858934[7] = 0;
   out_869288076907858934[8] = 0;
}
void h_28(double *state, double *unused, double *out_4024222538634710190) {
   out_4024222538634710190[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4213110940161671640) {
   out_4213110940161671640[0] = 1;
   out_4213110940161671640[1] = 0;
   out_4213110940161671640[2] = 0;
   out_4213110940161671640[3] = 0;
   out_4213110940161671640[4] = 0;
   out_4213110940161671640[5] = 0;
   out_4213110940161671640[6] = 0;
   out_4213110940161671640[7] = 0;
   out_4213110940161671640[8] = 0;
}
void h_31(double *state, double *unused, double *out_7235475747144226866) {
   out_7235475747144226866[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6526987647021189577) {
   out_6526987647021189577[0] = 0;
   out_6526987647021189577[1] = 0;
   out_6526987647021189577[2] = 0;
   out_6526987647021189577[3] = 0;
   out_6526987647021189577[4] = 0;
   out_6526987647021189577[5] = 0;
   out_6526987647021189577[6] = 0;
   out_6526987647021189577[7] = 0;
   out_6526987647021189577[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8939131331752933554) {
  err_fun(nom_x, delta_x, out_8939131331752933554);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4134102941266011107) {
  inv_err_fun(nom_x, true_x, out_4134102941266011107);
}
void car_H_mod_fun(double *state, double *out_8366129814821909934) {
  H_mod_fun(state, out_8366129814821909934);
}
void car_f_fun(double *state, double dt, double *out_3833305672384494666) {
  f_fun(state,  dt, out_3833305672384494666);
}
void car_F_fun(double *state, double dt, double *out_2882009860667348310) {
  F_fun(state,  dt, out_2882009860667348310);
}
void car_h_25(double *state, double *unused, double *out_3318328831680801391) {
  h_25(state, unused, out_3318328831680801391);
}
void car_H_25(double *state, double *unused, double *out_6557633608898150005) {
  H_25(state, unused, out_6557633608898150005);
}
void car_h_24(double *state, double *unused, double *out_2312509804207382212) {
  h_24(state, unused, out_2312509804207382212);
}
void car_H_24(double *state, double *unused, double *out_7142601137844892577) {
  H_24(state, unused, out_7142601137844892577);
}
void car_h_30(double *state, double *unused, double *out_3593522893965307280) {
  h_30(state, unused, out_3593522893965307280);
}
void car_H_30(double *state, double *unused, double *out_359056732593466750) {
  H_30(state, unused, out_359056732593466750);
}
void car_h_26(double *state, double *unused, double *out_2049206977115649555) {
  h_26(state, unused, out_2049206977115649555);
}
void car_H_26(double *state, double *unused, double *out_8147607145937345387) {
  H_26(state, unused, out_8147607145937345387);
}
void car_h_27(double *state, double *unused, double *out_3080712685733047755) {
  h_27(state, unused, out_3080712685733047755);
}
void car_H_27(double *state, double *unused, double *out_1815706579206958161) {
  H_27(state, unused, out_1815706579206958161);
}
void car_h_29(double *state, double *unused, double *out_556736842520974595) {
  h_29(state, unused, out_556736842520974595);
}
void car_H_29(double *state, double *unused, double *out_869288076907858934) {
  H_29(state, unused, out_869288076907858934);
}
void car_h_28(double *state, double *unused, double *out_4024222538634710190) {
  h_28(state, unused, out_4024222538634710190);
}
void car_H_28(double *state, double *unused, double *out_4213110940161671640) {
  H_28(state, unused, out_4213110940161671640);
}
void car_h_31(double *state, double *unused, double *out_7235475747144226866) {
  h_31(state, unused, out_7235475747144226866);
}
void car_H_31(double *state, double *unused, double *out_6526987647021189577) {
  H_31(state, unused, out_6526987647021189577);
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
