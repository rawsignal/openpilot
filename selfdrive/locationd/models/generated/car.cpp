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
void err_fun(double *nom_x, double *delta_x, double *out_1831389753275826188) {
   out_1831389753275826188[0] = delta_x[0] + nom_x[0];
   out_1831389753275826188[1] = delta_x[1] + nom_x[1];
   out_1831389753275826188[2] = delta_x[2] + nom_x[2];
   out_1831389753275826188[3] = delta_x[3] + nom_x[3];
   out_1831389753275826188[4] = delta_x[4] + nom_x[4];
   out_1831389753275826188[5] = delta_x[5] + nom_x[5];
   out_1831389753275826188[6] = delta_x[6] + nom_x[6];
   out_1831389753275826188[7] = delta_x[7] + nom_x[7];
   out_1831389753275826188[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4290228637053937851) {
   out_4290228637053937851[0] = -nom_x[0] + true_x[0];
   out_4290228637053937851[1] = -nom_x[1] + true_x[1];
   out_4290228637053937851[2] = -nom_x[2] + true_x[2];
   out_4290228637053937851[3] = -nom_x[3] + true_x[3];
   out_4290228637053937851[4] = -nom_x[4] + true_x[4];
   out_4290228637053937851[5] = -nom_x[5] + true_x[5];
   out_4290228637053937851[6] = -nom_x[6] + true_x[6];
   out_4290228637053937851[7] = -nom_x[7] + true_x[7];
   out_4290228637053937851[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2619348439350555686) {
   out_2619348439350555686[0] = 1.0;
   out_2619348439350555686[1] = 0.0;
   out_2619348439350555686[2] = 0.0;
   out_2619348439350555686[3] = 0.0;
   out_2619348439350555686[4] = 0.0;
   out_2619348439350555686[5] = 0.0;
   out_2619348439350555686[6] = 0.0;
   out_2619348439350555686[7] = 0.0;
   out_2619348439350555686[8] = 0.0;
   out_2619348439350555686[9] = 0.0;
   out_2619348439350555686[10] = 1.0;
   out_2619348439350555686[11] = 0.0;
   out_2619348439350555686[12] = 0.0;
   out_2619348439350555686[13] = 0.0;
   out_2619348439350555686[14] = 0.0;
   out_2619348439350555686[15] = 0.0;
   out_2619348439350555686[16] = 0.0;
   out_2619348439350555686[17] = 0.0;
   out_2619348439350555686[18] = 0.0;
   out_2619348439350555686[19] = 0.0;
   out_2619348439350555686[20] = 1.0;
   out_2619348439350555686[21] = 0.0;
   out_2619348439350555686[22] = 0.0;
   out_2619348439350555686[23] = 0.0;
   out_2619348439350555686[24] = 0.0;
   out_2619348439350555686[25] = 0.0;
   out_2619348439350555686[26] = 0.0;
   out_2619348439350555686[27] = 0.0;
   out_2619348439350555686[28] = 0.0;
   out_2619348439350555686[29] = 0.0;
   out_2619348439350555686[30] = 1.0;
   out_2619348439350555686[31] = 0.0;
   out_2619348439350555686[32] = 0.0;
   out_2619348439350555686[33] = 0.0;
   out_2619348439350555686[34] = 0.0;
   out_2619348439350555686[35] = 0.0;
   out_2619348439350555686[36] = 0.0;
   out_2619348439350555686[37] = 0.0;
   out_2619348439350555686[38] = 0.0;
   out_2619348439350555686[39] = 0.0;
   out_2619348439350555686[40] = 1.0;
   out_2619348439350555686[41] = 0.0;
   out_2619348439350555686[42] = 0.0;
   out_2619348439350555686[43] = 0.0;
   out_2619348439350555686[44] = 0.0;
   out_2619348439350555686[45] = 0.0;
   out_2619348439350555686[46] = 0.0;
   out_2619348439350555686[47] = 0.0;
   out_2619348439350555686[48] = 0.0;
   out_2619348439350555686[49] = 0.0;
   out_2619348439350555686[50] = 1.0;
   out_2619348439350555686[51] = 0.0;
   out_2619348439350555686[52] = 0.0;
   out_2619348439350555686[53] = 0.0;
   out_2619348439350555686[54] = 0.0;
   out_2619348439350555686[55] = 0.0;
   out_2619348439350555686[56] = 0.0;
   out_2619348439350555686[57] = 0.0;
   out_2619348439350555686[58] = 0.0;
   out_2619348439350555686[59] = 0.0;
   out_2619348439350555686[60] = 1.0;
   out_2619348439350555686[61] = 0.0;
   out_2619348439350555686[62] = 0.0;
   out_2619348439350555686[63] = 0.0;
   out_2619348439350555686[64] = 0.0;
   out_2619348439350555686[65] = 0.0;
   out_2619348439350555686[66] = 0.0;
   out_2619348439350555686[67] = 0.0;
   out_2619348439350555686[68] = 0.0;
   out_2619348439350555686[69] = 0.0;
   out_2619348439350555686[70] = 1.0;
   out_2619348439350555686[71] = 0.0;
   out_2619348439350555686[72] = 0.0;
   out_2619348439350555686[73] = 0.0;
   out_2619348439350555686[74] = 0.0;
   out_2619348439350555686[75] = 0.0;
   out_2619348439350555686[76] = 0.0;
   out_2619348439350555686[77] = 0.0;
   out_2619348439350555686[78] = 0.0;
   out_2619348439350555686[79] = 0.0;
   out_2619348439350555686[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_849567269053238190) {
   out_849567269053238190[0] = state[0];
   out_849567269053238190[1] = state[1];
   out_849567269053238190[2] = state[2];
   out_849567269053238190[3] = state[3];
   out_849567269053238190[4] = state[4];
   out_849567269053238190[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_849567269053238190[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_849567269053238190[7] = state[7];
   out_849567269053238190[8] = state[8];
}
void F_fun(double *state, double dt, double *out_9107872014031309504) {
   out_9107872014031309504[0] = 1;
   out_9107872014031309504[1] = 0;
   out_9107872014031309504[2] = 0;
   out_9107872014031309504[3] = 0;
   out_9107872014031309504[4] = 0;
   out_9107872014031309504[5] = 0;
   out_9107872014031309504[6] = 0;
   out_9107872014031309504[7] = 0;
   out_9107872014031309504[8] = 0;
   out_9107872014031309504[9] = 0;
   out_9107872014031309504[10] = 1;
   out_9107872014031309504[11] = 0;
   out_9107872014031309504[12] = 0;
   out_9107872014031309504[13] = 0;
   out_9107872014031309504[14] = 0;
   out_9107872014031309504[15] = 0;
   out_9107872014031309504[16] = 0;
   out_9107872014031309504[17] = 0;
   out_9107872014031309504[18] = 0;
   out_9107872014031309504[19] = 0;
   out_9107872014031309504[20] = 1;
   out_9107872014031309504[21] = 0;
   out_9107872014031309504[22] = 0;
   out_9107872014031309504[23] = 0;
   out_9107872014031309504[24] = 0;
   out_9107872014031309504[25] = 0;
   out_9107872014031309504[26] = 0;
   out_9107872014031309504[27] = 0;
   out_9107872014031309504[28] = 0;
   out_9107872014031309504[29] = 0;
   out_9107872014031309504[30] = 1;
   out_9107872014031309504[31] = 0;
   out_9107872014031309504[32] = 0;
   out_9107872014031309504[33] = 0;
   out_9107872014031309504[34] = 0;
   out_9107872014031309504[35] = 0;
   out_9107872014031309504[36] = 0;
   out_9107872014031309504[37] = 0;
   out_9107872014031309504[38] = 0;
   out_9107872014031309504[39] = 0;
   out_9107872014031309504[40] = 1;
   out_9107872014031309504[41] = 0;
   out_9107872014031309504[42] = 0;
   out_9107872014031309504[43] = 0;
   out_9107872014031309504[44] = 0;
   out_9107872014031309504[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_9107872014031309504[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_9107872014031309504[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9107872014031309504[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9107872014031309504[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_9107872014031309504[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_9107872014031309504[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_9107872014031309504[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_9107872014031309504[53] = -9.8000000000000007*dt;
   out_9107872014031309504[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_9107872014031309504[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_9107872014031309504[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9107872014031309504[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9107872014031309504[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_9107872014031309504[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_9107872014031309504[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_9107872014031309504[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9107872014031309504[62] = 0;
   out_9107872014031309504[63] = 0;
   out_9107872014031309504[64] = 0;
   out_9107872014031309504[65] = 0;
   out_9107872014031309504[66] = 0;
   out_9107872014031309504[67] = 0;
   out_9107872014031309504[68] = 0;
   out_9107872014031309504[69] = 0;
   out_9107872014031309504[70] = 1;
   out_9107872014031309504[71] = 0;
   out_9107872014031309504[72] = 0;
   out_9107872014031309504[73] = 0;
   out_9107872014031309504[74] = 0;
   out_9107872014031309504[75] = 0;
   out_9107872014031309504[76] = 0;
   out_9107872014031309504[77] = 0;
   out_9107872014031309504[78] = 0;
   out_9107872014031309504[79] = 0;
   out_9107872014031309504[80] = 1;
}
void h_25(double *state, double *unused, double *out_5454240522955423473) {
   out_5454240522955423473[0] = state[6];
}
void H_25(double *state, double *unused, double *out_9116371399240585637) {
   out_9116371399240585637[0] = 0;
   out_9116371399240585637[1] = 0;
   out_9116371399240585637[2] = 0;
   out_9116371399240585637[3] = 0;
   out_9116371399240585637[4] = 0;
   out_9116371399240585637[5] = 0;
   out_9116371399240585637[6] = 1;
   out_9116371399240585637[7] = 0;
   out_9116371399240585637[8] = 0;
}
void h_24(double *state, double *unused, double *out_8007148282328552359) {
   out_8007148282328552359[0] = state[4];
   out_8007148282328552359[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8859915192425627255) {
   out_8859915192425627255[0] = 0;
   out_8859915192425627255[1] = 0;
   out_8859915192425627255[2] = 0;
   out_8859915192425627255[3] = 0;
   out_8859915192425627255[4] = 1;
   out_8859915192425627255[5] = 0;
   out_8859915192425627255[6] = 0;
   out_8859915192425627255[7] = 0;
   out_8859915192425627255[8] = 0;
   out_8859915192425627255[9] = 0;
   out_8859915192425627255[10] = 0;
   out_8859915192425627255[11] = 0;
   out_8859915192425627255[12] = 0;
   out_8859915192425627255[13] = 0;
   out_8859915192425627255[14] = 1;
   out_8859915192425627255[15] = 0;
   out_8859915192425627255[16] = 0;
   out_8859915192425627255[17] = 0;
}
void h_30(double *state, double *unused, double *out_9091690051209445823) {
   out_9091690051209445823[0] = state[4];
}
void H_30(double *state, double *unused, double *out_9201033727325725909) {
   out_9201033727325725909[0] = 0;
   out_9201033727325725909[1] = 0;
   out_9201033727325725909[2] = 0;
   out_9201033727325725909[3] = 0;
   out_9201033727325725909[4] = 1;
   out_9201033727325725909[5] = 0;
   out_9201033727325725909[6] = 0;
   out_9201033727325725909[7] = 0;
   out_9201033727325725909[8] = 0;
}
void h_26(double *state, double *unused, double *out_148850902318785726) {
   out_148850902318785726[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5588869355594909755) {
   out_5588869355594909755[0] = 0;
   out_5588869355594909755[1] = 0;
   out_5588869355594909755[2] = 0;
   out_5588869355594909755[3] = 0;
   out_5588869355594909755[4] = 0;
   out_5588869355594909755[5] = 0;
   out_5588869355594909755[6] = 0;
   out_5588869355594909755[7] = 1;
   out_5588869355594909755[8] = 0;
}
void h_27(double *state, double *unused, double *out_7990830283093282498) {
   out_7990830283093282498[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7026270415525300998) {
   out_7026270415525300998[0] = 0;
   out_7026270415525300998[1] = 0;
   out_7026270415525300998[2] = 0;
   out_7026270415525300998[3] = 1;
   out_7026270415525300998[4] = 0;
   out_7026270415525300998[5] = 0;
   out_7026270415525300998[6] = 0;
   out_7026270415525300998[7] = 0;
   out_7026270415525300998[8] = 0;
}
void h_29(double *state, double *unused, double *out_4129146806664780846) {
   out_4129146806664780846[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5312907688655749965) {
   out_5312907688655749965[0] = 0;
   out_5312907688655749965[1] = 1;
   out_5312907688655749965[2] = 0;
   out_5312907688655749965[3] = 0;
   out_5312907688655749965[4] = 0;
   out_5312907688655749965[5] = 0;
   out_5312907688655749965[6] = 0;
   out_5312907688655749965[7] = 0;
   out_5312907688655749965[8] = 0;
}
void h_28(double *state, double *unused, double *out_727006636775409828) {
   out_727006636775409828[0] = state[0];
}
void H_28(double *state, double *unused, double *out_230508671586219391) {
   out_230508671586219391[0] = 1;
   out_230508671586219391[1] = 0;
   out_230508671586219391[2] = 0;
   out_230508671586219391[3] = 0;
   out_230508671586219391[4] = 0;
   out_230508671586219391[5] = 0;
   out_230508671586219391[6] = 0;
   out_230508671586219391[7] = 0;
   out_230508671586219391[8] = 0;
}
void h_31(double *state, double *unused, double *out_5531944786505239818) {
   out_5531944786505239818[0] = state[8];
}
void H_31(double *state, double *unused, double *out_9085725437363625209) {
   out_9085725437363625209[0] = 0;
   out_9085725437363625209[1] = 0;
   out_9085725437363625209[2] = 0;
   out_9085725437363625209[3] = 0;
   out_9085725437363625209[4] = 0;
   out_9085725437363625209[5] = 0;
   out_9085725437363625209[6] = 0;
   out_9085725437363625209[7] = 0;
   out_9085725437363625209[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1831389753275826188) {
  err_fun(nom_x, delta_x, out_1831389753275826188);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4290228637053937851) {
  inv_err_fun(nom_x, true_x, out_4290228637053937851);
}
void car_H_mod_fun(double *state, double *out_2619348439350555686) {
  H_mod_fun(state, out_2619348439350555686);
}
void car_f_fun(double *state, double dt, double *out_849567269053238190) {
  f_fun(state,  dt, out_849567269053238190);
}
void car_F_fun(double *state, double dt, double *out_9107872014031309504) {
  F_fun(state,  dt, out_9107872014031309504);
}
void car_h_25(double *state, double *unused, double *out_5454240522955423473) {
  h_25(state, unused, out_5454240522955423473);
}
void car_H_25(double *state, double *unused, double *out_9116371399240585637) {
  H_25(state, unused, out_9116371399240585637);
}
void car_h_24(double *state, double *unused, double *out_8007148282328552359) {
  h_24(state, unused, out_8007148282328552359);
}
void car_H_24(double *state, double *unused, double *out_8859915192425627255) {
  H_24(state, unused, out_8859915192425627255);
}
void car_h_30(double *state, double *unused, double *out_9091690051209445823) {
  h_30(state, unused, out_9091690051209445823);
}
void car_H_30(double *state, double *unused, double *out_9201033727325725909) {
  H_30(state, unused, out_9201033727325725909);
}
void car_h_26(double *state, double *unused, double *out_148850902318785726) {
  h_26(state, unused, out_148850902318785726);
}
void car_H_26(double *state, double *unused, double *out_5588869355594909755) {
  H_26(state, unused, out_5588869355594909755);
}
void car_h_27(double *state, double *unused, double *out_7990830283093282498) {
  h_27(state, unused, out_7990830283093282498);
}
void car_H_27(double *state, double *unused, double *out_7026270415525300998) {
  H_27(state, unused, out_7026270415525300998);
}
void car_h_29(double *state, double *unused, double *out_4129146806664780846) {
  h_29(state, unused, out_4129146806664780846);
}
void car_H_29(double *state, double *unused, double *out_5312907688655749965) {
  H_29(state, unused, out_5312907688655749965);
}
void car_h_28(double *state, double *unused, double *out_727006636775409828) {
  h_28(state, unused, out_727006636775409828);
}
void car_H_28(double *state, double *unused, double *out_230508671586219391) {
  H_28(state, unused, out_230508671586219391);
}
void car_h_31(double *state, double *unused, double *out_5531944786505239818) {
  h_31(state, unused, out_5531944786505239818);
}
void car_H_31(double *state, double *unused, double *out_9085725437363625209) {
  H_31(state, unused, out_9085725437363625209);
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
