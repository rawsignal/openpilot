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
void err_fun(double *nom_x, double *delta_x, double *out_2891105964222538946) {
   out_2891105964222538946[0] = delta_x[0] + nom_x[0];
   out_2891105964222538946[1] = delta_x[1] + nom_x[1];
   out_2891105964222538946[2] = delta_x[2] + nom_x[2];
   out_2891105964222538946[3] = delta_x[3] + nom_x[3];
   out_2891105964222538946[4] = delta_x[4] + nom_x[4];
   out_2891105964222538946[5] = delta_x[5] + nom_x[5];
   out_2891105964222538946[6] = delta_x[6] + nom_x[6];
   out_2891105964222538946[7] = delta_x[7] + nom_x[7];
   out_2891105964222538946[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6776165792917660510) {
   out_6776165792917660510[0] = -nom_x[0] + true_x[0];
   out_6776165792917660510[1] = -nom_x[1] + true_x[1];
   out_6776165792917660510[2] = -nom_x[2] + true_x[2];
   out_6776165792917660510[3] = -nom_x[3] + true_x[3];
   out_6776165792917660510[4] = -nom_x[4] + true_x[4];
   out_6776165792917660510[5] = -nom_x[5] + true_x[5];
   out_6776165792917660510[6] = -nom_x[6] + true_x[6];
   out_6776165792917660510[7] = -nom_x[7] + true_x[7];
   out_6776165792917660510[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_9150571167475995922) {
   out_9150571167475995922[0] = 1.0;
   out_9150571167475995922[1] = 0.0;
   out_9150571167475995922[2] = 0.0;
   out_9150571167475995922[3] = 0.0;
   out_9150571167475995922[4] = 0.0;
   out_9150571167475995922[5] = 0.0;
   out_9150571167475995922[6] = 0.0;
   out_9150571167475995922[7] = 0.0;
   out_9150571167475995922[8] = 0.0;
   out_9150571167475995922[9] = 0.0;
   out_9150571167475995922[10] = 1.0;
   out_9150571167475995922[11] = 0.0;
   out_9150571167475995922[12] = 0.0;
   out_9150571167475995922[13] = 0.0;
   out_9150571167475995922[14] = 0.0;
   out_9150571167475995922[15] = 0.0;
   out_9150571167475995922[16] = 0.0;
   out_9150571167475995922[17] = 0.0;
   out_9150571167475995922[18] = 0.0;
   out_9150571167475995922[19] = 0.0;
   out_9150571167475995922[20] = 1.0;
   out_9150571167475995922[21] = 0.0;
   out_9150571167475995922[22] = 0.0;
   out_9150571167475995922[23] = 0.0;
   out_9150571167475995922[24] = 0.0;
   out_9150571167475995922[25] = 0.0;
   out_9150571167475995922[26] = 0.0;
   out_9150571167475995922[27] = 0.0;
   out_9150571167475995922[28] = 0.0;
   out_9150571167475995922[29] = 0.0;
   out_9150571167475995922[30] = 1.0;
   out_9150571167475995922[31] = 0.0;
   out_9150571167475995922[32] = 0.0;
   out_9150571167475995922[33] = 0.0;
   out_9150571167475995922[34] = 0.0;
   out_9150571167475995922[35] = 0.0;
   out_9150571167475995922[36] = 0.0;
   out_9150571167475995922[37] = 0.0;
   out_9150571167475995922[38] = 0.0;
   out_9150571167475995922[39] = 0.0;
   out_9150571167475995922[40] = 1.0;
   out_9150571167475995922[41] = 0.0;
   out_9150571167475995922[42] = 0.0;
   out_9150571167475995922[43] = 0.0;
   out_9150571167475995922[44] = 0.0;
   out_9150571167475995922[45] = 0.0;
   out_9150571167475995922[46] = 0.0;
   out_9150571167475995922[47] = 0.0;
   out_9150571167475995922[48] = 0.0;
   out_9150571167475995922[49] = 0.0;
   out_9150571167475995922[50] = 1.0;
   out_9150571167475995922[51] = 0.0;
   out_9150571167475995922[52] = 0.0;
   out_9150571167475995922[53] = 0.0;
   out_9150571167475995922[54] = 0.0;
   out_9150571167475995922[55] = 0.0;
   out_9150571167475995922[56] = 0.0;
   out_9150571167475995922[57] = 0.0;
   out_9150571167475995922[58] = 0.0;
   out_9150571167475995922[59] = 0.0;
   out_9150571167475995922[60] = 1.0;
   out_9150571167475995922[61] = 0.0;
   out_9150571167475995922[62] = 0.0;
   out_9150571167475995922[63] = 0.0;
   out_9150571167475995922[64] = 0.0;
   out_9150571167475995922[65] = 0.0;
   out_9150571167475995922[66] = 0.0;
   out_9150571167475995922[67] = 0.0;
   out_9150571167475995922[68] = 0.0;
   out_9150571167475995922[69] = 0.0;
   out_9150571167475995922[70] = 1.0;
   out_9150571167475995922[71] = 0.0;
   out_9150571167475995922[72] = 0.0;
   out_9150571167475995922[73] = 0.0;
   out_9150571167475995922[74] = 0.0;
   out_9150571167475995922[75] = 0.0;
   out_9150571167475995922[76] = 0.0;
   out_9150571167475995922[77] = 0.0;
   out_9150571167475995922[78] = 0.0;
   out_9150571167475995922[79] = 0.0;
   out_9150571167475995922[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3162943793073791679) {
   out_3162943793073791679[0] = state[0];
   out_3162943793073791679[1] = state[1];
   out_3162943793073791679[2] = state[2];
   out_3162943793073791679[3] = state[3];
   out_3162943793073791679[4] = state[4];
   out_3162943793073791679[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3162943793073791679[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3162943793073791679[7] = state[7];
   out_3162943793073791679[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1510888866567458713) {
   out_1510888866567458713[0] = 1;
   out_1510888866567458713[1] = 0;
   out_1510888866567458713[2] = 0;
   out_1510888866567458713[3] = 0;
   out_1510888866567458713[4] = 0;
   out_1510888866567458713[5] = 0;
   out_1510888866567458713[6] = 0;
   out_1510888866567458713[7] = 0;
   out_1510888866567458713[8] = 0;
   out_1510888866567458713[9] = 0;
   out_1510888866567458713[10] = 1;
   out_1510888866567458713[11] = 0;
   out_1510888866567458713[12] = 0;
   out_1510888866567458713[13] = 0;
   out_1510888866567458713[14] = 0;
   out_1510888866567458713[15] = 0;
   out_1510888866567458713[16] = 0;
   out_1510888866567458713[17] = 0;
   out_1510888866567458713[18] = 0;
   out_1510888866567458713[19] = 0;
   out_1510888866567458713[20] = 1;
   out_1510888866567458713[21] = 0;
   out_1510888866567458713[22] = 0;
   out_1510888866567458713[23] = 0;
   out_1510888866567458713[24] = 0;
   out_1510888866567458713[25] = 0;
   out_1510888866567458713[26] = 0;
   out_1510888866567458713[27] = 0;
   out_1510888866567458713[28] = 0;
   out_1510888866567458713[29] = 0;
   out_1510888866567458713[30] = 1;
   out_1510888866567458713[31] = 0;
   out_1510888866567458713[32] = 0;
   out_1510888866567458713[33] = 0;
   out_1510888866567458713[34] = 0;
   out_1510888866567458713[35] = 0;
   out_1510888866567458713[36] = 0;
   out_1510888866567458713[37] = 0;
   out_1510888866567458713[38] = 0;
   out_1510888866567458713[39] = 0;
   out_1510888866567458713[40] = 1;
   out_1510888866567458713[41] = 0;
   out_1510888866567458713[42] = 0;
   out_1510888866567458713[43] = 0;
   out_1510888866567458713[44] = 0;
   out_1510888866567458713[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1510888866567458713[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1510888866567458713[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1510888866567458713[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1510888866567458713[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1510888866567458713[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1510888866567458713[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1510888866567458713[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1510888866567458713[53] = -9.8000000000000007*dt;
   out_1510888866567458713[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1510888866567458713[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1510888866567458713[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1510888866567458713[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1510888866567458713[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1510888866567458713[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1510888866567458713[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1510888866567458713[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1510888866567458713[62] = 0;
   out_1510888866567458713[63] = 0;
   out_1510888866567458713[64] = 0;
   out_1510888866567458713[65] = 0;
   out_1510888866567458713[66] = 0;
   out_1510888866567458713[67] = 0;
   out_1510888866567458713[68] = 0;
   out_1510888866567458713[69] = 0;
   out_1510888866567458713[70] = 1;
   out_1510888866567458713[71] = 0;
   out_1510888866567458713[72] = 0;
   out_1510888866567458713[73] = 0;
   out_1510888866567458713[74] = 0;
   out_1510888866567458713[75] = 0;
   out_1510888866567458713[76] = 0;
   out_1510888866567458713[77] = 0;
   out_1510888866567458713[78] = 0;
   out_1510888866567458713[79] = 0;
   out_1510888866567458713[80] = 1;
}
void h_25(double *state, double *unused, double *out_1634459234320078510) {
   out_1634459234320078510[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6659615981218129500) {
   out_6659615981218129500[0] = 0;
   out_6659615981218129500[1] = 0;
   out_6659615981218129500[2] = 0;
   out_6659615981218129500[3] = 0;
   out_6659615981218129500[4] = 0;
   out_6659615981218129500[5] = 0;
   out_6659615981218129500[6] = 1;
   out_6659615981218129500[7] = 0;
   out_6659615981218129500[8] = 0;
}
void h_24(double *state, double *unused, double *out_4880598876319921497) {
   out_4880598876319921497[0] = state[4];
   out_4880598876319921497[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2559062906422226891) {
   out_2559062906422226891[0] = 0;
   out_2559062906422226891[1] = 0;
   out_2559062906422226891[2] = 0;
   out_2559062906422226891[3] = 0;
   out_2559062906422226891[4] = 1;
   out_2559062906422226891[5] = 0;
   out_2559062906422226891[6] = 0;
   out_2559062906422226891[7] = 0;
   out_2559062906422226891[8] = 0;
   out_2559062906422226891[9] = 0;
   out_2559062906422226891[10] = 0;
   out_2559062906422226891[11] = 0;
   out_2559062906422226891[12] = 0;
   out_2559062906422226891[13] = 0;
   out_2559062906422226891[14] = 1;
   out_2559062906422226891[15] = 0;
   out_2559062906422226891[16] = 0;
   out_2559062906422226891[17] = 0;
}
void h_30(double *state, double *unused, double *out_1477272565968949365) {
   out_1477272565968949365[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6530277034074889430) {
   out_6530277034074889430[0] = 0;
   out_6530277034074889430[1] = 0;
   out_6530277034074889430[2] = 0;
   out_6530277034074889430[3] = 0;
   out_6530277034074889430[4] = 1;
   out_6530277034074889430[5] = 0;
   out_6530277034074889430[6] = 0;
   out_6530277034074889430[7] = 0;
   out_6530277034074889430[8] = 0;
}
void h_26(double *state, double *unused, double *out_873956681292471848) {
   out_873956681292471848[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2918112662344073276) {
   out_2918112662344073276[0] = 0;
   out_2918112662344073276[1] = 0;
   out_2918112662344073276[2] = 0;
   out_2918112662344073276[3] = 0;
   out_2918112662344073276[4] = 0;
   out_2918112662344073276[5] = 0;
   out_2918112662344073276[6] = 0;
   out_2918112662344073276[7] = 1;
   out_2918112662344073276[8] = 0;
}
void h_27(double *state, double *unused, double *out_7347509937686762255) {
   out_7347509937686762255[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4355513722274464519) {
   out_4355513722274464519[0] = 0;
   out_4355513722274464519[1] = 0;
   out_4355513722274464519[2] = 0;
   out_4355513722274464519[3] = 1;
   out_4355513722274464519[4] = 0;
   out_4355513722274464519[5] = 0;
   out_4355513722274464519[6] = 0;
   out_4355513722274464519[7] = 0;
   out_4355513722274464519[8] = 0;
}
void h_29(double *state, double *unused, double *out_7948928095300125809) {
   out_7948928095300125809[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2642150995404913486) {
   out_2642150995404913486[0] = 0;
   out_2642150995404913486[1] = 1;
   out_2642150995404913486[2] = 0;
   out_2642150995404913486[3] = 0;
   out_2642150995404913486[4] = 0;
   out_2642150995404913486[5] = 0;
   out_2642150995404913486[6] = 0;
   out_2642150995404913486[7] = 0;
   out_2642150995404913486[8] = 0;
}
void h_28(double *state, double *unused, double *out_2345836670025916610) {
   out_2345836670025916610[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2440248021664617088) {
   out_2440248021664617088[0] = 1;
   out_2440248021664617088[1] = 0;
   out_2440248021664617088[2] = 0;
   out_2440248021664617088[3] = 0;
   out_2440248021664617088[4] = 0;
   out_2440248021664617088[5] = 0;
   out_2440248021664617088[6] = 0;
   out_2440248021664617088[7] = 0;
   out_2440248021664617088[8] = 0;
}
void h_31(double *state, double *unused, double *out_4585917948526935459) {
   out_4585917948526935459[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6690261943095089928) {
   out_6690261943095089928[0] = 0;
   out_6690261943095089928[1] = 0;
   out_6690261943095089928[2] = 0;
   out_6690261943095089928[3] = 0;
   out_6690261943095089928[4] = 0;
   out_6690261943095089928[5] = 0;
   out_6690261943095089928[6] = 0;
   out_6690261943095089928[7] = 0;
   out_6690261943095089928[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2891105964222538946) {
  err_fun(nom_x, delta_x, out_2891105964222538946);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6776165792917660510) {
  inv_err_fun(nom_x, true_x, out_6776165792917660510);
}
void car_H_mod_fun(double *state, double *out_9150571167475995922) {
  H_mod_fun(state, out_9150571167475995922);
}
void car_f_fun(double *state, double dt, double *out_3162943793073791679) {
  f_fun(state,  dt, out_3162943793073791679);
}
void car_F_fun(double *state, double dt, double *out_1510888866567458713) {
  F_fun(state,  dt, out_1510888866567458713);
}
void car_h_25(double *state, double *unused, double *out_1634459234320078510) {
  h_25(state, unused, out_1634459234320078510);
}
void car_H_25(double *state, double *unused, double *out_6659615981218129500) {
  H_25(state, unused, out_6659615981218129500);
}
void car_h_24(double *state, double *unused, double *out_4880598876319921497) {
  h_24(state, unused, out_4880598876319921497);
}
void car_H_24(double *state, double *unused, double *out_2559062906422226891) {
  H_24(state, unused, out_2559062906422226891);
}
void car_h_30(double *state, double *unused, double *out_1477272565968949365) {
  h_30(state, unused, out_1477272565968949365);
}
void car_H_30(double *state, double *unused, double *out_6530277034074889430) {
  H_30(state, unused, out_6530277034074889430);
}
void car_h_26(double *state, double *unused, double *out_873956681292471848) {
  h_26(state, unused, out_873956681292471848);
}
void car_H_26(double *state, double *unused, double *out_2918112662344073276) {
  H_26(state, unused, out_2918112662344073276);
}
void car_h_27(double *state, double *unused, double *out_7347509937686762255) {
  h_27(state, unused, out_7347509937686762255);
}
void car_H_27(double *state, double *unused, double *out_4355513722274464519) {
  H_27(state, unused, out_4355513722274464519);
}
void car_h_29(double *state, double *unused, double *out_7948928095300125809) {
  h_29(state, unused, out_7948928095300125809);
}
void car_H_29(double *state, double *unused, double *out_2642150995404913486) {
  H_29(state, unused, out_2642150995404913486);
}
void car_h_28(double *state, double *unused, double *out_2345836670025916610) {
  h_28(state, unused, out_2345836670025916610);
}
void car_H_28(double *state, double *unused, double *out_2440248021664617088) {
  H_28(state, unused, out_2440248021664617088);
}
void car_h_31(double *state, double *unused, double *out_4585917948526935459) {
  h_31(state, unused, out_4585917948526935459);
}
void car_H_31(double *state, double *unused, double *out_6690261943095089928) {
  H_31(state, unused, out_6690261943095089928);
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
