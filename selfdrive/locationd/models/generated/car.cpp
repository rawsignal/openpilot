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
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2816734334840272051) {
   out_2816734334840272051[0] = delta_x[0] + nom_x[0];
   out_2816734334840272051[1] = delta_x[1] + nom_x[1];
   out_2816734334840272051[2] = delta_x[2] + nom_x[2];
   out_2816734334840272051[3] = delta_x[3] + nom_x[3];
   out_2816734334840272051[4] = delta_x[4] + nom_x[4];
   out_2816734334840272051[5] = delta_x[5] + nom_x[5];
   out_2816734334840272051[6] = delta_x[6] + nom_x[6];
   out_2816734334840272051[7] = delta_x[7] + nom_x[7];
   out_2816734334840272051[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3197403674388065693) {
   out_3197403674388065693[0] = -nom_x[0] + true_x[0];
   out_3197403674388065693[1] = -nom_x[1] + true_x[1];
   out_3197403674388065693[2] = -nom_x[2] + true_x[2];
   out_3197403674388065693[3] = -nom_x[3] + true_x[3];
   out_3197403674388065693[4] = -nom_x[4] + true_x[4];
   out_3197403674388065693[5] = -nom_x[5] + true_x[5];
   out_3197403674388065693[6] = -nom_x[6] + true_x[6];
   out_3197403674388065693[7] = -nom_x[7] + true_x[7];
   out_3197403674388065693[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3258567857202627240) {
   out_3258567857202627240[0] = 1.0;
   out_3258567857202627240[1] = 0.0;
   out_3258567857202627240[2] = 0.0;
   out_3258567857202627240[3] = 0.0;
   out_3258567857202627240[4] = 0.0;
   out_3258567857202627240[5] = 0.0;
   out_3258567857202627240[6] = 0.0;
   out_3258567857202627240[7] = 0.0;
   out_3258567857202627240[8] = 0.0;
   out_3258567857202627240[9] = 0.0;
   out_3258567857202627240[10] = 1.0;
   out_3258567857202627240[11] = 0.0;
   out_3258567857202627240[12] = 0.0;
   out_3258567857202627240[13] = 0.0;
   out_3258567857202627240[14] = 0.0;
   out_3258567857202627240[15] = 0.0;
   out_3258567857202627240[16] = 0.0;
   out_3258567857202627240[17] = 0.0;
   out_3258567857202627240[18] = 0.0;
   out_3258567857202627240[19] = 0.0;
   out_3258567857202627240[20] = 1.0;
   out_3258567857202627240[21] = 0.0;
   out_3258567857202627240[22] = 0.0;
   out_3258567857202627240[23] = 0.0;
   out_3258567857202627240[24] = 0.0;
   out_3258567857202627240[25] = 0.0;
   out_3258567857202627240[26] = 0.0;
   out_3258567857202627240[27] = 0.0;
   out_3258567857202627240[28] = 0.0;
   out_3258567857202627240[29] = 0.0;
   out_3258567857202627240[30] = 1.0;
   out_3258567857202627240[31] = 0.0;
   out_3258567857202627240[32] = 0.0;
   out_3258567857202627240[33] = 0.0;
   out_3258567857202627240[34] = 0.0;
   out_3258567857202627240[35] = 0.0;
   out_3258567857202627240[36] = 0.0;
   out_3258567857202627240[37] = 0.0;
   out_3258567857202627240[38] = 0.0;
   out_3258567857202627240[39] = 0.0;
   out_3258567857202627240[40] = 1.0;
   out_3258567857202627240[41] = 0.0;
   out_3258567857202627240[42] = 0.0;
   out_3258567857202627240[43] = 0.0;
   out_3258567857202627240[44] = 0.0;
   out_3258567857202627240[45] = 0.0;
   out_3258567857202627240[46] = 0.0;
   out_3258567857202627240[47] = 0.0;
   out_3258567857202627240[48] = 0.0;
   out_3258567857202627240[49] = 0.0;
   out_3258567857202627240[50] = 1.0;
   out_3258567857202627240[51] = 0.0;
   out_3258567857202627240[52] = 0.0;
   out_3258567857202627240[53] = 0.0;
   out_3258567857202627240[54] = 0.0;
   out_3258567857202627240[55] = 0.0;
   out_3258567857202627240[56] = 0.0;
   out_3258567857202627240[57] = 0.0;
   out_3258567857202627240[58] = 0.0;
   out_3258567857202627240[59] = 0.0;
   out_3258567857202627240[60] = 1.0;
   out_3258567857202627240[61] = 0.0;
   out_3258567857202627240[62] = 0.0;
   out_3258567857202627240[63] = 0.0;
   out_3258567857202627240[64] = 0.0;
   out_3258567857202627240[65] = 0.0;
   out_3258567857202627240[66] = 0.0;
   out_3258567857202627240[67] = 0.0;
   out_3258567857202627240[68] = 0.0;
   out_3258567857202627240[69] = 0.0;
   out_3258567857202627240[70] = 1.0;
   out_3258567857202627240[71] = 0.0;
   out_3258567857202627240[72] = 0.0;
   out_3258567857202627240[73] = 0.0;
   out_3258567857202627240[74] = 0.0;
   out_3258567857202627240[75] = 0.0;
   out_3258567857202627240[76] = 0.0;
   out_3258567857202627240[77] = 0.0;
   out_3258567857202627240[78] = 0.0;
   out_3258567857202627240[79] = 0.0;
   out_3258567857202627240[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4531498887012494036) {
   out_4531498887012494036[0] = state[0];
   out_4531498887012494036[1] = state[1];
   out_4531498887012494036[2] = state[2];
   out_4531498887012494036[3] = state[3];
   out_4531498887012494036[4] = state[4];
   out_4531498887012494036[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4531498887012494036[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4531498887012494036[7] = state[7];
   out_4531498887012494036[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2630502094007921965) {
   out_2630502094007921965[0] = 1;
   out_2630502094007921965[1] = 0;
   out_2630502094007921965[2] = 0;
   out_2630502094007921965[3] = 0;
   out_2630502094007921965[4] = 0;
   out_2630502094007921965[5] = 0;
   out_2630502094007921965[6] = 0;
   out_2630502094007921965[7] = 0;
   out_2630502094007921965[8] = 0;
   out_2630502094007921965[9] = 0;
   out_2630502094007921965[10] = 1;
   out_2630502094007921965[11] = 0;
   out_2630502094007921965[12] = 0;
   out_2630502094007921965[13] = 0;
   out_2630502094007921965[14] = 0;
   out_2630502094007921965[15] = 0;
   out_2630502094007921965[16] = 0;
   out_2630502094007921965[17] = 0;
   out_2630502094007921965[18] = 0;
   out_2630502094007921965[19] = 0;
   out_2630502094007921965[20] = 1;
   out_2630502094007921965[21] = 0;
   out_2630502094007921965[22] = 0;
   out_2630502094007921965[23] = 0;
   out_2630502094007921965[24] = 0;
   out_2630502094007921965[25] = 0;
   out_2630502094007921965[26] = 0;
   out_2630502094007921965[27] = 0;
   out_2630502094007921965[28] = 0;
   out_2630502094007921965[29] = 0;
   out_2630502094007921965[30] = 1;
   out_2630502094007921965[31] = 0;
   out_2630502094007921965[32] = 0;
   out_2630502094007921965[33] = 0;
   out_2630502094007921965[34] = 0;
   out_2630502094007921965[35] = 0;
   out_2630502094007921965[36] = 0;
   out_2630502094007921965[37] = 0;
   out_2630502094007921965[38] = 0;
   out_2630502094007921965[39] = 0;
   out_2630502094007921965[40] = 1;
   out_2630502094007921965[41] = 0;
   out_2630502094007921965[42] = 0;
   out_2630502094007921965[43] = 0;
   out_2630502094007921965[44] = 0;
   out_2630502094007921965[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2630502094007921965[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2630502094007921965[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2630502094007921965[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2630502094007921965[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2630502094007921965[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2630502094007921965[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2630502094007921965[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2630502094007921965[53] = -9.8000000000000007*dt;
   out_2630502094007921965[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2630502094007921965[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2630502094007921965[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2630502094007921965[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2630502094007921965[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2630502094007921965[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2630502094007921965[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2630502094007921965[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2630502094007921965[62] = 0;
   out_2630502094007921965[63] = 0;
   out_2630502094007921965[64] = 0;
   out_2630502094007921965[65] = 0;
   out_2630502094007921965[66] = 0;
   out_2630502094007921965[67] = 0;
   out_2630502094007921965[68] = 0;
   out_2630502094007921965[69] = 0;
   out_2630502094007921965[70] = 1;
   out_2630502094007921965[71] = 0;
   out_2630502094007921965[72] = 0;
   out_2630502094007921965[73] = 0;
   out_2630502094007921965[74] = 0;
   out_2630502094007921965[75] = 0;
   out_2630502094007921965[76] = 0;
   out_2630502094007921965[77] = 0;
   out_2630502094007921965[78] = 0;
   out_2630502094007921965[79] = 0;
   out_2630502094007921965[80] = 1;
}
void h_25(double *state, double *unused, double *out_7971611860832540066) {
   out_7971611860832540066[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5994987990594687139) {
   out_5994987990594687139[0] = 0;
   out_5994987990594687139[1] = 0;
   out_5994987990594687139[2] = 0;
   out_5994987990594687139[3] = 0;
   out_5994987990594687139[4] = 0;
   out_5994987990594687139[5] = 0;
   out_5994987990594687139[6] = 1;
   out_5994987990594687139[7] = 0;
   out_5994987990594687139[8] = 0;
}
void h_24(double *state, double *unused, double *out_3750464184291796320) {
   out_3750464184291796320[0] = state[4];
   out_3750464184291796320[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5797550625006678720) {
   out_5797550625006678720[0] = 0;
   out_5797550625006678720[1] = 0;
   out_5797550625006678720[2] = 0;
   out_5797550625006678720[3] = 0;
   out_5797550625006678720[4] = 1;
   out_5797550625006678720[5] = 0;
   out_5797550625006678720[6] = 0;
   out_5797550625006678720[7] = 0;
   out_5797550625006678720[8] = 0;
   out_5797550625006678720[9] = 0;
   out_5797550625006678720[10] = 0;
   out_5797550625006678720[11] = 0;
   out_5797550625006678720[12] = 0;
   out_5797550625006678720[13] = 0;
   out_5797550625006678720[14] = 1;
   out_5797550625006678720[15] = 0;
   out_5797550625006678720[16] = 0;
   out_5797550625006678720[17] = 0;
}
void h_30(double *state, double *unused, double *out_6837682684622989200) {
   out_6837682684622989200[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5865649043451447069) {
   out_5865649043451447069[0] = 0;
   out_5865649043451447069[1] = 0;
   out_5865649043451447069[2] = 0;
   out_5865649043451447069[3] = 0;
   out_5865649043451447069[4] = 1;
   out_5865649043451447069[5] = 0;
   out_5865649043451447069[6] = 0;
   out_5865649043451447069[7] = 0;
   out_5865649043451447069[8] = 0;
}
void h_26(double *state, double *unused, double *out_2144286487383851565) {
   out_2144286487383851565[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2253484671720630915) {
   out_2253484671720630915[0] = 0;
   out_2253484671720630915[1] = 0;
   out_2253484671720630915[2] = 0;
   out_2253484671720630915[3] = 0;
   out_2253484671720630915[4] = 0;
   out_2253484671720630915[5] = 0;
   out_2253484671720630915[6] = 0;
   out_2253484671720630915[7] = 1;
   out_2253484671720630915[8] = 0;
}
void h_27(double *state, double *unused, double *out_968849157039474287) {
   out_968849157039474287[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3690885731651022158) {
   out_3690885731651022158[0] = 0;
   out_3690885731651022158[1] = 0;
   out_3690885731651022158[2] = 0;
   out_3690885731651022158[3] = 1;
   out_3690885731651022158[4] = 0;
   out_3690885731651022158[5] = 0;
   out_3690885731651022158[6] = 0;
   out_3690885731651022158[7] = 0;
   out_3690885731651022158[8] = 0;
}
void h_29(double *state, double *unused, double *out_6010132851772032381) {
   out_6010132851772032381[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6375880387765839253) {
   out_6375880387765839253[0] = 0;
   out_6375880387765839253[1] = 1;
   out_6375880387765839253[2] = 0;
   out_6375880387765839253[3] = 0;
   out_6375880387765839253[4] = 0;
   out_6375880387765839253[5] = 0;
   out_6375880387765839253[6] = 0;
   out_6375880387765839253[7] = 0;
   out_6375880387765839253[8] = 0;
}
void h_28(double *state, double *unused, double *out_8889732234865235921) {
   out_8889732234865235921[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8339510659331165504) {
   out_8339510659331165504[0] = 1;
   out_8339510659331165504[1] = 0;
   out_8339510659331165504[2] = 0;
   out_8339510659331165504[3] = 0;
   out_8339510659331165504[4] = 0;
   out_8339510659331165504[5] = 0;
   out_8339510659331165504[6] = 0;
   out_8339510659331165504[7] = 0;
   out_8339510659331165504[8] = 0;
}
void h_31(double *state, double *unused, double *out_3730441146199301083) {
   out_3730441146199301083[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6025633952471647567) {
   out_6025633952471647567[0] = 0;
   out_6025633952471647567[1] = 0;
   out_6025633952471647567[2] = 0;
   out_6025633952471647567[3] = 0;
   out_6025633952471647567[4] = 0;
   out_6025633952471647567[5] = 0;
   out_6025633952471647567[6] = 0;
   out_6025633952471647567[7] = 0;
   out_6025633952471647567[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2816734334840272051) {
  err_fun(nom_x, delta_x, out_2816734334840272051);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3197403674388065693) {
  inv_err_fun(nom_x, true_x, out_3197403674388065693);
}
void car_H_mod_fun(double *state, double *out_3258567857202627240) {
  H_mod_fun(state, out_3258567857202627240);
}
void car_f_fun(double *state, double dt, double *out_4531498887012494036) {
  f_fun(state,  dt, out_4531498887012494036);
}
void car_F_fun(double *state, double dt, double *out_2630502094007921965) {
  F_fun(state,  dt, out_2630502094007921965);
}
void car_h_25(double *state, double *unused, double *out_7971611860832540066) {
  h_25(state, unused, out_7971611860832540066);
}
void car_H_25(double *state, double *unused, double *out_5994987990594687139) {
  H_25(state, unused, out_5994987990594687139);
}
void car_h_24(double *state, double *unused, double *out_3750464184291796320) {
  h_24(state, unused, out_3750464184291796320);
}
void car_H_24(double *state, double *unused, double *out_5797550625006678720) {
  H_24(state, unused, out_5797550625006678720);
}
void car_h_30(double *state, double *unused, double *out_6837682684622989200) {
  h_30(state, unused, out_6837682684622989200);
}
void car_H_30(double *state, double *unused, double *out_5865649043451447069) {
  H_30(state, unused, out_5865649043451447069);
}
void car_h_26(double *state, double *unused, double *out_2144286487383851565) {
  h_26(state, unused, out_2144286487383851565);
}
void car_H_26(double *state, double *unused, double *out_2253484671720630915) {
  H_26(state, unused, out_2253484671720630915);
}
void car_h_27(double *state, double *unused, double *out_968849157039474287) {
  h_27(state, unused, out_968849157039474287);
}
void car_H_27(double *state, double *unused, double *out_3690885731651022158) {
  H_27(state, unused, out_3690885731651022158);
}
void car_h_29(double *state, double *unused, double *out_6010132851772032381) {
  h_29(state, unused, out_6010132851772032381);
}
void car_H_29(double *state, double *unused, double *out_6375880387765839253) {
  H_29(state, unused, out_6375880387765839253);
}
void car_h_28(double *state, double *unused, double *out_8889732234865235921) {
  h_28(state, unused, out_8889732234865235921);
}
void car_H_28(double *state, double *unused, double *out_8339510659331165504) {
  H_28(state, unused, out_8339510659331165504);
}
void car_h_31(double *state, double *unused, double *out_3730441146199301083) {
  h_31(state, unused, out_3730441146199301083);
}
void car_H_31(double *state, double *unused, double *out_6025633952471647567) {
  H_31(state, unused, out_6025633952471647567);
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
