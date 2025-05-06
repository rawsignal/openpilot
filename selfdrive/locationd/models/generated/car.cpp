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
void err_fun(double *nom_x, double *delta_x, double *out_5230351242871143245) {
   out_5230351242871143245[0] = delta_x[0] + nom_x[0];
   out_5230351242871143245[1] = delta_x[1] + nom_x[1];
   out_5230351242871143245[2] = delta_x[2] + nom_x[2];
   out_5230351242871143245[3] = delta_x[3] + nom_x[3];
   out_5230351242871143245[4] = delta_x[4] + nom_x[4];
   out_5230351242871143245[5] = delta_x[5] + nom_x[5];
   out_5230351242871143245[6] = delta_x[6] + nom_x[6];
   out_5230351242871143245[7] = delta_x[7] + nom_x[7];
   out_5230351242871143245[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7994685084327859477) {
   out_7994685084327859477[0] = -nom_x[0] + true_x[0];
   out_7994685084327859477[1] = -nom_x[1] + true_x[1];
   out_7994685084327859477[2] = -nom_x[2] + true_x[2];
   out_7994685084327859477[3] = -nom_x[3] + true_x[3];
   out_7994685084327859477[4] = -nom_x[4] + true_x[4];
   out_7994685084327859477[5] = -nom_x[5] + true_x[5];
   out_7994685084327859477[6] = -nom_x[6] + true_x[6];
   out_7994685084327859477[7] = -nom_x[7] + true_x[7];
   out_7994685084327859477[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2156622796639092135) {
   out_2156622796639092135[0] = 1.0;
   out_2156622796639092135[1] = 0.0;
   out_2156622796639092135[2] = 0.0;
   out_2156622796639092135[3] = 0.0;
   out_2156622796639092135[4] = 0.0;
   out_2156622796639092135[5] = 0.0;
   out_2156622796639092135[6] = 0.0;
   out_2156622796639092135[7] = 0.0;
   out_2156622796639092135[8] = 0.0;
   out_2156622796639092135[9] = 0.0;
   out_2156622796639092135[10] = 1.0;
   out_2156622796639092135[11] = 0.0;
   out_2156622796639092135[12] = 0.0;
   out_2156622796639092135[13] = 0.0;
   out_2156622796639092135[14] = 0.0;
   out_2156622796639092135[15] = 0.0;
   out_2156622796639092135[16] = 0.0;
   out_2156622796639092135[17] = 0.0;
   out_2156622796639092135[18] = 0.0;
   out_2156622796639092135[19] = 0.0;
   out_2156622796639092135[20] = 1.0;
   out_2156622796639092135[21] = 0.0;
   out_2156622796639092135[22] = 0.0;
   out_2156622796639092135[23] = 0.0;
   out_2156622796639092135[24] = 0.0;
   out_2156622796639092135[25] = 0.0;
   out_2156622796639092135[26] = 0.0;
   out_2156622796639092135[27] = 0.0;
   out_2156622796639092135[28] = 0.0;
   out_2156622796639092135[29] = 0.0;
   out_2156622796639092135[30] = 1.0;
   out_2156622796639092135[31] = 0.0;
   out_2156622796639092135[32] = 0.0;
   out_2156622796639092135[33] = 0.0;
   out_2156622796639092135[34] = 0.0;
   out_2156622796639092135[35] = 0.0;
   out_2156622796639092135[36] = 0.0;
   out_2156622796639092135[37] = 0.0;
   out_2156622796639092135[38] = 0.0;
   out_2156622796639092135[39] = 0.0;
   out_2156622796639092135[40] = 1.0;
   out_2156622796639092135[41] = 0.0;
   out_2156622796639092135[42] = 0.0;
   out_2156622796639092135[43] = 0.0;
   out_2156622796639092135[44] = 0.0;
   out_2156622796639092135[45] = 0.0;
   out_2156622796639092135[46] = 0.0;
   out_2156622796639092135[47] = 0.0;
   out_2156622796639092135[48] = 0.0;
   out_2156622796639092135[49] = 0.0;
   out_2156622796639092135[50] = 1.0;
   out_2156622796639092135[51] = 0.0;
   out_2156622796639092135[52] = 0.0;
   out_2156622796639092135[53] = 0.0;
   out_2156622796639092135[54] = 0.0;
   out_2156622796639092135[55] = 0.0;
   out_2156622796639092135[56] = 0.0;
   out_2156622796639092135[57] = 0.0;
   out_2156622796639092135[58] = 0.0;
   out_2156622796639092135[59] = 0.0;
   out_2156622796639092135[60] = 1.0;
   out_2156622796639092135[61] = 0.0;
   out_2156622796639092135[62] = 0.0;
   out_2156622796639092135[63] = 0.0;
   out_2156622796639092135[64] = 0.0;
   out_2156622796639092135[65] = 0.0;
   out_2156622796639092135[66] = 0.0;
   out_2156622796639092135[67] = 0.0;
   out_2156622796639092135[68] = 0.0;
   out_2156622796639092135[69] = 0.0;
   out_2156622796639092135[70] = 1.0;
   out_2156622796639092135[71] = 0.0;
   out_2156622796639092135[72] = 0.0;
   out_2156622796639092135[73] = 0.0;
   out_2156622796639092135[74] = 0.0;
   out_2156622796639092135[75] = 0.0;
   out_2156622796639092135[76] = 0.0;
   out_2156622796639092135[77] = 0.0;
   out_2156622796639092135[78] = 0.0;
   out_2156622796639092135[79] = 0.0;
   out_2156622796639092135[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4839858682589434933) {
   out_4839858682589434933[0] = state[0];
   out_4839858682589434933[1] = state[1];
   out_4839858682589434933[2] = state[2];
   out_4839858682589434933[3] = state[3];
   out_4839858682589434933[4] = state[4];
   out_4839858682589434933[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4839858682589434933[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4839858682589434933[7] = state[7];
   out_4839858682589434933[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6752875690228243553) {
   out_6752875690228243553[0] = 1;
   out_6752875690228243553[1] = 0;
   out_6752875690228243553[2] = 0;
   out_6752875690228243553[3] = 0;
   out_6752875690228243553[4] = 0;
   out_6752875690228243553[5] = 0;
   out_6752875690228243553[6] = 0;
   out_6752875690228243553[7] = 0;
   out_6752875690228243553[8] = 0;
   out_6752875690228243553[9] = 0;
   out_6752875690228243553[10] = 1;
   out_6752875690228243553[11] = 0;
   out_6752875690228243553[12] = 0;
   out_6752875690228243553[13] = 0;
   out_6752875690228243553[14] = 0;
   out_6752875690228243553[15] = 0;
   out_6752875690228243553[16] = 0;
   out_6752875690228243553[17] = 0;
   out_6752875690228243553[18] = 0;
   out_6752875690228243553[19] = 0;
   out_6752875690228243553[20] = 1;
   out_6752875690228243553[21] = 0;
   out_6752875690228243553[22] = 0;
   out_6752875690228243553[23] = 0;
   out_6752875690228243553[24] = 0;
   out_6752875690228243553[25] = 0;
   out_6752875690228243553[26] = 0;
   out_6752875690228243553[27] = 0;
   out_6752875690228243553[28] = 0;
   out_6752875690228243553[29] = 0;
   out_6752875690228243553[30] = 1;
   out_6752875690228243553[31] = 0;
   out_6752875690228243553[32] = 0;
   out_6752875690228243553[33] = 0;
   out_6752875690228243553[34] = 0;
   out_6752875690228243553[35] = 0;
   out_6752875690228243553[36] = 0;
   out_6752875690228243553[37] = 0;
   out_6752875690228243553[38] = 0;
   out_6752875690228243553[39] = 0;
   out_6752875690228243553[40] = 1;
   out_6752875690228243553[41] = 0;
   out_6752875690228243553[42] = 0;
   out_6752875690228243553[43] = 0;
   out_6752875690228243553[44] = 0;
   out_6752875690228243553[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6752875690228243553[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6752875690228243553[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6752875690228243553[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6752875690228243553[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6752875690228243553[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6752875690228243553[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6752875690228243553[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6752875690228243553[53] = -9.8000000000000007*dt;
   out_6752875690228243553[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6752875690228243553[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6752875690228243553[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6752875690228243553[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6752875690228243553[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6752875690228243553[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6752875690228243553[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6752875690228243553[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6752875690228243553[62] = 0;
   out_6752875690228243553[63] = 0;
   out_6752875690228243553[64] = 0;
   out_6752875690228243553[65] = 0;
   out_6752875690228243553[66] = 0;
   out_6752875690228243553[67] = 0;
   out_6752875690228243553[68] = 0;
   out_6752875690228243553[69] = 0;
   out_6752875690228243553[70] = 1;
   out_6752875690228243553[71] = 0;
   out_6752875690228243553[72] = 0;
   out_6752875690228243553[73] = 0;
   out_6752875690228243553[74] = 0;
   out_6752875690228243553[75] = 0;
   out_6752875690228243553[76] = 0;
   out_6752875690228243553[77] = 0;
   out_6752875690228243553[78] = 0;
   out_6752875690228243553[79] = 0;
   out_6752875690228243553[80] = 1;
}
void h_25(double *state, double *unused, double *out_3012118820978252315) {
   out_3012118820978252315[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5764715236334767670) {
   out_5764715236334767670[0] = 0;
   out_5764715236334767670[1] = 0;
   out_5764715236334767670[2] = 0;
   out_5764715236334767670[3] = 0;
   out_5764715236334767670[4] = 0;
   out_5764715236334767670[5] = 0;
   out_5764715236334767670[6] = 1;
   out_5764715236334767670[7] = 0;
   out_5764715236334767670[8] = 0;
}
void h_24(double *state, double *unused, double *out_4008937272930415502) {
   out_4008937272930415502[0] = state[4];
   out_4008937272930415502[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3592065637329268104) {
   out_3592065637329268104[0] = 0;
   out_3592065637329268104[1] = 0;
   out_3592065637329268104[2] = 0;
   out_3592065637329268104[3] = 0;
   out_3592065637329268104[4] = 1;
   out_3592065637329268104[5] = 0;
   out_3592065637329268104[6] = 0;
   out_3592065637329268104[7] = 0;
   out_3592065637329268104[8] = 0;
   out_3592065637329268104[9] = 0;
   out_3592065637329268104[10] = 0;
   out_3592065637329268104[11] = 0;
   out_3592065637329268104[12] = 0;
   out_3592065637329268104[13] = 0;
   out_3592065637329268104[14] = 1;
   out_3592065637329268104[15] = 0;
   out_3592065637329268104[16] = 0;
   out_3592065637329268104[17] = 0;
}
void h_30(double *state, double *unused, double *out_4189849330954229981) {
   out_4189849330954229981[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1237018906207159472) {
   out_1237018906207159472[0] = 0;
   out_1237018906207159472[1] = 0;
   out_1237018906207159472[2] = 0;
   out_1237018906207159472[3] = 0;
   out_1237018906207159472[4] = 1;
   out_1237018906207159472[5] = 0;
   out_1237018906207159472[6] = 0;
   out_1237018906207159472[7] = 0;
   out_1237018906207159472[8] = 0;
}
void h_26(double *state, double *unused, double *out_7594326007187440774) {
   out_7594326007187440774[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2023211917460711446) {
   out_2023211917460711446[0] = 0;
   out_2023211917460711446[1] = 0;
   out_2023211917460711446[2] = 0;
   out_2023211917460711446[3] = 0;
   out_2023211917460711446[4] = 0;
   out_2023211917460711446[5] = 0;
   out_2023211917460711446[6] = 0;
   out_2023211917460711446[7] = 1;
   out_2023211917460711446[8] = 0;
}
void h_27(double *state, double *unused, double *out_7069448714047433521) {
   out_7069448714047433521[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3460612977391102689) {
   out_3460612977391102689[0] = 0;
   out_3460612977391102689[1] = 0;
   out_3460612977391102689[2] = 0;
   out_3460612977391102689[3] = 1;
   out_3460612977391102689[4] = 0;
   out_3460612977391102689[5] = 0;
   out_3460612977391102689[6] = 0;
   out_3460612977391102689[7] = 0;
   out_3460612977391102689[8] = 0;
}
void h_29(double *state, double *unused, double *out_8498909828761583679) {
   out_8498909828761583679[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1747250250521551656) {
   out_1747250250521551656[0] = 0;
   out_1747250250521551656[1] = 1;
   out_1747250250521551656[2] = 0;
   out_1747250250521551656[3] = 0;
   out_1747250250521551656[4] = 0;
   out_1747250250521551656[5] = 0;
   out_1747250250521551656[6] = 0;
   out_1747250250521551656[7] = 0;
   out_1747250250521551656[8] = 0;
}
void h_28(double *state, double *unused, double *out_5619310445668380139) {
   out_5619310445668380139[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3710880522086877907) {
   out_3710880522086877907[0] = 1;
   out_3710880522086877907[1] = 0;
   out_3710880522086877907[2] = 0;
   out_3710880522086877907[3] = 0;
   out_3710880522086877907[4] = 0;
   out_3710880522086877907[5] = 0;
   out_3710880522086877907[6] = 0;
   out_3710880522086877907[7] = 0;
   out_3710880522086877907[8] = 0;
}
void h_31(double *state, double *unused, double *out_7972818683428907302) {
   out_7972818683428907302[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1397003815227359970) {
   out_1397003815227359970[0] = 0;
   out_1397003815227359970[1] = 0;
   out_1397003815227359970[2] = 0;
   out_1397003815227359970[3] = 0;
   out_1397003815227359970[4] = 0;
   out_1397003815227359970[5] = 0;
   out_1397003815227359970[6] = 0;
   out_1397003815227359970[7] = 0;
   out_1397003815227359970[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5230351242871143245) {
  err_fun(nom_x, delta_x, out_5230351242871143245);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7994685084327859477) {
  inv_err_fun(nom_x, true_x, out_7994685084327859477);
}
void car_H_mod_fun(double *state, double *out_2156622796639092135) {
  H_mod_fun(state, out_2156622796639092135);
}
void car_f_fun(double *state, double dt, double *out_4839858682589434933) {
  f_fun(state,  dt, out_4839858682589434933);
}
void car_F_fun(double *state, double dt, double *out_6752875690228243553) {
  F_fun(state,  dt, out_6752875690228243553);
}
void car_h_25(double *state, double *unused, double *out_3012118820978252315) {
  h_25(state, unused, out_3012118820978252315);
}
void car_H_25(double *state, double *unused, double *out_5764715236334767670) {
  H_25(state, unused, out_5764715236334767670);
}
void car_h_24(double *state, double *unused, double *out_4008937272930415502) {
  h_24(state, unused, out_4008937272930415502);
}
void car_H_24(double *state, double *unused, double *out_3592065637329268104) {
  H_24(state, unused, out_3592065637329268104);
}
void car_h_30(double *state, double *unused, double *out_4189849330954229981) {
  h_30(state, unused, out_4189849330954229981);
}
void car_H_30(double *state, double *unused, double *out_1237018906207159472) {
  H_30(state, unused, out_1237018906207159472);
}
void car_h_26(double *state, double *unused, double *out_7594326007187440774) {
  h_26(state, unused, out_7594326007187440774);
}
void car_H_26(double *state, double *unused, double *out_2023211917460711446) {
  H_26(state, unused, out_2023211917460711446);
}
void car_h_27(double *state, double *unused, double *out_7069448714047433521) {
  h_27(state, unused, out_7069448714047433521);
}
void car_H_27(double *state, double *unused, double *out_3460612977391102689) {
  H_27(state, unused, out_3460612977391102689);
}
void car_h_29(double *state, double *unused, double *out_8498909828761583679) {
  h_29(state, unused, out_8498909828761583679);
}
void car_H_29(double *state, double *unused, double *out_1747250250521551656) {
  H_29(state, unused, out_1747250250521551656);
}
void car_h_28(double *state, double *unused, double *out_5619310445668380139) {
  h_28(state, unused, out_5619310445668380139);
}
void car_H_28(double *state, double *unused, double *out_3710880522086877907) {
  H_28(state, unused, out_3710880522086877907);
}
void car_h_31(double *state, double *unused, double *out_7972818683428907302) {
  h_31(state, unused, out_7972818683428907302);
}
void car_H_31(double *state, double *unused, double *out_1397003815227359970) {
  H_31(state, unused, out_1397003815227359970);
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
