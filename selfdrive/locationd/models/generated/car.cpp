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
void err_fun(double *nom_x, double *delta_x, double *out_1752089738275979241) {
   out_1752089738275979241[0] = delta_x[0] + nom_x[0];
   out_1752089738275979241[1] = delta_x[1] + nom_x[1];
   out_1752089738275979241[2] = delta_x[2] + nom_x[2];
   out_1752089738275979241[3] = delta_x[3] + nom_x[3];
   out_1752089738275979241[4] = delta_x[4] + nom_x[4];
   out_1752089738275979241[5] = delta_x[5] + nom_x[5];
   out_1752089738275979241[6] = delta_x[6] + nom_x[6];
   out_1752089738275979241[7] = delta_x[7] + nom_x[7];
   out_1752089738275979241[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1045694545913765962) {
   out_1045694545913765962[0] = -nom_x[0] + true_x[0];
   out_1045694545913765962[1] = -nom_x[1] + true_x[1];
   out_1045694545913765962[2] = -nom_x[2] + true_x[2];
   out_1045694545913765962[3] = -nom_x[3] + true_x[3];
   out_1045694545913765962[4] = -nom_x[4] + true_x[4];
   out_1045694545913765962[5] = -nom_x[5] + true_x[5];
   out_1045694545913765962[6] = -nom_x[6] + true_x[6];
   out_1045694545913765962[7] = -nom_x[7] + true_x[7];
   out_1045694545913765962[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3061534902965190884) {
   out_3061534902965190884[0] = 1.0;
   out_3061534902965190884[1] = 0.0;
   out_3061534902965190884[2] = 0.0;
   out_3061534902965190884[3] = 0.0;
   out_3061534902965190884[4] = 0.0;
   out_3061534902965190884[5] = 0.0;
   out_3061534902965190884[6] = 0.0;
   out_3061534902965190884[7] = 0.0;
   out_3061534902965190884[8] = 0.0;
   out_3061534902965190884[9] = 0.0;
   out_3061534902965190884[10] = 1.0;
   out_3061534902965190884[11] = 0.0;
   out_3061534902965190884[12] = 0.0;
   out_3061534902965190884[13] = 0.0;
   out_3061534902965190884[14] = 0.0;
   out_3061534902965190884[15] = 0.0;
   out_3061534902965190884[16] = 0.0;
   out_3061534902965190884[17] = 0.0;
   out_3061534902965190884[18] = 0.0;
   out_3061534902965190884[19] = 0.0;
   out_3061534902965190884[20] = 1.0;
   out_3061534902965190884[21] = 0.0;
   out_3061534902965190884[22] = 0.0;
   out_3061534902965190884[23] = 0.0;
   out_3061534902965190884[24] = 0.0;
   out_3061534902965190884[25] = 0.0;
   out_3061534902965190884[26] = 0.0;
   out_3061534902965190884[27] = 0.0;
   out_3061534902965190884[28] = 0.0;
   out_3061534902965190884[29] = 0.0;
   out_3061534902965190884[30] = 1.0;
   out_3061534902965190884[31] = 0.0;
   out_3061534902965190884[32] = 0.0;
   out_3061534902965190884[33] = 0.0;
   out_3061534902965190884[34] = 0.0;
   out_3061534902965190884[35] = 0.0;
   out_3061534902965190884[36] = 0.0;
   out_3061534902965190884[37] = 0.0;
   out_3061534902965190884[38] = 0.0;
   out_3061534902965190884[39] = 0.0;
   out_3061534902965190884[40] = 1.0;
   out_3061534902965190884[41] = 0.0;
   out_3061534902965190884[42] = 0.0;
   out_3061534902965190884[43] = 0.0;
   out_3061534902965190884[44] = 0.0;
   out_3061534902965190884[45] = 0.0;
   out_3061534902965190884[46] = 0.0;
   out_3061534902965190884[47] = 0.0;
   out_3061534902965190884[48] = 0.0;
   out_3061534902965190884[49] = 0.0;
   out_3061534902965190884[50] = 1.0;
   out_3061534902965190884[51] = 0.0;
   out_3061534902965190884[52] = 0.0;
   out_3061534902965190884[53] = 0.0;
   out_3061534902965190884[54] = 0.0;
   out_3061534902965190884[55] = 0.0;
   out_3061534902965190884[56] = 0.0;
   out_3061534902965190884[57] = 0.0;
   out_3061534902965190884[58] = 0.0;
   out_3061534902965190884[59] = 0.0;
   out_3061534902965190884[60] = 1.0;
   out_3061534902965190884[61] = 0.0;
   out_3061534902965190884[62] = 0.0;
   out_3061534902965190884[63] = 0.0;
   out_3061534902965190884[64] = 0.0;
   out_3061534902965190884[65] = 0.0;
   out_3061534902965190884[66] = 0.0;
   out_3061534902965190884[67] = 0.0;
   out_3061534902965190884[68] = 0.0;
   out_3061534902965190884[69] = 0.0;
   out_3061534902965190884[70] = 1.0;
   out_3061534902965190884[71] = 0.0;
   out_3061534902965190884[72] = 0.0;
   out_3061534902965190884[73] = 0.0;
   out_3061534902965190884[74] = 0.0;
   out_3061534902965190884[75] = 0.0;
   out_3061534902965190884[76] = 0.0;
   out_3061534902965190884[77] = 0.0;
   out_3061534902965190884[78] = 0.0;
   out_3061534902965190884[79] = 0.0;
   out_3061534902965190884[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5345978839819676021) {
   out_5345978839819676021[0] = state[0];
   out_5345978839819676021[1] = state[1];
   out_5345978839819676021[2] = state[2];
   out_5345978839819676021[3] = state[3];
   out_5345978839819676021[4] = state[4];
   out_5345978839819676021[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5345978839819676021[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5345978839819676021[7] = state[7];
   out_5345978839819676021[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4711065191693970871) {
   out_4711065191693970871[0] = 1;
   out_4711065191693970871[1] = 0;
   out_4711065191693970871[2] = 0;
   out_4711065191693970871[3] = 0;
   out_4711065191693970871[4] = 0;
   out_4711065191693970871[5] = 0;
   out_4711065191693970871[6] = 0;
   out_4711065191693970871[7] = 0;
   out_4711065191693970871[8] = 0;
   out_4711065191693970871[9] = 0;
   out_4711065191693970871[10] = 1;
   out_4711065191693970871[11] = 0;
   out_4711065191693970871[12] = 0;
   out_4711065191693970871[13] = 0;
   out_4711065191693970871[14] = 0;
   out_4711065191693970871[15] = 0;
   out_4711065191693970871[16] = 0;
   out_4711065191693970871[17] = 0;
   out_4711065191693970871[18] = 0;
   out_4711065191693970871[19] = 0;
   out_4711065191693970871[20] = 1;
   out_4711065191693970871[21] = 0;
   out_4711065191693970871[22] = 0;
   out_4711065191693970871[23] = 0;
   out_4711065191693970871[24] = 0;
   out_4711065191693970871[25] = 0;
   out_4711065191693970871[26] = 0;
   out_4711065191693970871[27] = 0;
   out_4711065191693970871[28] = 0;
   out_4711065191693970871[29] = 0;
   out_4711065191693970871[30] = 1;
   out_4711065191693970871[31] = 0;
   out_4711065191693970871[32] = 0;
   out_4711065191693970871[33] = 0;
   out_4711065191693970871[34] = 0;
   out_4711065191693970871[35] = 0;
   out_4711065191693970871[36] = 0;
   out_4711065191693970871[37] = 0;
   out_4711065191693970871[38] = 0;
   out_4711065191693970871[39] = 0;
   out_4711065191693970871[40] = 1;
   out_4711065191693970871[41] = 0;
   out_4711065191693970871[42] = 0;
   out_4711065191693970871[43] = 0;
   out_4711065191693970871[44] = 0;
   out_4711065191693970871[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4711065191693970871[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4711065191693970871[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4711065191693970871[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4711065191693970871[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4711065191693970871[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4711065191693970871[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4711065191693970871[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4711065191693970871[53] = -9.8000000000000007*dt;
   out_4711065191693970871[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4711065191693970871[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4711065191693970871[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4711065191693970871[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4711065191693970871[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4711065191693970871[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4711065191693970871[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4711065191693970871[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4711065191693970871[62] = 0;
   out_4711065191693970871[63] = 0;
   out_4711065191693970871[64] = 0;
   out_4711065191693970871[65] = 0;
   out_4711065191693970871[66] = 0;
   out_4711065191693970871[67] = 0;
   out_4711065191693970871[68] = 0;
   out_4711065191693970871[69] = 0;
   out_4711065191693970871[70] = 1;
   out_4711065191693970871[71] = 0;
   out_4711065191693970871[72] = 0;
   out_4711065191693970871[73] = 0;
   out_4711065191693970871[74] = 0;
   out_4711065191693970871[75] = 0;
   out_4711065191693970871[76] = 0;
   out_4711065191693970871[77] = 0;
   out_4711065191693970871[78] = 0;
   out_4711065191693970871[79] = 0;
   out_4711065191693970871[80] = 1;
}
void h_25(double *state, double *unused, double *out_3031350575474952037) {
   out_3031350575474952037[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8276550673710960652) {
   out_8276550673710960652[0] = 0;
   out_8276550673710960652[1] = 0;
   out_8276550673710960652[2] = 0;
   out_8276550673710960652[3] = 0;
   out_8276550673710960652[4] = 0;
   out_8276550673710960652[5] = 0;
   out_8276550673710960652[6] = 1;
   out_8276550673710960652[7] = 0;
   out_8276550673710960652[8] = 0;
}
void h_24(double *state, double *unused, double *out_1555058544644953461) {
   out_1555058544644953461[0] = state[4];
   out_1555058544644953461[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4718625074908637516) {
   out_4718625074908637516[0] = 0;
   out_4718625074908637516[1] = 0;
   out_4718625074908637516[2] = 0;
   out_4718625074908637516[3] = 0;
   out_4718625074908637516[4] = 1;
   out_4718625074908637516[5] = 0;
   out_4718625074908637516[6] = 0;
   out_4718625074908637516[7] = 0;
   out_4718625074908637516[8] = 0;
   out_4718625074908637516[9] = 0;
   out_4718625074908637516[10] = 0;
   out_4718625074908637516[11] = 0;
   out_4718625074908637516[12] = 0;
   out_4718625074908637516[13] = 0;
   out_4718625074908637516[14] = 1;
   out_4718625074908637516[15] = 0;
   out_4718625074908637516[16] = 0;
   out_4718625074908637516[17] = 0;
}
void h_30(double *state, double *unused, double *out_8644558271884248643) {
   out_8644558271884248643[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8405889620854200722) {
   out_8405889620854200722[0] = 0;
   out_8405889620854200722[1] = 0;
   out_8405889620854200722[2] = 0;
   out_8405889620854200722[3] = 0;
   out_8405889620854200722[4] = 1;
   out_8405889620854200722[5] = 0;
   out_8405889620854200722[6] = 0;
   out_8405889620854200722[7] = 0;
   out_8405889620854200722[8] = 0;
}
void h_26(double *state, double *unused, double *out_8634442000749161236) {
   out_8634442000749161236[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6428690081124534740) {
   out_6428690081124534740[0] = 0;
   out_6428690081124534740[1] = 0;
   out_6428690081124534740[2] = 0;
   out_6428690081124534740[3] = 0;
   out_6428690081124534740[4] = 0;
   out_6428690081124534740[5] = 0;
   out_6428690081124534740[6] = 0;
   out_6428690081124534740[7] = 1;
   out_6428690081124534740[8] = 0;
}
void h_27(double *state, double *unused, double *out_5989192836965524405) {
   out_5989192836965524405[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7866091141054925983) {
   out_7866091141054925983[0] = 0;
   out_7866091141054925983[1] = 0;
   out_7866091141054925983[2] = 0;
   out_7866091141054925983[3] = 1;
   out_7866091141054925983[4] = 0;
   out_7866091141054925983[5] = 0;
   out_7866091141054925983[6] = 0;
   out_7866091141054925983[7] = 0;
   out_7866091141054925983[8] = 0;
}
void h_29(double *state, double *unused, double *out_5832006168614395260) {
   out_5832006168614395260[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7895658276539808538) {
   out_7895658276539808538[0] = 0;
   out_7895658276539808538[1] = 1;
   out_7895658276539808538[2] = 0;
   out_7895658276539808538[3] = 0;
   out_7895658276539808538[4] = 0;
   out_7895658276539808538[5] = 0;
   out_7895658276539808538[6] = 0;
   out_7895658276539808538[7] = 0;
   out_7895658276539808538[8] = 0;
}
void h_28(double *state, double *unused, double *out_1445950597463176408) {
   out_1445950597463176408[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5932028004974482287) {
   out_5932028004974482287[0] = 1;
   out_5932028004974482287[1] = 0;
   out_5932028004974482287[2] = 0;
   out_5932028004974482287[3] = 0;
   out_5932028004974482287[4] = 0;
   out_5932028004974482287[5] = 0;
   out_5932028004974482287[6] = 0;
   out_5932028004974482287[7] = 0;
   out_5932028004974482287[8] = 0;
}
void h_31(double *state, double *unused, double *out_4731914681345720404) {
   out_4731914681345720404[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8245904711834000224) {
   out_8245904711834000224[0] = 0;
   out_8245904711834000224[1] = 0;
   out_8245904711834000224[2] = 0;
   out_8245904711834000224[3] = 0;
   out_8245904711834000224[4] = 0;
   out_8245904711834000224[5] = 0;
   out_8245904711834000224[6] = 0;
   out_8245904711834000224[7] = 0;
   out_8245904711834000224[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1752089738275979241) {
  err_fun(nom_x, delta_x, out_1752089738275979241);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1045694545913765962) {
  inv_err_fun(nom_x, true_x, out_1045694545913765962);
}
void car_H_mod_fun(double *state, double *out_3061534902965190884) {
  H_mod_fun(state, out_3061534902965190884);
}
void car_f_fun(double *state, double dt, double *out_5345978839819676021) {
  f_fun(state,  dt, out_5345978839819676021);
}
void car_F_fun(double *state, double dt, double *out_4711065191693970871) {
  F_fun(state,  dt, out_4711065191693970871);
}
void car_h_25(double *state, double *unused, double *out_3031350575474952037) {
  h_25(state, unused, out_3031350575474952037);
}
void car_H_25(double *state, double *unused, double *out_8276550673710960652) {
  H_25(state, unused, out_8276550673710960652);
}
void car_h_24(double *state, double *unused, double *out_1555058544644953461) {
  h_24(state, unused, out_1555058544644953461);
}
void car_H_24(double *state, double *unused, double *out_4718625074908637516) {
  H_24(state, unused, out_4718625074908637516);
}
void car_h_30(double *state, double *unused, double *out_8644558271884248643) {
  h_30(state, unused, out_8644558271884248643);
}
void car_H_30(double *state, double *unused, double *out_8405889620854200722) {
  H_30(state, unused, out_8405889620854200722);
}
void car_h_26(double *state, double *unused, double *out_8634442000749161236) {
  h_26(state, unused, out_8634442000749161236);
}
void car_H_26(double *state, double *unused, double *out_6428690081124534740) {
  H_26(state, unused, out_6428690081124534740);
}
void car_h_27(double *state, double *unused, double *out_5989192836965524405) {
  h_27(state, unused, out_5989192836965524405);
}
void car_H_27(double *state, double *unused, double *out_7866091141054925983) {
  H_27(state, unused, out_7866091141054925983);
}
void car_h_29(double *state, double *unused, double *out_5832006168614395260) {
  h_29(state, unused, out_5832006168614395260);
}
void car_H_29(double *state, double *unused, double *out_7895658276539808538) {
  H_29(state, unused, out_7895658276539808538);
}
void car_h_28(double *state, double *unused, double *out_1445950597463176408) {
  h_28(state, unused, out_1445950597463176408);
}
void car_H_28(double *state, double *unused, double *out_5932028004974482287) {
  H_28(state, unused, out_5932028004974482287);
}
void car_h_31(double *state, double *unused, double *out_4731914681345720404) {
  h_31(state, unused, out_4731914681345720404);
}
void car_H_31(double *state, double *unused, double *out_8245904711834000224) {
  H_31(state, unused, out_8245904711834000224);
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
