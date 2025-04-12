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
void err_fun(double *nom_x, double *delta_x, double *out_2788712361531455797) {
   out_2788712361531455797[0] = delta_x[0] + nom_x[0];
   out_2788712361531455797[1] = delta_x[1] + nom_x[1];
   out_2788712361531455797[2] = delta_x[2] + nom_x[2];
   out_2788712361531455797[3] = delta_x[3] + nom_x[3];
   out_2788712361531455797[4] = delta_x[4] + nom_x[4];
   out_2788712361531455797[5] = delta_x[5] + nom_x[5];
   out_2788712361531455797[6] = delta_x[6] + nom_x[6];
   out_2788712361531455797[7] = delta_x[7] + nom_x[7];
   out_2788712361531455797[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5243626477297637109) {
   out_5243626477297637109[0] = -nom_x[0] + true_x[0];
   out_5243626477297637109[1] = -nom_x[1] + true_x[1];
   out_5243626477297637109[2] = -nom_x[2] + true_x[2];
   out_5243626477297637109[3] = -nom_x[3] + true_x[3];
   out_5243626477297637109[4] = -nom_x[4] + true_x[4];
   out_5243626477297637109[5] = -nom_x[5] + true_x[5];
   out_5243626477297637109[6] = -nom_x[6] + true_x[6];
   out_5243626477297637109[7] = -nom_x[7] + true_x[7];
   out_5243626477297637109[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7309627436752179880) {
   out_7309627436752179880[0] = 1.0;
   out_7309627436752179880[1] = 0.0;
   out_7309627436752179880[2] = 0.0;
   out_7309627436752179880[3] = 0.0;
   out_7309627436752179880[4] = 0.0;
   out_7309627436752179880[5] = 0.0;
   out_7309627436752179880[6] = 0.0;
   out_7309627436752179880[7] = 0.0;
   out_7309627436752179880[8] = 0.0;
   out_7309627436752179880[9] = 0.0;
   out_7309627436752179880[10] = 1.0;
   out_7309627436752179880[11] = 0.0;
   out_7309627436752179880[12] = 0.0;
   out_7309627436752179880[13] = 0.0;
   out_7309627436752179880[14] = 0.0;
   out_7309627436752179880[15] = 0.0;
   out_7309627436752179880[16] = 0.0;
   out_7309627436752179880[17] = 0.0;
   out_7309627436752179880[18] = 0.0;
   out_7309627436752179880[19] = 0.0;
   out_7309627436752179880[20] = 1.0;
   out_7309627436752179880[21] = 0.0;
   out_7309627436752179880[22] = 0.0;
   out_7309627436752179880[23] = 0.0;
   out_7309627436752179880[24] = 0.0;
   out_7309627436752179880[25] = 0.0;
   out_7309627436752179880[26] = 0.0;
   out_7309627436752179880[27] = 0.0;
   out_7309627436752179880[28] = 0.0;
   out_7309627436752179880[29] = 0.0;
   out_7309627436752179880[30] = 1.0;
   out_7309627436752179880[31] = 0.0;
   out_7309627436752179880[32] = 0.0;
   out_7309627436752179880[33] = 0.0;
   out_7309627436752179880[34] = 0.0;
   out_7309627436752179880[35] = 0.0;
   out_7309627436752179880[36] = 0.0;
   out_7309627436752179880[37] = 0.0;
   out_7309627436752179880[38] = 0.0;
   out_7309627436752179880[39] = 0.0;
   out_7309627436752179880[40] = 1.0;
   out_7309627436752179880[41] = 0.0;
   out_7309627436752179880[42] = 0.0;
   out_7309627436752179880[43] = 0.0;
   out_7309627436752179880[44] = 0.0;
   out_7309627436752179880[45] = 0.0;
   out_7309627436752179880[46] = 0.0;
   out_7309627436752179880[47] = 0.0;
   out_7309627436752179880[48] = 0.0;
   out_7309627436752179880[49] = 0.0;
   out_7309627436752179880[50] = 1.0;
   out_7309627436752179880[51] = 0.0;
   out_7309627436752179880[52] = 0.0;
   out_7309627436752179880[53] = 0.0;
   out_7309627436752179880[54] = 0.0;
   out_7309627436752179880[55] = 0.0;
   out_7309627436752179880[56] = 0.0;
   out_7309627436752179880[57] = 0.0;
   out_7309627436752179880[58] = 0.0;
   out_7309627436752179880[59] = 0.0;
   out_7309627436752179880[60] = 1.0;
   out_7309627436752179880[61] = 0.0;
   out_7309627436752179880[62] = 0.0;
   out_7309627436752179880[63] = 0.0;
   out_7309627436752179880[64] = 0.0;
   out_7309627436752179880[65] = 0.0;
   out_7309627436752179880[66] = 0.0;
   out_7309627436752179880[67] = 0.0;
   out_7309627436752179880[68] = 0.0;
   out_7309627436752179880[69] = 0.0;
   out_7309627436752179880[70] = 1.0;
   out_7309627436752179880[71] = 0.0;
   out_7309627436752179880[72] = 0.0;
   out_7309627436752179880[73] = 0.0;
   out_7309627436752179880[74] = 0.0;
   out_7309627436752179880[75] = 0.0;
   out_7309627436752179880[76] = 0.0;
   out_7309627436752179880[77] = 0.0;
   out_7309627436752179880[78] = 0.0;
   out_7309627436752179880[79] = 0.0;
   out_7309627436752179880[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5495602249935962846) {
   out_5495602249935962846[0] = state[0];
   out_5495602249935962846[1] = state[1];
   out_5495602249935962846[2] = state[2];
   out_5495602249935962846[3] = state[3];
   out_5495602249935962846[4] = state[4];
   out_5495602249935962846[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5495602249935962846[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5495602249935962846[7] = state[7];
   out_5495602249935962846[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6467568700425071091) {
   out_6467568700425071091[0] = 1;
   out_6467568700425071091[1] = 0;
   out_6467568700425071091[2] = 0;
   out_6467568700425071091[3] = 0;
   out_6467568700425071091[4] = 0;
   out_6467568700425071091[5] = 0;
   out_6467568700425071091[6] = 0;
   out_6467568700425071091[7] = 0;
   out_6467568700425071091[8] = 0;
   out_6467568700425071091[9] = 0;
   out_6467568700425071091[10] = 1;
   out_6467568700425071091[11] = 0;
   out_6467568700425071091[12] = 0;
   out_6467568700425071091[13] = 0;
   out_6467568700425071091[14] = 0;
   out_6467568700425071091[15] = 0;
   out_6467568700425071091[16] = 0;
   out_6467568700425071091[17] = 0;
   out_6467568700425071091[18] = 0;
   out_6467568700425071091[19] = 0;
   out_6467568700425071091[20] = 1;
   out_6467568700425071091[21] = 0;
   out_6467568700425071091[22] = 0;
   out_6467568700425071091[23] = 0;
   out_6467568700425071091[24] = 0;
   out_6467568700425071091[25] = 0;
   out_6467568700425071091[26] = 0;
   out_6467568700425071091[27] = 0;
   out_6467568700425071091[28] = 0;
   out_6467568700425071091[29] = 0;
   out_6467568700425071091[30] = 1;
   out_6467568700425071091[31] = 0;
   out_6467568700425071091[32] = 0;
   out_6467568700425071091[33] = 0;
   out_6467568700425071091[34] = 0;
   out_6467568700425071091[35] = 0;
   out_6467568700425071091[36] = 0;
   out_6467568700425071091[37] = 0;
   out_6467568700425071091[38] = 0;
   out_6467568700425071091[39] = 0;
   out_6467568700425071091[40] = 1;
   out_6467568700425071091[41] = 0;
   out_6467568700425071091[42] = 0;
   out_6467568700425071091[43] = 0;
   out_6467568700425071091[44] = 0;
   out_6467568700425071091[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6467568700425071091[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6467568700425071091[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6467568700425071091[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6467568700425071091[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6467568700425071091[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6467568700425071091[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6467568700425071091[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6467568700425071091[53] = -9.8000000000000007*dt;
   out_6467568700425071091[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6467568700425071091[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6467568700425071091[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6467568700425071091[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6467568700425071091[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6467568700425071091[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6467568700425071091[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6467568700425071091[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6467568700425071091[62] = 0;
   out_6467568700425071091[63] = 0;
   out_6467568700425071091[64] = 0;
   out_6467568700425071091[65] = 0;
   out_6467568700425071091[66] = 0;
   out_6467568700425071091[67] = 0;
   out_6467568700425071091[68] = 0;
   out_6467568700425071091[69] = 0;
   out_6467568700425071091[70] = 1;
   out_6467568700425071091[71] = 0;
   out_6467568700425071091[72] = 0;
   out_6467568700425071091[73] = 0;
   out_6467568700425071091[74] = 0;
   out_6467568700425071091[75] = 0;
   out_6467568700425071091[76] = 0;
   out_6467568700425071091[77] = 0;
   out_6467568700425071091[78] = 0;
   out_6467568700425071091[79] = 0;
   out_6467568700425071091[80] = 1;
}
void h_25(double *state, double *unused, double *out_7832645508647381886) {
   out_7832645508647381886[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1943928411045134499) {
   out_1943928411045134499[0] = 0;
   out_1943928411045134499[1] = 0;
   out_1943928411045134499[2] = 0;
   out_1943928411045134499[3] = 0;
   out_1943928411045134499[4] = 0;
   out_1943928411045134499[5] = 0;
   out_1943928411045134499[6] = 1;
   out_1943928411045134499[7] = 0;
   out_1943928411045134499[8] = 0;
}
void h_24(double *state, double *unused, double *out_1226025131367495892) {
   out_1226025131367495892[0] = state[4];
   out_1226025131367495892[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2802580915921374535) {
   out_2802580915921374535[0] = 0;
   out_2802580915921374535[1] = 0;
   out_2802580915921374535[2] = 0;
   out_2802580915921374535[3] = 0;
   out_2802580915921374535[4] = 1;
   out_2802580915921374535[5] = 0;
   out_2802580915921374535[6] = 0;
   out_2802580915921374535[7] = 0;
   out_2802580915921374535[8] = 0;
   out_2802580915921374535[9] = 0;
   out_2802580915921374535[10] = 0;
   out_2802580915921374535[11] = 0;
   out_2802580915921374535[12] = 0;
   out_2802580915921374535[13] = 0;
   out_2802580915921374535[14] = 1;
   out_2802580915921374535[15] = 0;
   out_2802580915921374535[16] = 0;
   out_2802580915921374535[17] = 0;
}
void h_30(double *state, double *unused, double *out_3843263338711818794) {
   out_3843263338711818794[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8860618752536751254) {
   out_8860618752536751254[0] = 0;
   out_8860618752536751254[1] = 0;
   out_8860618752536751254[2] = 0;
   out_8860618752536751254[3] = 0;
   out_8860618752536751254[4] = 1;
   out_8860618752536751254[5] = 0;
   out_8860618752536751254[6] = 0;
   out_8860618752536751254[7] = 0;
   out_8860618752536751254[8] = 0;
}
void h_26(double *state, double *unused, double *out_8396322947517111767) {
   out_8396322947517111767[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1797574907828921725) {
   out_1797574907828921725[0] = 0;
   out_1797574907828921725[1] = 0;
   out_1797574907828921725[2] = 0;
   out_1797574907828921725[3] = 0;
   out_1797574907828921725[4] = 0;
   out_1797574907828921725[5] = 0;
   out_1797574907828921725[6] = 0;
   out_1797574907828921725[7] = 1;
   out_1797574907828921725[8] = 0;
}
void h_27(double *state, double *unused, double *out_1917014754913519550) {
   out_1917014754913519550[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6685855440736326343) {
   out_6685855440736326343[0] = 0;
   out_6685855440736326343[1] = 0;
   out_6685855440736326343[2] = 0;
   out_6685855440736326343[3] = 1;
   out_6685855440736326343[4] = 0;
   out_6685855440736326343[5] = 0;
   out_6685855440736326343[6] = 0;
   out_6685855440736326343[7] = 0;
   out_6685855440736326343[8] = 0;
}
void h_29(double *state, double *unused, double *out_1030711235441965411) {
   out_1030711235441965411[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4972492713866775310) {
   out_4972492713866775310[0] = 0;
   out_4972492713866775310[1] = 1;
   out_4972492713866775310[2] = 0;
   out_4972492713866775310[3] = 0;
   out_4972492713866775310[4] = 0;
   out_4972492713866775310[5] = 0;
   out_4972492713866775310[6] = 0;
   out_4972492713866775310[7] = 0;
   out_4972492713866775310[8] = 0;
}
void h_28(double *state, double *unused, double *out_6247245530635606257) {
   out_6247245530635606257[0] = state[0];
}
void H_28(double *state, double *unused, double *out_109906303202755264) {
   out_109906303202755264[0] = 1;
   out_109906303202755264[1] = 0;
   out_109906303202755264[2] = 0;
   out_109906303202755264[3] = 0;
   out_109906303202755264[4] = 0;
   out_109906303202755264[5] = 0;
   out_109906303202755264[6] = 0;
   out_109906303202755264[7] = 0;
   out_109906303202755264[8] = 0;
}
void h_31(double *state, double *unused, double *out_69380251826709445) {
   out_69380251826709445[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1974574372922094927) {
   out_1974574372922094927[0] = 0;
   out_1974574372922094927[1] = 0;
   out_1974574372922094927[2] = 0;
   out_1974574372922094927[3] = 0;
   out_1974574372922094927[4] = 0;
   out_1974574372922094927[5] = 0;
   out_1974574372922094927[6] = 0;
   out_1974574372922094927[7] = 0;
   out_1974574372922094927[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2788712361531455797) {
  err_fun(nom_x, delta_x, out_2788712361531455797);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5243626477297637109) {
  inv_err_fun(nom_x, true_x, out_5243626477297637109);
}
void car_H_mod_fun(double *state, double *out_7309627436752179880) {
  H_mod_fun(state, out_7309627436752179880);
}
void car_f_fun(double *state, double dt, double *out_5495602249935962846) {
  f_fun(state,  dt, out_5495602249935962846);
}
void car_F_fun(double *state, double dt, double *out_6467568700425071091) {
  F_fun(state,  dt, out_6467568700425071091);
}
void car_h_25(double *state, double *unused, double *out_7832645508647381886) {
  h_25(state, unused, out_7832645508647381886);
}
void car_H_25(double *state, double *unused, double *out_1943928411045134499) {
  H_25(state, unused, out_1943928411045134499);
}
void car_h_24(double *state, double *unused, double *out_1226025131367495892) {
  h_24(state, unused, out_1226025131367495892);
}
void car_H_24(double *state, double *unused, double *out_2802580915921374535) {
  H_24(state, unused, out_2802580915921374535);
}
void car_h_30(double *state, double *unused, double *out_3843263338711818794) {
  h_30(state, unused, out_3843263338711818794);
}
void car_H_30(double *state, double *unused, double *out_8860618752536751254) {
  H_30(state, unused, out_8860618752536751254);
}
void car_h_26(double *state, double *unused, double *out_8396322947517111767) {
  h_26(state, unused, out_8396322947517111767);
}
void car_H_26(double *state, double *unused, double *out_1797574907828921725) {
  H_26(state, unused, out_1797574907828921725);
}
void car_h_27(double *state, double *unused, double *out_1917014754913519550) {
  h_27(state, unused, out_1917014754913519550);
}
void car_H_27(double *state, double *unused, double *out_6685855440736326343) {
  H_27(state, unused, out_6685855440736326343);
}
void car_h_29(double *state, double *unused, double *out_1030711235441965411) {
  h_29(state, unused, out_1030711235441965411);
}
void car_H_29(double *state, double *unused, double *out_4972492713866775310) {
  H_29(state, unused, out_4972492713866775310);
}
void car_h_28(double *state, double *unused, double *out_6247245530635606257) {
  h_28(state, unused, out_6247245530635606257);
}
void car_H_28(double *state, double *unused, double *out_109906303202755264) {
  H_28(state, unused, out_109906303202755264);
}
void car_h_31(double *state, double *unused, double *out_69380251826709445) {
  h_31(state, unused, out_69380251826709445);
}
void car_H_31(double *state, double *unused, double *out_1974574372922094927) {
  H_31(state, unused, out_1974574372922094927);
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
