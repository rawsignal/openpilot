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
void err_fun(double *nom_x, double *delta_x, double *out_285877354863972642) {
   out_285877354863972642[0] = delta_x[0] + nom_x[0];
   out_285877354863972642[1] = delta_x[1] + nom_x[1];
   out_285877354863972642[2] = delta_x[2] + nom_x[2];
   out_285877354863972642[3] = delta_x[3] + nom_x[3];
   out_285877354863972642[4] = delta_x[4] + nom_x[4];
   out_285877354863972642[5] = delta_x[5] + nom_x[5];
   out_285877354863972642[6] = delta_x[6] + nom_x[6];
   out_285877354863972642[7] = delta_x[7] + nom_x[7];
   out_285877354863972642[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_865842154645548932) {
   out_865842154645548932[0] = -nom_x[0] + true_x[0];
   out_865842154645548932[1] = -nom_x[1] + true_x[1];
   out_865842154645548932[2] = -nom_x[2] + true_x[2];
   out_865842154645548932[3] = -nom_x[3] + true_x[3];
   out_865842154645548932[4] = -nom_x[4] + true_x[4];
   out_865842154645548932[5] = -nom_x[5] + true_x[5];
   out_865842154645548932[6] = -nom_x[6] + true_x[6];
   out_865842154645548932[7] = -nom_x[7] + true_x[7];
   out_865842154645548932[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2321332579323854249) {
   out_2321332579323854249[0] = 1.0;
   out_2321332579323854249[1] = 0.0;
   out_2321332579323854249[2] = 0.0;
   out_2321332579323854249[3] = 0.0;
   out_2321332579323854249[4] = 0.0;
   out_2321332579323854249[5] = 0.0;
   out_2321332579323854249[6] = 0.0;
   out_2321332579323854249[7] = 0.0;
   out_2321332579323854249[8] = 0.0;
   out_2321332579323854249[9] = 0.0;
   out_2321332579323854249[10] = 1.0;
   out_2321332579323854249[11] = 0.0;
   out_2321332579323854249[12] = 0.0;
   out_2321332579323854249[13] = 0.0;
   out_2321332579323854249[14] = 0.0;
   out_2321332579323854249[15] = 0.0;
   out_2321332579323854249[16] = 0.0;
   out_2321332579323854249[17] = 0.0;
   out_2321332579323854249[18] = 0.0;
   out_2321332579323854249[19] = 0.0;
   out_2321332579323854249[20] = 1.0;
   out_2321332579323854249[21] = 0.0;
   out_2321332579323854249[22] = 0.0;
   out_2321332579323854249[23] = 0.0;
   out_2321332579323854249[24] = 0.0;
   out_2321332579323854249[25] = 0.0;
   out_2321332579323854249[26] = 0.0;
   out_2321332579323854249[27] = 0.0;
   out_2321332579323854249[28] = 0.0;
   out_2321332579323854249[29] = 0.0;
   out_2321332579323854249[30] = 1.0;
   out_2321332579323854249[31] = 0.0;
   out_2321332579323854249[32] = 0.0;
   out_2321332579323854249[33] = 0.0;
   out_2321332579323854249[34] = 0.0;
   out_2321332579323854249[35] = 0.0;
   out_2321332579323854249[36] = 0.0;
   out_2321332579323854249[37] = 0.0;
   out_2321332579323854249[38] = 0.0;
   out_2321332579323854249[39] = 0.0;
   out_2321332579323854249[40] = 1.0;
   out_2321332579323854249[41] = 0.0;
   out_2321332579323854249[42] = 0.0;
   out_2321332579323854249[43] = 0.0;
   out_2321332579323854249[44] = 0.0;
   out_2321332579323854249[45] = 0.0;
   out_2321332579323854249[46] = 0.0;
   out_2321332579323854249[47] = 0.0;
   out_2321332579323854249[48] = 0.0;
   out_2321332579323854249[49] = 0.0;
   out_2321332579323854249[50] = 1.0;
   out_2321332579323854249[51] = 0.0;
   out_2321332579323854249[52] = 0.0;
   out_2321332579323854249[53] = 0.0;
   out_2321332579323854249[54] = 0.0;
   out_2321332579323854249[55] = 0.0;
   out_2321332579323854249[56] = 0.0;
   out_2321332579323854249[57] = 0.0;
   out_2321332579323854249[58] = 0.0;
   out_2321332579323854249[59] = 0.0;
   out_2321332579323854249[60] = 1.0;
   out_2321332579323854249[61] = 0.0;
   out_2321332579323854249[62] = 0.0;
   out_2321332579323854249[63] = 0.0;
   out_2321332579323854249[64] = 0.0;
   out_2321332579323854249[65] = 0.0;
   out_2321332579323854249[66] = 0.0;
   out_2321332579323854249[67] = 0.0;
   out_2321332579323854249[68] = 0.0;
   out_2321332579323854249[69] = 0.0;
   out_2321332579323854249[70] = 1.0;
   out_2321332579323854249[71] = 0.0;
   out_2321332579323854249[72] = 0.0;
   out_2321332579323854249[73] = 0.0;
   out_2321332579323854249[74] = 0.0;
   out_2321332579323854249[75] = 0.0;
   out_2321332579323854249[76] = 0.0;
   out_2321332579323854249[77] = 0.0;
   out_2321332579323854249[78] = 0.0;
   out_2321332579323854249[79] = 0.0;
   out_2321332579323854249[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8939805825965589821) {
   out_8939805825965589821[0] = state[0];
   out_8939805825965589821[1] = state[1];
   out_8939805825965589821[2] = state[2];
   out_8939805825965589821[3] = state[3];
   out_8939805825965589821[4] = state[4];
   out_8939805825965589821[5] = dt*(stiffness_front*(-state[2] - state[3] + state[7])*state[0]/state[1] + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/state[4] + ((-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/state[4] - state[4])*state[6] - 9.8100000000000005*state[8]) + state[5];
   out_8939805825965589821[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8939805825965589821[7] = state[7];
   out_8939805825965589821[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6594188117642331652) {
   out_6594188117642331652[0] = 1;
   out_6594188117642331652[1] = 0;
   out_6594188117642331652[2] = 0;
   out_6594188117642331652[3] = 0;
   out_6594188117642331652[4] = 0;
   out_6594188117642331652[5] = 0;
   out_6594188117642331652[6] = 0;
   out_6594188117642331652[7] = 0;
   out_6594188117642331652[8] = 0;
   out_6594188117642331652[9] = 0;
   out_6594188117642331652[10] = 1;
   out_6594188117642331652[11] = 0;
   out_6594188117642331652[12] = 0;
   out_6594188117642331652[13] = 0;
   out_6594188117642331652[14] = 0;
   out_6594188117642331652[15] = 0;
   out_6594188117642331652[16] = 0;
   out_6594188117642331652[17] = 0;
   out_6594188117642331652[18] = 0;
   out_6594188117642331652[19] = 0;
   out_6594188117642331652[20] = 1;
   out_6594188117642331652[21] = 0;
   out_6594188117642331652[22] = 0;
   out_6594188117642331652[23] = 0;
   out_6594188117642331652[24] = 0;
   out_6594188117642331652[25] = 0;
   out_6594188117642331652[26] = 0;
   out_6594188117642331652[27] = 0;
   out_6594188117642331652[28] = 0;
   out_6594188117642331652[29] = 0;
   out_6594188117642331652[30] = 1;
   out_6594188117642331652[31] = 0;
   out_6594188117642331652[32] = 0;
   out_6594188117642331652[33] = 0;
   out_6594188117642331652[34] = 0;
   out_6594188117642331652[35] = 0;
   out_6594188117642331652[36] = 0;
   out_6594188117642331652[37] = 0;
   out_6594188117642331652[38] = 0;
   out_6594188117642331652[39] = 0;
   out_6594188117642331652[40] = 1;
   out_6594188117642331652[41] = 0;
   out_6594188117642331652[42] = 0;
   out_6594188117642331652[43] = 0;
   out_6594188117642331652[44] = 0;
   out_6594188117642331652[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/state[1] + (-stiffness_front - stiffness_rear)*state[5]/state[4] + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/state[4]);
   out_6594188117642331652[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/pow(state[1], 2);
   out_6594188117642331652[47] = -dt*stiffness_front*state[0]/state[1];
   out_6594188117642331652[48] = -dt*stiffness_front*state[0]/state[1];
   out_6594188117642331652[49] = dt*(-(-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/pow(state[4], 2) + (-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/pow(state[4], 2) - 1)*state[6]);
   out_6594188117642331652[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/state[4] + 1;
   out_6594188117642331652[51] = dt*((-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/state[4] - state[4]);
   out_6594188117642331652[52] = dt*stiffness_front*state[0]/state[1];
   out_6594188117642331652[53] = -9.8100000000000005*dt;
   out_6594188117642331652[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6594188117642331652[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6594188117642331652[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6594188117642331652[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6594188117642331652[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6594188117642331652[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6594188117642331652[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6594188117642331652[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6594188117642331652[62] = 0;
   out_6594188117642331652[63] = 0;
   out_6594188117642331652[64] = 0;
   out_6594188117642331652[65] = 0;
   out_6594188117642331652[66] = 0;
   out_6594188117642331652[67] = 0;
   out_6594188117642331652[68] = 0;
   out_6594188117642331652[69] = 0;
   out_6594188117642331652[70] = 1;
   out_6594188117642331652[71] = 0;
   out_6594188117642331652[72] = 0;
   out_6594188117642331652[73] = 0;
   out_6594188117642331652[74] = 0;
   out_6594188117642331652[75] = 0;
   out_6594188117642331652[76] = 0;
   out_6594188117642331652[77] = 0;
   out_6594188117642331652[78] = 0;
   out_6594188117642331652[79] = 0;
   out_6594188117642331652[80] = 1;
}
void h_25(double *state, double *unused, double *out_3622607962420453030) {
   out_3622607962420453030[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2582359245860810591) {
   out_2582359245860810591[0] = 0;
   out_2582359245860810591[1] = 0;
   out_2582359245860810591[2] = 0;
   out_2582359245860810591[3] = 0;
   out_2582359245860810591[4] = 0;
   out_2582359245860810591[5] = 0;
   out_2582359245860810591[6] = 1;
   out_2582359245860810591[7] = 0;
   out_2582359245860810591[8] = 0;
}
void h_24(double *state, double *unused, double *out_8587894906978392811) {
   out_8587894906978392811[0] = state[4];
   out_8587894906978392811[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3427355854644505990) {
   out_3427355854644505990[0] = 0;
   out_3427355854644505990[1] = 0;
   out_3427355854644505990[2] = 0;
   out_3427355854644505990[3] = 0;
   out_3427355854644505990[4] = 1;
   out_3427355854644505990[5] = 0;
   out_3427355854644505990[6] = 0;
   out_3427355854644505990[7] = 0;
   out_3427355854644505990[8] = 0;
   out_3427355854644505990[9] = 0;
   out_3427355854644505990[10] = 0;
   out_3427355854644505990[11] = 0;
   out_3427355854644505990[12] = 0;
   out_3427355854644505990[13] = 0;
   out_3427355854644505990[14] = 1;
   out_3427355854644505990[15] = 0;
   out_3427355854644505990[16] = 0;
   out_3427355854644505990[17] = 0;
}
void h_30(double *state, double *unused, double *out_8008059677881605864) {
   out_8008059677881605864[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8947694486357124270) {
   out_8947694486357124270[0] = 0;
   out_8947694486357124270[1] = 0;
   out_8947694486357124270[2] = 0;
   out_8947694486357124270[3] = 0;
   out_8947694486357124270[4] = 1;
   out_8947694486357124270[5] = 0;
   out_8947694486357124270[6] = 0;
   out_8947694486357124270[7] = 0;
   out_8947694486357124270[8] = 0;
}
void h_26(double *state, double *unused, double *out_7730774847579258584) {
   out_7730774847579258584[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1159144073013245633) {
   out_1159144073013245633[0] = 0;
   out_1159144073013245633[1] = 0;
   out_1159144073013245633[2] = 0;
   out_1159144073013245633[3] = 0;
   out_1159144073013245633[4] = 0;
   out_1159144073013245633[5] = 0;
   out_1159144073013245633[6] = 0;
   out_1159144073013245633[7] = 1;
   out_1159144073013245633[8] = 0;
}
void h_27(double *state, double *unused, double *out_6803325635591764793) {
   out_6803325635591764793[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7324286275552002435) {
   out_7324286275552002435[0] = 0;
   out_7324286275552002435[1] = 0;
   out_7324286275552002435[2] = 0;
   out_7324286275552002435[3] = 1;
   out_7324286275552002435[4] = 0;
   out_7324286275552002435[5] = 0;
   out_7324286275552002435[6] = 0;
   out_7324286275552002435[7] = 0;
   out_7324286275552002435[8] = 0;
}
void h_29(double *state, double *unused, double *out_6159966003405800524) {
   out_6159966003405800524[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5610923548682451402) {
   out_5610923548682451402[0] = 0;
   out_5610923548682451402[1] = 1;
   out_5610923548682451402[2] = 0;
   out_5610923548682451402[3] = 0;
   out_5610923548682451402[4] = 0;
   out_5610923548682451402[5] = 0;
   out_5610923548682451402[6] = 0;
   out_5610923548682451402[7] = 0;
   out_5610923548682451402[8] = 0;
}
void h_28(double *state, double *unused, double *out_1801652367930919148) {
   out_1801652367930919148[0] = state[0];
}
void H_28(double *state, double *unused, double *out_528524531612920828) {
   out_528524531612920828[0] = 1;
   out_528524531612920828[1] = 0;
   out_528524531612920828[2] = 0;
   out_528524531612920828[3] = 0;
   out_528524531612920828[4] = 0;
   out_528524531612920828[5] = 0;
   out_528524531612920828[6] = 0;
   out_528524531612920828[7] = 0;
   out_528524531612920828[8] = 0;
}
void h_31(double *state, double *unused, double *out_6526040805289417513) {
   out_6526040805289417513[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2613005207737771019) {
   out_2613005207737771019[0] = 0;
   out_2613005207737771019[1] = 0;
   out_2613005207737771019[2] = 0;
   out_2613005207737771019[3] = 0;
   out_2613005207737771019[4] = 0;
   out_2613005207737771019[5] = 0;
   out_2613005207737771019[6] = 0;
   out_2613005207737771019[7] = 0;
   out_2613005207737771019[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_285877354863972642) {
  err_fun(nom_x, delta_x, out_285877354863972642);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_865842154645548932) {
  inv_err_fun(nom_x, true_x, out_865842154645548932);
}
void car_H_mod_fun(double *state, double *out_2321332579323854249) {
  H_mod_fun(state, out_2321332579323854249);
}
void car_f_fun(double *state, double dt, double *out_8939805825965589821) {
  f_fun(state,  dt, out_8939805825965589821);
}
void car_F_fun(double *state, double dt, double *out_6594188117642331652) {
  F_fun(state,  dt, out_6594188117642331652);
}
void car_h_25(double *state, double *unused, double *out_3622607962420453030) {
  h_25(state, unused, out_3622607962420453030);
}
void car_H_25(double *state, double *unused, double *out_2582359245860810591) {
  H_25(state, unused, out_2582359245860810591);
}
void car_h_24(double *state, double *unused, double *out_8587894906978392811) {
  h_24(state, unused, out_8587894906978392811);
}
void car_H_24(double *state, double *unused, double *out_3427355854644505990) {
  H_24(state, unused, out_3427355854644505990);
}
void car_h_30(double *state, double *unused, double *out_8008059677881605864) {
  h_30(state, unused, out_8008059677881605864);
}
void car_H_30(double *state, double *unused, double *out_8947694486357124270) {
  H_30(state, unused, out_8947694486357124270);
}
void car_h_26(double *state, double *unused, double *out_7730774847579258584) {
  h_26(state, unused, out_7730774847579258584);
}
void car_H_26(double *state, double *unused, double *out_1159144073013245633) {
  H_26(state, unused, out_1159144073013245633);
}
void car_h_27(double *state, double *unused, double *out_6803325635591764793) {
  h_27(state, unused, out_6803325635591764793);
}
void car_H_27(double *state, double *unused, double *out_7324286275552002435) {
  H_27(state, unused, out_7324286275552002435);
}
void car_h_29(double *state, double *unused, double *out_6159966003405800524) {
  h_29(state, unused, out_6159966003405800524);
}
void car_H_29(double *state, double *unused, double *out_5610923548682451402) {
  H_29(state, unused, out_5610923548682451402);
}
void car_h_28(double *state, double *unused, double *out_1801652367930919148) {
  h_28(state, unused, out_1801652367930919148);
}
void car_H_28(double *state, double *unused, double *out_528524531612920828) {
  H_28(state, unused, out_528524531612920828);
}
void car_h_31(double *state, double *unused, double *out_6526040805289417513) {
  h_31(state, unused, out_6526040805289417513);
}
void car_H_31(double *state, double *unused, double *out_2613005207737771019) {
  H_31(state, unused, out_2613005207737771019);
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
