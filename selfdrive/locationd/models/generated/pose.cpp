#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_455832145760308814) {
   out_455832145760308814[0] = delta_x[0] + nom_x[0];
   out_455832145760308814[1] = delta_x[1] + nom_x[1];
   out_455832145760308814[2] = delta_x[2] + nom_x[2];
   out_455832145760308814[3] = delta_x[3] + nom_x[3];
   out_455832145760308814[4] = delta_x[4] + nom_x[4];
   out_455832145760308814[5] = delta_x[5] + nom_x[5];
   out_455832145760308814[6] = delta_x[6] + nom_x[6];
   out_455832145760308814[7] = delta_x[7] + nom_x[7];
   out_455832145760308814[8] = delta_x[8] + nom_x[8];
   out_455832145760308814[9] = delta_x[9] + nom_x[9];
   out_455832145760308814[10] = delta_x[10] + nom_x[10];
   out_455832145760308814[11] = delta_x[11] + nom_x[11];
   out_455832145760308814[12] = delta_x[12] + nom_x[12];
   out_455832145760308814[13] = delta_x[13] + nom_x[13];
   out_455832145760308814[14] = delta_x[14] + nom_x[14];
   out_455832145760308814[15] = delta_x[15] + nom_x[15];
   out_455832145760308814[16] = delta_x[16] + nom_x[16];
   out_455832145760308814[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6833442360289991349) {
   out_6833442360289991349[0] = -nom_x[0] + true_x[0];
   out_6833442360289991349[1] = -nom_x[1] + true_x[1];
   out_6833442360289991349[2] = -nom_x[2] + true_x[2];
   out_6833442360289991349[3] = -nom_x[3] + true_x[3];
   out_6833442360289991349[4] = -nom_x[4] + true_x[4];
   out_6833442360289991349[5] = -nom_x[5] + true_x[5];
   out_6833442360289991349[6] = -nom_x[6] + true_x[6];
   out_6833442360289991349[7] = -nom_x[7] + true_x[7];
   out_6833442360289991349[8] = -nom_x[8] + true_x[8];
   out_6833442360289991349[9] = -nom_x[9] + true_x[9];
   out_6833442360289991349[10] = -nom_x[10] + true_x[10];
   out_6833442360289991349[11] = -nom_x[11] + true_x[11];
   out_6833442360289991349[12] = -nom_x[12] + true_x[12];
   out_6833442360289991349[13] = -nom_x[13] + true_x[13];
   out_6833442360289991349[14] = -nom_x[14] + true_x[14];
   out_6833442360289991349[15] = -nom_x[15] + true_x[15];
   out_6833442360289991349[16] = -nom_x[16] + true_x[16];
   out_6833442360289991349[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4443372719987989996) {
   out_4443372719987989996[0] = 1.0;
   out_4443372719987989996[1] = 0.0;
   out_4443372719987989996[2] = 0.0;
   out_4443372719987989996[3] = 0.0;
   out_4443372719987989996[4] = 0.0;
   out_4443372719987989996[5] = 0.0;
   out_4443372719987989996[6] = 0.0;
   out_4443372719987989996[7] = 0.0;
   out_4443372719987989996[8] = 0.0;
   out_4443372719987989996[9] = 0.0;
   out_4443372719987989996[10] = 0.0;
   out_4443372719987989996[11] = 0.0;
   out_4443372719987989996[12] = 0.0;
   out_4443372719987989996[13] = 0.0;
   out_4443372719987989996[14] = 0.0;
   out_4443372719987989996[15] = 0.0;
   out_4443372719987989996[16] = 0.0;
   out_4443372719987989996[17] = 0.0;
   out_4443372719987989996[18] = 0.0;
   out_4443372719987989996[19] = 1.0;
   out_4443372719987989996[20] = 0.0;
   out_4443372719987989996[21] = 0.0;
   out_4443372719987989996[22] = 0.0;
   out_4443372719987989996[23] = 0.0;
   out_4443372719987989996[24] = 0.0;
   out_4443372719987989996[25] = 0.0;
   out_4443372719987989996[26] = 0.0;
   out_4443372719987989996[27] = 0.0;
   out_4443372719987989996[28] = 0.0;
   out_4443372719987989996[29] = 0.0;
   out_4443372719987989996[30] = 0.0;
   out_4443372719987989996[31] = 0.0;
   out_4443372719987989996[32] = 0.0;
   out_4443372719987989996[33] = 0.0;
   out_4443372719987989996[34] = 0.0;
   out_4443372719987989996[35] = 0.0;
   out_4443372719987989996[36] = 0.0;
   out_4443372719987989996[37] = 0.0;
   out_4443372719987989996[38] = 1.0;
   out_4443372719987989996[39] = 0.0;
   out_4443372719987989996[40] = 0.0;
   out_4443372719987989996[41] = 0.0;
   out_4443372719987989996[42] = 0.0;
   out_4443372719987989996[43] = 0.0;
   out_4443372719987989996[44] = 0.0;
   out_4443372719987989996[45] = 0.0;
   out_4443372719987989996[46] = 0.0;
   out_4443372719987989996[47] = 0.0;
   out_4443372719987989996[48] = 0.0;
   out_4443372719987989996[49] = 0.0;
   out_4443372719987989996[50] = 0.0;
   out_4443372719987989996[51] = 0.0;
   out_4443372719987989996[52] = 0.0;
   out_4443372719987989996[53] = 0.0;
   out_4443372719987989996[54] = 0.0;
   out_4443372719987989996[55] = 0.0;
   out_4443372719987989996[56] = 0.0;
   out_4443372719987989996[57] = 1.0;
   out_4443372719987989996[58] = 0.0;
   out_4443372719987989996[59] = 0.0;
   out_4443372719987989996[60] = 0.0;
   out_4443372719987989996[61] = 0.0;
   out_4443372719987989996[62] = 0.0;
   out_4443372719987989996[63] = 0.0;
   out_4443372719987989996[64] = 0.0;
   out_4443372719987989996[65] = 0.0;
   out_4443372719987989996[66] = 0.0;
   out_4443372719987989996[67] = 0.0;
   out_4443372719987989996[68] = 0.0;
   out_4443372719987989996[69] = 0.0;
   out_4443372719987989996[70] = 0.0;
   out_4443372719987989996[71] = 0.0;
   out_4443372719987989996[72] = 0.0;
   out_4443372719987989996[73] = 0.0;
   out_4443372719987989996[74] = 0.0;
   out_4443372719987989996[75] = 0.0;
   out_4443372719987989996[76] = 1.0;
   out_4443372719987989996[77] = 0.0;
   out_4443372719987989996[78] = 0.0;
   out_4443372719987989996[79] = 0.0;
   out_4443372719987989996[80] = 0.0;
   out_4443372719987989996[81] = 0.0;
   out_4443372719987989996[82] = 0.0;
   out_4443372719987989996[83] = 0.0;
   out_4443372719987989996[84] = 0.0;
   out_4443372719987989996[85] = 0.0;
   out_4443372719987989996[86] = 0.0;
   out_4443372719987989996[87] = 0.0;
   out_4443372719987989996[88] = 0.0;
   out_4443372719987989996[89] = 0.0;
   out_4443372719987989996[90] = 0.0;
   out_4443372719987989996[91] = 0.0;
   out_4443372719987989996[92] = 0.0;
   out_4443372719987989996[93] = 0.0;
   out_4443372719987989996[94] = 0.0;
   out_4443372719987989996[95] = 1.0;
   out_4443372719987989996[96] = 0.0;
   out_4443372719987989996[97] = 0.0;
   out_4443372719987989996[98] = 0.0;
   out_4443372719987989996[99] = 0.0;
   out_4443372719987989996[100] = 0.0;
   out_4443372719987989996[101] = 0.0;
   out_4443372719987989996[102] = 0.0;
   out_4443372719987989996[103] = 0.0;
   out_4443372719987989996[104] = 0.0;
   out_4443372719987989996[105] = 0.0;
   out_4443372719987989996[106] = 0.0;
   out_4443372719987989996[107] = 0.0;
   out_4443372719987989996[108] = 0.0;
   out_4443372719987989996[109] = 0.0;
   out_4443372719987989996[110] = 0.0;
   out_4443372719987989996[111] = 0.0;
   out_4443372719987989996[112] = 0.0;
   out_4443372719987989996[113] = 0.0;
   out_4443372719987989996[114] = 1.0;
   out_4443372719987989996[115] = 0.0;
   out_4443372719987989996[116] = 0.0;
   out_4443372719987989996[117] = 0.0;
   out_4443372719987989996[118] = 0.0;
   out_4443372719987989996[119] = 0.0;
   out_4443372719987989996[120] = 0.0;
   out_4443372719987989996[121] = 0.0;
   out_4443372719987989996[122] = 0.0;
   out_4443372719987989996[123] = 0.0;
   out_4443372719987989996[124] = 0.0;
   out_4443372719987989996[125] = 0.0;
   out_4443372719987989996[126] = 0.0;
   out_4443372719987989996[127] = 0.0;
   out_4443372719987989996[128] = 0.0;
   out_4443372719987989996[129] = 0.0;
   out_4443372719987989996[130] = 0.0;
   out_4443372719987989996[131] = 0.0;
   out_4443372719987989996[132] = 0.0;
   out_4443372719987989996[133] = 1.0;
   out_4443372719987989996[134] = 0.0;
   out_4443372719987989996[135] = 0.0;
   out_4443372719987989996[136] = 0.0;
   out_4443372719987989996[137] = 0.0;
   out_4443372719987989996[138] = 0.0;
   out_4443372719987989996[139] = 0.0;
   out_4443372719987989996[140] = 0.0;
   out_4443372719987989996[141] = 0.0;
   out_4443372719987989996[142] = 0.0;
   out_4443372719987989996[143] = 0.0;
   out_4443372719987989996[144] = 0.0;
   out_4443372719987989996[145] = 0.0;
   out_4443372719987989996[146] = 0.0;
   out_4443372719987989996[147] = 0.0;
   out_4443372719987989996[148] = 0.0;
   out_4443372719987989996[149] = 0.0;
   out_4443372719987989996[150] = 0.0;
   out_4443372719987989996[151] = 0.0;
   out_4443372719987989996[152] = 1.0;
   out_4443372719987989996[153] = 0.0;
   out_4443372719987989996[154] = 0.0;
   out_4443372719987989996[155] = 0.0;
   out_4443372719987989996[156] = 0.0;
   out_4443372719987989996[157] = 0.0;
   out_4443372719987989996[158] = 0.0;
   out_4443372719987989996[159] = 0.0;
   out_4443372719987989996[160] = 0.0;
   out_4443372719987989996[161] = 0.0;
   out_4443372719987989996[162] = 0.0;
   out_4443372719987989996[163] = 0.0;
   out_4443372719987989996[164] = 0.0;
   out_4443372719987989996[165] = 0.0;
   out_4443372719987989996[166] = 0.0;
   out_4443372719987989996[167] = 0.0;
   out_4443372719987989996[168] = 0.0;
   out_4443372719987989996[169] = 0.0;
   out_4443372719987989996[170] = 0.0;
   out_4443372719987989996[171] = 1.0;
   out_4443372719987989996[172] = 0.0;
   out_4443372719987989996[173] = 0.0;
   out_4443372719987989996[174] = 0.0;
   out_4443372719987989996[175] = 0.0;
   out_4443372719987989996[176] = 0.0;
   out_4443372719987989996[177] = 0.0;
   out_4443372719987989996[178] = 0.0;
   out_4443372719987989996[179] = 0.0;
   out_4443372719987989996[180] = 0.0;
   out_4443372719987989996[181] = 0.0;
   out_4443372719987989996[182] = 0.0;
   out_4443372719987989996[183] = 0.0;
   out_4443372719987989996[184] = 0.0;
   out_4443372719987989996[185] = 0.0;
   out_4443372719987989996[186] = 0.0;
   out_4443372719987989996[187] = 0.0;
   out_4443372719987989996[188] = 0.0;
   out_4443372719987989996[189] = 0.0;
   out_4443372719987989996[190] = 1.0;
   out_4443372719987989996[191] = 0.0;
   out_4443372719987989996[192] = 0.0;
   out_4443372719987989996[193] = 0.0;
   out_4443372719987989996[194] = 0.0;
   out_4443372719987989996[195] = 0.0;
   out_4443372719987989996[196] = 0.0;
   out_4443372719987989996[197] = 0.0;
   out_4443372719987989996[198] = 0.0;
   out_4443372719987989996[199] = 0.0;
   out_4443372719987989996[200] = 0.0;
   out_4443372719987989996[201] = 0.0;
   out_4443372719987989996[202] = 0.0;
   out_4443372719987989996[203] = 0.0;
   out_4443372719987989996[204] = 0.0;
   out_4443372719987989996[205] = 0.0;
   out_4443372719987989996[206] = 0.0;
   out_4443372719987989996[207] = 0.0;
   out_4443372719987989996[208] = 0.0;
   out_4443372719987989996[209] = 1.0;
   out_4443372719987989996[210] = 0.0;
   out_4443372719987989996[211] = 0.0;
   out_4443372719987989996[212] = 0.0;
   out_4443372719987989996[213] = 0.0;
   out_4443372719987989996[214] = 0.0;
   out_4443372719987989996[215] = 0.0;
   out_4443372719987989996[216] = 0.0;
   out_4443372719987989996[217] = 0.0;
   out_4443372719987989996[218] = 0.0;
   out_4443372719987989996[219] = 0.0;
   out_4443372719987989996[220] = 0.0;
   out_4443372719987989996[221] = 0.0;
   out_4443372719987989996[222] = 0.0;
   out_4443372719987989996[223] = 0.0;
   out_4443372719987989996[224] = 0.0;
   out_4443372719987989996[225] = 0.0;
   out_4443372719987989996[226] = 0.0;
   out_4443372719987989996[227] = 0.0;
   out_4443372719987989996[228] = 1.0;
   out_4443372719987989996[229] = 0.0;
   out_4443372719987989996[230] = 0.0;
   out_4443372719987989996[231] = 0.0;
   out_4443372719987989996[232] = 0.0;
   out_4443372719987989996[233] = 0.0;
   out_4443372719987989996[234] = 0.0;
   out_4443372719987989996[235] = 0.0;
   out_4443372719987989996[236] = 0.0;
   out_4443372719987989996[237] = 0.0;
   out_4443372719987989996[238] = 0.0;
   out_4443372719987989996[239] = 0.0;
   out_4443372719987989996[240] = 0.0;
   out_4443372719987989996[241] = 0.0;
   out_4443372719987989996[242] = 0.0;
   out_4443372719987989996[243] = 0.0;
   out_4443372719987989996[244] = 0.0;
   out_4443372719987989996[245] = 0.0;
   out_4443372719987989996[246] = 0.0;
   out_4443372719987989996[247] = 1.0;
   out_4443372719987989996[248] = 0.0;
   out_4443372719987989996[249] = 0.0;
   out_4443372719987989996[250] = 0.0;
   out_4443372719987989996[251] = 0.0;
   out_4443372719987989996[252] = 0.0;
   out_4443372719987989996[253] = 0.0;
   out_4443372719987989996[254] = 0.0;
   out_4443372719987989996[255] = 0.0;
   out_4443372719987989996[256] = 0.0;
   out_4443372719987989996[257] = 0.0;
   out_4443372719987989996[258] = 0.0;
   out_4443372719987989996[259] = 0.0;
   out_4443372719987989996[260] = 0.0;
   out_4443372719987989996[261] = 0.0;
   out_4443372719987989996[262] = 0.0;
   out_4443372719987989996[263] = 0.0;
   out_4443372719987989996[264] = 0.0;
   out_4443372719987989996[265] = 0.0;
   out_4443372719987989996[266] = 1.0;
   out_4443372719987989996[267] = 0.0;
   out_4443372719987989996[268] = 0.0;
   out_4443372719987989996[269] = 0.0;
   out_4443372719987989996[270] = 0.0;
   out_4443372719987989996[271] = 0.0;
   out_4443372719987989996[272] = 0.0;
   out_4443372719987989996[273] = 0.0;
   out_4443372719987989996[274] = 0.0;
   out_4443372719987989996[275] = 0.0;
   out_4443372719987989996[276] = 0.0;
   out_4443372719987989996[277] = 0.0;
   out_4443372719987989996[278] = 0.0;
   out_4443372719987989996[279] = 0.0;
   out_4443372719987989996[280] = 0.0;
   out_4443372719987989996[281] = 0.0;
   out_4443372719987989996[282] = 0.0;
   out_4443372719987989996[283] = 0.0;
   out_4443372719987989996[284] = 0.0;
   out_4443372719987989996[285] = 1.0;
   out_4443372719987989996[286] = 0.0;
   out_4443372719987989996[287] = 0.0;
   out_4443372719987989996[288] = 0.0;
   out_4443372719987989996[289] = 0.0;
   out_4443372719987989996[290] = 0.0;
   out_4443372719987989996[291] = 0.0;
   out_4443372719987989996[292] = 0.0;
   out_4443372719987989996[293] = 0.0;
   out_4443372719987989996[294] = 0.0;
   out_4443372719987989996[295] = 0.0;
   out_4443372719987989996[296] = 0.0;
   out_4443372719987989996[297] = 0.0;
   out_4443372719987989996[298] = 0.0;
   out_4443372719987989996[299] = 0.0;
   out_4443372719987989996[300] = 0.0;
   out_4443372719987989996[301] = 0.0;
   out_4443372719987989996[302] = 0.0;
   out_4443372719987989996[303] = 0.0;
   out_4443372719987989996[304] = 1.0;
   out_4443372719987989996[305] = 0.0;
   out_4443372719987989996[306] = 0.0;
   out_4443372719987989996[307] = 0.0;
   out_4443372719987989996[308] = 0.0;
   out_4443372719987989996[309] = 0.0;
   out_4443372719987989996[310] = 0.0;
   out_4443372719987989996[311] = 0.0;
   out_4443372719987989996[312] = 0.0;
   out_4443372719987989996[313] = 0.0;
   out_4443372719987989996[314] = 0.0;
   out_4443372719987989996[315] = 0.0;
   out_4443372719987989996[316] = 0.0;
   out_4443372719987989996[317] = 0.0;
   out_4443372719987989996[318] = 0.0;
   out_4443372719987989996[319] = 0.0;
   out_4443372719987989996[320] = 0.0;
   out_4443372719987989996[321] = 0.0;
   out_4443372719987989996[322] = 0.0;
   out_4443372719987989996[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_7271509597229759153) {
   out_7271509597229759153[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_7271509597229759153[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_7271509597229759153[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_7271509597229759153[3] = dt*state[12] + state[3];
   out_7271509597229759153[4] = dt*state[13] + state[4];
   out_7271509597229759153[5] = dt*state[14] + state[5];
   out_7271509597229759153[6] = state[6];
   out_7271509597229759153[7] = state[7];
   out_7271509597229759153[8] = state[8];
   out_7271509597229759153[9] = state[9];
   out_7271509597229759153[10] = state[10];
   out_7271509597229759153[11] = state[11];
   out_7271509597229759153[12] = state[12];
   out_7271509597229759153[13] = state[13];
   out_7271509597229759153[14] = state[14];
   out_7271509597229759153[15] = state[15];
   out_7271509597229759153[16] = state[16];
   out_7271509597229759153[17] = state[17];
}
void F_fun(double *state, double dt, double *out_8398366488676661931) {
   out_8398366488676661931[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8398366488676661931[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8398366488676661931[2] = 0;
   out_8398366488676661931[3] = 0;
   out_8398366488676661931[4] = 0;
   out_8398366488676661931[5] = 0;
   out_8398366488676661931[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8398366488676661931[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8398366488676661931[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8398366488676661931[9] = 0;
   out_8398366488676661931[10] = 0;
   out_8398366488676661931[11] = 0;
   out_8398366488676661931[12] = 0;
   out_8398366488676661931[13] = 0;
   out_8398366488676661931[14] = 0;
   out_8398366488676661931[15] = 0;
   out_8398366488676661931[16] = 0;
   out_8398366488676661931[17] = 0;
   out_8398366488676661931[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8398366488676661931[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8398366488676661931[20] = 0;
   out_8398366488676661931[21] = 0;
   out_8398366488676661931[22] = 0;
   out_8398366488676661931[23] = 0;
   out_8398366488676661931[24] = 0;
   out_8398366488676661931[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8398366488676661931[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8398366488676661931[27] = 0;
   out_8398366488676661931[28] = 0;
   out_8398366488676661931[29] = 0;
   out_8398366488676661931[30] = 0;
   out_8398366488676661931[31] = 0;
   out_8398366488676661931[32] = 0;
   out_8398366488676661931[33] = 0;
   out_8398366488676661931[34] = 0;
   out_8398366488676661931[35] = 0;
   out_8398366488676661931[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8398366488676661931[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8398366488676661931[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8398366488676661931[39] = 0;
   out_8398366488676661931[40] = 0;
   out_8398366488676661931[41] = 0;
   out_8398366488676661931[42] = 0;
   out_8398366488676661931[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8398366488676661931[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8398366488676661931[45] = 0;
   out_8398366488676661931[46] = 0;
   out_8398366488676661931[47] = 0;
   out_8398366488676661931[48] = 0;
   out_8398366488676661931[49] = 0;
   out_8398366488676661931[50] = 0;
   out_8398366488676661931[51] = 0;
   out_8398366488676661931[52] = 0;
   out_8398366488676661931[53] = 0;
   out_8398366488676661931[54] = 0;
   out_8398366488676661931[55] = 0;
   out_8398366488676661931[56] = 0;
   out_8398366488676661931[57] = 1;
   out_8398366488676661931[58] = 0;
   out_8398366488676661931[59] = 0;
   out_8398366488676661931[60] = 0;
   out_8398366488676661931[61] = 0;
   out_8398366488676661931[62] = 0;
   out_8398366488676661931[63] = 0;
   out_8398366488676661931[64] = 0;
   out_8398366488676661931[65] = 0;
   out_8398366488676661931[66] = dt;
   out_8398366488676661931[67] = 0;
   out_8398366488676661931[68] = 0;
   out_8398366488676661931[69] = 0;
   out_8398366488676661931[70] = 0;
   out_8398366488676661931[71] = 0;
   out_8398366488676661931[72] = 0;
   out_8398366488676661931[73] = 0;
   out_8398366488676661931[74] = 0;
   out_8398366488676661931[75] = 0;
   out_8398366488676661931[76] = 1;
   out_8398366488676661931[77] = 0;
   out_8398366488676661931[78] = 0;
   out_8398366488676661931[79] = 0;
   out_8398366488676661931[80] = 0;
   out_8398366488676661931[81] = 0;
   out_8398366488676661931[82] = 0;
   out_8398366488676661931[83] = 0;
   out_8398366488676661931[84] = 0;
   out_8398366488676661931[85] = dt;
   out_8398366488676661931[86] = 0;
   out_8398366488676661931[87] = 0;
   out_8398366488676661931[88] = 0;
   out_8398366488676661931[89] = 0;
   out_8398366488676661931[90] = 0;
   out_8398366488676661931[91] = 0;
   out_8398366488676661931[92] = 0;
   out_8398366488676661931[93] = 0;
   out_8398366488676661931[94] = 0;
   out_8398366488676661931[95] = 1;
   out_8398366488676661931[96] = 0;
   out_8398366488676661931[97] = 0;
   out_8398366488676661931[98] = 0;
   out_8398366488676661931[99] = 0;
   out_8398366488676661931[100] = 0;
   out_8398366488676661931[101] = 0;
   out_8398366488676661931[102] = 0;
   out_8398366488676661931[103] = 0;
   out_8398366488676661931[104] = dt;
   out_8398366488676661931[105] = 0;
   out_8398366488676661931[106] = 0;
   out_8398366488676661931[107] = 0;
   out_8398366488676661931[108] = 0;
   out_8398366488676661931[109] = 0;
   out_8398366488676661931[110] = 0;
   out_8398366488676661931[111] = 0;
   out_8398366488676661931[112] = 0;
   out_8398366488676661931[113] = 0;
   out_8398366488676661931[114] = 1;
   out_8398366488676661931[115] = 0;
   out_8398366488676661931[116] = 0;
   out_8398366488676661931[117] = 0;
   out_8398366488676661931[118] = 0;
   out_8398366488676661931[119] = 0;
   out_8398366488676661931[120] = 0;
   out_8398366488676661931[121] = 0;
   out_8398366488676661931[122] = 0;
   out_8398366488676661931[123] = 0;
   out_8398366488676661931[124] = 0;
   out_8398366488676661931[125] = 0;
   out_8398366488676661931[126] = 0;
   out_8398366488676661931[127] = 0;
   out_8398366488676661931[128] = 0;
   out_8398366488676661931[129] = 0;
   out_8398366488676661931[130] = 0;
   out_8398366488676661931[131] = 0;
   out_8398366488676661931[132] = 0;
   out_8398366488676661931[133] = 1;
   out_8398366488676661931[134] = 0;
   out_8398366488676661931[135] = 0;
   out_8398366488676661931[136] = 0;
   out_8398366488676661931[137] = 0;
   out_8398366488676661931[138] = 0;
   out_8398366488676661931[139] = 0;
   out_8398366488676661931[140] = 0;
   out_8398366488676661931[141] = 0;
   out_8398366488676661931[142] = 0;
   out_8398366488676661931[143] = 0;
   out_8398366488676661931[144] = 0;
   out_8398366488676661931[145] = 0;
   out_8398366488676661931[146] = 0;
   out_8398366488676661931[147] = 0;
   out_8398366488676661931[148] = 0;
   out_8398366488676661931[149] = 0;
   out_8398366488676661931[150] = 0;
   out_8398366488676661931[151] = 0;
   out_8398366488676661931[152] = 1;
   out_8398366488676661931[153] = 0;
   out_8398366488676661931[154] = 0;
   out_8398366488676661931[155] = 0;
   out_8398366488676661931[156] = 0;
   out_8398366488676661931[157] = 0;
   out_8398366488676661931[158] = 0;
   out_8398366488676661931[159] = 0;
   out_8398366488676661931[160] = 0;
   out_8398366488676661931[161] = 0;
   out_8398366488676661931[162] = 0;
   out_8398366488676661931[163] = 0;
   out_8398366488676661931[164] = 0;
   out_8398366488676661931[165] = 0;
   out_8398366488676661931[166] = 0;
   out_8398366488676661931[167] = 0;
   out_8398366488676661931[168] = 0;
   out_8398366488676661931[169] = 0;
   out_8398366488676661931[170] = 0;
   out_8398366488676661931[171] = 1;
   out_8398366488676661931[172] = 0;
   out_8398366488676661931[173] = 0;
   out_8398366488676661931[174] = 0;
   out_8398366488676661931[175] = 0;
   out_8398366488676661931[176] = 0;
   out_8398366488676661931[177] = 0;
   out_8398366488676661931[178] = 0;
   out_8398366488676661931[179] = 0;
   out_8398366488676661931[180] = 0;
   out_8398366488676661931[181] = 0;
   out_8398366488676661931[182] = 0;
   out_8398366488676661931[183] = 0;
   out_8398366488676661931[184] = 0;
   out_8398366488676661931[185] = 0;
   out_8398366488676661931[186] = 0;
   out_8398366488676661931[187] = 0;
   out_8398366488676661931[188] = 0;
   out_8398366488676661931[189] = 0;
   out_8398366488676661931[190] = 1;
   out_8398366488676661931[191] = 0;
   out_8398366488676661931[192] = 0;
   out_8398366488676661931[193] = 0;
   out_8398366488676661931[194] = 0;
   out_8398366488676661931[195] = 0;
   out_8398366488676661931[196] = 0;
   out_8398366488676661931[197] = 0;
   out_8398366488676661931[198] = 0;
   out_8398366488676661931[199] = 0;
   out_8398366488676661931[200] = 0;
   out_8398366488676661931[201] = 0;
   out_8398366488676661931[202] = 0;
   out_8398366488676661931[203] = 0;
   out_8398366488676661931[204] = 0;
   out_8398366488676661931[205] = 0;
   out_8398366488676661931[206] = 0;
   out_8398366488676661931[207] = 0;
   out_8398366488676661931[208] = 0;
   out_8398366488676661931[209] = 1;
   out_8398366488676661931[210] = 0;
   out_8398366488676661931[211] = 0;
   out_8398366488676661931[212] = 0;
   out_8398366488676661931[213] = 0;
   out_8398366488676661931[214] = 0;
   out_8398366488676661931[215] = 0;
   out_8398366488676661931[216] = 0;
   out_8398366488676661931[217] = 0;
   out_8398366488676661931[218] = 0;
   out_8398366488676661931[219] = 0;
   out_8398366488676661931[220] = 0;
   out_8398366488676661931[221] = 0;
   out_8398366488676661931[222] = 0;
   out_8398366488676661931[223] = 0;
   out_8398366488676661931[224] = 0;
   out_8398366488676661931[225] = 0;
   out_8398366488676661931[226] = 0;
   out_8398366488676661931[227] = 0;
   out_8398366488676661931[228] = 1;
   out_8398366488676661931[229] = 0;
   out_8398366488676661931[230] = 0;
   out_8398366488676661931[231] = 0;
   out_8398366488676661931[232] = 0;
   out_8398366488676661931[233] = 0;
   out_8398366488676661931[234] = 0;
   out_8398366488676661931[235] = 0;
   out_8398366488676661931[236] = 0;
   out_8398366488676661931[237] = 0;
   out_8398366488676661931[238] = 0;
   out_8398366488676661931[239] = 0;
   out_8398366488676661931[240] = 0;
   out_8398366488676661931[241] = 0;
   out_8398366488676661931[242] = 0;
   out_8398366488676661931[243] = 0;
   out_8398366488676661931[244] = 0;
   out_8398366488676661931[245] = 0;
   out_8398366488676661931[246] = 0;
   out_8398366488676661931[247] = 1;
   out_8398366488676661931[248] = 0;
   out_8398366488676661931[249] = 0;
   out_8398366488676661931[250] = 0;
   out_8398366488676661931[251] = 0;
   out_8398366488676661931[252] = 0;
   out_8398366488676661931[253] = 0;
   out_8398366488676661931[254] = 0;
   out_8398366488676661931[255] = 0;
   out_8398366488676661931[256] = 0;
   out_8398366488676661931[257] = 0;
   out_8398366488676661931[258] = 0;
   out_8398366488676661931[259] = 0;
   out_8398366488676661931[260] = 0;
   out_8398366488676661931[261] = 0;
   out_8398366488676661931[262] = 0;
   out_8398366488676661931[263] = 0;
   out_8398366488676661931[264] = 0;
   out_8398366488676661931[265] = 0;
   out_8398366488676661931[266] = 1;
   out_8398366488676661931[267] = 0;
   out_8398366488676661931[268] = 0;
   out_8398366488676661931[269] = 0;
   out_8398366488676661931[270] = 0;
   out_8398366488676661931[271] = 0;
   out_8398366488676661931[272] = 0;
   out_8398366488676661931[273] = 0;
   out_8398366488676661931[274] = 0;
   out_8398366488676661931[275] = 0;
   out_8398366488676661931[276] = 0;
   out_8398366488676661931[277] = 0;
   out_8398366488676661931[278] = 0;
   out_8398366488676661931[279] = 0;
   out_8398366488676661931[280] = 0;
   out_8398366488676661931[281] = 0;
   out_8398366488676661931[282] = 0;
   out_8398366488676661931[283] = 0;
   out_8398366488676661931[284] = 0;
   out_8398366488676661931[285] = 1;
   out_8398366488676661931[286] = 0;
   out_8398366488676661931[287] = 0;
   out_8398366488676661931[288] = 0;
   out_8398366488676661931[289] = 0;
   out_8398366488676661931[290] = 0;
   out_8398366488676661931[291] = 0;
   out_8398366488676661931[292] = 0;
   out_8398366488676661931[293] = 0;
   out_8398366488676661931[294] = 0;
   out_8398366488676661931[295] = 0;
   out_8398366488676661931[296] = 0;
   out_8398366488676661931[297] = 0;
   out_8398366488676661931[298] = 0;
   out_8398366488676661931[299] = 0;
   out_8398366488676661931[300] = 0;
   out_8398366488676661931[301] = 0;
   out_8398366488676661931[302] = 0;
   out_8398366488676661931[303] = 0;
   out_8398366488676661931[304] = 1;
   out_8398366488676661931[305] = 0;
   out_8398366488676661931[306] = 0;
   out_8398366488676661931[307] = 0;
   out_8398366488676661931[308] = 0;
   out_8398366488676661931[309] = 0;
   out_8398366488676661931[310] = 0;
   out_8398366488676661931[311] = 0;
   out_8398366488676661931[312] = 0;
   out_8398366488676661931[313] = 0;
   out_8398366488676661931[314] = 0;
   out_8398366488676661931[315] = 0;
   out_8398366488676661931[316] = 0;
   out_8398366488676661931[317] = 0;
   out_8398366488676661931[318] = 0;
   out_8398366488676661931[319] = 0;
   out_8398366488676661931[320] = 0;
   out_8398366488676661931[321] = 0;
   out_8398366488676661931[322] = 0;
   out_8398366488676661931[323] = 1;
}
void h_4(double *state, double *unused, double *out_1122190748750831614) {
   out_1122190748750831614[0] = state[6] + state[9];
   out_1122190748750831614[1] = state[7] + state[10];
   out_1122190748750831614[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8215179929831897568) {
   out_8215179929831897568[0] = 0;
   out_8215179929831897568[1] = 0;
   out_8215179929831897568[2] = 0;
   out_8215179929831897568[3] = 0;
   out_8215179929831897568[4] = 0;
   out_8215179929831897568[5] = 0;
   out_8215179929831897568[6] = 1;
   out_8215179929831897568[7] = 0;
   out_8215179929831897568[8] = 0;
   out_8215179929831897568[9] = 1;
   out_8215179929831897568[10] = 0;
   out_8215179929831897568[11] = 0;
   out_8215179929831897568[12] = 0;
   out_8215179929831897568[13] = 0;
   out_8215179929831897568[14] = 0;
   out_8215179929831897568[15] = 0;
   out_8215179929831897568[16] = 0;
   out_8215179929831897568[17] = 0;
   out_8215179929831897568[18] = 0;
   out_8215179929831897568[19] = 0;
   out_8215179929831897568[20] = 0;
   out_8215179929831897568[21] = 0;
   out_8215179929831897568[22] = 0;
   out_8215179929831897568[23] = 0;
   out_8215179929831897568[24] = 0;
   out_8215179929831897568[25] = 1;
   out_8215179929831897568[26] = 0;
   out_8215179929831897568[27] = 0;
   out_8215179929831897568[28] = 1;
   out_8215179929831897568[29] = 0;
   out_8215179929831897568[30] = 0;
   out_8215179929831897568[31] = 0;
   out_8215179929831897568[32] = 0;
   out_8215179929831897568[33] = 0;
   out_8215179929831897568[34] = 0;
   out_8215179929831897568[35] = 0;
   out_8215179929831897568[36] = 0;
   out_8215179929831897568[37] = 0;
   out_8215179929831897568[38] = 0;
   out_8215179929831897568[39] = 0;
   out_8215179929831897568[40] = 0;
   out_8215179929831897568[41] = 0;
   out_8215179929831897568[42] = 0;
   out_8215179929831897568[43] = 0;
   out_8215179929831897568[44] = 1;
   out_8215179929831897568[45] = 0;
   out_8215179929831897568[46] = 0;
   out_8215179929831897568[47] = 1;
   out_8215179929831897568[48] = 0;
   out_8215179929831897568[49] = 0;
   out_8215179929831897568[50] = 0;
   out_8215179929831897568[51] = 0;
   out_8215179929831897568[52] = 0;
   out_8215179929831897568[53] = 0;
}
void h_10(double *state, double *unused, double *out_7012370241110020883) {
   out_7012370241110020883[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_7012370241110020883[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_7012370241110020883[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5502004992437324718) {
   out_5502004992437324718[0] = 0;
   out_5502004992437324718[1] = 9.8100000000000005*cos(state[1]);
   out_5502004992437324718[2] = 0;
   out_5502004992437324718[3] = 0;
   out_5502004992437324718[4] = -state[8];
   out_5502004992437324718[5] = state[7];
   out_5502004992437324718[6] = 0;
   out_5502004992437324718[7] = state[5];
   out_5502004992437324718[8] = -state[4];
   out_5502004992437324718[9] = 0;
   out_5502004992437324718[10] = 0;
   out_5502004992437324718[11] = 0;
   out_5502004992437324718[12] = 1;
   out_5502004992437324718[13] = 0;
   out_5502004992437324718[14] = 0;
   out_5502004992437324718[15] = 1;
   out_5502004992437324718[16] = 0;
   out_5502004992437324718[17] = 0;
   out_5502004992437324718[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5502004992437324718[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5502004992437324718[20] = 0;
   out_5502004992437324718[21] = state[8];
   out_5502004992437324718[22] = 0;
   out_5502004992437324718[23] = -state[6];
   out_5502004992437324718[24] = -state[5];
   out_5502004992437324718[25] = 0;
   out_5502004992437324718[26] = state[3];
   out_5502004992437324718[27] = 0;
   out_5502004992437324718[28] = 0;
   out_5502004992437324718[29] = 0;
   out_5502004992437324718[30] = 0;
   out_5502004992437324718[31] = 1;
   out_5502004992437324718[32] = 0;
   out_5502004992437324718[33] = 0;
   out_5502004992437324718[34] = 1;
   out_5502004992437324718[35] = 0;
   out_5502004992437324718[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5502004992437324718[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5502004992437324718[38] = 0;
   out_5502004992437324718[39] = -state[7];
   out_5502004992437324718[40] = state[6];
   out_5502004992437324718[41] = 0;
   out_5502004992437324718[42] = state[4];
   out_5502004992437324718[43] = -state[3];
   out_5502004992437324718[44] = 0;
   out_5502004992437324718[45] = 0;
   out_5502004992437324718[46] = 0;
   out_5502004992437324718[47] = 0;
   out_5502004992437324718[48] = 0;
   out_5502004992437324718[49] = 0;
   out_5502004992437324718[50] = 1;
   out_5502004992437324718[51] = 0;
   out_5502004992437324718[52] = 0;
   out_5502004992437324718[53] = 1;
}
void h_13(double *state, double *unused, double *out_354175508569982951) {
   out_354175508569982951[0] = state[3];
   out_354175508569982951[1] = state[4];
   out_354175508569982951[2] = state[5];
}
void H_13(double *state, double *unused, double *out_7019290318545321247) {
   out_7019290318545321247[0] = 0;
   out_7019290318545321247[1] = 0;
   out_7019290318545321247[2] = 0;
   out_7019290318545321247[3] = 1;
   out_7019290318545321247[4] = 0;
   out_7019290318545321247[5] = 0;
   out_7019290318545321247[6] = 0;
   out_7019290318545321247[7] = 0;
   out_7019290318545321247[8] = 0;
   out_7019290318545321247[9] = 0;
   out_7019290318545321247[10] = 0;
   out_7019290318545321247[11] = 0;
   out_7019290318545321247[12] = 0;
   out_7019290318545321247[13] = 0;
   out_7019290318545321247[14] = 0;
   out_7019290318545321247[15] = 0;
   out_7019290318545321247[16] = 0;
   out_7019290318545321247[17] = 0;
   out_7019290318545321247[18] = 0;
   out_7019290318545321247[19] = 0;
   out_7019290318545321247[20] = 0;
   out_7019290318545321247[21] = 0;
   out_7019290318545321247[22] = 1;
   out_7019290318545321247[23] = 0;
   out_7019290318545321247[24] = 0;
   out_7019290318545321247[25] = 0;
   out_7019290318545321247[26] = 0;
   out_7019290318545321247[27] = 0;
   out_7019290318545321247[28] = 0;
   out_7019290318545321247[29] = 0;
   out_7019290318545321247[30] = 0;
   out_7019290318545321247[31] = 0;
   out_7019290318545321247[32] = 0;
   out_7019290318545321247[33] = 0;
   out_7019290318545321247[34] = 0;
   out_7019290318545321247[35] = 0;
   out_7019290318545321247[36] = 0;
   out_7019290318545321247[37] = 0;
   out_7019290318545321247[38] = 0;
   out_7019290318545321247[39] = 0;
   out_7019290318545321247[40] = 0;
   out_7019290318545321247[41] = 1;
   out_7019290318545321247[42] = 0;
   out_7019290318545321247[43] = 0;
   out_7019290318545321247[44] = 0;
   out_7019290318545321247[45] = 0;
   out_7019290318545321247[46] = 0;
   out_7019290318545321247[47] = 0;
   out_7019290318545321247[48] = 0;
   out_7019290318545321247[49] = 0;
   out_7019290318545321247[50] = 0;
   out_7019290318545321247[51] = 0;
   out_7019290318545321247[52] = 0;
   out_7019290318545321247[53] = 0;
}
void h_14(double *state, double *unused, double *out_8860411994425898009) {
   out_8860411994425898009[0] = state[6];
   out_8860411994425898009[1] = state[7];
   out_8860411994425898009[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6268323287538169519) {
   out_6268323287538169519[0] = 0;
   out_6268323287538169519[1] = 0;
   out_6268323287538169519[2] = 0;
   out_6268323287538169519[3] = 0;
   out_6268323287538169519[4] = 0;
   out_6268323287538169519[5] = 0;
   out_6268323287538169519[6] = 1;
   out_6268323287538169519[7] = 0;
   out_6268323287538169519[8] = 0;
   out_6268323287538169519[9] = 0;
   out_6268323287538169519[10] = 0;
   out_6268323287538169519[11] = 0;
   out_6268323287538169519[12] = 0;
   out_6268323287538169519[13] = 0;
   out_6268323287538169519[14] = 0;
   out_6268323287538169519[15] = 0;
   out_6268323287538169519[16] = 0;
   out_6268323287538169519[17] = 0;
   out_6268323287538169519[18] = 0;
   out_6268323287538169519[19] = 0;
   out_6268323287538169519[20] = 0;
   out_6268323287538169519[21] = 0;
   out_6268323287538169519[22] = 0;
   out_6268323287538169519[23] = 0;
   out_6268323287538169519[24] = 0;
   out_6268323287538169519[25] = 1;
   out_6268323287538169519[26] = 0;
   out_6268323287538169519[27] = 0;
   out_6268323287538169519[28] = 0;
   out_6268323287538169519[29] = 0;
   out_6268323287538169519[30] = 0;
   out_6268323287538169519[31] = 0;
   out_6268323287538169519[32] = 0;
   out_6268323287538169519[33] = 0;
   out_6268323287538169519[34] = 0;
   out_6268323287538169519[35] = 0;
   out_6268323287538169519[36] = 0;
   out_6268323287538169519[37] = 0;
   out_6268323287538169519[38] = 0;
   out_6268323287538169519[39] = 0;
   out_6268323287538169519[40] = 0;
   out_6268323287538169519[41] = 0;
   out_6268323287538169519[42] = 0;
   out_6268323287538169519[43] = 0;
   out_6268323287538169519[44] = 1;
   out_6268323287538169519[45] = 0;
   out_6268323287538169519[46] = 0;
   out_6268323287538169519[47] = 0;
   out_6268323287538169519[48] = 0;
   out_6268323287538169519[49] = 0;
   out_6268323287538169519[50] = 0;
   out_6268323287538169519[51] = 0;
   out_6268323287538169519[52] = 0;
   out_6268323287538169519[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_455832145760308814) {
  err_fun(nom_x, delta_x, out_455832145760308814);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6833442360289991349) {
  inv_err_fun(nom_x, true_x, out_6833442360289991349);
}
void pose_H_mod_fun(double *state, double *out_4443372719987989996) {
  H_mod_fun(state, out_4443372719987989996);
}
void pose_f_fun(double *state, double dt, double *out_7271509597229759153) {
  f_fun(state,  dt, out_7271509597229759153);
}
void pose_F_fun(double *state, double dt, double *out_8398366488676661931) {
  F_fun(state,  dt, out_8398366488676661931);
}
void pose_h_4(double *state, double *unused, double *out_1122190748750831614) {
  h_4(state, unused, out_1122190748750831614);
}
void pose_H_4(double *state, double *unused, double *out_8215179929831897568) {
  H_4(state, unused, out_8215179929831897568);
}
void pose_h_10(double *state, double *unused, double *out_7012370241110020883) {
  h_10(state, unused, out_7012370241110020883);
}
void pose_H_10(double *state, double *unused, double *out_5502004992437324718) {
  H_10(state, unused, out_5502004992437324718);
}
void pose_h_13(double *state, double *unused, double *out_354175508569982951) {
  h_13(state, unused, out_354175508569982951);
}
void pose_H_13(double *state, double *unused, double *out_7019290318545321247) {
  H_13(state, unused, out_7019290318545321247);
}
void pose_h_14(double *state, double *unused, double *out_8860411994425898009) {
  h_14(state, unused, out_8860411994425898009);
}
void pose_H_14(double *state, double *unused, double *out_6268323287538169519) {
  H_14(state, unused, out_6268323287538169519);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
