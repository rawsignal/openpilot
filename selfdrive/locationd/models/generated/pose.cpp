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
void err_fun(double *nom_x, double *delta_x, double *out_5959269884630241271) {
   out_5959269884630241271[0] = delta_x[0] + nom_x[0];
   out_5959269884630241271[1] = delta_x[1] + nom_x[1];
   out_5959269884630241271[2] = delta_x[2] + nom_x[2];
   out_5959269884630241271[3] = delta_x[3] + nom_x[3];
   out_5959269884630241271[4] = delta_x[4] + nom_x[4];
   out_5959269884630241271[5] = delta_x[5] + nom_x[5];
   out_5959269884630241271[6] = delta_x[6] + nom_x[6];
   out_5959269884630241271[7] = delta_x[7] + nom_x[7];
   out_5959269884630241271[8] = delta_x[8] + nom_x[8];
   out_5959269884630241271[9] = delta_x[9] + nom_x[9];
   out_5959269884630241271[10] = delta_x[10] + nom_x[10];
   out_5959269884630241271[11] = delta_x[11] + nom_x[11];
   out_5959269884630241271[12] = delta_x[12] + nom_x[12];
   out_5959269884630241271[13] = delta_x[13] + nom_x[13];
   out_5959269884630241271[14] = delta_x[14] + nom_x[14];
   out_5959269884630241271[15] = delta_x[15] + nom_x[15];
   out_5959269884630241271[16] = delta_x[16] + nom_x[16];
   out_5959269884630241271[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7281047554654548610) {
   out_7281047554654548610[0] = -nom_x[0] + true_x[0];
   out_7281047554654548610[1] = -nom_x[1] + true_x[1];
   out_7281047554654548610[2] = -nom_x[2] + true_x[2];
   out_7281047554654548610[3] = -nom_x[3] + true_x[3];
   out_7281047554654548610[4] = -nom_x[4] + true_x[4];
   out_7281047554654548610[5] = -nom_x[5] + true_x[5];
   out_7281047554654548610[6] = -nom_x[6] + true_x[6];
   out_7281047554654548610[7] = -nom_x[7] + true_x[7];
   out_7281047554654548610[8] = -nom_x[8] + true_x[8];
   out_7281047554654548610[9] = -nom_x[9] + true_x[9];
   out_7281047554654548610[10] = -nom_x[10] + true_x[10];
   out_7281047554654548610[11] = -nom_x[11] + true_x[11];
   out_7281047554654548610[12] = -nom_x[12] + true_x[12];
   out_7281047554654548610[13] = -nom_x[13] + true_x[13];
   out_7281047554654548610[14] = -nom_x[14] + true_x[14];
   out_7281047554654548610[15] = -nom_x[15] + true_x[15];
   out_7281047554654548610[16] = -nom_x[16] + true_x[16];
   out_7281047554654548610[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6722182246202922039) {
   out_6722182246202922039[0] = 1.0;
   out_6722182246202922039[1] = 0.0;
   out_6722182246202922039[2] = 0.0;
   out_6722182246202922039[3] = 0.0;
   out_6722182246202922039[4] = 0.0;
   out_6722182246202922039[5] = 0.0;
   out_6722182246202922039[6] = 0.0;
   out_6722182246202922039[7] = 0.0;
   out_6722182246202922039[8] = 0.0;
   out_6722182246202922039[9] = 0.0;
   out_6722182246202922039[10] = 0.0;
   out_6722182246202922039[11] = 0.0;
   out_6722182246202922039[12] = 0.0;
   out_6722182246202922039[13] = 0.0;
   out_6722182246202922039[14] = 0.0;
   out_6722182246202922039[15] = 0.0;
   out_6722182246202922039[16] = 0.0;
   out_6722182246202922039[17] = 0.0;
   out_6722182246202922039[18] = 0.0;
   out_6722182246202922039[19] = 1.0;
   out_6722182246202922039[20] = 0.0;
   out_6722182246202922039[21] = 0.0;
   out_6722182246202922039[22] = 0.0;
   out_6722182246202922039[23] = 0.0;
   out_6722182246202922039[24] = 0.0;
   out_6722182246202922039[25] = 0.0;
   out_6722182246202922039[26] = 0.0;
   out_6722182246202922039[27] = 0.0;
   out_6722182246202922039[28] = 0.0;
   out_6722182246202922039[29] = 0.0;
   out_6722182246202922039[30] = 0.0;
   out_6722182246202922039[31] = 0.0;
   out_6722182246202922039[32] = 0.0;
   out_6722182246202922039[33] = 0.0;
   out_6722182246202922039[34] = 0.0;
   out_6722182246202922039[35] = 0.0;
   out_6722182246202922039[36] = 0.0;
   out_6722182246202922039[37] = 0.0;
   out_6722182246202922039[38] = 1.0;
   out_6722182246202922039[39] = 0.0;
   out_6722182246202922039[40] = 0.0;
   out_6722182246202922039[41] = 0.0;
   out_6722182246202922039[42] = 0.0;
   out_6722182246202922039[43] = 0.0;
   out_6722182246202922039[44] = 0.0;
   out_6722182246202922039[45] = 0.0;
   out_6722182246202922039[46] = 0.0;
   out_6722182246202922039[47] = 0.0;
   out_6722182246202922039[48] = 0.0;
   out_6722182246202922039[49] = 0.0;
   out_6722182246202922039[50] = 0.0;
   out_6722182246202922039[51] = 0.0;
   out_6722182246202922039[52] = 0.0;
   out_6722182246202922039[53] = 0.0;
   out_6722182246202922039[54] = 0.0;
   out_6722182246202922039[55] = 0.0;
   out_6722182246202922039[56] = 0.0;
   out_6722182246202922039[57] = 1.0;
   out_6722182246202922039[58] = 0.0;
   out_6722182246202922039[59] = 0.0;
   out_6722182246202922039[60] = 0.0;
   out_6722182246202922039[61] = 0.0;
   out_6722182246202922039[62] = 0.0;
   out_6722182246202922039[63] = 0.0;
   out_6722182246202922039[64] = 0.0;
   out_6722182246202922039[65] = 0.0;
   out_6722182246202922039[66] = 0.0;
   out_6722182246202922039[67] = 0.0;
   out_6722182246202922039[68] = 0.0;
   out_6722182246202922039[69] = 0.0;
   out_6722182246202922039[70] = 0.0;
   out_6722182246202922039[71] = 0.0;
   out_6722182246202922039[72] = 0.0;
   out_6722182246202922039[73] = 0.0;
   out_6722182246202922039[74] = 0.0;
   out_6722182246202922039[75] = 0.0;
   out_6722182246202922039[76] = 1.0;
   out_6722182246202922039[77] = 0.0;
   out_6722182246202922039[78] = 0.0;
   out_6722182246202922039[79] = 0.0;
   out_6722182246202922039[80] = 0.0;
   out_6722182246202922039[81] = 0.0;
   out_6722182246202922039[82] = 0.0;
   out_6722182246202922039[83] = 0.0;
   out_6722182246202922039[84] = 0.0;
   out_6722182246202922039[85] = 0.0;
   out_6722182246202922039[86] = 0.0;
   out_6722182246202922039[87] = 0.0;
   out_6722182246202922039[88] = 0.0;
   out_6722182246202922039[89] = 0.0;
   out_6722182246202922039[90] = 0.0;
   out_6722182246202922039[91] = 0.0;
   out_6722182246202922039[92] = 0.0;
   out_6722182246202922039[93] = 0.0;
   out_6722182246202922039[94] = 0.0;
   out_6722182246202922039[95] = 1.0;
   out_6722182246202922039[96] = 0.0;
   out_6722182246202922039[97] = 0.0;
   out_6722182246202922039[98] = 0.0;
   out_6722182246202922039[99] = 0.0;
   out_6722182246202922039[100] = 0.0;
   out_6722182246202922039[101] = 0.0;
   out_6722182246202922039[102] = 0.0;
   out_6722182246202922039[103] = 0.0;
   out_6722182246202922039[104] = 0.0;
   out_6722182246202922039[105] = 0.0;
   out_6722182246202922039[106] = 0.0;
   out_6722182246202922039[107] = 0.0;
   out_6722182246202922039[108] = 0.0;
   out_6722182246202922039[109] = 0.0;
   out_6722182246202922039[110] = 0.0;
   out_6722182246202922039[111] = 0.0;
   out_6722182246202922039[112] = 0.0;
   out_6722182246202922039[113] = 0.0;
   out_6722182246202922039[114] = 1.0;
   out_6722182246202922039[115] = 0.0;
   out_6722182246202922039[116] = 0.0;
   out_6722182246202922039[117] = 0.0;
   out_6722182246202922039[118] = 0.0;
   out_6722182246202922039[119] = 0.0;
   out_6722182246202922039[120] = 0.0;
   out_6722182246202922039[121] = 0.0;
   out_6722182246202922039[122] = 0.0;
   out_6722182246202922039[123] = 0.0;
   out_6722182246202922039[124] = 0.0;
   out_6722182246202922039[125] = 0.0;
   out_6722182246202922039[126] = 0.0;
   out_6722182246202922039[127] = 0.0;
   out_6722182246202922039[128] = 0.0;
   out_6722182246202922039[129] = 0.0;
   out_6722182246202922039[130] = 0.0;
   out_6722182246202922039[131] = 0.0;
   out_6722182246202922039[132] = 0.0;
   out_6722182246202922039[133] = 1.0;
   out_6722182246202922039[134] = 0.0;
   out_6722182246202922039[135] = 0.0;
   out_6722182246202922039[136] = 0.0;
   out_6722182246202922039[137] = 0.0;
   out_6722182246202922039[138] = 0.0;
   out_6722182246202922039[139] = 0.0;
   out_6722182246202922039[140] = 0.0;
   out_6722182246202922039[141] = 0.0;
   out_6722182246202922039[142] = 0.0;
   out_6722182246202922039[143] = 0.0;
   out_6722182246202922039[144] = 0.0;
   out_6722182246202922039[145] = 0.0;
   out_6722182246202922039[146] = 0.0;
   out_6722182246202922039[147] = 0.0;
   out_6722182246202922039[148] = 0.0;
   out_6722182246202922039[149] = 0.0;
   out_6722182246202922039[150] = 0.0;
   out_6722182246202922039[151] = 0.0;
   out_6722182246202922039[152] = 1.0;
   out_6722182246202922039[153] = 0.0;
   out_6722182246202922039[154] = 0.0;
   out_6722182246202922039[155] = 0.0;
   out_6722182246202922039[156] = 0.0;
   out_6722182246202922039[157] = 0.0;
   out_6722182246202922039[158] = 0.0;
   out_6722182246202922039[159] = 0.0;
   out_6722182246202922039[160] = 0.0;
   out_6722182246202922039[161] = 0.0;
   out_6722182246202922039[162] = 0.0;
   out_6722182246202922039[163] = 0.0;
   out_6722182246202922039[164] = 0.0;
   out_6722182246202922039[165] = 0.0;
   out_6722182246202922039[166] = 0.0;
   out_6722182246202922039[167] = 0.0;
   out_6722182246202922039[168] = 0.0;
   out_6722182246202922039[169] = 0.0;
   out_6722182246202922039[170] = 0.0;
   out_6722182246202922039[171] = 1.0;
   out_6722182246202922039[172] = 0.0;
   out_6722182246202922039[173] = 0.0;
   out_6722182246202922039[174] = 0.0;
   out_6722182246202922039[175] = 0.0;
   out_6722182246202922039[176] = 0.0;
   out_6722182246202922039[177] = 0.0;
   out_6722182246202922039[178] = 0.0;
   out_6722182246202922039[179] = 0.0;
   out_6722182246202922039[180] = 0.0;
   out_6722182246202922039[181] = 0.0;
   out_6722182246202922039[182] = 0.0;
   out_6722182246202922039[183] = 0.0;
   out_6722182246202922039[184] = 0.0;
   out_6722182246202922039[185] = 0.0;
   out_6722182246202922039[186] = 0.0;
   out_6722182246202922039[187] = 0.0;
   out_6722182246202922039[188] = 0.0;
   out_6722182246202922039[189] = 0.0;
   out_6722182246202922039[190] = 1.0;
   out_6722182246202922039[191] = 0.0;
   out_6722182246202922039[192] = 0.0;
   out_6722182246202922039[193] = 0.0;
   out_6722182246202922039[194] = 0.0;
   out_6722182246202922039[195] = 0.0;
   out_6722182246202922039[196] = 0.0;
   out_6722182246202922039[197] = 0.0;
   out_6722182246202922039[198] = 0.0;
   out_6722182246202922039[199] = 0.0;
   out_6722182246202922039[200] = 0.0;
   out_6722182246202922039[201] = 0.0;
   out_6722182246202922039[202] = 0.0;
   out_6722182246202922039[203] = 0.0;
   out_6722182246202922039[204] = 0.0;
   out_6722182246202922039[205] = 0.0;
   out_6722182246202922039[206] = 0.0;
   out_6722182246202922039[207] = 0.0;
   out_6722182246202922039[208] = 0.0;
   out_6722182246202922039[209] = 1.0;
   out_6722182246202922039[210] = 0.0;
   out_6722182246202922039[211] = 0.0;
   out_6722182246202922039[212] = 0.0;
   out_6722182246202922039[213] = 0.0;
   out_6722182246202922039[214] = 0.0;
   out_6722182246202922039[215] = 0.0;
   out_6722182246202922039[216] = 0.0;
   out_6722182246202922039[217] = 0.0;
   out_6722182246202922039[218] = 0.0;
   out_6722182246202922039[219] = 0.0;
   out_6722182246202922039[220] = 0.0;
   out_6722182246202922039[221] = 0.0;
   out_6722182246202922039[222] = 0.0;
   out_6722182246202922039[223] = 0.0;
   out_6722182246202922039[224] = 0.0;
   out_6722182246202922039[225] = 0.0;
   out_6722182246202922039[226] = 0.0;
   out_6722182246202922039[227] = 0.0;
   out_6722182246202922039[228] = 1.0;
   out_6722182246202922039[229] = 0.0;
   out_6722182246202922039[230] = 0.0;
   out_6722182246202922039[231] = 0.0;
   out_6722182246202922039[232] = 0.0;
   out_6722182246202922039[233] = 0.0;
   out_6722182246202922039[234] = 0.0;
   out_6722182246202922039[235] = 0.0;
   out_6722182246202922039[236] = 0.0;
   out_6722182246202922039[237] = 0.0;
   out_6722182246202922039[238] = 0.0;
   out_6722182246202922039[239] = 0.0;
   out_6722182246202922039[240] = 0.0;
   out_6722182246202922039[241] = 0.0;
   out_6722182246202922039[242] = 0.0;
   out_6722182246202922039[243] = 0.0;
   out_6722182246202922039[244] = 0.0;
   out_6722182246202922039[245] = 0.0;
   out_6722182246202922039[246] = 0.0;
   out_6722182246202922039[247] = 1.0;
   out_6722182246202922039[248] = 0.0;
   out_6722182246202922039[249] = 0.0;
   out_6722182246202922039[250] = 0.0;
   out_6722182246202922039[251] = 0.0;
   out_6722182246202922039[252] = 0.0;
   out_6722182246202922039[253] = 0.0;
   out_6722182246202922039[254] = 0.0;
   out_6722182246202922039[255] = 0.0;
   out_6722182246202922039[256] = 0.0;
   out_6722182246202922039[257] = 0.0;
   out_6722182246202922039[258] = 0.0;
   out_6722182246202922039[259] = 0.0;
   out_6722182246202922039[260] = 0.0;
   out_6722182246202922039[261] = 0.0;
   out_6722182246202922039[262] = 0.0;
   out_6722182246202922039[263] = 0.0;
   out_6722182246202922039[264] = 0.0;
   out_6722182246202922039[265] = 0.0;
   out_6722182246202922039[266] = 1.0;
   out_6722182246202922039[267] = 0.0;
   out_6722182246202922039[268] = 0.0;
   out_6722182246202922039[269] = 0.0;
   out_6722182246202922039[270] = 0.0;
   out_6722182246202922039[271] = 0.0;
   out_6722182246202922039[272] = 0.0;
   out_6722182246202922039[273] = 0.0;
   out_6722182246202922039[274] = 0.0;
   out_6722182246202922039[275] = 0.0;
   out_6722182246202922039[276] = 0.0;
   out_6722182246202922039[277] = 0.0;
   out_6722182246202922039[278] = 0.0;
   out_6722182246202922039[279] = 0.0;
   out_6722182246202922039[280] = 0.0;
   out_6722182246202922039[281] = 0.0;
   out_6722182246202922039[282] = 0.0;
   out_6722182246202922039[283] = 0.0;
   out_6722182246202922039[284] = 0.0;
   out_6722182246202922039[285] = 1.0;
   out_6722182246202922039[286] = 0.0;
   out_6722182246202922039[287] = 0.0;
   out_6722182246202922039[288] = 0.0;
   out_6722182246202922039[289] = 0.0;
   out_6722182246202922039[290] = 0.0;
   out_6722182246202922039[291] = 0.0;
   out_6722182246202922039[292] = 0.0;
   out_6722182246202922039[293] = 0.0;
   out_6722182246202922039[294] = 0.0;
   out_6722182246202922039[295] = 0.0;
   out_6722182246202922039[296] = 0.0;
   out_6722182246202922039[297] = 0.0;
   out_6722182246202922039[298] = 0.0;
   out_6722182246202922039[299] = 0.0;
   out_6722182246202922039[300] = 0.0;
   out_6722182246202922039[301] = 0.0;
   out_6722182246202922039[302] = 0.0;
   out_6722182246202922039[303] = 0.0;
   out_6722182246202922039[304] = 1.0;
   out_6722182246202922039[305] = 0.0;
   out_6722182246202922039[306] = 0.0;
   out_6722182246202922039[307] = 0.0;
   out_6722182246202922039[308] = 0.0;
   out_6722182246202922039[309] = 0.0;
   out_6722182246202922039[310] = 0.0;
   out_6722182246202922039[311] = 0.0;
   out_6722182246202922039[312] = 0.0;
   out_6722182246202922039[313] = 0.0;
   out_6722182246202922039[314] = 0.0;
   out_6722182246202922039[315] = 0.0;
   out_6722182246202922039[316] = 0.0;
   out_6722182246202922039[317] = 0.0;
   out_6722182246202922039[318] = 0.0;
   out_6722182246202922039[319] = 0.0;
   out_6722182246202922039[320] = 0.0;
   out_6722182246202922039[321] = 0.0;
   out_6722182246202922039[322] = 0.0;
   out_6722182246202922039[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5905758100805214049) {
   out_5905758100805214049[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5905758100805214049[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5905758100805214049[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5905758100805214049[3] = dt*state[12] + state[3];
   out_5905758100805214049[4] = dt*state[13] + state[4];
   out_5905758100805214049[5] = dt*state[14] + state[5];
   out_5905758100805214049[6] = state[6];
   out_5905758100805214049[7] = state[7];
   out_5905758100805214049[8] = state[8];
   out_5905758100805214049[9] = state[9];
   out_5905758100805214049[10] = state[10];
   out_5905758100805214049[11] = state[11];
   out_5905758100805214049[12] = state[12];
   out_5905758100805214049[13] = state[13];
   out_5905758100805214049[14] = state[14];
   out_5905758100805214049[15] = state[15];
   out_5905758100805214049[16] = state[16];
   out_5905758100805214049[17] = state[17];
}
void F_fun(double *state, double dt, double *out_8803043244110166487) {
   out_8803043244110166487[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8803043244110166487[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8803043244110166487[2] = 0;
   out_8803043244110166487[3] = 0;
   out_8803043244110166487[4] = 0;
   out_8803043244110166487[5] = 0;
   out_8803043244110166487[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8803043244110166487[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8803043244110166487[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8803043244110166487[9] = 0;
   out_8803043244110166487[10] = 0;
   out_8803043244110166487[11] = 0;
   out_8803043244110166487[12] = 0;
   out_8803043244110166487[13] = 0;
   out_8803043244110166487[14] = 0;
   out_8803043244110166487[15] = 0;
   out_8803043244110166487[16] = 0;
   out_8803043244110166487[17] = 0;
   out_8803043244110166487[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8803043244110166487[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8803043244110166487[20] = 0;
   out_8803043244110166487[21] = 0;
   out_8803043244110166487[22] = 0;
   out_8803043244110166487[23] = 0;
   out_8803043244110166487[24] = 0;
   out_8803043244110166487[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8803043244110166487[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8803043244110166487[27] = 0;
   out_8803043244110166487[28] = 0;
   out_8803043244110166487[29] = 0;
   out_8803043244110166487[30] = 0;
   out_8803043244110166487[31] = 0;
   out_8803043244110166487[32] = 0;
   out_8803043244110166487[33] = 0;
   out_8803043244110166487[34] = 0;
   out_8803043244110166487[35] = 0;
   out_8803043244110166487[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8803043244110166487[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8803043244110166487[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8803043244110166487[39] = 0;
   out_8803043244110166487[40] = 0;
   out_8803043244110166487[41] = 0;
   out_8803043244110166487[42] = 0;
   out_8803043244110166487[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8803043244110166487[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8803043244110166487[45] = 0;
   out_8803043244110166487[46] = 0;
   out_8803043244110166487[47] = 0;
   out_8803043244110166487[48] = 0;
   out_8803043244110166487[49] = 0;
   out_8803043244110166487[50] = 0;
   out_8803043244110166487[51] = 0;
   out_8803043244110166487[52] = 0;
   out_8803043244110166487[53] = 0;
   out_8803043244110166487[54] = 0;
   out_8803043244110166487[55] = 0;
   out_8803043244110166487[56] = 0;
   out_8803043244110166487[57] = 1;
   out_8803043244110166487[58] = 0;
   out_8803043244110166487[59] = 0;
   out_8803043244110166487[60] = 0;
   out_8803043244110166487[61] = 0;
   out_8803043244110166487[62] = 0;
   out_8803043244110166487[63] = 0;
   out_8803043244110166487[64] = 0;
   out_8803043244110166487[65] = 0;
   out_8803043244110166487[66] = dt;
   out_8803043244110166487[67] = 0;
   out_8803043244110166487[68] = 0;
   out_8803043244110166487[69] = 0;
   out_8803043244110166487[70] = 0;
   out_8803043244110166487[71] = 0;
   out_8803043244110166487[72] = 0;
   out_8803043244110166487[73] = 0;
   out_8803043244110166487[74] = 0;
   out_8803043244110166487[75] = 0;
   out_8803043244110166487[76] = 1;
   out_8803043244110166487[77] = 0;
   out_8803043244110166487[78] = 0;
   out_8803043244110166487[79] = 0;
   out_8803043244110166487[80] = 0;
   out_8803043244110166487[81] = 0;
   out_8803043244110166487[82] = 0;
   out_8803043244110166487[83] = 0;
   out_8803043244110166487[84] = 0;
   out_8803043244110166487[85] = dt;
   out_8803043244110166487[86] = 0;
   out_8803043244110166487[87] = 0;
   out_8803043244110166487[88] = 0;
   out_8803043244110166487[89] = 0;
   out_8803043244110166487[90] = 0;
   out_8803043244110166487[91] = 0;
   out_8803043244110166487[92] = 0;
   out_8803043244110166487[93] = 0;
   out_8803043244110166487[94] = 0;
   out_8803043244110166487[95] = 1;
   out_8803043244110166487[96] = 0;
   out_8803043244110166487[97] = 0;
   out_8803043244110166487[98] = 0;
   out_8803043244110166487[99] = 0;
   out_8803043244110166487[100] = 0;
   out_8803043244110166487[101] = 0;
   out_8803043244110166487[102] = 0;
   out_8803043244110166487[103] = 0;
   out_8803043244110166487[104] = dt;
   out_8803043244110166487[105] = 0;
   out_8803043244110166487[106] = 0;
   out_8803043244110166487[107] = 0;
   out_8803043244110166487[108] = 0;
   out_8803043244110166487[109] = 0;
   out_8803043244110166487[110] = 0;
   out_8803043244110166487[111] = 0;
   out_8803043244110166487[112] = 0;
   out_8803043244110166487[113] = 0;
   out_8803043244110166487[114] = 1;
   out_8803043244110166487[115] = 0;
   out_8803043244110166487[116] = 0;
   out_8803043244110166487[117] = 0;
   out_8803043244110166487[118] = 0;
   out_8803043244110166487[119] = 0;
   out_8803043244110166487[120] = 0;
   out_8803043244110166487[121] = 0;
   out_8803043244110166487[122] = 0;
   out_8803043244110166487[123] = 0;
   out_8803043244110166487[124] = 0;
   out_8803043244110166487[125] = 0;
   out_8803043244110166487[126] = 0;
   out_8803043244110166487[127] = 0;
   out_8803043244110166487[128] = 0;
   out_8803043244110166487[129] = 0;
   out_8803043244110166487[130] = 0;
   out_8803043244110166487[131] = 0;
   out_8803043244110166487[132] = 0;
   out_8803043244110166487[133] = 1;
   out_8803043244110166487[134] = 0;
   out_8803043244110166487[135] = 0;
   out_8803043244110166487[136] = 0;
   out_8803043244110166487[137] = 0;
   out_8803043244110166487[138] = 0;
   out_8803043244110166487[139] = 0;
   out_8803043244110166487[140] = 0;
   out_8803043244110166487[141] = 0;
   out_8803043244110166487[142] = 0;
   out_8803043244110166487[143] = 0;
   out_8803043244110166487[144] = 0;
   out_8803043244110166487[145] = 0;
   out_8803043244110166487[146] = 0;
   out_8803043244110166487[147] = 0;
   out_8803043244110166487[148] = 0;
   out_8803043244110166487[149] = 0;
   out_8803043244110166487[150] = 0;
   out_8803043244110166487[151] = 0;
   out_8803043244110166487[152] = 1;
   out_8803043244110166487[153] = 0;
   out_8803043244110166487[154] = 0;
   out_8803043244110166487[155] = 0;
   out_8803043244110166487[156] = 0;
   out_8803043244110166487[157] = 0;
   out_8803043244110166487[158] = 0;
   out_8803043244110166487[159] = 0;
   out_8803043244110166487[160] = 0;
   out_8803043244110166487[161] = 0;
   out_8803043244110166487[162] = 0;
   out_8803043244110166487[163] = 0;
   out_8803043244110166487[164] = 0;
   out_8803043244110166487[165] = 0;
   out_8803043244110166487[166] = 0;
   out_8803043244110166487[167] = 0;
   out_8803043244110166487[168] = 0;
   out_8803043244110166487[169] = 0;
   out_8803043244110166487[170] = 0;
   out_8803043244110166487[171] = 1;
   out_8803043244110166487[172] = 0;
   out_8803043244110166487[173] = 0;
   out_8803043244110166487[174] = 0;
   out_8803043244110166487[175] = 0;
   out_8803043244110166487[176] = 0;
   out_8803043244110166487[177] = 0;
   out_8803043244110166487[178] = 0;
   out_8803043244110166487[179] = 0;
   out_8803043244110166487[180] = 0;
   out_8803043244110166487[181] = 0;
   out_8803043244110166487[182] = 0;
   out_8803043244110166487[183] = 0;
   out_8803043244110166487[184] = 0;
   out_8803043244110166487[185] = 0;
   out_8803043244110166487[186] = 0;
   out_8803043244110166487[187] = 0;
   out_8803043244110166487[188] = 0;
   out_8803043244110166487[189] = 0;
   out_8803043244110166487[190] = 1;
   out_8803043244110166487[191] = 0;
   out_8803043244110166487[192] = 0;
   out_8803043244110166487[193] = 0;
   out_8803043244110166487[194] = 0;
   out_8803043244110166487[195] = 0;
   out_8803043244110166487[196] = 0;
   out_8803043244110166487[197] = 0;
   out_8803043244110166487[198] = 0;
   out_8803043244110166487[199] = 0;
   out_8803043244110166487[200] = 0;
   out_8803043244110166487[201] = 0;
   out_8803043244110166487[202] = 0;
   out_8803043244110166487[203] = 0;
   out_8803043244110166487[204] = 0;
   out_8803043244110166487[205] = 0;
   out_8803043244110166487[206] = 0;
   out_8803043244110166487[207] = 0;
   out_8803043244110166487[208] = 0;
   out_8803043244110166487[209] = 1;
   out_8803043244110166487[210] = 0;
   out_8803043244110166487[211] = 0;
   out_8803043244110166487[212] = 0;
   out_8803043244110166487[213] = 0;
   out_8803043244110166487[214] = 0;
   out_8803043244110166487[215] = 0;
   out_8803043244110166487[216] = 0;
   out_8803043244110166487[217] = 0;
   out_8803043244110166487[218] = 0;
   out_8803043244110166487[219] = 0;
   out_8803043244110166487[220] = 0;
   out_8803043244110166487[221] = 0;
   out_8803043244110166487[222] = 0;
   out_8803043244110166487[223] = 0;
   out_8803043244110166487[224] = 0;
   out_8803043244110166487[225] = 0;
   out_8803043244110166487[226] = 0;
   out_8803043244110166487[227] = 0;
   out_8803043244110166487[228] = 1;
   out_8803043244110166487[229] = 0;
   out_8803043244110166487[230] = 0;
   out_8803043244110166487[231] = 0;
   out_8803043244110166487[232] = 0;
   out_8803043244110166487[233] = 0;
   out_8803043244110166487[234] = 0;
   out_8803043244110166487[235] = 0;
   out_8803043244110166487[236] = 0;
   out_8803043244110166487[237] = 0;
   out_8803043244110166487[238] = 0;
   out_8803043244110166487[239] = 0;
   out_8803043244110166487[240] = 0;
   out_8803043244110166487[241] = 0;
   out_8803043244110166487[242] = 0;
   out_8803043244110166487[243] = 0;
   out_8803043244110166487[244] = 0;
   out_8803043244110166487[245] = 0;
   out_8803043244110166487[246] = 0;
   out_8803043244110166487[247] = 1;
   out_8803043244110166487[248] = 0;
   out_8803043244110166487[249] = 0;
   out_8803043244110166487[250] = 0;
   out_8803043244110166487[251] = 0;
   out_8803043244110166487[252] = 0;
   out_8803043244110166487[253] = 0;
   out_8803043244110166487[254] = 0;
   out_8803043244110166487[255] = 0;
   out_8803043244110166487[256] = 0;
   out_8803043244110166487[257] = 0;
   out_8803043244110166487[258] = 0;
   out_8803043244110166487[259] = 0;
   out_8803043244110166487[260] = 0;
   out_8803043244110166487[261] = 0;
   out_8803043244110166487[262] = 0;
   out_8803043244110166487[263] = 0;
   out_8803043244110166487[264] = 0;
   out_8803043244110166487[265] = 0;
   out_8803043244110166487[266] = 1;
   out_8803043244110166487[267] = 0;
   out_8803043244110166487[268] = 0;
   out_8803043244110166487[269] = 0;
   out_8803043244110166487[270] = 0;
   out_8803043244110166487[271] = 0;
   out_8803043244110166487[272] = 0;
   out_8803043244110166487[273] = 0;
   out_8803043244110166487[274] = 0;
   out_8803043244110166487[275] = 0;
   out_8803043244110166487[276] = 0;
   out_8803043244110166487[277] = 0;
   out_8803043244110166487[278] = 0;
   out_8803043244110166487[279] = 0;
   out_8803043244110166487[280] = 0;
   out_8803043244110166487[281] = 0;
   out_8803043244110166487[282] = 0;
   out_8803043244110166487[283] = 0;
   out_8803043244110166487[284] = 0;
   out_8803043244110166487[285] = 1;
   out_8803043244110166487[286] = 0;
   out_8803043244110166487[287] = 0;
   out_8803043244110166487[288] = 0;
   out_8803043244110166487[289] = 0;
   out_8803043244110166487[290] = 0;
   out_8803043244110166487[291] = 0;
   out_8803043244110166487[292] = 0;
   out_8803043244110166487[293] = 0;
   out_8803043244110166487[294] = 0;
   out_8803043244110166487[295] = 0;
   out_8803043244110166487[296] = 0;
   out_8803043244110166487[297] = 0;
   out_8803043244110166487[298] = 0;
   out_8803043244110166487[299] = 0;
   out_8803043244110166487[300] = 0;
   out_8803043244110166487[301] = 0;
   out_8803043244110166487[302] = 0;
   out_8803043244110166487[303] = 0;
   out_8803043244110166487[304] = 1;
   out_8803043244110166487[305] = 0;
   out_8803043244110166487[306] = 0;
   out_8803043244110166487[307] = 0;
   out_8803043244110166487[308] = 0;
   out_8803043244110166487[309] = 0;
   out_8803043244110166487[310] = 0;
   out_8803043244110166487[311] = 0;
   out_8803043244110166487[312] = 0;
   out_8803043244110166487[313] = 0;
   out_8803043244110166487[314] = 0;
   out_8803043244110166487[315] = 0;
   out_8803043244110166487[316] = 0;
   out_8803043244110166487[317] = 0;
   out_8803043244110166487[318] = 0;
   out_8803043244110166487[319] = 0;
   out_8803043244110166487[320] = 0;
   out_8803043244110166487[321] = 0;
   out_8803043244110166487[322] = 0;
   out_8803043244110166487[323] = 1;
}
void h_4(double *state, double *unused, double *out_4745763628231792235) {
   out_4745763628231792235[0] = state[6] + state[9];
   out_4745763628231792235[1] = state[7] + state[10];
   out_4745763628231792235[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_4796039147815908771) {
   out_4796039147815908771[0] = 0;
   out_4796039147815908771[1] = 0;
   out_4796039147815908771[2] = 0;
   out_4796039147815908771[3] = 0;
   out_4796039147815908771[4] = 0;
   out_4796039147815908771[5] = 0;
   out_4796039147815908771[6] = 1;
   out_4796039147815908771[7] = 0;
   out_4796039147815908771[8] = 0;
   out_4796039147815908771[9] = 1;
   out_4796039147815908771[10] = 0;
   out_4796039147815908771[11] = 0;
   out_4796039147815908771[12] = 0;
   out_4796039147815908771[13] = 0;
   out_4796039147815908771[14] = 0;
   out_4796039147815908771[15] = 0;
   out_4796039147815908771[16] = 0;
   out_4796039147815908771[17] = 0;
   out_4796039147815908771[18] = 0;
   out_4796039147815908771[19] = 0;
   out_4796039147815908771[20] = 0;
   out_4796039147815908771[21] = 0;
   out_4796039147815908771[22] = 0;
   out_4796039147815908771[23] = 0;
   out_4796039147815908771[24] = 0;
   out_4796039147815908771[25] = 1;
   out_4796039147815908771[26] = 0;
   out_4796039147815908771[27] = 0;
   out_4796039147815908771[28] = 1;
   out_4796039147815908771[29] = 0;
   out_4796039147815908771[30] = 0;
   out_4796039147815908771[31] = 0;
   out_4796039147815908771[32] = 0;
   out_4796039147815908771[33] = 0;
   out_4796039147815908771[34] = 0;
   out_4796039147815908771[35] = 0;
   out_4796039147815908771[36] = 0;
   out_4796039147815908771[37] = 0;
   out_4796039147815908771[38] = 0;
   out_4796039147815908771[39] = 0;
   out_4796039147815908771[40] = 0;
   out_4796039147815908771[41] = 0;
   out_4796039147815908771[42] = 0;
   out_4796039147815908771[43] = 0;
   out_4796039147815908771[44] = 1;
   out_4796039147815908771[45] = 0;
   out_4796039147815908771[46] = 0;
   out_4796039147815908771[47] = 1;
   out_4796039147815908771[48] = 0;
   out_4796039147815908771[49] = 0;
   out_4796039147815908771[50] = 0;
   out_4796039147815908771[51] = 0;
   out_4796039147815908771[52] = 0;
   out_4796039147815908771[53] = 0;
}
void h_10(double *state, double *unused, double *out_6977620553933084530) {
   out_6977620553933084530[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_6977620553933084530[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_6977620553933084530[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3695487615244328658) {
   out_3695487615244328658[0] = 0;
   out_3695487615244328658[1] = 9.8100000000000005*cos(state[1]);
   out_3695487615244328658[2] = 0;
   out_3695487615244328658[3] = 0;
   out_3695487615244328658[4] = -state[8];
   out_3695487615244328658[5] = state[7];
   out_3695487615244328658[6] = 0;
   out_3695487615244328658[7] = state[5];
   out_3695487615244328658[8] = -state[4];
   out_3695487615244328658[9] = 0;
   out_3695487615244328658[10] = 0;
   out_3695487615244328658[11] = 0;
   out_3695487615244328658[12] = 1;
   out_3695487615244328658[13] = 0;
   out_3695487615244328658[14] = 0;
   out_3695487615244328658[15] = 1;
   out_3695487615244328658[16] = 0;
   out_3695487615244328658[17] = 0;
   out_3695487615244328658[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3695487615244328658[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3695487615244328658[20] = 0;
   out_3695487615244328658[21] = state[8];
   out_3695487615244328658[22] = 0;
   out_3695487615244328658[23] = -state[6];
   out_3695487615244328658[24] = -state[5];
   out_3695487615244328658[25] = 0;
   out_3695487615244328658[26] = state[3];
   out_3695487615244328658[27] = 0;
   out_3695487615244328658[28] = 0;
   out_3695487615244328658[29] = 0;
   out_3695487615244328658[30] = 0;
   out_3695487615244328658[31] = 1;
   out_3695487615244328658[32] = 0;
   out_3695487615244328658[33] = 0;
   out_3695487615244328658[34] = 1;
   out_3695487615244328658[35] = 0;
   out_3695487615244328658[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3695487615244328658[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3695487615244328658[38] = 0;
   out_3695487615244328658[39] = -state[7];
   out_3695487615244328658[40] = state[6];
   out_3695487615244328658[41] = 0;
   out_3695487615244328658[42] = state[4];
   out_3695487615244328658[43] = -state[3];
   out_3695487615244328658[44] = 0;
   out_3695487615244328658[45] = 0;
   out_3695487615244328658[46] = 0;
   out_3695487615244328658[47] = 0;
   out_3695487615244328658[48] = 0;
   out_3695487615244328658[49] = 0;
   out_3695487615244328658[50] = 1;
   out_3695487615244328658[51] = 0;
   out_3695487615244328658[52] = 0;
   out_3695487615244328658[53] = 1;
}
void h_13(double *state, double *unused, double *out_7820235432546942220) {
   out_7820235432546942220[0] = state[3];
   out_7820235432546942220[1] = state[4];
   out_7820235432546942220[2] = state[5];
}
void H_13(double *state, double *unused, double *out_1583765322483575970) {
   out_1583765322483575970[0] = 0;
   out_1583765322483575970[1] = 0;
   out_1583765322483575970[2] = 0;
   out_1583765322483575970[3] = 1;
   out_1583765322483575970[4] = 0;
   out_1583765322483575970[5] = 0;
   out_1583765322483575970[6] = 0;
   out_1583765322483575970[7] = 0;
   out_1583765322483575970[8] = 0;
   out_1583765322483575970[9] = 0;
   out_1583765322483575970[10] = 0;
   out_1583765322483575970[11] = 0;
   out_1583765322483575970[12] = 0;
   out_1583765322483575970[13] = 0;
   out_1583765322483575970[14] = 0;
   out_1583765322483575970[15] = 0;
   out_1583765322483575970[16] = 0;
   out_1583765322483575970[17] = 0;
   out_1583765322483575970[18] = 0;
   out_1583765322483575970[19] = 0;
   out_1583765322483575970[20] = 0;
   out_1583765322483575970[21] = 0;
   out_1583765322483575970[22] = 1;
   out_1583765322483575970[23] = 0;
   out_1583765322483575970[24] = 0;
   out_1583765322483575970[25] = 0;
   out_1583765322483575970[26] = 0;
   out_1583765322483575970[27] = 0;
   out_1583765322483575970[28] = 0;
   out_1583765322483575970[29] = 0;
   out_1583765322483575970[30] = 0;
   out_1583765322483575970[31] = 0;
   out_1583765322483575970[32] = 0;
   out_1583765322483575970[33] = 0;
   out_1583765322483575970[34] = 0;
   out_1583765322483575970[35] = 0;
   out_1583765322483575970[36] = 0;
   out_1583765322483575970[37] = 0;
   out_1583765322483575970[38] = 0;
   out_1583765322483575970[39] = 0;
   out_1583765322483575970[40] = 0;
   out_1583765322483575970[41] = 1;
   out_1583765322483575970[42] = 0;
   out_1583765322483575970[43] = 0;
   out_1583765322483575970[44] = 0;
   out_1583765322483575970[45] = 0;
   out_1583765322483575970[46] = 0;
   out_1583765322483575970[47] = 0;
   out_1583765322483575970[48] = 0;
   out_1583765322483575970[49] = 0;
   out_1583765322483575970[50] = 0;
   out_1583765322483575970[51] = 0;
   out_1583765322483575970[52] = 0;
   out_1583765322483575970[53] = 0;
}
void h_14(double *state, double *unused, double *out_2348946808153196977) {
   out_2348946808153196977[0] = state[6];
   out_2348946808153196977[1] = state[7];
   out_2348946808153196977[2] = state[8];
}
void H_14(double *state, double *unused, double *out_832798291476424242) {
   out_832798291476424242[0] = 0;
   out_832798291476424242[1] = 0;
   out_832798291476424242[2] = 0;
   out_832798291476424242[3] = 0;
   out_832798291476424242[4] = 0;
   out_832798291476424242[5] = 0;
   out_832798291476424242[6] = 1;
   out_832798291476424242[7] = 0;
   out_832798291476424242[8] = 0;
   out_832798291476424242[9] = 0;
   out_832798291476424242[10] = 0;
   out_832798291476424242[11] = 0;
   out_832798291476424242[12] = 0;
   out_832798291476424242[13] = 0;
   out_832798291476424242[14] = 0;
   out_832798291476424242[15] = 0;
   out_832798291476424242[16] = 0;
   out_832798291476424242[17] = 0;
   out_832798291476424242[18] = 0;
   out_832798291476424242[19] = 0;
   out_832798291476424242[20] = 0;
   out_832798291476424242[21] = 0;
   out_832798291476424242[22] = 0;
   out_832798291476424242[23] = 0;
   out_832798291476424242[24] = 0;
   out_832798291476424242[25] = 1;
   out_832798291476424242[26] = 0;
   out_832798291476424242[27] = 0;
   out_832798291476424242[28] = 0;
   out_832798291476424242[29] = 0;
   out_832798291476424242[30] = 0;
   out_832798291476424242[31] = 0;
   out_832798291476424242[32] = 0;
   out_832798291476424242[33] = 0;
   out_832798291476424242[34] = 0;
   out_832798291476424242[35] = 0;
   out_832798291476424242[36] = 0;
   out_832798291476424242[37] = 0;
   out_832798291476424242[38] = 0;
   out_832798291476424242[39] = 0;
   out_832798291476424242[40] = 0;
   out_832798291476424242[41] = 0;
   out_832798291476424242[42] = 0;
   out_832798291476424242[43] = 0;
   out_832798291476424242[44] = 1;
   out_832798291476424242[45] = 0;
   out_832798291476424242[46] = 0;
   out_832798291476424242[47] = 0;
   out_832798291476424242[48] = 0;
   out_832798291476424242[49] = 0;
   out_832798291476424242[50] = 0;
   out_832798291476424242[51] = 0;
   out_832798291476424242[52] = 0;
   out_832798291476424242[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_5959269884630241271) {
  err_fun(nom_x, delta_x, out_5959269884630241271);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7281047554654548610) {
  inv_err_fun(nom_x, true_x, out_7281047554654548610);
}
void pose_H_mod_fun(double *state, double *out_6722182246202922039) {
  H_mod_fun(state, out_6722182246202922039);
}
void pose_f_fun(double *state, double dt, double *out_5905758100805214049) {
  f_fun(state,  dt, out_5905758100805214049);
}
void pose_F_fun(double *state, double dt, double *out_8803043244110166487) {
  F_fun(state,  dt, out_8803043244110166487);
}
void pose_h_4(double *state, double *unused, double *out_4745763628231792235) {
  h_4(state, unused, out_4745763628231792235);
}
void pose_H_4(double *state, double *unused, double *out_4796039147815908771) {
  H_4(state, unused, out_4796039147815908771);
}
void pose_h_10(double *state, double *unused, double *out_6977620553933084530) {
  h_10(state, unused, out_6977620553933084530);
}
void pose_H_10(double *state, double *unused, double *out_3695487615244328658) {
  H_10(state, unused, out_3695487615244328658);
}
void pose_h_13(double *state, double *unused, double *out_7820235432546942220) {
  h_13(state, unused, out_7820235432546942220);
}
void pose_H_13(double *state, double *unused, double *out_1583765322483575970) {
  H_13(state, unused, out_1583765322483575970);
}
void pose_h_14(double *state, double *unused, double *out_2348946808153196977) {
  h_14(state, unused, out_2348946808153196977);
}
void pose_H_14(double *state, double *unused, double *out_832798291476424242) {
  H_14(state, unused, out_832798291476424242);
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
