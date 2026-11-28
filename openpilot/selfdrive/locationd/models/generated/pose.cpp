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
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_72747944530161733) {
   out_72747944530161733[0] = delta_x[0] + nom_x[0];
   out_72747944530161733[1] = delta_x[1] + nom_x[1];
   out_72747944530161733[2] = delta_x[2] + nom_x[2];
   out_72747944530161733[3] = delta_x[3] + nom_x[3];
   out_72747944530161733[4] = delta_x[4] + nom_x[4];
   out_72747944530161733[5] = delta_x[5] + nom_x[5];
   out_72747944530161733[6] = delta_x[6] + nom_x[6];
   out_72747944530161733[7] = delta_x[7] + nom_x[7];
   out_72747944530161733[8] = delta_x[8] + nom_x[8];
   out_72747944530161733[9] = delta_x[9] + nom_x[9];
   out_72747944530161733[10] = delta_x[10] + nom_x[10];
   out_72747944530161733[11] = delta_x[11] + nom_x[11];
   out_72747944530161733[12] = delta_x[12] + nom_x[12];
   out_72747944530161733[13] = delta_x[13] + nom_x[13];
   out_72747944530161733[14] = delta_x[14] + nom_x[14];
   out_72747944530161733[15] = delta_x[15] + nom_x[15];
   out_72747944530161733[16] = delta_x[16] + nom_x[16];
   out_72747944530161733[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7058365865363402022) {
   out_7058365865363402022[0] = -nom_x[0] + true_x[0];
   out_7058365865363402022[1] = -nom_x[1] + true_x[1];
   out_7058365865363402022[2] = -nom_x[2] + true_x[2];
   out_7058365865363402022[3] = -nom_x[3] + true_x[3];
   out_7058365865363402022[4] = -nom_x[4] + true_x[4];
   out_7058365865363402022[5] = -nom_x[5] + true_x[5];
   out_7058365865363402022[6] = -nom_x[6] + true_x[6];
   out_7058365865363402022[7] = -nom_x[7] + true_x[7];
   out_7058365865363402022[8] = -nom_x[8] + true_x[8];
   out_7058365865363402022[9] = -nom_x[9] + true_x[9];
   out_7058365865363402022[10] = -nom_x[10] + true_x[10];
   out_7058365865363402022[11] = -nom_x[11] + true_x[11];
   out_7058365865363402022[12] = -nom_x[12] + true_x[12];
   out_7058365865363402022[13] = -nom_x[13] + true_x[13];
   out_7058365865363402022[14] = -nom_x[14] + true_x[14];
   out_7058365865363402022[15] = -nom_x[15] + true_x[15];
   out_7058365865363402022[16] = -nom_x[16] + true_x[16];
   out_7058365865363402022[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6187177415367410681) {
   out_6187177415367410681[0] = 1.0;
   out_6187177415367410681[1] = 0.0;
   out_6187177415367410681[2] = 0.0;
   out_6187177415367410681[3] = 0.0;
   out_6187177415367410681[4] = 0.0;
   out_6187177415367410681[5] = 0.0;
   out_6187177415367410681[6] = 0.0;
   out_6187177415367410681[7] = 0.0;
   out_6187177415367410681[8] = 0.0;
   out_6187177415367410681[9] = 0.0;
   out_6187177415367410681[10] = 0.0;
   out_6187177415367410681[11] = 0.0;
   out_6187177415367410681[12] = 0.0;
   out_6187177415367410681[13] = 0.0;
   out_6187177415367410681[14] = 0.0;
   out_6187177415367410681[15] = 0.0;
   out_6187177415367410681[16] = 0.0;
   out_6187177415367410681[17] = 0.0;
   out_6187177415367410681[18] = 0.0;
   out_6187177415367410681[19] = 1.0;
   out_6187177415367410681[20] = 0.0;
   out_6187177415367410681[21] = 0.0;
   out_6187177415367410681[22] = 0.0;
   out_6187177415367410681[23] = 0.0;
   out_6187177415367410681[24] = 0.0;
   out_6187177415367410681[25] = 0.0;
   out_6187177415367410681[26] = 0.0;
   out_6187177415367410681[27] = 0.0;
   out_6187177415367410681[28] = 0.0;
   out_6187177415367410681[29] = 0.0;
   out_6187177415367410681[30] = 0.0;
   out_6187177415367410681[31] = 0.0;
   out_6187177415367410681[32] = 0.0;
   out_6187177415367410681[33] = 0.0;
   out_6187177415367410681[34] = 0.0;
   out_6187177415367410681[35] = 0.0;
   out_6187177415367410681[36] = 0.0;
   out_6187177415367410681[37] = 0.0;
   out_6187177415367410681[38] = 1.0;
   out_6187177415367410681[39] = 0.0;
   out_6187177415367410681[40] = 0.0;
   out_6187177415367410681[41] = 0.0;
   out_6187177415367410681[42] = 0.0;
   out_6187177415367410681[43] = 0.0;
   out_6187177415367410681[44] = 0.0;
   out_6187177415367410681[45] = 0.0;
   out_6187177415367410681[46] = 0.0;
   out_6187177415367410681[47] = 0.0;
   out_6187177415367410681[48] = 0.0;
   out_6187177415367410681[49] = 0.0;
   out_6187177415367410681[50] = 0.0;
   out_6187177415367410681[51] = 0.0;
   out_6187177415367410681[52] = 0.0;
   out_6187177415367410681[53] = 0.0;
   out_6187177415367410681[54] = 0.0;
   out_6187177415367410681[55] = 0.0;
   out_6187177415367410681[56] = 0.0;
   out_6187177415367410681[57] = 1.0;
   out_6187177415367410681[58] = 0.0;
   out_6187177415367410681[59] = 0.0;
   out_6187177415367410681[60] = 0.0;
   out_6187177415367410681[61] = 0.0;
   out_6187177415367410681[62] = 0.0;
   out_6187177415367410681[63] = 0.0;
   out_6187177415367410681[64] = 0.0;
   out_6187177415367410681[65] = 0.0;
   out_6187177415367410681[66] = 0.0;
   out_6187177415367410681[67] = 0.0;
   out_6187177415367410681[68] = 0.0;
   out_6187177415367410681[69] = 0.0;
   out_6187177415367410681[70] = 0.0;
   out_6187177415367410681[71] = 0.0;
   out_6187177415367410681[72] = 0.0;
   out_6187177415367410681[73] = 0.0;
   out_6187177415367410681[74] = 0.0;
   out_6187177415367410681[75] = 0.0;
   out_6187177415367410681[76] = 1.0;
   out_6187177415367410681[77] = 0.0;
   out_6187177415367410681[78] = 0.0;
   out_6187177415367410681[79] = 0.0;
   out_6187177415367410681[80] = 0.0;
   out_6187177415367410681[81] = 0.0;
   out_6187177415367410681[82] = 0.0;
   out_6187177415367410681[83] = 0.0;
   out_6187177415367410681[84] = 0.0;
   out_6187177415367410681[85] = 0.0;
   out_6187177415367410681[86] = 0.0;
   out_6187177415367410681[87] = 0.0;
   out_6187177415367410681[88] = 0.0;
   out_6187177415367410681[89] = 0.0;
   out_6187177415367410681[90] = 0.0;
   out_6187177415367410681[91] = 0.0;
   out_6187177415367410681[92] = 0.0;
   out_6187177415367410681[93] = 0.0;
   out_6187177415367410681[94] = 0.0;
   out_6187177415367410681[95] = 1.0;
   out_6187177415367410681[96] = 0.0;
   out_6187177415367410681[97] = 0.0;
   out_6187177415367410681[98] = 0.0;
   out_6187177415367410681[99] = 0.0;
   out_6187177415367410681[100] = 0.0;
   out_6187177415367410681[101] = 0.0;
   out_6187177415367410681[102] = 0.0;
   out_6187177415367410681[103] = 0.0;
   out_6187177415367410681[104] = 0.0;
   out_6187177415367410681[105] = 0.0;
   out_6187177415367410681[106] = 0.0;
   out_6187177415367410681[107] = 0.0;
   out_6187177415367410681[108] = 0.0;
   out_6187177415367410681[109] = 0.0;
   out_6187177415367410681[110] = 0.0;
   out_6187177415367410681[111] = 0.0;
   out_6187177415367410681[112] = 0.0;
   out_6187177415367410681[113] = 0.0;
   out_6187177415367410681[114] = 1.0;
   out_6187177415367410681[115] = 0.0;
   out_6187177415367410681[116] = 0.0;
   out_6187177415367410681[117] = 0.0;
   out_6187177415367410681[118] = 0.0;
   out_6187177415367410681[119] = 0.0;
   out_6187177415367410681[120] = 0.0;
   out_6187177415367410681[121] = 0.0;
   out_6187177415367410681[122] = 0.0;
   out_6187177415367410681[123] = 0.0;
   out_6187177415367410681[124] = 0.0;
   out_6187177415367410681[125] = 0.0;
   out_6187177415367410681[126] = 0.0;
   out_6187177415367410681[127] = 0.0;
   out_6187177415367410681[128] = 0.0;
   out_6187177415367410681[129] = 0.0;
   out_6187177415367410681[130] = 0.0;
   out_6187177415367410681[131] = 0.0;
   out_6187177415367410681[132] = 0.0;
   out_6187177415367410681[133] = 1.0;
   out_6187177415367410681[134] = 0.0;
   out_6187177415367410681[135] = 0.0;
   out_6187177415367410681[136] = 0.0;
   out_6187177415367410681[137] = 0.0;
   out_6187177415367410681[138] = 0.0;
   out_6187177415367410681[139] = 0.0;
   out_6187177415367410681[140] = 0.0;
   out_6187177415367410681[141] = 0.0;
   out_6187177415367410681[142] = 0.0;
   out_6187177415367410681[143] = 0.0;
   out_6187177415367410681[144] = 0.0;
   out_6187177415367410681[145] = 0.0;
   out_6187177415367410681[146] = 0.0;
   out_6187177415367410681[147] = 0.0;
   out_6187177415367410681[148] = 0.0;
   out_6187177415367410681[149] = 0.0;
   out_6187177415367410681[150] = 0.0;
   out_6187177415367410681[151] = 0.0;
   out_6187177415367410681[152] = 1.0;
   out_6187177415367410681[153] = 0.0;
   out_6187177415367410681[154] = 0.0;
   out_6187177415367410681[155] = 0.0;
   out_6187177415367410681[156] = 0.0;
   out_6187177415367410681[157] = 0.0;
   out_6187177415367410681[158] = 0.0;
   out_6187177415367410681[159] = 0.0;
   out_6187177415367410681[160] = 0.0;
   out_6187177415367410681[161] = 0.0;
   out_6187177415367410681[162] = 0.0;
   out_6187177415367410681[163] = 0.0;
   out_6187177415367410681[164] = 0.0;
   out_6187177415367410681[165] = 0.0;
   out_6187177415367410681[166] = 0.0;
   out_6187177415367410681[167] = 0.0;
   out_6187177415367410681[168] = 0.0;
   out_6187177415367410681[169] = 0.0;
   out_6187177415367410681[170] = 0.0;
   out_6187177415367410681[171] = 1.0;
   out_6187177415367410681[172] = 0.0;
   out_6187177415367410681[173] = 0.0;
   out_6187177415367410681[174] = 0.0;
   out_6187177415367410681[175] = 0.0;
   out_6187177415367410681[176] = 0.0;
   out_6187177415367410681[177] = 0.0;
   out_6187177415367410681[178] = 0.0;
   out_6187177415367410681[179] = 0.0;
   out_6187177415367410681[180] = 0.0;
   out_6187177415367410681[181] = 0.0;
   out_6187177415367410681[182] = 0.0;
   out_6187177415367410681[183] = 0.0;
   out_6187177415367410681[184] = 0.0;
   out_6187177415367410681[185] = 0.0;
   out_6187177415367410681[186] = 0.0;
   out_6187177415367410681[187] = 0.0;
   out_6187177415367410681[188] = 0.0;
   out_6187177415367410681[189] = 0.0;
   out_6187177415367410681[190] = 1.0;
   out_6187177415367410681[191] = 0.0;
   out_6187177415367410681[192] = 0.0;
   out_6187177415367410681[193] = 0.0;
   out_6187177415367410681[194] = 0.0;
   out_6187177415367410681[195] = 0.0;
   out_6187177415367410681[196] = 0.0;
   out_6187177415367410681[197] = 0.0;
   out_6187177415367410681[198] = 0.0;
   out_6187177415367410681[199] = 0.0;
   out_6187177415367410681[200] = 0.0;
   out_6187177415367410681[201] = 0.0;
   out_6187177415367410681[202] = 0.0;
   out_6187177415367410681[203] = 0.0;
   out_6187177415367410681[204] = 0.0;
   out_6187177415367410681[205] = 0.0;
   out_6187177415367410681[206] = 0.0;
   out_6187177415367410681[207] = 0.0;
   out_6187177415367410681[208] = 0.0;
   out_6187177415367410681[209] = 1.0;
   out_6187177415367410681[210] = 0.0;
   out_6187177415367410681[211] = 0.0;
   out_6187177415367410681[212] = 0.0;
   out_6187177415367410681[213] = 0.0;
   out_6187177415367410681[214] = 0.0;
   out_6187177415367410681[215] = 0.0;
   out_6187177415367410681[216] = 0.0;
   out_6187177415367410681[217] = 0.0;
   out_6187177415367410681[218] = 0.0;
   out_6187177415367410681[219] = 0.0;
   out_6187177415367410681[220] = 0.0;
   out_6187177415367410681[221] = 0.0;
   out_6187177415367410681[222] = 0.0;
   out_6187177415367410681[223] = 0.0;
   out_6187177415367410681[224] = 0.0;
   out_6187177415367410681[225] = 0.0;
   out_6187177415367410681[226] = 0.0;
   out_6187177415367410681[227] = 0.0;
   out_6187177415367410681[228] = 1.0;
   out_6187177415367410681[229] = 0.0;
   out_6187177415367410681[230] = 0.0;
   out_6187177415367410681[231] = 0.0;
   out_6187177415367410681[232] = 0.0;
   out_6187177415367410681[233] = 0.0;
   out_6187177415367410681[234] = 0.0;
   out_6187177415367410681[235] = 0.0;
   out_6187177415367410681[236] = 0.0;
   out_6187177415367410681[237] = 0.0;
   out_6187177415367410681[238] = 0.0;
   out_6187177415367410681[239] = 0.0;
   out_6187177415367410681[240] = 0.0;
   out_6187177415367410681[241] = 0.0;
   out_6187177415367410681[242] = 0.0;
   out_6187177415367410681[243] = 0.0;
   out_6187177415367410681[244] = 0.0;
   out_6187177415367410681[245] = 0.0;
   out_6187177415367410681[246] = 0.0;
   out_6187177415367410681[247] = 1.0;
   out_6187177415367410681[248] = 0.0;
   out_6187177415367410681[249] = 0.0;
   out_6187177415367410681[250] = 0.0;
   out_6187177415367410681[251] = 0.0;
   out_6187177415367410681[252] = 0.0;
   out_6187177415367410681[253] = 0.0;
   out_6187177415367410681[254] = 0.0;
   out_6187177415367410681[255] = 0.0;
   out_6187177415367410681[256] = 0.0;
   out_6187177415367410681[257] = 0.0;
   out_6187177415367410681[258] = 0.0;
   out_6187177415367410681[259] = 0.0;
   out_6187177415367410681[260] = 0.0;
   out_6187177415367410681[261] = 0.0;
   out_6187177415367410681[262] = 0.0;
   out_6187177415367410681[263] = 0.0;
   out_6187177415367410681[264] = 0.0;
   out_6187177415367410681[265] = 0.0;
   out_6187177415367410681[266] = 1.0;
   out_6187177415367410681[267] = 0.0;
   out_6187177415367410681[268] = 0.0;
   out_6187177415367410681[269] = 0.0;
   out_6187177415367410681[270] = 0.0;
   out_6187177415367410681[271] = 0.0;
   out_6187177415367410681[272] = 0.0;
   out_6187177415367410681[273] = 0.0;
   out_6187177415367410681[274] = 0.0;
   out_6187177415367410681[275] = 0.0;
   out_6187177415367410681[276] = 0.0;
   out_6187177415367410681[277] = 0.0;
   out_6187177415367410681[278] = 0.0;
   out_6187177415367410681[279] = 0.0;
   out_6187177415367410681[280] = 0.0;
   out_6187177415367410681[281] = 0.0;
   out_6187177415367410681[282] = 0.0;
   out_6187177415367410681[283] = 0.0;
   out_6187177415367410681[284] = 0.0;
   out_6187177415367410681[285] = 1.0;
   out_6187177415367410681[286] = 0.0;
   out_6187177415367410681[287] = 0.0;
   out_6187177415367410681[288] = 0.0;
   out_6187177415367410681[289] = 0.0;
   out_6187177415367410681[290] = 0.0;
   out_6187177415367410681[291] = 0.0;
   out_6187177415367410681[292] = 0.0;
   out_6187177415367410681[293] = 0.0;
   out_6187177415367410681[294] = 0.0;
   out_6187177415367410681[295] = 0.0;
   out_6187177415367410681[296] = 0.0;
   out_6187177415367410681[297] = 0.0;
   out_6187177415367410681[298] = 0.0;
   out_6187177415367410681[299] = 0.0;
   out_6187177415367410681[300] = 0.0;
   out_6187177415367410681[301] = 0.0;
   out_6187177415367410681[302] = 0.0;
   out_6187177415367410681[303] = 0.0;
   out_6187177415367410681[304] = 1.0;
   out_6187177415367410681[305] = 0.0;
   out_6187177415367410681[306] = 0.0;
   out_6187177415367410681[307] = 0.0;
   out_6187177415367410681[308] = 0.0;
   out_6187177415367410681[309] = 0.0;
   out_6187177415367410681[310] = 0.0;
   out_6187177415367410681[311] = 0.0;
   out_6187177415367410681[312] = 0.0;
   out_6187177415367410681[313] = 0.0;
   out_6187177415367410681[314] = 0.0;
   out_6187177415367410681[315] = 0.0;
   out_6187177415367410681[316] = 0.0;
   out_6187177415367410681[317] = 0.0;
   out_6187177415367410681[318] = 0.0;
   out_6187177415367410681[319] = 0.0;
   out_6187177415367410681[320] = 0.0;
   out_6187177415367410681[321] = 0.0;
   out_6187177415367410681[322] = 0.0;
   out_6187177415367410681[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_8681455416555502143) {
   out_8681455416555502143[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_8681455416555502143[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_8681455416555502143[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_8681455416555502143[3] = dt*state[12] + state[3];
   out_8681455416555502143[4] = dt*state[13] + state[4];
   out_8681455416555502143[5] = dt*state[14] + state[5];
   out_8681455416555502143[6] = state[6];
   out_8681455416555502143[7] = state[7];
   out_8681455416555502143[8] = state[8];
   out_8681455416555502143[9] = state[9];
   out_8681455416555502143[10] = state[10];
   out_8681455416555502143[11] = state[11];
   out_8681455416555502143[12] = state[12];
   out_8681455416555502143[13] = state[13];
   out_8681455416555502143[14] = state[14];
   out_8681455416555502143[15] = state[15];
   out_8681455416555502143[16] = state[16];
   out_8681455416555502143[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3542904472886764692) {
   out_3542904472886764692[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3542904472886764692[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3542904472886764692[2] = 0;
   out_3542904472886764692[3] = 0;
   out_3542904472886764692[4] = 0;
   out_3542904472886764692[5] = 0;
   out_3542904472886764692[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3542904472886764692[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3542904472886764692[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3542904472886764692[9] = 0;
   out_3542904472886764692[10] = 0;
   out_3542904472886764692[11] = 0;
   out_3542904472886764692[12] = 0;
   out_3542904472886764692[13] = 0;
   out_3542904472886764692[14] = 0;
   out_3542904472886764692[15] = 0;
   out_3542904472886764692[16] = 0;
   out_3542904472886764692[17] = 0;
   out_3542904472886764692[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3542904472886764692[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3542904472886764692[20] = 0;
   out_3542904472886764692[21] = 0;
   out_3542904472886764692[22] = 0;
   out_3542904472886764692[23] = 0;
   out_3542904472886764692[24] = 0;
   out_3542904472886764692[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3542904472886764692[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3542904472886764692[27] = 0;
   out_3542904472886764692[28] = 0;
   out_3542904472886764692[29] = 0;
   out_3542904472886764692[30] = 0;
   out_3542904472886764692[31] = 0;
   out_3542904472886764692[32] = 0;
   out_3542904472886764692[33] = 0;
   out_3542904472886764692[34] = 0;
   out_3542904472886764692[35] = 0;
   out_3542904472886764692[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3542904472886764692[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3542904472886764692[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3542904472886764692[39] = 0;
   out_3542904472886764692[40] = 0;
   out_3542904472886764692[41] = 0;
   out_3542904472886764692[42] = 0;
   out_3542904472886764692[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3542904472886764692[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3542904472886764692[45] = 0;
   out_3542904472886764692[46] = 0;
   out_3542904472886764692[47] = 0;
   out_3542904472886764692[48] = 0;
   out_3542904472886764692[49] = 0;
   out_3542904472886764692[50] = 0;
   out_3542904472886764692[51] = 0;
   out_3542904472886764692[52] = 0;
   out_3542904472886764692[53] = 0;
   out_3542904472886764692[54] = 0;
   out_3542904472886764692[55] = 0;
   out_3542904472886764692[56] = 0;
   out_3542904472886764692[57] = 1;
   out_3542904472886764692[58] = 0;
   out_3542904472886764692[59] = 0;
   out_3542904472886764692[60] = 0;
   out_3542904472886764692[61] = 0;
   out_3542904472886764692[62] = 0;
   out_3542904472886764692[63] = 0;
   out_3542904472886764692[64] = 0;
   out_3542904472886764692[65] = 0;
   out_3542904472886764692[66] = dt;
   out_3542904472886764692[67] = 0;
   out_3542904472886764692[68] = 0;
   out_3542904472886764692[69] = 0;
   out_3542904472886764692[70] = 0;
   out_3542904472886764692[71] = 0;
   out_3542904472886764692[72] = 0;
   out_3542904472886764692[73] = 0;
   out_3542904472886764692[74] = 0;
   out_3542904472886764692[75] = 0;
   out_3542904472886764692[76] = 1;
   out_3542904472886764692[77] = 0;
   out_3542904472886764692[78] = 0;
   out_3542904472886764692[79] = 0;
   out_3542904472886764692[80] = 0;
   out_3542904472886764692[81] = 0;
   out_3542904472886764692[82] = 0;
   out_3542904472886764692[83] = 0;
   out_3542904472886764692[84] = 0;
   out_3542904472886764692[85] = dt;
   out_3542904472886764692[86] = 0;
   out_3542904472886764692[87] = 0;
   out_3542904472886764692[88] = 0;
   out_3542904472886764692[89] = 0;
   out_3542904472886764692[90] = 0;
   out_3542904472886764692[91] = 0;
   out_3542904472886764692[92] = 0;
   out_3542904472886764692[93] = 0;
   out_3542904472886764692[94] = 0;
   out_3542904472886764692[95] = 1;
   out_3542904472886764692[96] = 0;
   out_3542904472886764692[97] = 0;
   out_3542904472886764692[98] = 0;
   out_3542904472886764692[99] = 0;
   out_3542904472886764692[100] = 0;
   out_3542904472886764692[101] = 0;
   out_3542904472886764692[102] = 0;
   out_3542904472886764692[103] = 0;
   out_3542904472886764692[104] = dt;
   out_3542904472886764692[105] = 0;
   out_3542904472886764692[106] = 0;
   out_3542904472886764692[107] = 0;
   out_3542904472886764692[108] = 0;
   out_3542904472886764692[109] = 0;
   out_3542904472886764692[110] = 0;
   out_3542904472886764692[111] = 0;
   out_3542904472886764692[112] = 0;
   out_3542904472886764692[113] = 0;
   out_3542904472886764692[114] = 1;
   out_3542904472886764692[115] = 0;
   out_3542904472886764692[116] = 0;
   out_3542904472886764692[117] = 0;
   out_3542904472886764692[118] = 0;
   out_3542904472886764692[119] = 0;
   out_3542904472886764692[120] = 0;
   out_3542904472886764692[121] = 0;
   out_3542904472886764692[122] = 0;
   out_3542904472886764692[123] = 0;
   out_3542904472886764692[124] = 0;
   out_3542904472886764692[125] = 0;
   out_3542904472886764692[126] = 0;
   out_3542904472886764692[127] = 0;
   out_3542904472886764692[128] = 0;
   out_3542904472886764692[129] = 0;
   out_3542904472886764692[130] = 0;
   out_3542904472886764692[131] = 0;
   out_3542904472886764692[132] = 0;
   out_3542904472886764692[133] = 1;
   out_3542904472886764692[134] = 0;
   out_3542904472886764692[135] = 0;
   out_3542904472886764692[136] = 0;
   out_3542904472886764692[137] = 0;
   out_3542904472886764692[138] = 0;
   out_3542904472886764692[139] = 0;
   out_3542904472886764692[140] = 0;
   out_3542904472886764692[141] = 0;
   out_3542904472886764692[142] = 0;
   out_3542904472886764692[143] = 0;
   out_3542904472886764692[144] = 0;
   out_3542904472886764692[145] = 0;
   out_3542904472886764692[146] = 0;
   out_3542904472886764692[147] = 0;
   out_3542904472886764692[148] = 0;
   out_3542904472886764692[149] = 0;
   out_3542904472886764692[150] = 0;
   out_3542904472886764692[151] = 0;
   out_3542904472886764692[152] = 1;
   out_3542904472886764692[153] = 0;
   out_3542904472886764692[154] = 0;
   out_3542904472886764692[155] = 0;
   out_3542904472886764692[156] = 0;
   out_3542904472886764692[157] = 0;
   out_3542904472886764692[158] = 0;
   out_3542904472886764692[159] = 0;
   out_3542904472886764692[160] = 0;
   out_3542904472886764692[161] = 0;
   out_3542904472886764692[162] = 0;
   out_3542904472886764692[163] = 0;
   out_3542904472886764692[164] = 0;
   out_3542904472886764692[165] = 0;
   out_3542904472886764692[166] = 0;
   out_3542904472886764692[167] = 0;
   out_3542904472886764692[168] = 0;
   out_3542904472886764692[169] = 0;
   out_3542904472886764692[170] = 0;
   out_3542904472886764692[171] = 1;
   out_3542904472886764692[172] = 0;
   out_3542904472886764692[173] = 0;
   out_3542904472886764692[174] = 0;
   out_3542904472886764692[175] = 0;
   out_3542904472886764692[176] = 0;
   out_3542904472886764692[177] = 0;
   out_3542904472886764692[178] = 0;
   out_3542904472886764692[179] = 0;
   out_3542904472886764692[180] = 0;
   out_3542904472886764692[181] = 0;
   out_3542904472886764692[182] = 0;
   out_3542904472886764692[183] = 0;
   out_3542904472886764692[184] = 0;
   out_3542904472886764692[185] = 0;
   out_3542904472886764692[186] = 0;
   out_3542904472886764692[187] = 0;
   out_3542904472886764692[188] = 0;
   out_3542904472886764692[189] = 0;
   out_3542904472886764692[190] = 1;
   out_3542904472886764692[191] = 0;
   out_3542904472886764692[192] = 0;
   out_3542904472886764692[193] = 0;
   out_3542904472886764692[194] = 0;
   out_3542904472886764692[195] = 0;
   out_3542904472886764692[196] = 0;
   out_3542904472886764692[197] = 0;
   out_3542904472886764692[198] = 0;
   out_3542904472886764692[199] = 0;
   out_3542904472886764692[200] = 0;
   out_3542904472886764692[201] = 0;
   out_3542904472886764692[202] = 0;
   out_3542904472886764692[203] = 0;
   out_3542904472886764692[204] = 0;
   out_3542904472886764692[205] = 0;
   out_3542904472886764692[206] = 0;
   out_3542904472886764692[207] = 0;
   out_3542904472886764692[208] = 0;
   out_3542904472886764692[209] = 1;
   out_3542904472886764692[210] = 0;
   out_3542904472886764692[211] = 0;
   out_3542904472886764692[212] = 0;
   out_3542904472886764692[213] = 0;
   out_3542904472886764692[214] = 0;
   out_3542904472886764692[215] = 0;
   out_3542904472886764692[216] = 0;
   out_3542904472886764692[217] = 0;
   out_3542904472886764692[218] = 0;
   out_3542904472886764692[219] = 0;
   out_3542904472886764692[220] = 0;
   out_3542904472886764692[221] = 0;
   out_3542904472886764692[222] = 0;
   out_3542904472886764692[223] = 0;
   out_3542904472886764692[224] = 0;
   out_3542904472886764692[225] = 0;
   out_3542904472886764692[226] = 0;
   out_3542904472886764692[227] = 0;
   out_3542904472886764692[228] = 1;
   out_3542904472886764692[229] = 0;
   out_3542904472886764692[230] = 0;
   out_3542904472886764692[231] = 0;
   out_3542904472886764692[232] = 0;
   out_3542904472886764692[233] = 0;
   out_3542904472886764692[234] = 0;
   out_3542904472886764692[235] = 0;
   out_3542904472886764692[236] = 0;
   out_3542904472886764692[237] = 0;
   out_3542904472886764692[238] = 0;
   out_3542904472886764692[239] = 0;
   out_3542904472886764692[240] = 0;
   out_3542904472886764692[241] = 0;
   out_3542904472886764692[242] = 0;
   out_3542904472886764692[243] = 0;
   out_3542904472886764692[244] = 0;
   out_3542904472886764692[245] = 0;
   out_3542904472886764692[246] = 0;
   out_3542904472886764692[247] = 1;
   out_3542904472886764692[248] = 0;
   out_3542904472886764692[249] = 0;
   out_3542904472886764692[250] = 0;
   out_3542904472886764692[251] = 0;
   out_3542904472886764692[252] = 0;
   out_3542904472886764692[253] = 0;
   out_3542904472886764692[254] = 0;
   out_3542904472886764692[255] = 0;
   out_3542904472886764692[256] = 0;
   out_3542904472886764692[257] = 0;
   out_3542904472886764692[258] = 0;
   out_3542904472886764692[259] = 0;
   out_3542904472886764692[260] = 0;
   out_3542904472886764692[261] = 0;
   out_3542904472886764692[262] = 0;
   out_3542904472886764692[263] = 0;
   out_3542904472886764692[264] = 0;
   out_3542904472886764692[265] = 0;
   out_3542904472886764692[266] = 1;
   out_3542904472886764692[267] = 0;
   out_3542904472886764692[268] = 0;
   out_3542904472886764692[269] = 0;
   out_3542904472886764692[270] = 0;
   out_3542904472886764692[271] = 0;
   out_3542904472886764692[272] = 0;
   out_3542904472886764692[273] = 0;
   out_3542904472886764692[274] = 0;
   out_3542904472886764692[275] = 0;
   out_3542904472886764692[276] = 0;
   out_3542904472886764692[277] = 0;
   out_3542904472886764692[278] = 0;
   out_3542904472886764692[279] = 0;
   out_3542904472886764692[280] = 0;
   out_3542904472886764692[281] = 0;
   out_3542904472886764692[282] = 0;
   out_3542904472886764692[283] = 0;
   out_3542904472886764692[284] = 0;
   out_3542904472886764692[285] = 1;
   out_3542904472886764692[286] = 0;
   out_3542904472886764692[287] = 0;
   out_3542904472886764692[288] = 0;
   out_3542904472886764692[289] = 0;
   out_3542904472886764692[290] = 0;
   out_3542904472886764692[291] = 0;
   out_3542904472886764692[292] = 0;
   out_3542904472886764692[293] = 0;
   out_3542904472886764692[294] = 0;
   out_3542904472886764692[295] = 0;
   out_3542904472886764692[296] = 0;
   out_3542904472886764692[297] = 0;
   out_3542904472886764692[298] = 0;
   out_3542904472886764692[299] = 0;
   out_3542904472886764692[300] = 0;
   out_3542904472886764692[301] = 0;
   out_3542904472886764692[302] = 0;
   out_3542904472886764692[303] = 0;
   out_3542904472886764692[304] = 1;
   out_3542904472886764692[305] = 0;
   out_3542904472886764692[306] = 0;
   out_3542904472886764692[307] = 0;
   out_3542904472886764692[308] = 0;
   out_3542904472886764692[309] = 0;
   out_3542904472886764692[310] = 0;
   out_3542904472886764692[311] = 0;
   out_3542904472886764692[312] = 0;
   out_3542904472886764692[313] = 0;
   out_3542904472886764692[314] = 0;
   out_3542904472886764692[315] = 0;
   out_3542904472886764692[316] = 0;
   out_3542904472886764692[317] = 0;
   out_3542904472886764692[318] = 0;
   out_3542904472886764692[319] = 0;
   out_3542904472886764692[320] = 0;
   out_3542904472886764692[321] = 0;
   out_3542904472886764692[322] = 0;
   out_3542904472886764692[323] = 1;
}
void h_4(double *state, double *unused, double *out_4697645595739621983) {
   out_4697645595739621983[0] = state[6] + state[9];
   out_4697645595739621983[1] = state[7] + state[10];
   out_4697645595739621983[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2757184250690410661) {
   out_2757184250690410661[0] = 0;
   out_2757184250690410661[1] = 0;
   out_2757184250690410661[2] = 0;
   out_2757184250690410661[3] = 0;
   out_2757184250690410661[4] = 0;
   out_2757184250690410661[5] = 0;
   out_2757184250690410661[6] = 1;
   out_2757184250690410661[7] = 0;
   out_2757184250690410661[8] = 0;
   out_2757184250690410661[9] = 1;
   out_2757184250690410661[10] = 0;
   out_2757184250690410661[11] = 0;
   out_2757184250690410661[12] = 0;
   out_2757184250690410661[13] = 0;
   out_2757184250690410661[14] = 0;
   out_2757184250690410661[15] = 0;
   out_2757184250690410661[16] = 0;
   out_2757184250690410661[17] = 0;
   out_2757184250690410661[18] = 0;
   out_2757184250690410661[19] = 0;
   out_2757184250690410661[20] = 0;
   out_2757184250690410661[21] = 0;
   out_2757184250690410661[22] = 0;
   out_2757184250690410661[23] = 0;
   out_2757184250690410661[24] = 0;
   out_2757184250690410661[25] = 1;
   out_2757184250690410661[26] = 0;
   out_2757184250690410661[27] = 0;
   out_2757184250690410661[28] = 1;
   out_2757184250690410661[29] = 0;
   out_2757184250690410661[30] = 0;
   out_2757184250690410661[31] = 0;
   out_2757184250690410661[32] = 0;
   out_2757184250690410661[33] = 0;
   out_2757184250690410661[34] = 0;
   out_2757184250690410661[35] = 0;
   out_2757184250690410661[36] = 0;
   out_2757184250690410661[37] = 0;
   out_2757184250690410661[38] = 0;
   out_2757184250690410661[39] = 0;
   out_2757184250690410661[40] = 0;
   out_2757184250690410661[41] = 0;
   out_2757184250690410661[42] = 0;
   out_2757184250690410661[43] = 0;
   out_2757184250690410661[44] = 1;
   out_2757184250690410661[45] = 0;
   out_2757184250690410661[46] = 0;
   out_2757184250690410661[47] = 1;
   out_2757184250690410661[48] = 0;
   out_2757184250690410661[49] = 0;
   out_2757184250690410661[50] = 0;
   out_2757184250690410661[51] = 0;
   out_2757184250690410661[52] = 0;
   out_2757184250690410661[53] = 0;
}
void h_10(double *state, double *unused, double *out_8842851692336888185) {
   out_8842851692336888185[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8842851692336888185[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8842851692336888185[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5833963184882541613) {
   out_5833963184882541613[0] = 0;
   out_5833963184882541613[1] = 9.8100000000000005*cos(state[1]);
   out_5833963184882541613[2] = 0;
   out_5833963184882541613[3] = 0;
   out_5833963184882541613[4] = -state[8];
   out_5833963184882541613[5] = state[7];
   out_5833963184882541613[6] = 0;
   out_5833963184882541613[7] = state[5];
   out_5833963184882541613[8] = -state[4];
   out_5833963184882541613[9] = 0;
   out_5833963184882541613[10] = 0;
   out_5833963184882541613[11] = 0;
   out_5833963184882541613[12] = 1;
   out_5833963184882541613[13] = 0;
   out_5833963184882541613[14] = 0;
   out_5833963184882541613[15] = 1;
   out_5833963184882541613[16] = 0;
   out_5833963184882541613[17] = 0;
   out_5833963184882541613[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5833963184882541613[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5833963184882541613[20] = 0;
   out_5833963184882541613[21] = state[8];
   out_5833963184882541613[22] = 0;
   out_5833963184882541613[23] = -state[6];
   out_5833963184882541613[24] = -state[5];
   out_5833963184882541613[25] = 0;
   out_5833963184882541613[26] = state[3];
   out_5833963184882541613[27] = 0;
   out_5833963184882541613[28] = 0;
   out_5833963184882541613[29] = 0;
   out_5833963184882541613[30] = 0;
   out_5833963184882541613[31] = 1;
   out_5833963184882541613[32] = 0;
   out_5833963184882541613[33] = 0;
   out_5833963184882541613[34] = 1;
   out_5833963184882541613[35] = 0;
   out_5833963184882541613[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5833963184882541613[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5833963184882541613[38] = 0;
   out_5833963184882541613[39] = -state[7];
   out_5833963184882541613[40] = state[6];
   out_5833963184882541613[41] = 0;
   out_5833963184882541613[42] = state[4];
   out_5833963184882541613[43] = -state[3];
   out_5833963184882541613[44] = 0;
   out_5833963184882541613[45] = 0;
   out_5833963184882541613[46] = 0;
   out_5833963184882541613[47] = 0;
   out_5833963184882541613[48] = 0;
   out_5833963184882541613[49] = 0;
   out_5833963184882541613[50] = 1;
   out_5833963184882541613[51] = 0;
   out_5833963184882541613[52] = 0;
   out_5833963184882541613[53] = 1;
}
void h_13(double *state, double *unused, double *out_7087867461883945740) {
   out_7087867461883945740[0] = state[3];
   out_7087867461883945740[1] = state[4];
   out_7087867461883945740[2] = state[5];
}
void H_13(double *state, double *unused, double *out_455089574641922140) {
   out_455089574641922140[0] = 0;
   out_455089574641922140[1] = 0;
   out_455089574641922140[2] = 0;
   out_455089574641922140[3] = 1;
   out_455089574641922140[4] = 0;
   out_455089574641922140[5] = 0;
   out_455089574641922140[6] = 0;
   out_455089574641922140[7] = 0;
   out_455089574641922140[8] = 0;
   out_455089574641922140[9] = 0;
   out_455089574641922140[10] = 0;
   out_455089574641922140[11] = 0;
   out_455089574641922140[12] = 0;
   out_455089574641922140[13] = 0;
   out_455089574641922140[14] = 0;
   out_455089574641922140[15] = 0;
   out_455089574641922140[16] = 0;
   out_455089574641922140[17] = 0;
   out_455089574641922140[18] = 0;
   out_455089574641922140[19] = 0;
   out_455089574641922140[20] = 0;
   out_455089574641922140[21] = 0;
   out_455089574641922140[22] = 1;
   out_455089574641922140[23] = 0;
   out_455089574641922140[24] = 0;
   out_455089574641922140[25] = 0;
   out_455089574641922140[26] = 0;
   out_455089574641922140[27] = 0;
   out_455089574641922140[28] = 0;
   out_455089574641922140[29] = 0;
   out_455089574641922140[30] = 0;
   out_455089574641922140[31] = 0;
   out_455089574641922140[32] = 0;
   out_455089574641922140[33] = 0;
   out_455089574641922140[34] = 0;
   out_455089574641922140[35] = 0;
   out_455089574641922140[36] = 0;
   out_455089574641922140[37] = 0;
   out_455089574641922140[38] = 0;
   out_455089574641922140[39] = 0;
   out_455089574641922140[40] = 0;
   out_455089574641922140[41] = 1;
   out_455089574641922140[42] = 0;
   out_455089574641922140[43] = 0;
   out_455089574641922140[44] = 0;
   out_455089574641922140[45] = 0;
   out_455089574641922140[46] = 0;
   out_455089574641922140[47] = 0;
   out_455089574641922140[48] = 0;
   out_455089574641922140[49] = 0;
   out_455089574641922140[50] = 0;
   out_455089574641922140[51] = 0;
   out_455089574641922140[52] = 0;
   out_455089574641922140[53] = 0;
}
void h_14(double *state, double *unused, double *out_2045247785068682723) {
   out_2045247785068682723[0] = state[6];
   out_2045247785068682723[1] = state[7];
   out_2045247785068682723[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5839972682985782957) {
   out_5839972682985782957[0] = 0;
   out_5839972682985782957[1] = 0;
   out_5839972682985782957[2] = 0;
   out_5839972682985782957[3] = 0;
   out_5839972682985782957[4] = 0;
   out_5839972682985782957[5] = 0;
   out_5839972682985782957[6] = 1;
   out_5839972682985782957[7] = 0;
   out_5839972682985782957[8] = 0;
   out_5839972682985782957[9] = 0;
   out_5839972682985782957[10] = 0;
   out_5839972682985782957[11] = 0;
   out_5839972682985782957[12] = 0;
   out_5839972682985782957[13] = 0;
   out_5839972682985782957[14] = 0;
   out_5839972682985782957[15] = 0;
   out_5839972682985782957[16] = 0;
   out_5839972682985782957[17] = 0;
   out_5839972682985782957[18] = 0;
   out_5839972682985782957[19] = 0;
   out_5839972682985782957[20] = 0;
   out_5839972682985782957[21] = 0;
   out_5839972682985782957[22] = 0;
   out_5839972682985782957[23] = 0;
   out_5839972682985782957[24] = 0;
   out_5839972682985782957[25] = 1;
   out_5839972682985782957[26] = 0;
   out_5839972682985782957[27] = 0;
   out_5839972682985782957[28] = 0;
   out_5839972682985782957[29] = 0;
   out_5839972682985782957[30] = 0;
   out_5839972682985782957[31] = 0;
   out_5839972682985782957[32] = 0;
   out_5839972682985782957[33] = 0;
   out_5839972682985782957[34] = 0;
   out_5839972682985782957[35] = 0;
   out_5839972682985782957[36] = 0;
   out_5839972682985782957[37] = 0;
   out_5839972682985782957[38] = 0;
   out_5839972682985782957[39] = 0;
   out_5839972682985782957[40] = 0;
   out_5839972682985782957[41] = 0;
   out_5839972682985782957[42] = 0;
   out_5839972682985782957[43] = 0;
   out_5839972682985782957[44] = 1;
   out_5839972682985782957[45] = 0;
   out_5839972682985782957[46] = 0;
   out_5839972682985782957[47] = 0;
   out_5839972682985782957[48] = 0;
   out_5839972682985782957[49] = 0;
   out_5839972682985782957[50] = 0;
   out_5839972682985782957[51] = 0;
   out_5839972682985782957[52] = 0;
   out_5839972682985782957[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_72747944530161733) {
  err_fun(nom_x, delta_x, out_72747944530161733);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7058365865363402022) {
  inv_err_fun(nom_x, true_x, out_7058365865363402022);
}
void pose_H_mod_fun(double *state, double *out_6187177415367410681) {
  H_mod_fun(state, out_6187177415367410681);
}
void pose_f_fun(double *state, double dt, double *out_8681455416555502143) {
  f_fun(state,  dt, out_8681455416555502143);
}
void pose_F_fun(double *state, double dt, double *out_3542904472886764692) {
  F_fun(state,  dt, out_3542904472886764692);
}
void pose_h_4(double *state, double *unused, double *out_4697645595739621983) {
  h_4(state, unused, out_4697645595739621983);
}
void pose_H_4(double *state, double *unused, double *out_2757184250690410661) {
  H_4(state, unused, out_2757184250690410661);
}
void pose_h_10(double *state, double *unused, double *out_8842851692336888185) {
  h_10(state, unused, out_8842851692336888185);
}
void pose_H_10(double *state, double *unused, double *out_5833963184882541613) {
  H_10(state, unused, out_5833963184882541613);
}
void pose_h_13(double *state, double *unused, double *out_7087867461883945740) {
  h_13(state, unused, out_7087867461883945740);
}
void pose_H_13(double *state, double *unused, double *out_455089574641922140) {
  H_13(state, unused, out_455089574641922140);
}
void pose_h_14(double *state, double *unused, double *out_2045247785068682723) {
  h_14(state, unused, out_2045247785068682723);
}
void pose_H_14(double *state, double *unused, double *out_5839972682985782957) {
  H_14(state, unused, out_5839972682985782957);
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
