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
void err_fun(double *nom_x, double *delta_x, double *out_3543435702888968549) {
   out_3543435702888968549[0] = delta_x[0] + nom_x[0];
   out_3543435702888968549[1] = delta_x[1] + nom_x[1];
   out_3543435702888968549[2] = delta_x[2] + nom_x[2];
   out_3543435702888968549[3] = delta_x[3] + nom_x[3];
   out_3543435702888968549[4] = delta_x[4] + nom_x[4];
   out_3543435702888968549[5] = delta_x[5] + nom_x[5];
   out_3543435702888968549[6] = delta_x[6] + nom_x[6];
   out_3543435702888968549[7] = delta_x[7] + nom_x[7];
   out_3543435702888968549[8] = delta_x[8] + nom_x[8];
   out_3543435702888968549[9] = delta_x[9] + nom_x[9];
   out_3543435702888968549[10] = delta_x[10] + nom_x[10];
   out_3543435702888968549[11] = delta_x[11] + nom_x[11];
   out_3543435702888968549[12] = delta_x[12] + nom_x[12];
   out_3543435702888968549[13] = delta_x[13] + nom_x[13];
   out_3543435702888968549[14] = delta_x[14] + nom_x[14];
   out_3543435702888968549[15] = delta_x[15] + nom_x[15];
   out_3543435702888968549[16] = delta_x[16] + nom_x[16];
   out_3543435702888968549[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2416974275063884114) {
   out_2416974275063884114[0] = -nom_x[0] + true_x[0];
   out_2416974275063884114[1] = -nom_x[1] + true_x[1];
   out_2416974275063884114[2] = -nom_x[2] + true_x[2];
   out_2416974275063884114[3] = -nom_x[3] + true_x[3];
   out_2416974275063884114[4] = -nom_x[4] + true_x[4];
   out_2416974275063884114[5] = -nom_x[5] + true_x[5];
   out_2416974275063884114[6] = -nom_x[6] + true_x[6];
   out_2416974275063884114[7] = -nom_x[7] + true_x[7];
   out_2416974275063884114[8] = -nom_x[8] + true_x[8];
   out_2416974275063884114[9] = -nom_x[9] + true_x[9];
   out_2416974275063884114[10] = -nom_x[10] + true_x[10];
   out_2416974275063884114[11] = -nom_x[11] + true_x[11];
   out_2416974275063884114[12] = -nom_x[12] + true_x[12];
   out_2416974275063884114[13] = -nom_x[13] + true_x[13];
   out_2416974275063884114[14] = -nom_x[14] + true_x[14];
   out_2416974275063884114[15] = -nom_x[15] + true_x[15];
   out_2416974275063884114[16] = -nom_x[16] + true_x[16];
   out_2416974275063884114[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_8476558897598068373) {
   out_8476558897598068373[0] = 1.0;
   out_8476558897598068373[1] = 0.0;
   out_8476558897598068373[2] = 0.0;
   out_8476558897598068373[3] = 0.0;
   out_8476558897598068373[4] = 0.0;
   out_8476558897598068373[5] = 0.0;
   out_8476558897598068373[6] = 0.0;
   out_8476558897598068373[7] = 0.0;
   out_8476558897598068373[8] = 0.0;
   out_8476558897598068373[9] = 0.0;
   out_8476558897598068373[10] = 0.0;
   out_8476558897598068373[11] = 0.0;
   out_8476558897598068373[12] = 0.0;
   out_8476558897598068373[13] = 0.0;
   out_8476558897598068373[14] = 0.0;
   out_8476558897598068373[15] = 0.0;
   out_8476558897598068373[16] = 0.0;
   out_8476558897598068373[17] = 0.0;
   out_8476558897598068373[18] = 0.0;
   out_8476558897598068373[19] = 1.0;
   out_8476558897598068373[20] = 0.0;
   out_8476558897598068373[21] = 0.0;
   out_8476558897598068373[22] = 0.0;
   out_8476558897598068373[23] = 0.0;
   out_8476558897598068373[24] = 0.0;
   out_8476558897598068373[25] = 0.0;
   out_8476558897598068373[26] = 0.0;
   out_8476558897598068373[27] = 0.0;
   out_8476558897598068373[28] = 0.0;
   out_8476558897598068373[29] = 0.0;
   out_8476558897598068373[30] = 0.0;
   out_8476558897598068373[31] = 0.0;
   out_8476558897598068373[32] = 0.0;
   out_8476558897598068373[33] = 0.0;
   out_8476558897598068373[34] = 0.0;
   out_8476558897598068373[35] = 0.0;
   out_8476558897598068373[36] = 0.0;
   out_8476558897598068373[37] = 0.0;
   out_8476558897598068373[38] = 1.0;
   out_8476558897598068373[39] = 0.0;
   out_8476558897598068373[40] = 0.0;
   out_8476558897598068373[41] = 0.0;
   out_8476558897598068373[42] = 0.0;
   out_8476558897598068373[43] = 0.0;
   out_8476558897598068373[44] = 0.0;
   out_8476558897598068373[45] = 0.0;
   out_8476558897598068373[46] = 0.0;
   out_8476558897598068373[47] = 0.0;
   out_8476558897598068373[48] = 0.0;
   out_8476558897598068373[49] = 0.0;
   out_8476558897598068373[50] = 0.0;
   out_8476558897598068373[51] = 0.0;
   out_8476558897598068373[52] = 0.0;
   out_8476558897598068373[53] = 0.0;
   out_8476558897598068373[54] = 0.0;
   out_8476558897598068373[55] = 0.0;
   out_8476558897598068373[56] = 0.0;
   out_8476558897598068373[57] = 1.0;
   out_8476558897598068373[58] = 0.0;
   out_8476558897598068373[59] = 0.0;
   out_8476558897598068373[60] = 0.0;
   out_8476558897598068373[61] = 0.0;
   out_8476558897598068373[62] = 0.0;
   out_8476558897598068373[63] = 0.0;
   out_8476558897598068373[64] = 0.0;
   out_8476558897598068373[65] = 0.0;
   out_8476558897598068373[66] = 0.0;
   out_8476558897598068373[67] = 0.0;
   out_8476558897598068373[68] = 0.0;
   out_8476558897598068373[69] = 0.0;
   out_8476558897598068373[70] = 0.0;
   out_8476558897598068373[71] = 0.0;
   out_8476558897598068373[72] = 0.0;
   out_8476558897598068373[73] = 0.0;
   out_8476558897598068373[74] = 0.0;
   out_8476558897598068373[75] = 0.0;
   out_8476558897598068373[76] = 1.0;
   out_8476558897598068373[77] = 0.0;
   out_8476558897598068373[78] = 0.0;
   out_8476558897598068373[79] = 0.0;
   out_8476558897598068373[80] = 0.0;
   out_8476558897598068373[81] = 0.0;
   out_8476558897598068373[82] = 0.0;
   out_8476558897598068373[83] = 0.0;
   out_8476558897598068373[84] = 0.0;
   out_8476558897598068373[85] = 0.0;
   out_8476558897598068373[86] = 0.0;
   out_8476558897598068373[87] = 0.0;
   out_8476558897598068373[88] = 0.0;
   out_8476558897598068373[89] = 0.0;
   out_8476558897598068373[90] = 0.0;
   out_8476558897598068373[91] = 0.0;
   out_8476558897598068373[92] = 0.0;
   out_8476558897598068373[93] = 0.0;
   out_8476558897598068373[94] = 0.0;
   out_8476558897598068373[95] = 1.0;
   out_8476558897598068373[96] = 0.0;
   out_8476558897598068373[97] = 0.0;
   out_8476558897598068373[98] = 0.0;
   out_8476558897598068373[99] = 0.0;
   out_8476558897598068373[100] = 0.0;
   out_8476558897598068373[101] = 0.0;
   out_8476558897598068373[102] = 0.0;
   out_8476558897598068373[103] = 0.0;
   out_8476558897598068373[104] = 0.0;
   out_8476558897598068373[105] = 0.0;
   out_8476558897598068373[106] = 0.0;
   out_8476558897598068373[107] = 0.0;
   out_8476558897598068373[108] = 0.0;
   out_8476558897598068373[109] = 0.0;
   out_8476558897598068373[110] = 0.0;
   out_8476558897598068373[111] = 0.0;
   out_8476558897598068373[112] = 0.0;
   out_8476558897598068373[113] = 0.0;
   out_8476558897598068373[114] = 1.0;
   out_8476558897598068373[115] = 0.0;
   out_8476558897598068373[116] = 0.0;
   out_8476558897598068373[117] = 0.0;
   out_8476558897598068373[118] = 0.0;
   out_8476558897598068373[119] = 0.0;
   out_8476558897598068373[120] = 0.0;
   out_8476558897598068373[121] = 0.0;
   out_8476558897598068373[122] = 0.0;
   out_8476558897598068373[123] = 0.0;
   out_8476558897598068373[124] = 0.0;
   out_8476558897598068373[125] = 0.0;
   out_8476558897598068373[126] = 0.0;
   out_8476558897598068373[127] = 0.0;
   out_8476558897598068373[128] = 0.0;
   out_8476558897598068373[129] = 0.0;
   out_8476558897598068373[130] = 0.0;
   out_8476558897598068373[131] = 0.0;
   out_8476558897598068373[132] = 0.0;
   out_8476558897598068373[133] = 1.0;
   out_8476558897598068373[134] = 0.0;
   out_8476558897598068373[135] = 0.0;
   out_8476558897598068373[136] = 0.0;
   out_8476558897598068373[137] = 0.0;
   out_8476558897598068373[138] = 0.0;
   out_8476558897598068373[139] = 0.0;
   out_8476558897598068373[140] = 0.0;
   out_8476558897598068373[141] = 0.0;
   out_8476558897598068373[142] = 0.0;
   out_8476558897598068373[143] = 0.0;
   out_8476558897598068373[144] = 0.0;
   out_8476558897598068373[145] = 0.0;
   out_8476558897598068373[146] = 0.0;
   out_8476558897598068373[147] = 0.0;
   out_8476558897598068373[148] = 0.0;
   out_8476558897598068373[149] = 0.0;
   out_8476558897598068373[150] = 0.0;
   out_8476558897598068373[151] = 0.0;
   out_8476558897598068373[152] = 1.0;
   out_8476558897598068373[153] = 0.0;
   out_8476558897598068373[154] = 0.0;
   out_8476558897598068373[155] = 0.0;
   out_8476558897598068373[156] = 0.0;
   out_8476558897598068373[157] = 0.0;
   out_8476558897598068373[158] = 0.0;
   out_8476558897598068373[159] = 0.0;
   out_8476558897598068373[160] = 0.0;
   out_8476558897598068373[161] = 0.0;
   out_8476558897598068373[162] = 0.0;
   out_8476558897598068373[163] = 0.0;
   out_8476558897598068373[164] = 0.0;
   out_8476558897598068373[165] = 0.0;
   out_8476558897598068373[166] = 0.0;
   out_8476558897598068373[167] = 0.0;
   out_8476558897598068373[168] = 0.0;
   out_8476558897598068373[169] = 0.0;
   out_8476558897598068373[170] = 0.0;
   out_8476558897598068373[171] = 1.0;
   out_8476558897598068373[172] = 0.0;
   out_8476558897598068373[173] = 0.0;
   out_8476558897598068373[174] = 0.0;
   out_8476558897598068373[175] = 0.0;
   out_8476558897598068373[176] = 0.0;
   out_8476558897598068373[177] = 0.0;
   out_8476558897598068373[178] = 0.0;
   out_8476558897598068373[179] = 0.0;
   out_8476558897598068373[180] = 0.0;
   out_8476558897598068373[181] = 0.0;
   out_8476558897598068373[182] = 0.0;
   out_8476558897598068373[183] = 0.0;
   out_8476558897598068373[184] = 0.0;
   out_8476558897598068373[185] = 0.0;
   out_8476558897598068373[186] = 0.0;
   out_8476558897598068373[187] = 0.0;
   out_8476558897598068373[188] = 0.0;
   out_8476558897598068373[189] = 0.0;
   out_8476558897598068373[190] = 1.0;
   out_8476558897598068373[191] = 0.0;
   out_8476558897598068373[192] = 0.0;
   out_8476558897598068373[193] = 0.0;
   out_8476558897598068373[194] = 0.0;
   out_8476558897598068373[195] = 0.0;
   out_8476558897598068373[196] = 0.0;
   out_8476558897598068373[197] = 0.0;
   out_8476558897598068373[198] = 0.0;
   out_8476558897598068373[199] = 0.0;
   out_8476558897598068373[200] = 0.0;
   out_8476558897598068373[201] = 0.0;
   out_8476558897598068373[202] = 0.0;
   out_8476558897598068373[203] = 0.0;
   out_8476558897598068373[204] = 0.0;
   out_8476558897598068373[205] = 0.0;
   out_8476558897598068373[206] = 0.0;
   out_8476558897598068373[207] = 0.0;
   out_8476558897598068373[208] = 0.0;
   out_8476558897598068373[209] = 1.0;
   out_8476558897598068373[210] = 0.0;
   out_8476558897598068373[211] = 0.0;
   out_8476558897598068373[212] = 0.0;
   out_8476558897598068373[213] = 0.0;
   out_8476558897598068373[214] = 0.0;
   out_8476558897598068373[215] = 0.0;
   out_8476558897598068373[216] = 0.0;
   out_8476558897598068373[217] = 0.0;
   out_8476558897598068373[218] = 0.0;
   out_8476558897598068373[219] = 0.0;
   out_8476558897598068373[220] = 0.0;
   out_8476558897598068373[221] = 0.0;
   out_8476558897598068373[222] = 0.0;
   out_8476558897598068373[223] = 0.0;
   out_8476558897598068373[224] = 0.0;
   out_8476558897598068373[225] = 0.0;
   out_8476558897598068373[226] = 0.0;
   out_8476558897598068373[227] = 0.0;
   out_8476558897598068373[228] = 1.0;
   out_8476558897598068373[229] = 0.0;
   out_8476558897598068373[230] = 0.0;
   out_8476558897598068373[231] = 0.0;
   out_8476558897598068373[232] = 0.0;
   out_8476558897598068373[233] = 0.0;
   out_8476558897598068373[234] = 0.0;
   out_8476558897598068373[235] = 0.0;
   out_8476558897598068373[236] = 0.0;
   out_8476558897598068373[237] = 0.0;
   out_8476558897598068373[238] = 0.0;
   out_8476558897598068373[239] = 0.0;
   out_8476558897598068373[240] = 0.0;
   out_8476558897598068373[241] = 0.0;
   out_8476558897598068373[242] = 0.0;
   out_8476558897598068373[243] = 0.0;
   out_8476558897598068373[244] = 0.0;
   out_8476558897598068373[245] = 0.0;
   out_8476558897598068373[246] = 0.0;
   out_8476558897598068373[247] = 1.0;
   out_8476558897598068373[248] = 0.0;
   out_8476558897598068373[249] = 0.0;
   out_8476558897598068373[250] = 0.0;
   out_8476558897598068373[251] = 0.0;
   out_8476558897598068373[252] = 0.0;
   out_8476558897598068373[253] = 0.0;
   out_8476558897598068373[254] = 0.0;
   out_8476558897598068373[255] = 0.0;
   out_8476558897598068373[256] = 0.0;
   out_8476558897598068373[257] = 0.0;
   out_8476558897598068373[258] = 0.0;
   out_8476558897598068373[259] = 0.0;
   out_8476558897598068373[260] = 0.0;
   out_8476558897598068373[261] = 0.0;
   out_8476558897598068373[262] = 0.0;
   out_8476558897598068373[263] = 0.0;
   out_8476558897598068373[264] = 0.0;
   out_8476558897598068373[265] = 0.0;
   out_8476558897598068373[266] = 1.0;
   out_8476558897598068373[267] = 0.0;
   out_8476558897598068373[268] = 0.0;
   out_8476558897598068373[269] = 0.0;
   out_8476558897598068373[270] = 0.0;
   out_8476558897598068373[271] = 0.0;
   out_8476558897598068373[272] = 0.0;
   out_8476558897598068373[273] = 0.0;
   out_8476558897598068373[274] = 0.0;
   out_8476558897598068373[275] = 0.0;
   out_8476558897598068373[276] = 0.0;
   out_8476558897598068373[277] = 0.0;
   out_8476558897598068373[278] = 0.0;
   out_8476558897598068373[279] = 0.0;
   out_8476558897598068373[280] = 0.0;
   out_8476558897598068373[281] = 0.0;
   out_8476558897598068373[282] = 0.0;
   out_8476558897598068373[283] = 0.0;
   out_8476558897598068373[284] = 0.0;
   out_8476558897598068373[285] = 1.0;
   out_8476558897598068373[286] = 0.0;
   out_8476558897598068373[287] = 0.0;
   out_8476558897598068373[288] = 0.0;
   out_8476558897598068373[289] = 0.0;
   out_8476558897598068373[290] = 0.0;
   out_8476558897598068373[291] = 0.0;
   out_8476558897598068373[292] = 0.0;
   out_8476558897598068373[293] = 0.0;
   out_8476558897598068373[294] = 0.0;
   out_8476558897598068373[295] = 0.0;
   out_8476558897598068373[296] = 0.0;
   out_8476558897598068373[297] = 0.0;
   out_8476558897598068373[298] = 0.0;
   out_8476558897598068373[299] = 0.0;
   out_8476558897598068373[300] = 0.0;
   out_8476558897598068373[301] = 0.0;
   out_8476558897598068373[302] = 0.0;
   out_8476558897598068373[303] = 0.0;
   out_8476558897598068373[304] = 1.0;
   out_8476558897598068373[305] = 0.0;
   out_8476558897598068373[306] = 0.0;
   out_8476558897598068373[307] = 0.0;
   out_8476558897598068373[308] = 0.0;
   out_8476558897598068373[309] = 0.0;
   out_8476558897598068373[310] = 0.0;
   out_8476558897598068373[311] = 0.0;
   out_8476558897598068373[312] = 0.0;
   out_8476558897598068373[313] = 0.0;
   out_8476558897598068373[314] = 0.0;
   out_8476558897598068373[315] = 0.0;
   out_8476558897598068373[316] = 0.0;
   out_8476558897598068373[317] = 0.0;
   out_8476558897598068373[318] = 0.0;
   out_8476558897598068373[319] = 0.0;
   out_8476558897598068373[320] = 0.0;
   out_8476558897598068373[321] = 0.0;
   out_8476558897598068373[322] = 0.0;
   out_8476558897598068373[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_4641425989573363532) {
   out_4641425989573363532[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_4641425989573363532[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_4641425989573363532[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_4641425989573363532[3] = dt*state[12] + state[3];
   out_4641425989573363532[4] = dt*state[13] + state[4];
   out_4641425989573363532[5] = dt*state[14] + state[5];
   out_4641425989573363532[6] = state[6];
   out_4641425989573363532[7] = state[7];
   out_4641425989573363532[8] = state[8];
   out_4641425989573363532[9] = state[9];
   out_4641425989573363532[10] = state[10];
   out_4641425989573363532[11] = state[11];
   out_4641425989573363532[12] = state[12];
   out_4641425989573363532[13] = state[13];
   out_4641425989573363532[14] = state[14];
   out_4641425989573363532[15] = state[15];
   out_4641425989573363532[16] = state[16];
   out_4641425989573363532[17] = state[17];
}
void F_fun(double *state, double dt, double *out_8960990753670273174) {
   out_8960990753670273174[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8960990753670273174[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8960990753670273174[2] = 0;
   out_8960990753670273174[3] = 0;
   out_8960990753670273174[4] = 0;
   out_8960990753670273174[5] = 0;
   out_8960990753670273174[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8960990753670273174[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8960990753670273174[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8960990753670273174[9] = 0;
   out_8960990753670273174[10] = 0;
   out_8960990753670273174[11] = 0;
   out_8960990753670273174[12] = 0;
   out_8960990753670273174[13] = 0;
   out_8960990753670273174[14] = 0;
   out_8960990753670273174[15] = 0;
   out_8960990753670273174[16] = 0;
   out_8960990753670273174[17] = 0;
   out_8960990753670273174[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8960990753670273174[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8960990753670273174[20] = 0;
   out_8960990753670273174[21] = 0;
   out_8960990753670273174[22] = 0;
   out_8960990753670273174[23] = 0;
   out_8960990753670273174[24] = 0;
   out_8960990753670273174[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8960990753670273174[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8960990753670273174[27] = 0;
   out_8960990753670273174[28] = 0;
   out_8960990753670273174[29] = 0;
   out_8960990753670273174[30] = 0;
   out_8960990753670273174[31] = 0;
   out_8960990753670273174[32] = 0;
   out_8960990753670273174[33] = 0;
   out_8960990753670273174[34] = 0;
   out_8960990753670273174[35] = 0;
   out_8960990753670273174[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8960990753670273174[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8960990753670273174[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8960990753670273174[39] = 0;
   out_8960990753670273174[40] = 0;
   out_8960990753670273174[41] = 0;
   out_8960990753670273174[42] = 0;
   out_8960990753670273174[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8960990753670273174[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8960990753670273174[45] = 0;
   out_8960990753670273174[46] = 0;
   out_8960990753670273174[47] = 0;
   out_8960990753670273174[48] = 0;
   out_8960990753670273174[49] = 0;
   out_8960990753670273174[50] = 0;
   out_8960990753670273174[51] = 0;
   out_8960990753670273174[52] = 0;
   out_8960990753670273174[53] = 0;
   out_8960990753670273174[54] = 0;
   out_8960990753670273174[55] = 0;
   out_8960990753670273174[56] = 0;
   out_8960990753670273174[57] = 1;
   out_8960990753670273174[58] = 0;
   out_8960990753670273174[59] = 0;
   out_8960990753670273174[60] = 0;
   out_8960990753670273174[61] = 0;
   out_8960990753670273174[62] = 0;
   out_8960990753670273174[63] = 0;
   out_8960990753670273174[64] = 0;
   out_8960990753670273174[65] = 0;
   out_8960990753670273174[66] = dt;
   out_8960990753670273174[67] = 0;
   out_8960990753670273174[68] = 0;
   out_8960990753670273174[69] = 0;
   out_8960990753670273174[70] = 0;
   out_8960990753670273174[71] = 0;
   out_8960990753670273174[72] = 0;
   out_8960990753670273174[73] = 0;
   out_8960990753670273174[74] = 0;
   out_8960990753670273174[75] = 0;
   out_8960990753670273174[76] = 1;
   out_8960990753670273174[77] = 0;
   out_8960990753670273174[78] = 0;
   out_8960990753670273174[79] = 0;
   out_8960990753670273174[80] = 0;
   out_8960990753670273174[81] = 0;
   out_8960990753670273174[82] = 0;
   out_8960990753670273174[83] = 0;
   out_8960990753670273174[84] = 0;
   out_8960990753670273174[85] = dt;
   out_8960990753670273174[86] = 0;
   out_8960990753670273174[87] = 0;
   out_8960990753670273174[88] = 0;
   out_8960990753670273174[89] = 0;
   out_8960990753670273174[90] = 0;
   out_8960990753670273174[91] = 0;
   out_8960990753670273174[92] = 0;
   out_8960990753670273174[93] = 0;
   out_8960990753670273174[94] = 0;
   out_8960990753670273174[95] = 1;
   out_8960990753670273174[96] = 0;
   out_8960990753670273174[97] = 0;
   out_8960990753670273174[98] = 0;
   out_8960990753670273174[99] = 0;
   out_8960990753670273174[100] = 0;
   out_8960990753670273174[101] = 0;
   out_8960990753670273174[102] = 0;
   out_8960990753670273174[103] = 0;
   out_8960990753670273174[104] = dt;
   out_8960990753670273174[105] = 0;
   out_8960990753670273174[106] = 0;
   out_8960990753670273174[107] = 0;
   out_8960990753670273174[108] = 0;
   out_8960990753670273174[109] = 0;
   out_8960990753670273174[110] = 0;
   out_8960990753670273174[111] = 0;
   out_8960990753670273174[112] = 0;
   out_8960990753670273174[113] = 0;
   out_8960990753670273174[114] = 1;
   out_8960990753670273174[115] = 0;
   out_8960990753670273174[116] = 0;
   out_8960990753670273174[117] = 0;
   out_8960990753670273174[118] = 0;
   out_8960990753670273174[119] = 0;
   out_8960990753670273174[120] = 0;
   out_8960990753670273174[121] = 0;
   out_8960990753670273174[122] = 0;
   out_8960990753670273174[123] = 0;
   out_8960990753670273174[124] = 0;
   out_8960990753670273174[125] = 0;
   out_8960990753670273174[126] = 0;
   out_8960990753670273174[127] = 0;
   out_8960990753670273174[128] = 0;
   out_8960990753670273174[129] = 0;
   out_8960990753670273174[130] = 0;
   out_8960990753670273174[131] = 0;
   out_8960990753670273174[132] = 0;
   out_8960990753670273174[133] = 1;
   out_8960990753670273174[134] = 0;
   out_8960990753670273174[135] = 0;
   out_8960990753670273174[136] = 0;
   out_8960990753670273174[137] = 0;
   out_8960990753670273174[138] = 0;
   out_8960990753670273174[139] = 0;
   out_8960990753670273174[140] = 0;
   out_8960990753670273174[141] = 0;
   out_8960990753670273174[142] = 0;
   out_8960990753670273174[143] = 0;
   out_8960990753670273174[144] = 0;
   out_8960990753670273174[145] = 0;
   out_8960990753670273174[146] = 0;
   out_8960990753670273174[147] = 0;
   out_8960990753670273174[148] = 0;
   out_8960990753670273174[149] = 0;
   out_8960990753670273174[150] = 0;
   out_8960990753670273174[151] = 0;
   out_8960990753670273174[152] = 1;
   out_8960990753670273174[153] = 0;
   out_8960990753670273174[154] = 0;
   out_8960990753670273174[155] = 0;
   out_8960990753670273174[156] = 0;
   out_8960990753670273174[157] = 0;
   out_8960990753670273174[158] = 0;
   out_8960990753670273174[159] = 0;
   out_8960990753670273174[160] = 0;
   out_8960990753670273174[161] = 0;
   out_8960990753670273174[162] = 0;
   out_8960990753670273174[163] = 0;
   out_8960990753670273174[164] = 0;
   out_8960990753670273174[165] = 0;
   out_8960990753670273174[166] = 0;
   out_8960990753670273174[167] = 0;
   out_8960990753670273174[168] = 0;
   out_8960990753670273174[169] = 0;
   out_8960990753670273174[170] = 0;
   out_8960990753670273174[171] = 1;
   out_8960990753670273174[172] = 0;
   out_8960990753670273174[173] = 0;
   out_8960990753670273174[174] = 0;
   out_8960990753670273174[175] = 0;
   out_8960990753670273174[176] = 0;
   out_8960990753670273174[177] = 0;
   out_8960990753670273174[178] = 0;
   out_8960990753670273174[179] = 0;
   out_8960990753670273174[180] = 0;
   out_8960990753670273174[181] = 0;
   out_8960990753670273174[182] = 0;
   out_8960990753670273174[183] = 0;
   out_8960990753670273174[184] = 0;
   out_8960990753670273174[185] = 0;
   out_8960990753670273174[186] = 0;
   out_8960990753670273174[187] = 0;
   out_8960990753670273174[188] = 0;
   out_8960990753670273174[189] = 0;
   out_8960990753670273174[190] = 1;
   out_8960990753670273174[191] = 0;
   out_8960990753670273174[192] = 0;
   out_8960990753670273174[193] = 0;
   out_8960990753670273174[194] = 0;
   out_8960990753670273174[195] = 0;
   out_8960990753670273174[196] = 0;
   out_8960990753670273174[197] = 0;
   out_8960990753670273174[198] = 0;
   out_8960990753670273174[199] = 0;
   out_8960990753670273174[200] = 0;
   out_8960990753670273174[201] = 0;
   out_8960990753670273174[202] = 0;
   out_8960990753670273174[203] = 0;
   out_8960990753670273174[204] = 0;
   out_8960990753670273174[205] = 0;
   out_8960990753670273174[206] = 0;
   out_8960990753670273174[207] = 0;
   out_8960990753670273174[208] = 0;
   out_8960990753670273174[209] = 1;
   out_8960990753670273174[210] = 0;
   out_8960990753670273174[211] = 0;
   out_8960990753670273174[212] = 0;
   out_8960990753670273174[213] = 0;
   out_8960990753670273174[214] = 0;
   out_8960990753670273174[215] = 0;
   out_8960990753670273174[216] = 0;
   out_8960990753670273174[217] = 0;
   out_8960990753670273174[218] = 0;
   out_8960990753670273174[219] = 0;
   out_8960990753670273174[220] = 0;
   out_8960990753670273174[221] = 0;
   out_8960990753670273174[222] = 0;
   out_8960990753670273174[223] = 0;
   out_8960990753670273174[224] = 0;
   out_8960990753670273174[225] = 0;
   out_8960990753670273174[226] = 0;
   out_8960990753670273174[227] = 0;
   out_8960990753670273174[228] = 1;
   out_8960990753670273174[229] = 0;
   out_8960990753670273174[230] = 0;
   out_8960990753670273174[231] = 0;
   out_8960990753670273174[232] = 0;
   out_8960990753670273174[233] = 0;
   out_8960990753670273174[234] = 0;
   out_8960990753670273174[235] = 0;
   out_8960990753670273174[236] = 0;
   out_8960990753670273174[237] = 0;
   out_8960990753670273174[238] = 0;
   out_8960990753670273174[239] = 0;
   out_8960990753670273174[240] = 0;
   out_8960990753670273174[241] = 0;
   out_8960990753670273174[242] = 0;
   out_8960990753670273174[243] = 0;
   out_8960990753670273174[244] = 0;
   out_8960990753670273174[245] = 0;
   out_8960990753670273174[246] = 0;
   out_8960990753670273174[247] = 1;
   out_8960990753670273174[248] = 0;
   out_8960990753670273174[249] = 0;
   out_8960990753670273174[250] = 0;
   out_8960990753670273174[251] = 0;
   out_8960990753670273174[252] = 0;
   out_8960990753670273174[253] = 0;
   out_8960990753670273174[254] = 0;
   out_8960990753670273174[255] = 0;
   out_8960990753670273174[256] = 0;
   out_8960990753670273174[257] = 0;
   out_8960990753670273174[258] = 0;
   out_8960990753670273174[259] = 0;
   out_8960990753670273174[260] = 0;
   out_8960990753670273174[261] = 0;
   out_8960990753670273174[262] = 0;
   out_8960990753670273174[263] = 0;
   out_8960990753670273174[264] = 0;
   out_8960990753670273174[265] = 0;
   out_8960990753670273174[266] = 1;
   out_8960990753670273174[267] = 0;
   out_8960990753670273174[268] = 0;
   out_8960990753670273174[269] = 0;
   out_8960990753670273174[270] = 0;
   out_8960990753670273174[271] = 0;
   out_8960990753670273174[272] = 0;
   out_8960990753670273174[273] = 0;
   out_8960990753670273174[274] = 0;
   out_8960990753670273174[275] = 0;
   out_8960990753670273174[276] = 0;
   out_8960990753670273174[277] = 0;
   out_8960990753670273174[278] = 0;
   out_8960990753670273174[279] = 0;
   out_8960990753670273174[280] = 0;
   out_8960990753670273174[281] = 0;
   out_8960990753670273174[282] = 0;
   out_8960990753670273174[283] = 0;
   out_8960990753670273174[284] = 0;
   out_8960990753670273174[285] = 1;
   out_8960990753670273174[286] = 0;
   out_8960990753670273174[287] = 0;
   out_8960990753670273174[288] = 0;
   out_8960990753670273174[289] = 0;
   out_8960990753670273174[290] = 0;
   out_8960990753670273174[291] = 0;
   out_8960990753670273174[292] = 0;
   out_8960990753670273174[293] = 0;
   out_8960990753670273174[294] = 0;
   out_8960990753670273174[295] = 0;
   out_8960990753670273174[296] = 0;
   out_8960990753670273174[297] = 0;
   out_8960990753670273174[298] = 0;
   out_8960990753670273174[299] = 0;
   out_8960990753670273174[300] = 0;
   out_8960990753670273174[301] = 0;
   out_8960990753670273174[302] = 0;
   out_8960990753670273174[303] = 0;
   out_8960990753670273174[304] = 1;
   out_8960990753670273174[305] = 0;
   out_8960990753670273174[306] = 0;
   out_8960990753670273174[307] = 0;
   out_8960990753670273174[308] = 0;
   out_8960990753670273174[309] = 0;
   out_8960990753670273174[310] = 0;
   out_8960990753670273174[311] = 0;
   out_8960990753670273174[312] = 0;
   out_8960990753670273174[313] = 0;
   out_8960990753670273174[314] = 0;
   out_8960990753670273174[315] = 0;
   out_8960990753670273174[316] = 0;
   out_8960990753670273174[317] = 0;
   out_8960990753670273174[318] = 0;
   out_8960990753670273174[319] = 0;
   out_8960990753670273174[320] = 0;
   out_8960990753670273174[321] = 0;
   out_8960990753670273174[322] = 0;
   out_8960990753670273174[323] = 1;
}
void h_4(double *state, double *unused, double *out_6682348964168201837) {
   out_6682348964168201837[0] = state[6] + state[9];
   out_6682348964168201837[1] = state[7] + state[10];
   out_6682348964168201837[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_261637334484920765) {
   out_261637334484920765[0] = 0;
   out_261637334484920765[1] = 0;
   out_261637334484920765[2] = 0;
   out_261637334484920765[3] = 0;
   out_261637334484920765[4] = 0;
   out_261637334484920765[5] = 0;
   out_261637334484920765[6] = 1;
   out_261637334484920765[7] = 0;
   out_261637334484920765[8] = 0;
   out_261637334484920765[9] = 1;
   out_261637334484920765[10] = 0;
   out_261637334484920765[11] = 0;
   out_261637334484920765[12] = 0;
   out_261637334484920765[13] = 0;
   out_261637334484920765[14] = 0;
   out_261637334484920765[15] = 0;
   out_261637334484920765[16] = 0;
   out_261637334484920765[17] = 0;
   out_261637334484920765[18] = 0;
   out_261637334484920765[19] = 0;
   out_261637334484920765[20] = 0;
   out_261637334484920765[21] = 0;
   out_261637334484920765[22] = 0;
   out_261637334484920765[23] = 0;
   out_261637334484920765[24] = 0;
   out_261637334484920765[25] = 1;
   out_261637334484920765[26] = 0;
   out_261637334484920765[27] = 0;
   out_261637334484920765[28] = 1;
   out_261637334484920765[29] = 0;
   out_261637334484920765[30] = 0;
   out_261637334484920765[31] = 0;
   out_261637334484920765[32] = 0;
   out_261637334484920765[33] = 0;
   out_261637334484920765[34] = 0;
   out_261637334484920765[35] = 0;
   out_261637334484920765[36] = 0;
   out_261637334484920765[37] = 0;
   out_261637334484920765[38] = 0;
   out_261637334484920765[39] = 0;
   out_261637334484920765[40] = 0;
   out_261637334484920765[41] = 0;
   out_261637334484920765[42] = 0;
   out_261637334484920765[43] = 0;
   out_261637334484920765[44] = 1;
   out_261637334484920765[45] = 0;
   out_261637334484920765[46] = 0;
   out_261637334484920765[47] = 1;
   out_261637334484920765[48] = 0;
   out_261637334484920765[49] = 0;
   out_261637334484920765[50] = 0;
   out_261637334484920765[51] = 0;
   out_261637334484920765[52] = 0;
   out_261637334484920765[53] = 0;
}
void h_10(double *state, double *unused, double *out_5527329664152594749) {
   out_5527329664152594749[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5527329664152594749[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5527329664152594749[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_1417237105866251151) {
   out_1417237105866251151[0] = 0;
   out_1417237105866251151[1] = 9.8100000000000005*cos(state[1]);
   out_1417237105866251151[2] = 0;
   out_1417237105866251151[3] = 0;
   out_1417237105866251151[4] = -state[8];
   out_1417237105866251151[5] = state[7];
   out_1417237105866251151[6] = 0;
   out_1417237105866251151[7] = state[5];
   out_1417237105866251151[8] = -state[4];
   out_1417237105866251151[9] = 0;
   out_1417237105866251151[10] = 0;
   out_1417237105866251151[11] = 0;
   out_1417237105866251151[12] = 1;
   out_1417237105866251151[13] = 0;
   out_1417237105866251151[14] = 0;
   out_1417237105866251151[15] = 1;
   out_1417237105866251151[16] = 0;
   out_1417237105866251151[17] = 0;
   out_1417237105866251151[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_1417237105866251151[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_1417237105866251151[20] = 0;
   out_1417237105866251151[21] = state[8];
   out_1417237105866251151[22] = 0;
   out_1417237105866251151[23] = -state[6];
   out_1417237105866251151[24] = -state[5];
   out_1417237105866251151[25] = 0;
   out_1417237105866251151[26] = state[3];
   out_1417237105866251151[27] = 0;
   out_1417237105866251151[28] = 0;
   out_1417237105866251151[29] = 0;
   out_1417237105866251151[30] = 0;
   out_1417237105866251151[31] = 1;
   out_1417237105866251151[32] = 0;
   out_1417237105866251151[33] = 0;
   out_1417237105866251151[34] = 1;
   out_1417237105866251151[35] = 0;
   out_1417237105866251151[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_1417237105866251151[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_1417237105866251151[38] = 0;
   out_1417237105866251151[39] = -state[7];
   out_1417237105866251151[40] = state[6];
   out_1417237105866251151[41] = 0;
   out_1417237105866251151[42] = state[4];
   out_1417237105866251151[43] = -state[3];
   out_1417237105866251151[44] = 0;
   out_1417237105866251151[45] = 0;
   out_1417237105866251151[46] = 0;
   out_1417237105866251151[47] = 0;
   out_1417237105866251151[48] = 0;
   out_1417237105866251151[49] = 0;
   out_1417237105866251151[50] = 1;
   out_1417237105866251151[51] = 0;
   out_1417237105866251151[52] = 0;
   out_1417237105866251151[53] = 1;
}
void h_13(double *state, double *unused, double *out_7750677098364519715) {
   out_7750677098364519715[0] = state[3];
   out_7750677098364519715[1] = state[4];
   out_7750677098364519715[2] = state[5];
}
void H_13(double *state, double *unused, double *out_3473911159817253566) {
   out_3473911159817253566[0] = 0;
   out_3473911159817253566[1] = 0;
   out_3473911159817253566[2] = 0;
   out_3473911159817253566[3] = 1;
   out_3473911159817253566[4] = 0;
   out_3473911159817253566[5] = 0;
   out_3473911159817253566[6] = 0;
   out_3473911159817253566[7] = 0;
   out_3473911159817253566[8] = 0;
   out_3473911159817253566[9] = 0;
   out_3473911159817253566[10] = 0;
   out_3473911159817253566[11] = 0;
   out_3473911159817253566[12] = 0;
   out_3473911159817253566[13] = 0;
   out_3473911159817253566[14] = 0;
   out_3473911159817253566[15] = 0;
   out_3473911159817253566[16] = 0;
   out_3473911159817253566[17] = 0;
   out_3473911159817253566[18] = 0;
   out_3473911159817253566[19] = 0;
   out_3473911159817253566[20] = 0;
   out_3473911159817253566[21] = 0;
   out_3473911159817253566[22] = 1;
   out_3473911159817253566[23] = 0;
   out_3473911159817253566[24] = 0;
   out_3473911159817253566[25] = 0;
   out_3473911159817253566[26] = 0;
   out_3473911159817253566[27] = 0;
   out_3473911159817253566[28] = 0;
   out_3473911159817253566[29] = 0;
   out_3473911159817253566[30] = 0;
   out_3473911159817253566[31] = 0;
   out_3473911159817253566[32] = 0;
   out_3473911159817253566[33] = 0;
   out_3473911159817253566[34] = 0;
   out_3473911159817253566[35] = 0;
   out_3473911159817253566[36] = 0;
   out_3473911159817253566[37] = 0;
   out_3473911159817253566[38] = 0;
   out_3473911159817253566[39] = 0;
   out_3473911159817253566[40] = 0;
   out_3473911159817253566[41] = 1;
   out_3473911159817253566[42] = 0;
   out_3473911159817253566[43] = 0;
   out_3473911159817253566[44] = 0;
   out_3473911159817253566[45] = 0;
   out_3473911159817253566[46] = 0;
   out_3473911159817253566[47] = 0;
   out_3473911159817253566[48] = 0;
   out_3473911159817253566[49] = 0;
   out_3473911159817253566[50] = 0;
   out_3473911159817253566[51] = 0;
   out_3473911159817253566[52] = 0;
   out_3473911159817253566[53] = 0;
}
void h_14(double *state, double *unused, double *out_4559486557289584562) {
   out_4559486557289584562[0] = state[6];
   out_4559486557289584562[1] = state[7];
   out_4559486557289584562[2] = state[8];
}
void H_14(double *state, double *unused, double *out_4224878190824405294) {
   out_4224878190824405294[0] = 0;
   out_4224878190824405294[1] = 0;
   out_4224878190824405294[2] = 0;
   out_4224878190824405294[3] = 0;
   out_4224878190824405294[4] = 0;
   out_4224878190824405294[5] = 0;
   out_4224878190824405294[6] = 1;
   out_4224878190824405294[7] = 0;
   out_4224878190824405294[8] = 0;
   out_4224878190824405294[9] = 0;
   out_4224878190824405294[10] = 0;
   out_4224878190824405294[11] = 0;
   out_4224878190824405294[12] = 0;
   out_4224878190824405294[13] = 0;
   out_4224878190824405294[14] = 0;
   out_4224878190824405294[15] = 0;
   out_4224878190824405294[16] = 0;
   out_4224878190824405294[17] = 0;
   out_4224878190824405294[18] = 0;
   out_4224878190824405294[19] = 0;
   out_4224878190824405294[20] = 0;
   out_4224878190824405294[21] = 0;
   out_4224878190824405294[22] = 0;
   out_4224878190824405294[23] = 0;
   out_4224878190824405294[24] = 0;
   out_4224878190824405294[25] = 1;
   out_4224878190824405294[26] = 0;
   out_4224878190824405294[27] = 0;
   out_4224878190824405294[28] = 0;
   out_4224878190824405294[29] = 0;
   out_4224878190824405294[30] = 0;
   out_4224878190824405294[31] = 0;
   out_4224878190824405294[32] = 0;
   out_4224878190824405294[33] = 0;
   out_4224878190824405294[34] = 0;
   out_4224878190824405294[35] = 0;
   out_4224878190824405294[36] = 0;
   out_4224878190824405294[37] = 0;
   out_4224878190824405294[38] = 0;
   out_4224878190824405294[39] = 0;
   out_4224878190824405294[40] = 0;
   out_4224878190824405294[41] = 0;
   out_4224878190824405294[42] = 0;
   out_4224878190824405294[43] = 0;
   out_4224878190824405294[44] = 1;
   out_4224878190824405294[45] = 0;
   out_4224878190824405294[46] = 0;
   out_4224878190824405294[47] = 0;
   out_4224878190824405294[48] = 0;
   out_4224878190824405294[49] = 0;
   out_4224878190824405294[50] = 0;
   out_4224878190824405294[51] = 0;
   out_4224878190824405294[52] = 0;
   out_4224878190824405294[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_3543435702888968549) {
  err_fun(nom_x, delta_x, out_3543435702888968549);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2416974275063884114) {
  inv_err_fun(nom_x, true_x, out_2416974275063884114);
}
void pose_H_mod_fun(double *state, double *out_8476558897598068373) {
  H_mod_fun(state, out_8476558897598068373);
}
void pose_f_fun(double *state, double dt, double *out_4641425989573363532) {
  f_fun(state,  dt, out_4641425989573363532);
}
void pose_F_fun(double *state, double dt, double *out_8960990753670273174) {
  F_fun(state,  dt, out_8960990753670273174);
}
void pose_h_4(double *state, double *unused, double *out_6682348964168201837) {
  h_4(state, unused, out_6682348964168201837);
}
void pose_H_4(double *state, double *unused, double *out_261637334484920765) {
  H_4(state, unused, out_261637334484920765);
}
void pose_h_10(double *state, double *unused, double *out_5527329664152594749) {
  h_10(state, unused, out_5527329664152594749);
}
void pose_H_10(double *state, double *unused, double *out_1417237105866251151) {
  H_10(state, unused, out_1417237105866251151);
}
void pose_h_13(double *state, double *unused, double *out_7750677098364519715) {
  h_13(state, unused, out_7750677098364519715);
}
void pose_H_13(double *state, double *unused, double *out_3473911159817253566) {
  H_13(state, unused, out_3473911159817253566);
}
void pose_h_14(double *state, double *unused, double *out_4559486557289584562) {
  h_14(state, unused, out_4559486557289584562);
}
void pose_H_14(double *state, double *unused, double *out_4224878190824405294) {
  H_14(state, unused, out_4224878190824405294);
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
