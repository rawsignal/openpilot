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
void err_fun(double *nom_x, double *delta_x, double *out_8989729873251700153) {
   out_8989729873251700153[0] = delta_x[0] + nom_x[0];
   out_8989729873251700153[1] = delta_x[1] + nom_x[1];
   out_8989729873251700153[2] = delta_x[2] + nom_x[2];
   out_8989729873251700153[3] = delta_x[3] + nom_x[3];
   out_8989729873251700153[4] = delta_x[4] + nom_x[4];
   out_8989729873251700153[5] = delta_x[5] + nom_x[5];
   out_8989729873251700153[6] = delta_x[6] + nom_x[6];
   out_8989729873251700153[7] = delta_x[7] + nom_x[7];
   out_8989729873251700153[8] = delta_x[8] + nom_x[8];
   out_8989729873251700153[9] = delta_x[9] + nom_x[9];
   out_8989729873251700153[10] = delta_x[10] + nom_x[10];
   out_8989729873251700153[11] = delta_x[11] + nom_x[11];
   out_8989729873251700153[12] = delta_x[12] + nom_x[12];
   out_8989729873251700153[13] = delta_x[13] + nom_x[13];
   out_8989729873251700153[14] = delta_x[14] + nom_x[14];
   out_8989729873251700153[15] = delta_x[15] + nom_x[15];
   out_8989729873251700153[16] = delta_x[16] + nom_x[16];
   out_8989729873251700153[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5494395439268003082) {
   out_5494395439268003082[0] = -nom_x[0] + true_x[0];
   out_5494395439268003082[1] = -nom_x[1] + true_x[1];
   out_5494395439268003082[2] = -nom_x[2] + true_x[2];
   out_5494395439268003082[3] = -nom_x[3] + true_x[3];
   out_5494395439268003082[4] = -nom_x[4] + true_x[4];
   out_5494395439268003082[5] = -nom_x[5] + true_x[5];
   out_5494395439268003082[6] = -nom_x[6] + true_x[6];
   out_5494395439268003082[7] = -nom_x[7] + true_x[7];
   out_5494395439268003082[8] = -nom_x[8] + true_x[8];
   out_5494395439268003082[9] = -nom_x[9] + true_x[9];
   out_5494395439268003082[10] = -nom_x[10] + true_x[10];
   out_5494395439268003082[11] = -nom_x[11] + true_x[11];
   out_5494395439268003082[12] = -nom_x[12] + true_x[12];
   out_5494395439268003082[13] = -nom_x[13] + true_x[13];
   out_5494395439268003082[14] = -nom_x[14] + true_x[14];
   out_5494395439268003082[15] = -nom_x[15] + true_x[15];
   out_5494395439268003082[16] = -nom_x[16] + true_x[16];
   out_5494395439268003082[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4818367292979666309) {
   out_4818367292979666309[0] = 1.0;
   out_4818367292979666309[1] = 0.0;
   out_4818367292979666309[2] = 0.0;
   out_4818367292979666309[3] = 0.0;
   out_4818367292979666309[4] = 0.0;
   out_4818367292979666309[5] = 0.0;
   out_4818367292979666309[6] = 0.0;
   out_4818367292979666309[7] = 0.0;
   out_4818367292979666309[8] = 0.0;
   out_4818367292979666309[9] = 0.0;
   out_4818367292979666309[10] = 0.0;
   out_4818367292979666309[11] = 0.0;
   out_4818367292979666309[12] = 0.0;
   out_4818367292979666309[13] = 0.0;
   out_4818367292979666309[14] = 0.0;
   out_4818367292979666309[15] = 0.0;
   out_4818367292979666309[16] = 0.0;
   out_4818367292979666309[17] = 0.0;
   out_4818367292979666309[18] = 0.0;
   out_4818367292979666309[19] = 1.0;
   out_4818367292979666309[20] = 0.0;
   out_4818367292979666309[21] = 0.0;
   out_4818367292979666309[22] = 0.0;
   out_4818367292979666309[23] = 0.0;
   out_4818367292979666309[24] = 0.0;
   out_4818367292979666309[25] = 0.0;
   out_4818367292979666309[26] = 0.0;
   out_4818367292979666309[27] = 0.0;
   out_4818367292979666309[28] = 0.0;
   out_4818367292979666309[29] = 0.0;
   out_4818367292979666309[30] = 0.0;
   out_4818367292979666309[31] = 0.0;
   out_4818367292979666309[32] = 0.0;
   out_4818367292979666309[33] = 0.0;
   out_4818367292979666309[34] = 0.0;
   out_4818367292979666309[35] = 0.0;
   out_4818367292979666309[36] = 0.0;
   out_4818367292979666309[37] = 0.0;
   out_4818367292979666309[38] = 1.0;
   out_4818367292979666309[39] = 0.0;
   out_4818367292979666309[40] = 0.0;
   out_4818367292979666309[41] = 0.0;
   out_4818367292979666309[42] = 0.0;
   out_4818367292979666309[43] = 0.0;
   out_4818367292979666309[44] = 0.0;
   out_4818367292979666309[45] = 0.0;
   out_4818367292979666309[46] = 0.0;
   out_4818367292979666309[47] = 0.0;
   out_4818367292979666309[48] = 0.0;
   out_4818367292979666309[49] = 0.0;
   out_4818367292979666309[50] = 0.0;
   out_4818367292979666309[51] = 0.0;
   out_4818367292979666309[52] = 0.0;
   out_4818367292979666309[53] = 0.0;
   out_4818367292979666309[54] = 0.0;
   out_4818367292979666309[55] = 0.0;
   out_4818367292979666309[56] = 0.0;
   out_4818367292979666309[57] = 1.0;
   out_4818367292979666309[58] = 0.0;
   out_4818367292979666309[59] = 0.0;
   out_4818367292979666309[60] = 0.0;
   out_4818367292979666309[61] = 0.0;
   out_4818367292979666309[62] = 0.0;
   out_4818367292979666309[63] = 0.0;
   out_4818367292979666309[64] = 0.0;
   out_4818367292979666309[65] = 0.0;
   out_4818367292979666309[66] = 0.0;
   out_4818367292979666309[67] = 0.0;
   out_4818367292979666309[68] = 0.0;
   out_4818367292979666309[69] = 0.0;
   out_4818367292979666309[70] = 0.0;
   out_4818367292979666309[71] = 0.0;
   out_4818367292979666309[72] = 0.0;
   out_4818367292979666309[73] = 0.0;
   out_4818367292979666309[74] = 0.0;
   out_4818367292979666309[75] = 0.0;
   out_4818367292979666309[76] = 1.0;
   out_4818367292979666309[77] = 0.0;
   out_4818367292979666309[78] = 0.0;
   out_4818367292979666309[79] = 0.0;
   out_4818367292979666309[80] = 0.0;
   out_4818367292979666309[81] = 0.0;
   out_4818367292979666309[82] = 0.0;
   out_4818367292979666309[83] = 0.0;
   out_4818367292979666309[84] = 0.0;
   out_4818367292979666309[85] = 0.0;
   out_4818367292979666309[86] = 0.0;
   out_4818367292979666309[87] = 0.0;
   out_4818367292979666309[88] = 0.0;
   out_4818367292979666309[89] = 0.0;
   out_4818367292979666309[90] = 0.0;
   out_4818367292979666309[91] = 0.0;
   out_4818367292979666309[92] = 0.0;
   out_4818367292979666309[93] = 0.0;
   out_4818367292979666309[94] = 0.0;
   out_4818367292979666309[95] = 1.0;
   out_4818367292979666309[96] = 0.0;
   out_4818367292979666309[97] = 0.0;
   out_4818367292979666309[98] = 0.0;
   out_4818367292979666309[99] = 0.0;
   out_4818367292979666309[100] = 0.0;
   out_4818367292979666309[101] = 0.0;
   out_4818367292979666309[102] = 0.0;
   out_4818367292979666309[103] = 0.0;
   out_4818367292979666309[104] = 0.0;
   out_4818367292979666309[105] = 0.0;
   out_4818367292979666309[106] = 0.0;
   out_4818367292979666309[107] = 0.0;
   out_4818367292979666309[108] = 0.0;
   out_4818367292979666309[109] = 0.0;
   out_4818367292979666309[110] = 0.0;
   out_4818367292979666309[111] = 0.0;
   out_4818367292979666309[112] = 0.0;
   out_4818367292979666309[113] = 0.0;
   out_4818367292979666309[114] = 1.0;
   out_4818367292979666309[115] = 0.0;
   out_4818367292979666309[116] = 0.0;
   out_4818367292979666309[117] = 0.0;
   out_4818367292979666309[118] = 0.0;
   out_4818367292979666309[119] = 0.0;
   out_4818367292979666309[120] = 0.0;
   out_4818367292979666309[121] = 0.0;
   out_4818367292979666309[122] = 0.0;
   out_4818367292979666309[123] = 0.0;
   out_4818367292979666309[124] = 0.0;
   out_4818367292979666309[125] = 0.0;
   out_4818367292979666309[126] = 0.0;
   out_4818367292979666309[127] = 0.0;
   out_4818367292979666309[128] = 0.0;
   out_4818367292979666309[129] = 0.0;
   out_4818367292979666309[130] = 0.0;
   out_4818367292979666309[131] = 0.0;
   out_4818367292979666309[132] = 0.0;
   out_4818367292979666309[133] = 1.0;
   out_4818367292979666309[134] = 0.0;
   out_4818367292979666309[135] = 0.0;
   out_4818367292979666309[136] = 0.0;
   out_4818367292979666309[137] = 0.0;
   out_4818367292979666309[138] = 0.0;
   out_4818367292979666309[139] = 0.0;
   out_4818367292979666309[140] = 0.0;
   out_4818367292979666309[141] = 0.0;
   out_4818367292979666309[142] = 0.0;
   out_4818367292979666309[143] = 0.0;
   out_4818367292979666309[144] = 0.0;
   out_4818367292979666309[145] = 0.0;
   out_4818367292979666309[146] = 0.0;
   out_4818367292979666309[147] = 0.0;
   out_4818367292979666309[148] = 0.0;
   out_4818367292979666309[149] = 0.0;
   out_4818367292979666309[150] = 0.0;
   out_4818367292979666309[151] = 0.0;
   out_4818367292979666309[152] = 1.0;
   out_4818367292979666309[153] = 0.0;
   out_4818367292979666309[154] = 0.0;
   out_4818367292979666309[155] = 0.0;
   out_4818367292979666309[156] = 0.0;
   out_4818367292979666309[157] = 0.0;
   out_4818367292979666309[158] = 0.0;
   out_4818367292979666309[159] = 0.0;
   out_4818367292979666309[160] = 0.0;
   out_4818367292979666309[161] = 0.0;
   out_4818367292979666309[162] = 0.0;
   out_4818367292979666309[163] = 0.0;
   out_4818367292979666309[164] = 0.0;
   out_4818367292979666309[165] = 0.0;
   out_4818367292979666309[166] = 0.0;
   out_4818367292979666309[167] = 0.0;
   out_4818367292979666309[168] = 0.0;
   out_4818367292979666309[169] = 0.0;
   out_4818367292979666309[170] = 0.0;
   out_4818367292979666309[171] = 1.0;
   out_4818367292979666309[172] = 0.0;
   out_4818367292979666309[173] = 0.0;
   out_4818367292979666309[174] = 0.0;
   out_4818367292979666309[175] = 0.0;
   out_4818367292979666309[176] = 0.0;
   out_4818367292979666309[177] = 0.0;
   out_4818367292979666309[178] = 0.0;
   out_4818367292979666309[179] = 0.0;
   out_4818367292979666309[180] = 0.0;
   out_4818367292979666309[181] = 0.0;
   out_4818367292979666309[182] = 0.0;
   out_4818367292979666309[183] = 0.0;
   out_4818367292979666309[184] = 0.0;
   out_4818367292979666309[185] = 0.0;
   out_4818367292979666309[186] = 0.0;
   out_4818367292979666309[187] = 0.0;
   out_4818367292979666309[188] = 0.0;
   out_4818367292979666309[189] = 0.0;
   out_4818367292979666309[190] = 1.0;
   out_4818367292979666309[191] = 0.0;
   out_4818367292979666309[192] = 0.0;
   out_4818367292979666309[193] = 0.0;
   out_4818367292979666309[194] = 0.0;
   out_4818367292979666309[195] = 0.0;
   out_4818367292979666309[196] = 0.0;
   out_4818367292979666309[197] = 0.0;
   out_4818367292979666309[198] = 0.0;
   out_4818367292979666309[199] = 0.0;
   out_4818367292979666309[200] = 0.0;
   out_4818367292979666309[201] = 0.0;
   out_4818367292979666309[202] = 0.0;
   out_4818367292979666309[203] = 0.0;
   out_4818367292979666309[204] = 0.0;
   out_4818367292979666309[205] = 0.0;
   out_4818367292979666309[206] = 0.0;
   out_4818367292979666309[207] = 0.0;
   out_4818367292979666309[208] = 0.0;
   out_4818367292979666309[209] = 1.0;
   out_4818367292979666309[210] = 0.0;
   out_4818367292979666309[211] = 0.0;
   out_4818367292979666309[212] = 0.0;
   out_4818367292979666309[213] = 0.0;
   out_4818367292979666309[214] = 0.0;
   out_4818367292979666309[215] = 0.0;
   out_4818367292979666309[216] = 0.0;
   out_4818367292979666309[217] = 0.0;
   out_4818367292979666309[218] = 0.0;
   out_4818367292979666309[219] = 0.0;
   out_4818367292979666309[220] = 0.0;
   out_4818367292979666309[221] = 0.0;
   out_4818367292979666309[222] = 0.0;
   out_4818367292979666309[223] = 0.0;
   out_4818367292979666309[224] = 0.0;
   out_4818367292979666309[225] = 0.0;
   out_4818367292979666309[226] = 0.0;
   out_4818367292979666309[227] = 0.0;
   out_4818367292979666309[228] = 1.0;
   out_4818367292979666309[229] = 0.0;
   out_4818367292979666309[230] = 0.0;
   out_4818367292979666309[231] = 0.0;
   out_4818367292979666309[232] = 0.0;
   out_4818367292979666309[233] = 0.0;
   out_4818367292979666309[234] = 0.0;
   out_4818367292979666309[235] = 0.0;
   out_4818367292979666309[236] = 0.0;
   out_4818367292979666309[237] = 0.0;
   out_4818367292979666309[238] = 0.0;
   out_4818367292979666309[239] = 0.0;
   out_4818367292979666309[240] = 0.0;
   out_4818367292979666309[241] = 0.0;
   out_4818367292979666309[242] = 0.0;
   out_4818367292979666309[243] = 0.0;
   out_4818367292979666309[244] = 0.0;
   out_4818367292979666309[245] = 0.0;
   out_4818367292979666309[246] = 0.0;
   out_4818367292979666309[247] = 1.0;
   out_4818367292979666309[248] = 0.0;
   out_4818367292979666309[249] = 0.0;
   out_4818367292979666309[250] = 0.0;
   out_4818367292979666309[251] = 0.0;
   out_4818367292979666309[252] = 0.0;
   out_4818367292979666309[253] = 0.0;
   out_4818367292979666309[254] = 0.0;
   out_4818367292979666309[255] = 0.0;
   out_4818367292979666309[256] = 0.0;
   out_4818367292979666309[257] = 0.0;
   out_4818367292979666309[258] = 0.0;
   out_4818367292979666309[259] = 0.0;
   out_4818367292979666309[260] = 0.0;
   out_4818367292979666309[261] = 0.0;
   out_4818367292979666309[262] = 0.0;
   out_4818367292979666309[263] = 0.0;
   out_4818367292979666309[264] = 0.0;
   out_4818367292979666309[265] = 0.0;
   out_4818367292979666309[266] = 1.0;
   out_4818367292979666309[267] = 0.0;
   out_4818367292979666309[268] = 0.0;
   out_4818367292979666309[269] = 0.0;
   out_4818367292979666309[270] = 0.0;
   out_4818367292979666309[271] = 0.0;
   out_4818367292979666309[272] = 0.0;
   out_4818367292979666309[273] = 0.0;
   out_4818367292979666309[274] = 0.0;
   out_4818367292979666309[275] = 0.0;
   out_4818367292979666309[276] = 0.0;
   out_4818367292979666309[277] = 0.0;
   out_4818367292979666309[278] = 0.0;
   out_4818367292979666309[279] = 0.0;
   out_4818367292979666309[280] = 0.0;
   out_4818367292979666309[281] = 0.0;
   out_4818367292979666309[282] = 0.0;
   out_4818367292979666309[283] = 0.0;
   out_4818367292979666309[284] = 0.0;
   out_4818367292979666309[285] = 1.0;
   out_4818367292979666309[286] = 0.0;
   out_4818367292979666309[287] = 0.0;
   out_4818367292979666309[288] = 0.0;
   out_4818367292979666309[289] = 0.0;
   out_4818367292979666309[290] = 0.0;
   out_4818367292979666309[291] = 0.0;
   out_4818367292979666309[292] = 0.0;
   out_4818367292979666309[293] = 0.0;
   out_4818367292979666309[294] = 0.0;
   out_4818367292979666309[295] = 0.0;
   out_4818367292979666309[296] = 0.0;
   out_4818367292979666309[297] = 0.0;
   out_4818367292979666309[298] = 0.0;
   out_4818367292979666309[299] = 0.0;
   out_4818367292979666309[300] = 0.0;
   out_4818367292979666309[301] = 0.0;
   out_4818367292979666309[302] = 0.0;
   out_4818367292979666309[303] = 0.0;
   out_4818367292979666309[304] = 1.0;
   out_4818367292979666309[305] = 0.0;
   out_4818367292979666309[306] = 0.0;
   out_4818367292979666309[307] = 0.0;
   out_4818367292979666309[308] = 0.0;
   out_4818367292979666309[309] = 0.0;
   out_4818367292979666309[310] = 0.0;
   out_4818367292979666309[311] = 0.0;
   out_4818367292979666309[312] = 0.0;
   out_4818367292979666309[313] = 0.0;
   out_4818367292979666309[314] = 0.0;
   out_4818367292979666309[315] = 0.0;
   out_4818367292979666309[316] = 0.0;
   out_4818367292979666309[317] = 0.0;
   out_4818367292979666309[318] = 0.0;
   out_4818367292979666309[319] = 0.0;
   out_4818367292979666309[320] = 0.0;
   out_4818367292979666309[321] = 0.0;
   out_4818367292979666309[322] = 0.0;
   out_4818367292979666309[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5283112497490037971) {
   out_5283112497490037971[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5283112497490037971[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5283112497490037971[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5283112497490037971[3] = dt*state[12] + state[3];
   out_5283112497490037971[4] = dt*state[13] + state[4];
   out_5283112497490037971[5] = dt*state[14] + state[5];
   out_5283112497490037971[6] = state[6];
   out_5283112497490037971[7] = state[7];
   out_5283112497490037971[8] = state[8];
   out_5283112497490037971[9] = state[9];
   out_5283112497490037971[10] = state[10];
   out_5283112497490037971[11] = state[11];
   out_5283112497490037971[12] = state[12];
   out_5283112497490037971[13] = state[13];
   out_5283112497490037971[14] = state[14];
   out_5283112497490037971[15] = state[15];
   out_5283112497490037971[16] = state[16];
   out_5283112497490037971[17] = state[17];
}
void F_fun(double *state, double dt, double *out_4041536850565338045) {
   out_4041536850565338045[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4041536850565338045[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4041536850565338045[2] = 0;
   out_4041536850565338045[3] = 0;
   out_4041536850565338045[4] = 0;
   out_4041536850565338045[5] = 0;
   out_4041536850565338045[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4041536850565338045[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4041536850565338045[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4041536850565338045[9] = 0;
   out_4041536850565338045[10] = 0;
   out_4041536850565338045[11] = 0;
   out_4041536850565338045[12] = 0;
   out_4041536850565338045[13] = 0;
   out_4041536850565338045[14] = 0;
   out_4041536850565338045[15] = 0;
   out_4041536850565338045[16] = 0;
   out_4041536850565338045[17] = 0;
   out_4041536850565338045[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4041536850565338045[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4041536850565338045[20] = 0;
   out_4041536850565338045[21] = 0;
   out_4041536850565338045[22] = 0;
   out_4041536850565338045[23] = 0;
   out_4041536850565338045[24] = 0;
   out_4041536850565338045[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4041536850565338045[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4041536850565338045[27] = 0;
   out_4041536850565338045[28] = 0;
   out_4041536850565338045[29] = 0;
   out_4041536850565338045[30] = 0;
   out_4041536850565338045[31] = 0;
   out_4041536850565338045[32] = 0;
   out_4041536850565338045[33] = 0;
   out_4041536850565338045[34] = 0;
   out_4041536850565338045[35] = 0;
   out_4041536850565338045[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4041536850565338045[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4041536850565338045[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4041536850565338045[39] = 0;
   out_4041536850565338045[40] = 0;
   out_4041536850565338045[41] = 0;
   out_4041536850565338045[42] = 0;
   out_4041536850565338045[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4041536850565338045[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4041536850565338045[45] = 0;
   out_4041536850565338045[46] = 0;
   out_4041536850565338045[47] = 0;
   out_4041536850565338045[48] = 0;
   out_4041536850565338045[49] = 0;
   out_4041536850565338045[50] = 0;
   out_4041536850565338045[51] = 0;
   out_4041536850565338045[52] = 0;
   out_4041536850565338045[53] = 0;
   out_4041536850565338045[54] = 0;
   out_4041536850565338045[55] = 0;
   out_4041536850565338045[56] = 0;
   out_4041536850565338045[57] = 1;
   out_4041536850565338045[58] = 0;
   out_4041536850565338045[59] = 0;
   out_4041536850565338045[60] = 0;
   out_4041536850565338045[61] = 0;
   out_4041536850565338045[62] = 0;
   out_4041536850565338045[63] = 0;
   out_4041536850565338045[64] = 0;
   out_4041536850565338045[65] = 0;
   out_4041536850565338045[66] = dt;
   out_4041536850565338045[67] = 0;
   out_4041536850565338045[68] = 0;
   out_4041536850565338045[69] = 0;
   out_4041536850565338045[70] = 0;
   out_4041536850565338045[71] = 0;
   out_4041536850565338045[72] = 0;
   out_4041536850565338045[73] = 0;
   out_4041536850565338045[74] = 0;
   out_4041536850565338045[75] = 0;
   out_4041536850565338045[76] = 1;
   out_4041536850565338045[77] = 0;
   out_4041536850565338045[78] = 0;
   out_4041536850565338045[79] = 0;
   out_4041536850565338045[80] = 0;
   out_4041536850565338045[81] = 0;
   out_4041536850565338045[82] = 0;
   out_4041536850565338045[83] = 0;
   out_4041536850565338045[84] = 0;
   out_4041536850565338045[85] = dt;
   out_4041536850565338045[86] = 0;
   out_4041536850565338045[87] = 0;
   out_4041536850565338045[88] = 0;
   out_4041536850565338045[89] = 0;
   out_4041536850565338045[90] = 0;
   out_4041536850565338045[91] = 0;
   out_4041536850565338045[92] = 0;
   out_4041536850565338045[93] = 0;
   out_4041536850565338045[94] = 0;
   out_4041536850565338045[95] = 1;
   out_4041536850565338045[96] = 0;
   out_4041536850565338045[97] = 0;
   out_4041536850565338045[98] = 0;
   out_4041536850565338045[99] = 0;
   out_4041536850565338045[100] = 0;
   out_4041536850565338045[101] = 0;
   out_4041536850565338045[102] = 0;
   out_4041536850565338045[103] = 0;
   out_4041536850565338045[104] = dt;
   out_4041536850565338045[105] = 0;
   out_4041536850565338045[106] = 0;
   out_4041536850565338045[107] = 0;
   out_4041536850565338045[108] = 0;
   out_4041536850565338045[109] = 0;
   out_4041536850565338045[110] = 0;
   out_4041536850565338045[111] = 0;
   out_4041536850565338045[112] = 0;
   out_4041536850565338045[113] = 0;
   out_4041536850565338045[114] = 1;
   out_4041536850565338045[115] = 0;
   out_4041536850565338045[116] = 0;
   out_4041536850565338045[117] = 0;
   out_4041536850565338045[118] = 0;
   out_4041536850565338045[119] = 0;
   out_4041536850565338045[120] = 0;
   out_4041536850565338045[121] = 0;
   out_4041536850565338045[122] = 0;
   out_4041536850565338045[123] = 0;
   out_4041536850565338045[124] = 0;
   out_4041536850565338045[125] = 0;
   out_4041536850565338045[126] = 0;
   out_4041536850565338045[127] = 0;
   out_4041536850565338045[128] = 0;
   out_4041536850565338045[129] = 0;
   out_4041536850565338045[130] = 0;
   out_4041536850565338045[131] = 0;
   out_4041536850565338045[132] = 0;
   out_4041536850565338045[133] = 1;
   out_4041536850565338045[134] = 0;
   out_4041536850565338045[135] = 0;
   out_4041536850565338045[136] = 0;
   out_4041536850565338045[137] = 0;
   out_4041536850565338045[138] = 0;
   out_4041536850565338045[139] = 0;
   out_4041536850565338045[140] = 0;
   out_4041536850565338045[141] = 0;
   out_4041536850565338045[142] = 0;
   out_4041536850565338045[143] = 0;
   out_4041536850565338045[144] = 0;
   out_4041536850565338045[145] = 0;
   out_4041536850565338045[146] = 0;
   out_4041536850565338045[147] = 0;
   out_4041536850565338045[148] = 0;
   out_4041536850565338045[149] = 0;
   out_4041536850565338045[150] = 0;
   out_4041536850565338045[151] = 0;
   out_4041536850565338045[152] = 1;
   out_4041536850565338045[153] = 0;
   out_4041536850565338045[154] = 0;
   out_4041536850565338045[155] = 0;
   out_4041536850565338045[156] = 0;
   out_4041536850565338045[157] = 0;
   out_4041536850565338045[158] = 0;
   out_4041536850565338045[159] = 0;
   out_4041536850565338045[160] = 0;
   out_4041536850565338045[161] = 0;
   out_4041536850565338045[162] = 0;
   out_4041536850565338045[163] = 0;
   out_4041536850565338045[164] = 0;
   out_4041536850565338045[165] = 0;
   out_4041536850565338045[166] = 0;
   out_4041536850565338045[167] = 0;
   out_4041536850565338045[168] = 0;
   out_4041536850565338045[169] = 0;
   out_4041536850565338045[170] = 0;
   out_4041536850565338045[171] = 1;
   out_4041536850565338045[172] = 0;
   out_4041536850565338045[173] = 0;
   out_4041536850565338045[174] = 0;
   out_4041536850565338045[175] = 0;
   out_4041536850565338045[176] = 0;
   out_4041536850565338045[177] = 0;
   out_4041536850565338045[178] = 0;
   out_4041536850565338045[179] = 0;
   out_4041536850565338045[180] = 0;
   out_4041536850565338045[181] = 0;
   out_4041536850565338045[182] = 0;
   out_4041536850565338045[183] = 0;
   out_4041536850565338045[184] = 0;
   out_4041536850565338045[185] = 0;
   out_4041536850565338045[186] = 0;
   out_4041536850565338045[187] = 0;
   out_4041536850565338045[188] = 0;
   out_4041536850565338045[189] = 0;
   out_4041536850565338045[190] = 1;
   out_4041536850565338045[191] = 0;
   out_4041536850565338045[192] = 0;
   out_4041536850565338045[193] = 0;
   out_4041536850565338045[194] = 0;
   out_4041536850565338045[195] = 0;
   out_4041536850565338045[196] = 0;
   out_4041536850565338045[197] = 0;
   out_4041536850565338045[198] = 0;
   out_4041536850565338045[199] = 0;
   out_4041536850565338045[200] = 0;
   out_4041536850565338045[201] = 0;
   out_4041536850565338045[202] = 0;
   out_4041536850565338045[203] = 0;
   out_4041536850565338045[204] = 0;
   out_4041536850565338045[205] = 0;
   out_4041536850565338045[206] = 0;
   out_4041536850565338045[207] = 0;
   out_4041536850565338045[208] = 0;
   out_4041536850565338045[209] = 1;
   out_4041536850565338045[210] = 0;
   out_4041536850565338045[211] = 0;
   out_4041536850565338045[212] = 0;
   out_4041536850565338045[213] = 0;
   out_4041536850565338045[214] = 0;
   out_4041536850565338045[215] = 0;
   out_4041536850565338045[216] = 0;
   out_4041536850565338045[217] = 0;
   out_4041536850565338045[218] = 0;
   out_4041536850565338045[219] = 0;
   out_4041536850565338045[220] = 0;
   out_4041536850565338045[221] = 0;
   out_4041536850565338045[222] = 0;
   out_4041536850565338045[223] = 0;
   out_4041536850565338045[224] = 0;
   out_4041536850565338045[225] = 0;
   out_4041536850565338045[226] = 0;
   out_4041536850565338045[227] = 0;
   out_4041536850565338045[228] = 1;
   out_4041536850565338045[229] = 0;
   out_4041536850565338045[230] = 0;
   out_4041536850565338045[231] = 0;
   out_4041536850565338045[232] = 0;
   out_4041536850565338045[233] = 0;
   out_4041536850565338045[234] = 0;
   out_4041536850565338045[235] = 0;
   out_4041536850565338045[236] = 0;
   out_4041536850565338045[237] = 0;
   out_4041536850565338045[238] = 0;
   out_4041536850565338045[239] = 0;
   out_4041536850565338045[240] = 0;
   out_4041536850565338045[241] = 0;
   out_4041536850565338045[242] = 0;
   out_4041536850565338045[243] = 0;
   out_4041536850565338045[244] = 0;
   out_4041536850565338045[245] = 0;
   out_4041536850565338045[246] = 0;
   out_4041536850565338045[247] = 1;
   out_4041536850565338045[248] = 0;
   out_4041536850565338045[249] = 0;
   out_4041536850565338045[250] = 0;
   out_4041536850565338045[251] = 0;
   out_4041536850565338045[252] = 0;
   out_4041536850565338045[253] = 0;
   out_4041536850565338045[254] = 0;
   out_4041536850565338045[255] = 0;
   out_4041536850565338045[256] = 0;
   out_4041536850565338045[257] = 0;
   out_4041536850565338045[258] = 0;
   out_4041536850565338045[259] = 0;
   out_4041536850565338045[260] = 0;
   out_4041536850565338045[261] = 0;
   out_4041536850565338045[262] = 0;
   out_4041536850565338045[263] = 0;
   out_4041536850565338045[264] = 0;
   out_4041536850565338045[265] = 0;
   out_4041536850565338045[266] = 1;
   out_4041536850565338045[267] = 0;
   out_4041536850565338045[268] = 0;
   out_4041536850565338045[269] = 0;
   out_4041536850565338045[270] = 0;
   out_4041536850565338045[271] = 0;
   out_4041536850565338045[272] = 0;
   out_4041536850565338045[273] = 0;
   out_4041536850565338045[274] = 0;
   out_4041536850565338045[275] = 0;
   out_4041536850565338045[276] = 0;
   out_4041536850565338045[277] = 0;
   out_4041536850565338045[278] = 0;
   out_4041536850565338045[279] = 0;
   out_4041536850565338045[280] = 0;
   out_4041536850565338045[281] = 0;
   out_4041536850565338045[282] = 0;
   out_4041536850565338045[283] = 0;
   out_4041536850565338045[284] = 0;
   out_4041536850565338045[285] = 1;
   out_4041536850565338045[286] = 0;
   out_4041536850565338045[287] = 0;
   out_4041536850565338045[288] = 0;
   out_4041536850565338045[289] = 0;
   out_4041536850565338045[290] = 0;
   out_4041536850565338045[291] = 0;
   out_4041536850565338045[292] = 0;
   out_4041536850565338045[293] = 0;
   out_4041536850565338045[294] = 0;
   out_4041536850565338045[295] = 0;
   out_4041536850565338045[296] = 0;
   out_4041536850565338045[297] = 0;
   out_4041536850565338045[298] = 0;
   out_4041536850565338045[299] = 0;
   out_4041536850565338045[300] = 0;
   out_4041536850565338045[301] = 0;
   out_4041536850565338045[302] = 0;
   out_4041536850565338045[303] = 0;
   out_4041536850565338045[304] = 1;
   out_4041536850565338045[305] = 0;
   out_4041536850565338045[306] = 0;
   out_4041536850565338045[307] = 0;
   out_4041536850565338045[308] = 0;
   out_4041536850565338045[309] = 0;
   out_4041536850565338045[310] = 0;
   out_4041536850565338045[311] = 0;
   out_4041536850565338045[312] = 0;
   out_4041536850565338045[313] = 0;
   out_4041536850565338045[314] = 0;
   out_4041536850565338045[315] = 0;
   out_4041536850565338045[316] = 0;
   out_4041536850565338045[317] = 0;
   out_4041536850565338045[318] = 0;
   out_4041536850565338045[319] = 0;
   out_4041536850565338045[320] = 0;
   out_4041536850565338045[321] = 0;
   out_4041536850565338045[322] = 0;
   out_4041536850565338045[323] = 1;
}
void h_4(double *state, double *unused, double *out_2536598407606234528) {
   out_2536598407606234528[0] = state[6] + state[9];
   out_2536598407606234528[1] = state[7] + state[10];
   out_2536598407606234528[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2430636927818656010) {
   out_2430636927818656010[0] = 0;
   out_2430636927818656010[1] = 0;
   out_2430636927818656010[2] = 0;
   out_2430636927818656010[3] = 0;
   out_2430636927818656010[4] = 0;
   out_2430636927818656010[5] = 0;
   out_2430636927818656010[6] = 1;
   out_2430636927818656010[7] = 0;
   out_2430636927818656010[8] = 0;
   out_2430636927818656010[9] = 1;
   out_2430636927818656010[10] = 0;
   out_2430636927818656010[11] = 0;
   out_2430636927818656010[12] = 0;
   out_2430636927818656010[13] = 0;
   out_2430636927818656010[14] = 0;
   out_2430636927818656010[15] = 0;
   out_2430636927818656010[16] = 0;
   out_2430636927818656010[17] = 0;
   out_2430636927818656010[18] = 0;
   out_2430636927818656010[19] = 0;
   out_2430636927818656010[20] = 0;
   out_2430636927818656010[21] = 0;
   out_2430636927818656010[22] = 0;
   out_2430636927818656010[23] = 0;
   out_2430636927818656010[24] = 0;
   out_2430636927818656010[25] = 1;
   out_2430636927818656010[26] = 0;
   out_2430636927818656010[27] = 0;
   out_2430636927818656010[28] = 1;
   out_2430636927818656010[29] = 0;
   out_2430636927818656010[30] = 0;
   out_2430636927818656010[31] = 0;
   out_2430636927818656010[32] = 0;
   out_2430636927818656010[33] = 0;
   out_2430636927818656010[34] = 0;
   out_2430636927818656010[35] = 0;
   out_2430636927818656010[36] = 0;
   out_2430636927818656010[37] = 0;
   out_2430636927818656010[38] = 0;
   out_2430636927818656010[39] = 0;
   out_2430636927818656010[40] = 0;
   out_2430636927818656010[41] = 0;
   out_2430636927818656010[42] = 0;
   out_2430636927818656010[43] = 0;
   out_2430636927818656010[44] = 1;
   out_2430636927818656010[45] = 0;
   out_2430636927818656010[46] = 0;
   out_2430636927818656010[47] = 1;
   out_2430636927818656010[48] = 0;
   out_2430636927818656010[49] = 0;
   out_2430636927818656010[50] = 0;
   out_2430636927818656010[51] = 0;
   out_2430636927818656010[52] = 0;
   out_2430636927818656010[53] = 0;
}
void h_10(double *state, double *unused, double *out_1438571128295234105) {
   out_1438571128295234105[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_1438571128295234105[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_1438571128295234105[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5878179630925345156) {
   out_5878179630925345156[0] = 0;
   out_5878179630925345156[1] = 9.8100000000000005*cos(state[1]);
   out_5878179630925345156[2] = 0;
   out_5878179630925345156[3] = 0;
   out_5878179630925345156[4] = -state[8];
   out_5878179630925345156[5] = state[7];
   out_5878179630925345156[6] = 0;
   out_5878179630925345156[7] = state[5];
   out_5878179630925345156[8] = -state[4];
   out_5878179630925345156[9] = 0;
   out_5878179630925345156[10] = 0;
   out_5878179630925345156[11] = 0;
   out_5878179630925345156[12] = 1;
   out_5878179630925345156[13] = 0;
   out_5878179630925345156[14] = 0;
   out_5878179630925345156[15] = 1;
   out_5878179630925345156[16] = 0;
   out_5878179630925345156[17] = 0;
   out_5878179630925345156[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5878179630925345156[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5878179630925345156[20] = 0;
   out_5878179630925345156[21] = state[8];
   out_5878179630925345156[22] = 0;
   out_5878179630925345156[23] = -state[6];
   out_5878179630925345156[24] = -state[5];
   out_5878179630925345156[25] = 0;
   out_5878179630925345156[26] = state[3];
   out_5878179630925345156[27] = 0;
   out_5878179630925345156[28] = 0;
   out_5878179630925345156[29] = 0;
   out_5878179630925345156[30] = 0;
   out_5878179630925345156[31] = 1;
   out_5878179630925345156[32] = 0;
   out_5878179630925345156[33] = 0;
   out_5878179630925345156[34] = 1;
   out_5878179630925345156[35] = 0;
   out_5878179630925345156[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5878179630925345156[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5878179630925345156[38] = 0;
   out_5878179630925345156[39] = -state[7];
   out_5878179630925345156[40] = state[6];
   out_5878179630925345156[41] = 0;
   out_5878179630925345156[42] = state[4];
   out_5878179630925345156[43] = -state[3];
   out_5878179630925345156[44] = 0;
   out_5878179630925345156[45] = 0;
   out_5878179630925345156[46] = 0;
   out_5878179630925345156[47] = 0;
   out_5878179630925345156[48] = 0;
   out_5878179630925345156[49] = 0;
   out_5878179630925345156[50] = 1;
   out_5878179630925345156[51] = 0;
   out_5878179630925345156[52] = 0;
   out_5878179630925345156[53] = 1;
}
void h_13(double *state, double *unused, double *out_3737352406153045413) {
   out_3737352406153045413[0] = state[3];
   out_3737352406153045413[1] = state[4];
   out_3737352406153045413[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5642910753150988811) {
   out_5642910753150988811[0] = 0;
   out_5642910753150988811[1] = 0;
   out_5642910753150988811[2] = 0;
   out_5642910753150988811[3] = 1;
   out_5642910753150988811[4] = 0;
   out_5642910753150988811[5] = 0;
   out_5642910753150988811[6] = 0;
   out_5642910753150988811[7] = 0;
   out_5642910753150988811[8] = 0;
   out_5642910753150988811[9] = 0;
   out_5642910753150988811[10] = 0;
   out_5642910753150988811[11] = 0;
   out_5642910753150988811[12] = 0;
   out_5642910753150988811[13] = 0;
   out_5642910753150988811[14] = 0;
   out_5642910753150988811[15] = 0;
   out_5642910753150988811[16] = 0;
   out_5642910753150988811[17] = 0;
   out_5642910753150988811[18] = 0;
   out_5642910753150988811[19] = 0;
   out_5642910753150988811[20] = 0;
   out_5642910753150988811[21] = 0;
   out_5642910753150988811[22] = 1;
   out_5642910753150988811[23] = 0;
   out_5642910753150988811[24] = 0;
   out_5642910753150988811[25] = 0;
   out_5642910753150988811[26] = 0;
   out_5642910753150988811[27] = 0;
   out_5642910753150988811[28] = 0;
   out_5642910753150988811[29] = 0;
   out_5642910753150988811[30] = 0;
   out_5642910753150988811[31] = 0;
   out_5642910753150988811[32] = 0;
   out_5642910753150988811[33] = 0;
   out_5642910753150988811[34] = 0;
   out_5642910753150988811[35] = 0;
   out_5642910753150988811[36] = 0;
   out_5642910753150988811[37] = 0;
   out_5642910753150988811[38] = 0;
   out_5642910753150988811[39] = 0;
   out_5642910753150988811[40] = 0;
   out_5642910753150988811[41] = 1;
   out_5642910753150988811[42] = 0;
   out_5642910753150988811[43] = 0;
   out_5642910753150988811[44] = 0;
   out_5642910753150988811[45] = 0;
   out_5642910753150988811[46] = 0;
   out_5642910753150988811[47] = 0;
   out_5642910753150988811[48] = 0;
   out_5642910753150988811[49] = 0;
   out_5642910753150988811[50] = 0;
   out_5642910753150988811[51] = 0;
   out_5642910753150988811[52] = 0;
   out_5642910753150988811[53] = 0;
}
void h_14(double *state, double *unused, double *out_1881635978908402004) {
   out_1881635978908402004[0] = state[6];
   out_1881635978908402004[1] = state[7];
   out_1881635978908402004[2] = state[8];
}
void H_14(double *state, double *unused, double *out_652151504476716286) {
   out_652151504476716286[0] = 0;
   out_652151504476716286[1] = 0;
   out_652151504476716286[2] = 0;
   out_652151504476716286[3] = 0;
   out_652151504476716286[4] = 0;
   out_652151504476716286[5] = 0;
   out_652151504476716286[6] = 1;
   out_652151504476716286[7] = 0;
   out_652151504476716286[8] = 0;
   out_652151504476716286[9] = 0;
   out_652151504476716286[10] = 0;
   out_652151504476716286[11] = 0;
   out_652151504476716286[12] = 0;
   out_652151504476716286[13] = 0;
   out_652151504476716286[14] = 0;
   out_652151504476716286[15] = 0;
   out_652151504476716286[16] = 0;
   out_652151504476716286[17] = 0;
   out_652151504476716286[18] = 0;
   out_652151504476716286[19] = 0;
   out_652151504476716286[20] = 0;
   out_652151504476716286[21] = 0;
   out_652151504476716286[22] = 0;
   out_652151504476716286[23] = 0;
   out_652151504476716286[24] = 0;
   out_652151504476716286[25] = 1;
   out_652151504476716286[26] = 0;
   out_652151504476716286[27] = 0;
   out_652151504476716286[28] = 0;
   out_652151504476716286[29] = 0;
   out_652151504476716286[30] = 0;
   out_652151504476716286[31] = 0;
   out_652151504476716286[32] = 0;
   out_652151504476716286[33] = 0;
   out_652151504476716286[34] = 0;
   out_652151504476716286[35] = 0;
   out_652151504476716286[36] = 0;
   out_652151504476716286[37] = 0;
   out_652151504476716286[38] = 0;
   out_652151504476716286[39] = 0;
   out_652151504476716286[40] = 0;
   out_652151504476716286[41] = 0;
   out_652151504476716286[42] = 0;
   out_652151504476716286[43] = 0;
   out_652151504476716286[44] = 1;
   out_652151504476716286[45] = 0;
   out_652151504476716286[46] = 0;
   out_652151504476716286[47] = 0;
   out_652151504476716286[48] = 0;
   out_652151504476716286[49] = 0;
   out_652151504476716286[50] = 0;
   out_652151504476716286[51] = 0;
   out_652151504476716286[52] = 0;
   out_652151504476716286[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_8989729873251700153) {
  err_fun(nom_x, delta_x, out_8989729873251700153);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5494395439268003082) {
  inv_err_fun(nom_x, true_x, out_5494395439268003082);
}
void pose_H_mod_fun(double *state, double *out_4818367292979666309) {
  H_mod_fun(state, out_4818367292979666309);
}
void pose_f_fun(double *state, double dt, double *out_5283112497490037971) {
  f_fun(state,  dt, out_5283112497490037971);
}
void pose_F_fun(double *state, double dt, double *out_4041536850565338045) {
  F_fun(state,  dt, out_4041536850565338045);
}
void pose_h_4(double *state, double *unused, double *out_2536598407606234528) {
  h_4(state, unused, out_2536598407606234528);
}
void pose_H_4(double *state, double *unused, double *out_2430636927818656010) {
  H_4(state, unused, out_2430636927818656010);
}
void pose_h_10(double *state, double *unused, double *out_1438571128295234105) {
  h_10(state, unused, out_1438571128295234105);
}
void pose_H_10(double *state, double *unused, double *out_5878179630925345156) {
  H_10(state, unused, out_5878179630925345156);
}
void pose_h_13(double *state, double *unused, double *out_3737352406153045413) {
  h_13(state, unused, out_3737352406153045413);
}
void pose_H_13(double *state, double *unused, double *out_5642910753150988811) {
  H_13(state, unused, out_5642910753150988811);
}
void pose_h_14(double *state, double *unused, double *out_1881635978908402004) {
  h_14(state, unused, out_1881635978908402004);
}
void pose_H_14(double *state, double *unused, double *out_652151504476716286) {
  H_14(state, unused, out_652151504476716286);
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
