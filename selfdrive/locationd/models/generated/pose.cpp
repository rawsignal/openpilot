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
void err_fun(double *nom_x, double *delta_x, double *out_2107472926716733093) {
   out_2107472926716733093[0] = delta_x[0] + nom_x[0];
   out_2107472926716733093[1] = delta_x[1] + nom_x[1];
   out_2107472926716733093[2] = delta_x[2] + nom_x[2];
   out_2107472926716733093[3] = delta_x[3] + nom_x[3];
   out_2107472926716733093[4] = delta_x[4] + nom_x[4];
   out_2107472926716733093[5] = delta_x[5] + nom_x[5];
   out_2107472926716733093[6] = delta_x[6] + nom_x[6];
   out_2107472926716733093[7] = delta_x[7] + nom_x[7];
   out_2107472926716733093[8] = delta_x[8] + nom_x[8];
   out_2107472926716733093[9] = delta_x[9] + nom_x[9];
   out_2107472926716733093[10] = delta_x[10] + nom_x[10];
   out_2107472926716733093[11] = delta_x[11] + nom_x[11];
   out_2107472926716733093[12] = delta_x[12] + nom_x[12];
   out_2107472926716733093[13] = delta_x[13] + nom_x[13];
   out_2107472926716733093[14] = delta_x[14] + nom_x[14];
   out_2107472926716733093[15] = delta_x[15] + nom_x[15];
   out_2107472926716733093[16] = delta_x[16] + nom_x[16];
   out_2107472926716733093[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7606941711378929712) {
   out_7606941711378929712[0] = -nom_x[0] + true_x[0];
   out_7606941711378929712[1] = -nom_x[1] + true_x[1];
   out_7606941711378929712[2] = -nom_x[2] + true_x[2];
   out_7606941711378929712[3] = -nom_x[3] + true_x[3];
   out_7606941711378929712[4] = -nom_x[4] + true_x[4];
   out_7606941711378929712[5] = -nom_x[5] + true_x[5];
   out_7606941711378929712[6] = -nom_x[6] + true_x[6];
   out_7606941711378929712[7] = -nom_x[7] + true_x[7];
   out_7606941711378929712[8] = -nom_x[8] + true_x[8];
   out_7606941711378929712[9] = -nom_x[9] + true_x[9];
   out_7606941711378929712[10] = -nom_x[10] + true_x[10];
   out_7606941711378929712[11] = -nom_x[11] + true_x[11];
   out_7606941711378929712[12] = -nom_x[12] + true_x[12];
   out_7606941711378929712[13] = -nom_x[13] + true_x[13];
   out_7606941711378929712[14] = -nom_x[14] + true_x[14];
   out_7606941711378929712[15] = -nom_x[15] + true_x[15];
   out_7606941711378929712[16] = -nom_x[16] + true_x[16];
   out_7606941711378929712[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_2474305163268538487) {
   out_2474305163268538487[0] = 1.0;
   out_2474305163268538487[1] = 0.0;
   out_2474305163268538487[2] = 0.0;
   out_2474305163268538487[3] = 0.0;
   out_2474305163268538487[4] = 0.0;
   out_2474305163268538487[5] = 0.0;
   out_2474305163268538487[6] = 0.0;
   out_2474305163268538487[7] = 0.0;
   out_2474305163268538487[8] = 0.0;
   out_2474305163268538487[9] = 0.0;
   out_2474305163268538487[10] = 0.0;
   out_2474305163268538487[11] = 0.0;
   out_2474305163268538487[12] = 0.0;
   out_2474305163268538487[13] = 0.0;
   out_2474305163268538487[14] = 0.0;
   out_2474305163268538487[15] = 0.0;
   out_2474305163268538487[16] = 0.0;
   out_2474305163268538487[17] = 0.0;
   out_2474305163268538487[18] = 0.0;
   out_2474305163268538487[19] = 1.0;
   out_2474305163268538487[20] = 0.0;
   out_2474305163268538487[21] = 0.0;
   out_2474305163268538487[22] = 0.0;
   out_2474305163268538487[23] = 0.0;
   out_2474305163268538487[24] = 0.0;
   out_2474305163268538487[25] = 0.0;
   out_2474305163268538487[26] = 0.0;
   out_2474305163268538487[27] = 0.0;
   out_2474305163268538487[28] = 0.0;
   out_2474305163268538487[29] = 0.0;
   out_2474305163268538487[30] = 0.0;
   out_2474305163268538487[31] = 0.0;
   out_2474305163268538487[32] = 0.0;
   out_2474305163268538487[33] = 0.0;
   out_2474305163268538487[34] = 0.0;
   out_2474305163268538487[35] = 0.0;
   out_2474305163268538487[36] = 0.0;
   out_2474305163268538487[37] = 0.0;
   out_2474305163268538487[38] = 1.0;
   out_2474305163268538487[39] = 0.0;
   out_2474305163268538487[40] = 0.0;
   out_2474305163268538487[41] = 0.0;
   out_2474305163268538487[42] = 0.0;
   out_2474305163268538487[43] = 0.0;
   out_2474305163268538487[44] = 0.0;
   out_2474305163268538487[45] = 0.0;
   out_2474305163268538487[46] = 0.0;
   out_2474305163268538487[47] = 0.0;
   out_2474305163268538487[48] = 0.0;
   out_2474305163268538487[49] = 0.0;
   out_2474305163268538487[50] = 0.0;
   out_2474305163268538487[51] = 0.0;
   out_2474305163268538487[52] = 0.0;
   out_2474305163268538487[53] = 0.0;
   out_2474305163268538487[54] = 0.0;
   out_2474305163268538487[55] = 0.0;
   out_2474305163268538487[56] = 0.0;
   out_2474305163268538487[57] = 1.0;
   out_2474305163268538487[58] = 0.0;
   out_2474305163268538487[59] = 0.0;
   out_2474305163268538487[60] = 0.0;
   out_2474305163268538487[61] = 0.0;
   out_2474305163268538487[62] = 0.0;
   out_2474305163268538487[63] = 0.0;
   out_2474305163268538487[64] = 0.0;
   out_2474305163268538487[65] = 0.0;
   out_2474305163268538487[66] = 0.0;
   out_2474305163268538487[67] = 0.0;
   out_2474305163268538487[68] = 0.0;
   out_2474305163268538487[69] = 0.0;
   out_2474305163268538487[70] = 0.0;
   out_2474305163268538487[71] = 0.0;
   out_2474305163268538487[72] = 0.0;
   out_2474305163268538487[73] = 0.0;
   out_2474305163268538487[74] = 0.0;
   out_2474305163268538487[75] = 0.0;
   out_2474305163268538487[76] = 1.0;
   out_2474305163268538487[77] = 0.0;
   out_2474305163268538487[78] = 0.0;
   out_2474305163268538487[79] = 0.0;
   out_2474305163268538487[80] = 0.0;
   out_2474305163268538487[81] = 0.0;
   out_2474305163268538487[82] = 0.0;
   out_2474305163268538487[83] = 0.0;
   out_2474305163268538487[84] = 0.0;
   out_2474305163268538487[85] = 0.0;
   out_2474305163268538487[86] = 0.0;
   out_2474305163268538487[87] = 0.0;
   out_2474305163268538487[88] = 0.0;
   out_2474305163268538487[89] = 0.0;
   out_2474305163268538487[90] = 0.0;
   out_2474305163268538487[91] = 0.0;
   out_2474305163268538487[92] = 0.0;
   out_2474305163268538487[93] = 0.0;
   out_2474305163268538487[94] = 0.0;
   out_2474305163268538487[95] = 1.0;
   out_2474305163268538487[96] = 0.0;
   out_2474305163268538487[97] = 0.0;
   out_2474305163268538487[98] = 0.0;
   out_2474305163268538487[99] = 0.0;
   out_2474305163268538487[100] = 0.0;
   out_2474305163268538487[101] = 0.0;
   out_2474305163268538487[102] = 0.0;
   out_2474305163268538487[103] = 0.0;
   out_2474305163268538487[104] = 0.0;
   out_2474305163268538487[105] = 0.0;
   out_2474305163268538487[106] = 0.0;
   out_2474305163268538487[107] = 0.0;
   out_2474305163268538487[108] = 0.0;
   out_2474305163268538487[109] = 0.0;
   out_2474305163268538487[110] = 0.0;
   out_2474305163268538487[111] = 0.0;
   out_2474305163268538487[112] = 0.0;
   out_2474305163268538487[113] = 0.0;
   out_2474305163268538487[114] = 1.0;
   out_2474305163268538487[115] = 0.0;
   out_2474305163268538487[116] = 0.0;
   out_2474305163268538487[117] = 0.0;
   out_2474305163268538487[118] = 0.0;
   out_2474305163268538487[119] = 0.0;
   out_2474305163268538487[120] = 0.0;
   out_2474305163268538487[121] = 0.0;
   out_2474305163268538487[122] = 0.0;
   out_2474305163268538487[123] = 0.0;
   out_2474305163268538487[124] = 0.0;
   out_2474305163268538487[125] = 0.0;
   out_2474305163268538487[126] = 0.0;
   out_2474305163268538487[127] = 0.0;
   out_2474305163268538487[128] = 0.0;
   out_2474305163268538487[129] = 0.0;
   out_2474305163268538487[130] = 0.0;
   out_2474305163268538487[131] = 0.0;
   out_2474305163268538487[132] = 0.0;
   out_2474305163268538487[133] = 1.0;
   out_2474305163268538487[134] = 0.0;
   out_2474305163268538487[135] = 0.0;
   out_2474305163268538487[136] = 0.0;
   out_2474305163268538487[137] = 0.0;
   out_2474305163268538487[138] = 0.0;
   out_2474305163268538487[139] = 0.0;
   out_2474305163268538487[140] = 0.0;
   out_2474305163268538487[141] = 0.0;
   out_2474305163268538487[142] = 0.0;
   out_2474305163268538487[143] = 0.0;
   out_2474305163268538487[144] = 0.0;
   out_2474305163268538487[145] = 0.0;
   out_2474305163268538487[146] = 0.0;
   out_2474305163268538487[147] = 0.0;
   out_2474305163268538487[148] = 0.0;
   out_2474305163268538487[149] = 0.0;
   out_2474305163268538487[150] = 0.0;
   out_2474305163268538487[151] = 0.0;
   out_2474305163268538487[152] = 1.0;
   out_2474305163268538487[153] = 0.0;
   out_2474305163268538487[154] = 0.0;
   out_2474305163268538487[155] = 0.0;
   out_2474305163268538487[156] = 0.0;
   out_2474305163268538487[157] = 0.0;
   out_2474305163268538487[158] = 0.0;
   out_2474305163268538487[159] = 0.0;
   out_2474305163268538487[160] = 0.0;
   out_2474305163268538487[161] = 0.0;
   out_2474305163268538487[162] = 0.0;
   out_2474305163268538487[163] = 0.0;
   out_2474305163268538487[164] = 0.0;
   out_2474305163268538487[165] = 0.0;
   out_2474305163268538487[166] = 0.0;
   out_2474305163268538487[167] = 0.0;
   out_2474305163268538487[168] = 0.0;
   out_2474305163268538487[169] = 0.0;
   out_2474305163268538487[170] = 0.0;
   out_2474305163268538487[171] = 1.0;
   out_2474305163268538487[172] = 0.0;
   out_2474305163268538487[173] = 0.0;
   out_2474305163268538487[174] = 0.0;
   out_2474305163268538487[175] = 0.0;
   out_2474305163268538487[176] = 0.0;
   out_2474305163268538487[177] = 0.0;
   out_2474305163268538487[178] = 0.0;
   out_2474305163268538487[179] = 0.0;
   out_2474305163268538487[180] = 0.0;
   out_2474305163268538487[181] = 0.0;
   out_2474305163268538487[182] = 0.0;
   out_2474305163268538487[183] = 0.0;
   out_2474305163268538487[184] = 0.0;
   out_2474305163268538487[185] = 0.0;
   out_2474305163268538487[186] = 0.0;
   out_2474305163268538487[187] = 0.0;
   out_2474305163268538487[188] = 0.0;
   out_2474305163268538487[189] = 0.0;
   out_2474305163268538487[190] = 1.0;
   out_2474305163268538487[191] = 0.0;
   out_2474305163268538487[192] = 0.0;
   out_2474305163268538487[193] = 0.0;
   out_2474305163268538487[194] = 0.0;
   out_2474305163268538487[195] = 0.0;
   out_2474305163268538487[196] = 0.0;
   out_2474305163268538487[197] = 0.0;
   out_2474305163268538487[198] = 0.0;
   out_2474305163268538487[199] = 0.0;
   out_2474305163268538487[200] = 0.0;
   out_2474305163268538487[201] = 0.0;
   out_2474305163268538487[202] = 0.0;
   out_2474305163268538487[203] = 0.0;
   out_2474305163268538487[204] = 0.0;
   out_2474305163268538487[205] = 0.0;
   out_2474305163268538487[206] = 0.0;
   out_2474305163268538487[207] = 0.0;
   out_2474305163268538487[208] = 0.0;
   out_2474305163268538487[209] = 1.0;
   out_2474305163268538487[210] = 0.0;
   out_2474305163268538487[211] = 0.0;
   out_2474305163268538487[212] = 0.0;
   out_2474305163268538487[213] = 0.0;
   out_2474305163268538487[214] = 0.0;
   out_2474305163268538487[215] = 0.0;
   out_2474305163268538487[216] = 0.0;
   out_2474305163268538487[217] = 0.0;
   out_2474305163268538487[218] = 0.0;
   out_2474305163268538487[219] = 0.0;
   out_2474305163268538487[220] = 0.0;
   out_2474305163268538487[221] = 0.0;
   out_2474305163268538487[222] = 0.0;
   out_2474305163268538487[223] = 0.0;
   out_2474305163268538487[224] = 0.0;
   out_2474305163268538487[225] = 0.0;
   out_2474305163268538487[226] = 0.0;
   out_2474305163268538487[227] = 0.0;
   out_2474305163268538487[228] = 1.0;
   out_2474305163268538487[229] = 0.0;
   out_2474305163268538487[230] = 0.0;
   out_2474305163268538487[231] = 0.0;
   out_2474305163268538487[232] = 0.0;
   out_2474305163268538487[233] = 0.0;
   out_2474305163268538487[234] = 0.0;
   out_2474305163268538487[235] = 0.0;
   out_2474305163268538487[236] = 0.0;
   out_2474305163268538487[237] = 0.0;
   out_2474305163268538487[238] = 0.0;
   out_2474305163268538487[239] = 0.0;
   out_2474305163268538487[240] = 0.0;
   out_2474305163268538487[241] = 0.0;
   out_2474305163268538487[242] = 0.0;
   out_2474305163268538487[243] = 0.0;
   out_2474305163268538487[244] = 0.0;
   out_2474305163268538487[245] = 0.0;
   out_2474305163268538487[246] = 0.0;
   out_2474305163268538487[247] = 1.0;
   out_2474305163268538487[248] = 0.0;
   out_2474305163268538487[249] = 0.0;
   out_2474305163268538487[250] = 0.0;
   out_2474305163268538487[251] = 0.0;
   out_2474305163268538487[252] = 0.0;
   out_2474305163268538487[253] = 0.0;
   out_2474305163268538487[254] = 0.0;
   out_2474305163268538487[255] = 0.0;
   out_2474305163268538487[256] = 0.0;
   out_2474305163268538487[257] = 0.0;
   out_2474305163268538487[258] = 0.0;
   out_2474305163268538487[259] = 0.0;
   out_2474305163268538487[260] = 0.0;
   out_2474305163268538487[261] = 0.0;
   out_2474305163268538487[262] = 0.0;
   out_2474305163268538487[263] = 0.0;
   out_2474305163268538487[264] = 0.0;
   out_2474305163268538487[265] = 0.0;
   out_2474305163268538487[266] = 1.0;
   out_2474305163268538487[267] = 0.0;
   out_2474305163268538487[268] = 0.0;
   out_2474305163268538487[269] = 0.0;
   out_2474305163268538487[270] = 0.0;
   out_2474305163268538487[271] = 0.0;
   out_2474305163268538487[272] = 0.0;
   out_2474305163268538487[273] = 0.0;
   out_2474305163268538487[274] = 0.0;
   out_2474305163268538487[275] = 0.0;
   out_2474305163268538487[276] = 0.0;
   out_2474305163268538487[277] = 0.0;
   out_2474305163268538487[278] = 0.0;
   out_2474305163268538487[279] = 0.0;
   out_2474305163268538487[280] = 0.0;
   out_2474305163268538487[281] = 0.0;
   out_2474305163268538487[282] = 0.0;
   out_2474305163268538487[283] = 0.0;
   out_2474305163268538487[284] = 0.0;
   out_2474305163268538487[285] = 1.0;
   out_2474305163268538487[286] = 0.0;
   out_2474305163268538487[287] = 0.0;
   out_2474305163268538487[288] = 0.0;
   out_2474305163268538487[289] = 0.0;
   out_2474305163268538487[290] = 0.0;
   out_2474305163268538487[291] = 0.0;
   out_2474305163268538487[292] = 0.0;
   out_2474305163268538487[293] = 0.0;
   out_2474305163268538487[294] = 0.0;
   out_2474305163268538487[295] = 0.0;
   out_2474305163268538487[296] = 0.0;
   out_2474305163268538487[297] = 0.0;
   out_2474305163268538487[298] = 0.0;
   out_2474305163268538487[299] = 0.0;
   out_2474305163268538487[300] = 0.0;
   out_2474305163268538487[301] = 0.0;
   out_2474305163268538487[302] = 0.0;
   out_2474305163268538487[303] = 0.0;
   out_2474305163268538487[304] = 1.0;
   out_2474305163268538487[305] = 0.0;
   out_2474305163268538487[306] = 0.0;
   out_2474305163268538487[307] = 0.0;
   out_2474305163268538487[308] = 0.0;
   out_2474305163268538487[309] = 0.0;
   out_2474305163268538487[310] = 0.0;
   out_2474305163268538487[311] = 0.0;
   out_2474305163268538487[312] = 0.0;
   out_2474305163268538487[313] = 0.0;
   out_2474305163268538487[314] = 0.0;
   out_2474305163268538487[315] = 0.0;
   out_2474305163268538487[316] = 0.0;
   out_2474305163268538487[317] = 0.0;
   out_2474305163268538487[318] = 0.0;
   out_2474305163268538487[319] = 0.0;
   out_2474305163268538487[320] = 0.0;
   out_2474305163268538487[321] = 0.0;
   out_2474305163268538487[322] = 0.0;
   out_2474305163268538487[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_6855082133400824491) {
   out_6855082133400824491[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_6855082133400824491[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_6855082133400824491[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_6855082133400824491[3] = dt*state[12] + state[3];
   out_6855082133400824491[4] = dt*state[13] + state[4];
   out_6855082133400824491[5] = dt*state[14] + state[5];
   out_6855082133400824491[6] = state[6];
   out_6855082133400824491[7] = state[7];
   out_6855082133400824491[8] = state[8];
   out_6855082133400824491[9] = state[9];
   out_6855082133400824491[10] = state[10];
   out_6855082133400824491[11] = state[11];
   out_6855082133400824491[12] = state[12];
   out_6855082133400824491[13] = state[13];
   out_6855082133400824491[14] = state[14];
   out_6855082133400824491[15] = state[15];
   out_6855082133400824491[16] = state[16];
   out_6855082133400824491[17] = state[17];
}
void F_fun(double *state, double dt, double *out_5712260729693103006) {
   out_5712260729693103006[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5712260729693103006[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5712260729693103006[2] = 0;
   out_5712260729693103006[3] = 0;
   out_5712260729693103006[4] = 0;
   out_5712260729693103006[5] = 0;
   out_5712260729693103006[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5712260729693103006[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5712260729693103006[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5712260729693103006[9] = 0;
   out_5712260729693103006[10] = 0;
   out_5712260729693103006[11] = 0;
   out_5712260729693103006[12] = 0;
   out_5712260729693103006[13] = 0;
   out_5712260729693103006[14] = 0;
   out_5712260729693103006[15] = 0;
   out_5712260729693103006[16] = 0;
   out_5712260729693103006[17] = 0;
   out_5712260729693103006[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5712260729693103006[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5712260729693103006[20] = 0;
   out_5712260729693103006[21] = 0;
   out_5712260729693103006[22] = 0;
   out_5712260729693103006[23] = 0;
   out_5712260729693103006[24] = 0;
   out_5712260729693103006[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5712260729693103006[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5712260729693103006[27] = 0;
   out_5712260729693103006[28] = 0;
   out_5712260729693103006[29] = 0;
   out_5712260729693103006[30] = 0;
   out_5712260729693103006[31] = 0;
   out_5712260729693103006[32] = 0;
   out_5712260729693103006[33] = 0;
   out_5712260729693103006[34] = 0;
   out_5712260729693103006[35] = 0;
   out_5712260729693103006[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5712260729693103006[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5712260729693103006[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5712260729693103006[39] = 0;
   out_5712260729693103006[40] = 0;
   out_5712260729693103006[41] = 0;
   out_5712260729693103006[42] = 0;
   out_5712260729693103006[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5712260729693103006[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5712260729693103006[45] = 0;
   out_5712260729693103006[46] = 0;
   out_5712260729693103006[47] = 0;
   out_5712260729693103006[48] = 0;
   out_5712260729693103006[49] = 0;
   out_5712260729693103006[50] = 0;
   out_5712260729693103006[51] = 0;
   out_5712260729693103006[52] = 0;
   out_5712260729693103006[53] = 0;
   out_5712260729693103006[54] = 0;
   out_5712260729693103006[55] = 0;
   out_5712260729693103006[56] = 0;
   out_5712260729693103006[57] = 1;
   out_5712260729693103006[58] = 0;
   out_5712260729693103006[59] = 0;
   out_5712260729693103006[60] = 0;
   out_5712260729693103006[61] = 0;
   out_5712260729693103006[62] = 0;
   out_5712260729693103006[63] = 0;
   out_5712260729693103006[64] = 0;
   out_5712260729693103006[65] = 0;
   out_5712260729693103006[66] = dt;
   out_5712260729693103006[67] = 0;
   out_5712260729693103006[68] = 0;
   out_5712260729693103006[69] = 0;
   out_5712260729693103006[70] = 0;
   out_5712260729693103006[71] = 0;
   out_5712260729693103006[72] = 0;
   out_5712260729693103006[73] = 0;
   out_5712260729693103006[74] = 0;
   out_5712260729693103006[75] = 0;
   out_5712260729693103006[76] = 1;
   out_5712260729693103006[77] = 0;
   out_5712260729693103006[78] = 0;
   out_5712260729693103006[79] = 0;
   out_5712260729693103006[80] = 0;
   out_5712260729693103006[81] = 0;
   out_5712260729693103006[82] = 0;
   out_5712260729693103006[83] = 0;
   out_5712260729693103006[84] = 0;
   out_5712260729693103006[85] = dt;
   out_5712260729693103006[86] = 0;
   out_5712260729693103006[87] = 0;
   out_5712260729693103006[88] = 0;
   out_5712260729693103006[89] = 0;
   out_5712260729693103006[90] = 0;
   out_5712260729693103006[91] = 0;
   out_5712260729693103006[92] = 0;
   out_5712260729693103006[93] = 0;
   out_5712260729693103006[94] = 0;
   out_5712260729693103006[95] = 1;
   out_5712260729693103006[96] = 0;
   out_5712260729693103006[97] = 0;
   out_5712260729693103006[98] = 0;
   out_5712260729693103006[99] = 0;
   out_5712260729693103006[100] = 0;
   out_5712260729693103006[101] = 0;
   out_5712260729693103006[102] = 0;
   out_5712260729693103006[103] = 0;
   out_5712260729693103006[104] = dt;
   out_5712260729693103006[105] = 0;
   out_5712260729693103006[106] = 0;
   out_5712260729693103006[107] = 0;
   out_5712260729693103006[108] = 0;
   out_5712260729693103006[109] = 0;
   out_5712260729693103006[110] = 0;
   out_5712260729693103006[111] = 0;
   out_5712260729693103006[112] = 0;
   out_5712260729693103006[113] = 0;
   out_5712260729693103006[114] = 1;
   out_5712260729693103006[115] = 0;
   out_5712260729693103006[116] = 0;
   out_5712260729693103006[117] = 0;
   out_5712260729693103006[118] = 0;
   out_5712260729693103006[119] = 0;
   out_5712260729693103006[120] = 0;
   out_5712260729693103006[121] = 0;
   out_5712260729693103006[122] = 0;
   out_5712260729693103006[123] = 0;
   out_5712260729693103006[124] = 0;
   out_5712260729693103006[125] = 0;
   out_5712260729693103006[126] = 0;
   out_5712260729693103006[127] = 0;
   out_5712260729693103006[128] = 0;
   out_5712260729693103006[129] = 0;
   out_5712260729693103006[130] = 0;
   out_5712260729693103006[131] = 0;
   out_5712260729693103006[132] = 0;
   out_5712260729693103006[133] = 1;
   out_5712260729693103006[134] = 0;
   out_5712260729693103006[135] = 0;
   out_5712260729693103006[136] = 0;
   out_5712260729693103006[137] = 0;
   out_5712260729693103006[138] = 0;
   out_5712260729693103006[139] = 0;
   out_5712260729693103006[140] = 0;
   out_5712260729693103006[141] = 0;
   out_5712260729693103006[142] = 0;
   out_5712260729693103006[143] = 0;
   out_5712260729693103006[144] = 0;
   out_5712260729693103006[145] = 0;
   out_5712260729693103006[146] = 0;
   out_5712260729693103006[147] = 0;
   out_5712260729693103006[148] = 0;
   out_5712260729693103006[149] = 0;
   out_5712260729693103006[150] = 0;
   out_5712260729693103006[151] = 0;
   out_5712260729693103006[152] = 1;
   out_5712260729693103006[153] = 0;
   out_5712260729693103006[154] = 0;
   out_5712260729693103006[155] = 0;
   out_5712260729693103006[156] = 0;
   out_5712260729693103006[157] = 0;
   out_5712260729693103006[158] = 0;
   out_5712260729693103006[159] = 0;
   out_5712260729693103006[160] = 0;
   out_5712260729693103006[161] = 0;
   out_5712260729693103006[162] = 0;
   out_5712260729693103006[163] = 0;
   out_5712260729693103006[164] = 0;
   out_5712260729693103006[165] = 0;
   out_5712260729693103006[166] = 0;
   out_5712260729693103006[167] = 0;
   out_5712260729693103006[168] = 0;
   out_5712260729693103006[169] = 0;
   out_5712260729693103006[170] = 0;
   out_5712260729693103006[171] = 1;
   out_5712260729693103006[172] = 0;
   out_5712260729693103006[173] = 0;
   out_5712260729693103006[174] = 0;
   out_5712260729693103006[175] = 0;
   out_5712260729693103006[176] = 0;
   out_5712260729693103006[177] = 0;
   out_5712260729693103006[178] = 0;
   out_5712260729693103006[179] = 0;
   out_5712260729693103006[180] = 0;
   out_5712260729693103006[181] = 0;
   out_5712260729693103006[182] = 0;
   out_5712260729693103006[183] = 0;
   out_5712260729693103006[184] = 0;
   out_5712260729693103006[185] = 0;
   out_5712260729693103006[186] = 0;
   out_5712260729693103006[187] = 0;
   out_5712260729693103006[188] = 0;
   out_5712260729693103006[189] = 0;
   out_5712260729693103006[190] = 1;
   out_5712260729693103006[191] = 0;
   out_5712260729693103006[192] = 0;
   out_5712260729693103006[193] = 0;
   out_5712260729693103006[194] = 0;
   out_5712260729693103006[195] = 0;
   out_5712260729693103006[196] = 0;
   out_5712260729693103006[197] = 0;
   out_5712260729693103006[198] = 0;
   out_5712260729693103006[199] = 0;
   out_5712260729693103006[200] = 0;
   out_5712260729693103006[201] = 0;
   out_5712260729693103006[202] = 0;
   out_5712260729693103006[203] = 0;
   out_5712260729693103006[204] = 0;
   out_5712260729693103006[205] = 0;
   out_5712260729693103006[206] = 0;
   out_5712260729693103006[207] = 0;
   out_5712260729693103006[208] = 0;
   out_5712260729693103006[209] = 1;
   out_5712260729693103006[210] = 0;
   out_5712260729693103006[211] = 0;
   out_5712260729693103006[212] = 0;
   out_5712260729693103006[213] = 0;
   out_5712260729693103006[214] = 0;
   out_5712260729693103006[215] = 0;
   out_5712260729693103006[216] = 0;
   out_5712260729693103006[217] = 0;
   out_5712260729693103006[218] = 0;
   out_5712260729693103006[219] = 0;
   out_5712260729693103006[220] = 0;
   out_5712260729693103006[221] = 0;
   out_5712260729693103006[222] = 0;
   out_5712260729693103006[223] = 0;
   out_5712260729693103006[224] = 0;
   out_5712260729693103006[225] = 0;
   out_5712260729693103006[226] = 0;
   out_5712260729693103006[227] = 0;
   out_5712260729693103006[228] = 1;
   out_5712260729693103006[229] = 0;
   out_5712260729693103006[230] = 0;
   out_5712260729693103006[231] = 0;
   out_5712260729693103006[232] = 0;
   out_5712260729693103006[233] = 0;
   out_5712260729693103006[234] = 0;
   out_5712260729693103006[235] = 0;
   out_5712260729693103006[236] = 0;
   out_5712260729693103006[237] = 0;
   out_5712260729693103006[238] = 0;
   out_5712260729693103006[239] = 0;
   out_5712260729693103006[240] = 0;
   out_5712260729693103006[241] = 0;
   out_5712260729693103006[242] = 0;
   out_5712260729693103006[243] = 0;
   out_5712260729693103006[244] = 0;
   out_5712260729693103006[245] = 0;
   out_5712260729693103006[246] = 0;
   out_5712260729693103006[247] = 1;
   out_5712260729693103006[248] = 0;
   out_5712260729693103006[249] = 0;
   out_5712260729693103006[250] = 0;
   out_5712260729693103006[251] = 0;
   out_5712260729693103006[252] = 0;
   out_5712260729693103006[253] = 0;
   out_5712260729693103006[254] = 0;
   out_5712260729693103006[255] = 0;
   out_5712260729693103006[256] = 0;
   out_5712260729693103006[257] = 0;
   out_5712260729693103006[258] = 0;
   out_5712260729693103006[259] = 0;
   out_5712260729693103006[260] = 0;
   out_5712260729693103006[261] = 0;
   out_5712260729693103006[262] = 0;
   out_5712260729693103006[263] = 0;
   out_5712260729693103006[264] = 0;
   out_5712260729693103006[265] = 0;
   out_5712260729693103006[266] = 1;
   out_5712260729693103006[267] = 0;
   out_5712260729693103006[268] = 0;
   out_5712260729693103006[269] = 0;
   out_5712260729693103006[270] = 0;
   out_5712260729693103006[271] = 0;
   out_5712260729693103006[272] = 0;
   out_5712260729693103006[273] = 0;
   out_5712260729693103006[274] = 0;
   out_5712260729693103006[275] = 0;
   out_5712260729693103006[276] = 0;
   out_5712260729693103006[277] = 0;
   out_5712260729693103006[278] = 0;
   out_5712260729693103006[279] = 0;
   out_5712260729693103006[280] = 0;
   out_5712260729693103006[281] = 0;
   out_5712260729693103006[282] = 0;
   out_5712260729693103006[283] = 0;
   out_5712260729693103006[284] = 0;
   out_5712260729693103006[285] = 1;
   out_5712260729693103006[286] = 0;
   out_5712260729693103006[287] = 0;
   out_5712260729693103006[288] = 0;
   out_5712260729693103006[289] = 0;
   out_5712260729693103006[290] = 0;
   out_5712260729693103006[291] = 0;
   out_5712260729693103006[292] = 0;
   out_5712260729693103006[293] = 0;
   out_5712260729693103006[294] = 0;
   out_5712260729693103006[295] = 0;
   out_5712260729693103006[296] = 0;
   out_5712260729693103006[297] = 0;
   out_5712260729693103006[298] = 0;
   out_5712260729693103006[299] = 0;
   out_5712260729693103006[300] = 0;
   out_5712260729693103006[301] = 0;
   out_5712260729693103006[302] = 0;
   out_5712260729693103006[303] = 0;
   out_5712260729693103006[304] = 1;
   out_5712260729693103006[305] = 0;
   out_5712260729693103006[306] = 0;
   out_5712260729693103006[307] = 0;
   out_5712260729693103006[308] = 0;
   out_5712260729693103006[309] = 0;
   out_5712260729693103006[310] = 0;
   out_5712260729693103006[311] = 0;
   out_5712260729693103006[312] = 0;
   out_5712260729693103006[313] = 0;
   out_5712260729693103006[314] = 0;
   out_5712260729693103006[315] = 0;
   out_5712260729693103006[316] = 0;
   out_5712260729693103006[317] = 0;
   out_5712260729693103006[318] = 0;
   out_5712260729693103006[319] = 0;
   out_5712260729693103006[320] = 0;
   out_5712260729693103006[321] = 0;
   out_5712260729693103006[322] = 0;
   out_5712260729693103006[323] = 1;
}
void h_4(double *state, double *unused, double *out_7382434891101514169) {
   out_7382434891101514169[0] = state[6] + state[9];
   out_7382434891101514169[1] = state[7] + state[10];
   out_7382434891101514169[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2678213221770542248) {
   out_2678213221770542248[0] = 0;
   out_2678213221770542248[1] = 0;
   out_2678213221770542248[2] = 0;
   out_2678213221770542248[3] = 0;
   out_2678213221770542248[4] = 0;
   out_2678213221770542248[5] = 0;
   out_2678213221770542248[6] = 1;
   out_2678213221770542248[7] = 0;
   out_2678213221770542248[8] = 0;
   out_2678213221770542248[9] = 1;
   out_2678213221770542248[10] = 0;
   out_2678213221770542248[11] = 0;
   out_2678213221770542248[12] = 0;
   out_2678213221770542248[13] = 0;
   out_2678213221770542248[14] = 0;
   out_2678213221770542248[15] = 0;
   out_2678213221770542248[16] = 0;
   out_2678213221770542248[17] = 0;
   out_2678213221770542248[18] = 0;
   out_2678213221770542248[19] = 0;
   out_2678213221770542248[20] = 0;
   out_2678213221770542248[21] = 0;
   out_2678213221770542248[22] = 0;
   out_2678213221770542248[23] = 0;
   out_2678213221770542248[24] = 0;
   out_2678213221770542248[25] = 1;
   out_2678213221770542248[26] = 0;
   out_2678213221770542248[27] = 0;
   out_2678213221770542248[28] = 1;
   out_2678213221770542248[29] = 0;
   out_2678213221770542248[30] = 0;
   out_2678213221770542248[31] = 0;
   out_2678213221770542248[32] = 0;
   out_2678213221770542248[33] = 0;
   out_2678213221770542248[34] = 0;
   out_2678213221770542248[35] = 0;
   out_2678213221770542248[36] = 0;
   out_2678213221770542248[37] = 0;
   out_2678213221770542248[38] = 0;
   out_2678213221770542248[39] = 0;
   out_2678213221770542248[40] = 0;
   out_2678213221770542248[41] = 0;
   out_2678213221770542248[42] = 0;
   out_2678213221770542248[43] = 0;
   out_2678213221770542248[44] = 1;
   out_2678213221770542248[45] = 0;
   out_2678213221770542248[46] = 0;
   out_2678213221770542248[47] = 1;
   out_2678213221770542248[48] = 0;
   out_2678213221770542248[49] = 0;
   out_2678213221770542248[50] = 0;
   out_2678213221770542248[51] = 0;
   out_2678213221770542248[52] = 0;
   out_2678213221770542248[53] = 0;
}
void h_10(double *state, double *unused, double *out_3695414532195514518) {
   out_3695414532195514518[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_3695414532195514518[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_3695414532195514518[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_4693822457708176897) {
   out_4693822457708176897[0] = 0;
   out_4693822457708176897[1] = 9.8100000000000005*cos(state[1]);
   out_4693822457708176897[2] = 0;
   out_4693822457708176897[3] = 0;
   out_4693822457708176897[4] = -state[8];
   out_4693822457708176897[5] = state[7];
   out_4693822457708176897[6] = 0;
   out_4693822457708176897[7] = state[5];
   out_4693822457708176897[8] = -state[4];
   out_4693822457708176897[9] = 0;
   out_4693822457708176897[10] = 0;
   out_4693822457708176897[11] = 0;
   out_4693822457708176897[12] = 1;
   out_4693822457708176897[13] = 0;
   out_4693822457708176897[14] = 0;
   out_4693822457708176897[15] = 1;
   out_4693822457708176897[16] = 0;
   out_4693822457708176897[17] = 0;
   out_4693822457708176897[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_4693822457708176897[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_4693822457708176897[20] = 0;
   out_4693822457708176897[21] = state[8];
   out_4693822457708176897[22] = 0;
   out_4693822457708176897[23] = -state[6];
   out_4693822457708176897[24] = -state[5];
   out_4693822457708176897[25] = 0;
   out_4693822457708176897[26] = state[3];
   out_4693822457708176897[27] = 0;
   out_4693822457708176897[28] = 0;
   out_4693822457708176897[29] = 0;
   out_4693822457708176897[30] = 0;
   out_4693822457708176897[31] = 1;
   out_4693822457708176897[32] = 0;
   out_4693822457708176897[33] = 0;
   out_4693822457708176897[34] = 1;
   out_4693822457708176897[35] = 0;
   out_4693822457708176897[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_4693822457708176897[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_4693822457708176897[38] = 0;
   out_4693822457708176897[39] = -state[7];
   out_4693822457708176897[40] = state[6];
   out_4693822457708176897[41] = 0;
   out_4693822457708176897[42] = state[4];
   out_4693822457708176897[43] = -state[3];
   out_4693822457708176897[44] = 0;
   out_4693822457708176897[45] = 0;
   out_4693822457708176897[46] = 0;
   out_4693822457708176897[47] = 0;
   out_4693822457708176897[48] = 0;
   out_4693822457708176897[49] = 0;
   out_4693822457708176897[50] = 1;
   out_4693822457708176897[51] = 0;
   out_4693822457708176897[52] = 0;
   out_4693822457708176897[53] = 1;
}
void h_13(double *state, double *unused, double *out_1948308378304519843) {
   out_1948308378304519843[0] = state[3];
   out_1948308378304519843[1] = state[4];
   out_1948308378304519843[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5890487047102875049) {
   out_5890487047102875049[0] = 0;
   out_5890487047102875049[1] = 0;
   out_5890487047102875049[2] = 0;
   out_5890487047102875049[3] = 1;
   out_5890487047102875049[4] = 0;
   out_5890487047102875049[5] = 0;
   out_5890487047102875049[6] = 0;
   out_5890487047102875049[7] = 0;
   out_5890487047102875049[8] = 0;
   out_5890487047102875049[9] = 0;
   out_5890487047102875049[10] = 0;
   out_5890487047102875049[11] = 0;
   out_5890487047102875049[12] = 0;
   out_5890487047102875049[13] = 0;
   out_5890487047102875049[14] = 0;
   out_5890487047102875049[15] = 0;
   out_5890487047102875049[16] = 0;
   out_5890487047102875049[17] = 0;
   out_5890487047102875049[18] = 0;
   out_5890487047102875049[19] = 0;
   out_5890487047102875049[20] = 0;
   out_5890487047102875049[21] = 0;
   out_5890487047102875049[22] = 1;
   out_5890487047102875049[23] = 0;
   out_5890487047102875049[24] = 0;
   out_5890487047102875049[25] = 0;
   out_5890487047102875049[26] = 0;
   out_5890487047102875049[27] = 0;
   out_5890487047102875049[28] = 0;
   out_5890487047102875049[29] = 0;
   out_5890487047102875049[30] = 0;
   out_5890487047102875049[31] = 0;
   out_5890487047102875049[32] = 0;
   out_5890487047102875049[33] = 0;
   out_5890487047102875049[34] = 0;
   out_5890487047102875049[35] = 0;
   out_5890487047102875049[36] = 0;
   out_5890487047102875049[37] = 0;
   out_5890487047102875049[38] = 0;
   out_5890487047102875049[39] = 0;
   out_5890487047102875049[40] = 0;
   out_5890487047102875049[41] = 1;
   out_5890487047102875049[42] = 0;
   out_5890487047102875049[43] = 0;
   out_5890487047102875049[44] = 0;
   out_5890487047102875049[45] = 0;
   out_5890487047102875049[46] = 0;
   out_5890487047102875049[47] = 0;
   out_5890487047102875049[48] = 0;
   out_5890487047102875049[49] = 0;
   out_5890487047102875049[50] = 0;
   out_5890487047102875049[51] = 0;
   out_5890487047102875049[52] = 0;
   out_5890487047102875049[53] = 0;
}
void h_14(double *state, double *unused, double *out_3950222992602798817) {
   out_3950222992602798817[0] = state[6];
   out_3950222992602798817[1] = state[7];
   out_3950222992602798817[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6641454078110026777) {
   out_6641454078110026777[0] = 0;
   out_6641454078110026777[1] = 0;
   out_6641454078110026777[2] = 0;
   out_6641454078110026777[3] = 0;
   out_6641454078110026777[4] = 0;
   out_6641454078110026777[5] = 0;
   out_6641454078110026777[6] = 1;
   out_6641454078110026777[7] = 0;
   out_6641454078110026777[8] = 0;
   out_6641454078110026777[9] = 0;
   out_6641454078110026777[10] = 0;
   out_6641454078110026777[11] = 0;
   out_6641454078110026777[12] = 0;
   out_6641454078110026777[13] = 0;
   out_6641454078110026777[14] = 0;
   out_6641454078110026777[15] = 0;
   out_6641454078110026777[16] = 0;
   out_6641454078110026777[17] = 0;
   out_6641454078110026777[18] = 0;
   out_6641454078110026777[19] = 0;
   out_6641454078110026777[20] = 0;
   out_6641454078110026777[21] = 0;
   out_6641454078110026777[22] = 0;
   out_6641454078110026777[23] = 0;
   out_6641454078110026777[24] = 0;
   out_6641454078110026777[25] = 1;
   out_6641454078110026777[26] = 0;
   out_6641454078110026777[27] = 0;
   out_6641454078110026777[28] = 0;
   out_6641454078110026777[29] = 0;
   out_6641454078110026777[30] = 0;
   out_6641454078110026777[31] = 0;
   out_6641454078110026777[32] = 0;
   out_6641454078110026777[33] = 0;
   out_6641454078110026777[34] = 0;
   out_6641454078110026777[35] = 0;
   out_6641454078110026777[36] = 0;
   out_6641454078110026777[37] = 0;
   out_6641454078110026777[38] = 0;
   out_6641454078110026777[39] = 0;
   out_6641454078110026777[40] = 0;
   out_6641454078110026777[41] = 0;
   out_6641454078110026777[42] = 0;
   out_6641454078110026777[43] = 0;
   out_6641454078110026777[44] = 1;
   out_6641454078110026777[45] = 0;
   out_6641454078110026777[46] = 0;
   out_6641454078110026777[47] = 0;
   out_6641454078110026777[48] = 0;
   out_6641454078110026777[49] = 0;
   out_6641454078110026777[50] = 0;
   out_6641454078110026777[51] = 0;
   out_6641454078110026777[52] = 0;
   out_6641454078110026777[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_2107472926716733093) {
  err_fun(nom_x, delta_x, out_2107472926716733093);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7606941711378929712) {
  inv_err_fun(nom_x, true_x, out_7606941711378929712);
}
void pose_H_mod_fun(double *state, double *out_2474305163268538487) {
  H_mod_fun(state, out_2474305163268538487);
}
void pose_f_fun(double *state, double dt, double *out_6855082133400824491) {
  f_fun(state,  dt, out_6855082133400824491);
}
void pose_F_fun(double *state, double dt, double *out_5712260729693103006) {
  F_fun(state,  dt, out_5712260729693103006);
}
void pose_h_4(double *state, double *unused, double *out_7382434891101514169) {
  h_4(state, unused, out_7382434891101514169);
}
void pose_H_4(double *state, double *unused, double *out_2678213221770542248) {
  H_4(state, unused, out_2678213221770542248);
}
void pose_h_10(double *state, double *unused, double *out_3695414532195514518) {
  h_10(state, unused, out_3695414532195514518);
}
void pose_H_10(double *state, double *unused, double *out_4693822457708176897) {
  H_10(state, unused, out_4693822457708176897);
}
void pose_h_13(double *state, double *unused, double *out_1948308378304519843) {
  h_13(state, unused, out_1948308378304519843);
}
void pose_H_13(double *state, double *unused, double *out_5890487047102875049) {
  H_13(state, unused, out_5890487047102875049);
}
void pose_h_14(double *state, double *unused, double *out_3950222992602798817) {
  h_14(state, unused, out_3950222992602798817);
}
void pose_H_14(double *state, double *unused, double *out_6641454078110026777) {
  H_14(state, unused, out_6641454078110026777);
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
