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
void err_fun(double *nom_x, double *delta_x, double *out_6666156999559179096) {
   out_6666156999559179096[0] = delta_x[0] + nom_x[0];
   out_6666156999559179096[1] = delta_x[1] + nom_x[1];
   out_6666156999559179096[2] = delta_x[2] + nom_x[2];
   out_6666156999559179096[3] = delta_x[3] + nom_x[3];
   out_6666156999559179096[4] = delta_x[4] + nom_x[4];
   out_6666156999559179096[5] = delta_x[5] + nom_x[5];
   out_6666156999559179096[6] = delta_x[6] + nom_x[6];
   out_6666156999559179096[7] = delta_x[7] + nom_x[7];
   out_6666156999559179096[8] = delta_x[8] + nom_x[8];
   out_6666156999559179096[9] = delta_x[9] + nom_x[9];
   out_6666156999559179096[10] = delta_x[10] + nom_x[10];
   out_6666156999559179096[11] = delta_x[11] + nom_x[11];
   out_6666156999559179096[12] = delta_x[12] + nom_x[12];
   out_6666156999559179096[13] = delta_x[13] + nom_x[13];
   out_6666156999559179096[14] = delta_x[14] + nom_x[14];
   out_6666156999559179096[15] = delta_x[15] + nom_x[15];
   out_6666156999559179096[16] = delta_x[16] + nom_x[16];
   out_6666156999559179096[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6383049424053173820) {
   out_6383049424053173820[0] = -nom_x[0] + true_x[0];
   out_6383049424053173820[1] = -nom_x[1] + true_x[1];
   out_6383049424053173820[2] = -nom_x[2] + true_x[2];
   out_6383049424053173820[3] = -nom_x[3] + true_x[3];
   out_6383049424053173820[4] = -nom_x[4] + true_x[4];
   out_6383049424053173820[5] = -nom_x[5] + true_x[5];
   out_6383049424053173820[6] = -nom_x[6] + true_x[6];
   out_6383049424053173820[7] = -nom_x[7] + true_x[7];
   out_6383049424053173820[8] = -nom_x[8] + true_x[8];
   out_6383049424053173820[9] = -nom_x[9] + true_x[9];
   out_6383049424053173820[10] = -nom_x[10] + true_x[10];
   out_6383049424053173820[11] = -nom_x[11] + true_x[11];
   out_6383049424053173820[12] = -nom_x[12] + true_x[12];
   out_6383049424053173820[13] = -nom_x[13] + true_x[13];
   out_6383049424053173820[14] = -nom_x[14] + true_x[14];
   out_6383049424053173820[15] = -nom_x[15] + true_x[15];
   out_6383049424053173820[16] = -nom_x[16] + true_x[16];
   out_6383049424053173820[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_7560663382806042444) {
   out_7560663382806042444[0] = 1.0;
   out_7560663382806042444[1] = 0.0;
   out_7560663382806042444[2] = 0.0;
   out_7560663382806042444[3] = 0.0;
   out_7560663382806042444[4] = 0.0;
   out_7560663382806042444[5] = 0.0;
   out_7560663382806042444[6] = 0.0;
   out_7560663382806042444[7] = 0.0;
   out_7560663382806042444[8] = 0.0;
   out_7560663382806042444[9] = 0.0;
   out_7560663382806042444[10] = 0.0;
   out_7560663382806042444[11] = 0.0;
   out_7560663382806042444[12] = 0.0;
   out_7560663382806042444[13] = 0.0;
   out_7560663382806042444[14] = 0.0;
   out_7560663382806042444[15] = 0.0;
   out_7560663382806042444[16] = 0.0;
   out_7560663382806042444[17] = 0.0;
   out_7560663382806042444[18] = 0.0;
   out_7560663382806042444[19] = 1.0;
   out_7560663382806042444[20] = 0.0;
   out_7560663382806042444[21] = 0.0;
   out_7560663382806042444[22] = 0.0;
   out_7560663382806042444[23] = 0.0;
   out_7560663382806042444[24] = 0.0;
   out_7560663382806042444[25] = 0.0;
   out_7560663382806042444[26] = 0.0;
   out_7560663382806042444[27] = 0.0;
   out_7560663382806042444[28] = 0.0;
   out_7560663382806042444[29] = 0.0;
   out_7560663382806042444[30] = 0.0;
   out_7560663382806042444[31] = 0.0;
   out_7560663382806042444[32] = 0.0;
   out_7560663382806042444[33] = 0.0;
   out_7560663382806042444[34] = 0.0;
   out_7560663382806042444[35] = 0.0;
   out_7560663382806042444[36] = 0.0;
   out_7560663382806042444[37] = 0.0;
   out_7560663382806042444[38] = 1.0;
   out_7560663382806042444[39] = 0.0;
   out_7560663382806042444[40] = 0.0;
   out_7560663382806042444[41] = 0.0;
   out_7560663382806042444[42] = 0.0;
   out_7560663382806042444[43] = 0.0;
   out_7560663382806042444[44] = 0.0;
   out_7560663382806042444[45] = 0.0;
   out_7560663382806042444[46] = 0.0;
   out_7560663382806042444[47] = 0.0;
   out_7560663382806042444[48] = 0.0;
   out_7560663382806042444[49] = 0.0;
   out_7560663382806042444[50] = 0.0;
   out_7560663382806042444[51] = 0.0;
   out_7560663382806042444[52] = 0.0;
   out_7560663382806042444[53] = 0.0;
   out_7560663382806042444[54] = 0.0;
   out_7560663382806042444[55] = 0.0;
   out_7560663382806042444[56] = 0.0;
   out_7560663382806042444[57] = 1.0;
   out_7560663382806042444[58] = 0.0;
   out_7560663382806042444[59] = 0.0;
   out_7560663382806042444[60] = 0.0;
   out_7560663382806042444[61] = 0.0;
   out_7560663382806042444[62] = 0.0;
   out_7560663382806042444[63] = 0.0;
   out_7560663382806042444[64] = 0.0;
   out_7560663382806042444[65] = 0.0;
   out_7560663382806042444[66] = 0.0;
   out_7560663382806042444[67] = 0.0;
   out_7560663382806042444[68] = 0.0;
   out_7560663382806042444[69] = 0.0;
   out_7560663382806042444[70] = 0.0;
   out_7560663382806042444[71] = 0.0;
   out_7560663382806042444[72] = 0.0;
   out_7560663382806042444[73] = 0.0;
   out_7560663382806042444[74] = 0.0;
   out_7560663382806042444[75] = 0.0;
   out_7560663382806042444[76] = 1.0;
   out_7560663382806042444[77] = 0.0;
   out_7560663382806042444[78] = 0.0;
   out_7560663382806042444[79] = 0.0;
   out_7560663382806042444[80] = 0.0;
   out_7560663382806042444[81] = 0.0;
   out_7560663382806042444[82] = 0.0;
   out_7560663382806042444[83] = 0.0;
   out_7560663382806042444[84] = 0.0;
   out_7560663382806042444[85] = 0.0;
   out_7560663382806042444[86] = 0.0;
   out_7560663382806042444[87] = 0.0;
   out_7560663382806042444[88] = 0.0;
   out_7560663382806042444[89] = 0.0;
   out_7560663382806042444[90] = 0.0;
   out_7560663382806042444[91] = 0.0;
   out_7560663382806042444[92] = 0.0;
   out_7560663382806042444[93] = 0.0;
   out_7560663382806042444[94] = 0.0;
   out_7560663382806042444[95] = 1.0;
   out_7560663382806042444[96] = 0.0;
   out_7560663382806042444[97] = 0.0;
   out_7560663382806042444[98] = 0.0;
   out_7560663382806042444[99] = 0.0;
   out_7560663382806042444[100] = 0.0;
   out_7560663382806042444[101] = 0.0;
   out_7560663382806042444[102] = 0.0;
   out_7560663382806042444[103] = 0.0;
   out_7560663382806042444[104] = 0.0;
   out_7560663382806042444[105] = 0.0;
   out_7560663382806042444[106] = 0.0;
   out_7560663382806042444[107] = 0.0;
   out_7560663382806042444[108] = 0.0;
   out_7560663382806042444[109] = 0.0;
   out_7560663382806042444[110] = 0.0;
   out_7560663382806042444[111] = 0.0;
   out_7560663382806042444[112] = 0.0;
   out_7560663382806042444[113] = 0.0;
   out_7560663382806042444[114] = 1.0;
   out_7560663382806042444[115] = 0.0;
   out_7560663382806042444[116] = 0.0;
   out_7560663382806042444[117] = 0.0;
   out_7560663382806042444[118] = 0.0;
   out_7560663382806042444[119] = 0.0;
   out_7560663382806042444[120] = 0.0;
   out_7560663382806042444[121] = 0.0;
   out_7560663382806042444[122] = 0.0;
   out_7560663382806042444[123] = 0.0;
   out_7560663382806042444[124] = 0.0;
   out_7560663382806042444[125] = 0.0;
   out_7560663382806042444[126] = 0.0;
   out_7560663382806042444[127] = 0.0;
   out_7560663382806042444[128] = 0.0;
   out_7560663382806042444[129] = 0.0;
   out_7560663382806042444[130] = 0.0;
   out_7560663382806042444[131] = 0.0;
   out_7560663382806042444[132] = 0.0;
   out_7560663382806042444[133] = 1.0;
   out_7560663382806042444[134] = 0.0;
   out_7560663382806042444[135] = 0.0;
   out_7560663382806042444[136] = 0.0;
   out_7560663382806042444[137] = 0.0;
   out_7560663382806042444[138] = 0.0;
   out_7560663382806042444[139] = 0.0;
   out_7560663382806042444[140] = 0.0;
   out_7560663382806042444[141] = 0.0;
   out_7560663382806042444[142] = 0.0;
   out_7560663382806042444[143] = 0.0;
   out_7560663382806042444[144] = 0.0;
   out_7560663382806042444[145] = 0.0;
   out_7560663382806042444[146] = 0.0;
   out_7560663382806042444[147] = 0.0;
   out_7560663382806042444[148] = 0.0;
   out_7560663382806042444[149] = 0.0;
   out_7560663382806042444[150] = 0.0;
   out_7560663382806042444[151] = 0.0;
   out_7560663382806042444[152] = 1.0;
   out_7560663382806042444[153] = 0.0;
   out_7560663382806042444[154] = 0.0;
   out_7560663382806042444[155] = 0.0;
   out_7560663382806042444[156] = 0.0;
   out_7560663382806042444[157] = 0.0;
   out_7560663382806042444[158] = 0.0;
   out_7560663382806042444[159] = 0.0;
   out_7560663382806042444[160] = 0.0;
   out_7560663382806042444[161] = 0.0;
   out_7560663382806042444[162] = 0.0;
   out_7560663382806042444[163] = 0.0;
   out_7560663382806042444[164] = 0.0;
   out_7560663382806042444[165] = 0.0;
   out_7560663382806042444[166] = 0.0;
   out_7560663382806042444[167] = 0.0;
   out_7560663382806042444[168] = 0.0;
   out_7560663382806042444[169] = 0.0;
   out_7560663382806042444[170] = 0.0;
   out_7560663382806042444[171] = 1.0;
   out_7560663382806042444[172] = 0.0;
   out_7560663382806042444[173] = 0.0;
   out_7560663382806042444[174] = 0.0;
   out_7560663382806042444[175] = 0.0;
   out_7560663382806042444[176] = 0.0;
   out_7560663382806042444[177] = 0.0;
   out_7560663382806042444[178] = 0.0;
   out_7560663382806042444[179] = 0.0;
   out_7560663382806042444[180] = 0.0;
   out_7560663382806042444[181] = 0.0;
   out_7560663382806042444[182] = 0.0;
   out_7560663382806042444[183] = 0.0;
   out_7560663382806042444[184] = 0.0;
   out_7560663382806042444[185] = 0.0;
   out_7560663382806042444[186] = 0.0;
   out_7560663382806042444[187] = 0.0;
   out_7560663382806042444[188] = 0.0;
   out_7560663382806042444[189] = 0.0;
   out_7560663382806042444[190] = 1.0;
   out_7560663382806042444[191] = 0.0;
   out_7560663382806042444[192] = 0.0;
   out_7560663382806042444[193] = 0.0;
   out_7560663382806042444[194] = 0.0;
   out_7560663382806042444[195] = 0.0;
   out_7560663382806042444[196] = 0.0;
   out_7560663382806042444[197] = 0.0;
   out_7560663382806042444[198] = 0.0;
   out_7560663382806042444[199] = 0.0;
   out_7560663382806042444[200] = 0.0;
   out_7560663382806042444[201] = 0.0;
   out_7560663382806042444[202] = 0.0;
   out_7560663382806042444[203] = 0.0;
   out_7560663382806042444[204] = 0.0;
   out_7560663382806042444[205] = 0.0;
   out_7560663382806042444[206] = 0.0;
   out_7560663382806042444[207] = 0.0;
   out_7560663382806042444[208] = 0.0;
   out_7560663382806042444[209] = 1.0;
   out_7560663382806042444[210] = 0.0;
   out_7560663382806042444[211] = 0.0;
   out_7560663382806042444[212] = 0.0;
   out_7560663382806042444[213] = 0.0;
   out_7560663382806042444[214] = 0.0;
   out_7560663382806042444[215] = 0.0;
   out_7560663382806042444[216] = 0.0;
   out_7560663382806042444[217] = 0.0;
   out_7560663382806042444[218] = 0.0;
   out_7560663382806042444[219] = 0.0;
   out_7560663382806042444[220] = 0.0;
   out_7560663382806042444[221] = 0.0;
   out_7560663382806042444[222] = 0.0;
   out_7560663382806042444[223] = 0.0;
   out_7560663382806042444[224] = 0.0;
   out_7560663382806042444[225] = 0.0;
   out_7560663382806042444[226] = 0.0;
   out_7560663382806042444[227] = 0.0;
   out_7560663382806042444[228] = 1.0;
   out_7560663382806042444[229] = 0.0;
   out_7560663382806042444[230] = 0.0;
   out_7560663382806042444[231] = 0.0;
   out_7560663382806042444[232] = 0.0;
   out_7560663382806042444[233] = 0.0;
   out_7560663382806042444[234] = 0.0;
   out_7560663382806042444[235] = 0.0;
   out_7560663382806042444[236] = 0.0;
   out_7560663382806042444[237] = 0.0;
   out_7560663382806042444[238] = 0.0;
   out_7560663382806042444[239] = 0.0;
   out_7560663382806042444[240] = 0.0;
   out_7560663382806042444[241] = 0.0;
   out_7560663382806042444[242] = 0.0;
   out_7560663382806042444[243] = 0.0;
   out_7560663382806042444[244] = 0.0;
   out_7560663382806042444[245] = 0.0;
   out_7560663382806042444[246] = 0.0;
   out_7560663382806042444[247] = 1.0;
   out_7560663382806042444[248] = 0.0;
   out_7560663382806042444[249] = 0.0;
   out_7560663382806042444[250] = 0.0;
   out_7560663382806042444[251] = 0.0;
   out_7560663382806042444[252] = 0.0;
   out_7560663382806042444[253] = 0.0;
   out_7560663382806042444[254] = 0.0;
   out_7560663382806042444[255] = 0.0;
   out_7560663382806042444[256] = 0.0;
   out_7560663382806042444[257] = 0.0;
   out_7560663382806042444[258] = 0.0;
   out_7560663382806042444[259] = 0.0;
   out_7560663382806042444[260] = 0.0;
   out_7560663382806042444[261] = 0.0;
   out_7560663382806042444[262] = 0.0;
   out_7560663382806042444[263] = 0.0;
   out_7560663382806042444[264] = 0.0;
   out_7560663382806042444[265] = 0.0;
   out_7560663382806042444[266] = 1.0;
   out_7560663382806042444[267] = 0.0;
   out_7560663382806042444[268] = 0.0;
   out_7560663382806042444[269] = 0.0;
   out_7560663382806042444[270] = 0.0;
   out_7560663382806042444[271] = 0.0;
   out_7560663382806042444[272] = 0.0;
   out_7560663382806042444[273] = 0.0;
   out_7560663382806042444[274] = 0.0;
   out_7560663382806042444[275] = 0.0;
   out_7560663382806042444[276] = 0.0;
   out_7560663382806042444[277] = 0.0;
   out_7560663382806042444[278] = 0.0;
   out_7560663382806042444[279] = 0.0;
   out_7560663382806042444[280] = 0.0;
   out_7560663382806042444[281] = 0.0;
   out_7560663382806042444[282] = 0.0;
   out_7560663382806042444[283] = 0.0;
   out_7560663382806042444[284] = 0.0;
   out_7560663382806042444[285] = 1.0;
   out_7560663382806042444[286] = 0.0;
   out_7560663382806042444[287] = 0.0;
   out_7560663382806042444[288] = 0.0;
   out_7560663382806042444[289] = 0.0;
   out_7560663382806042444[290] = 0.0;
   out_7560663382806042444[291] = 0.0;
   out_7560663382806042444[292] = 0.0;
   out_7560663382806042444[293] = 0.0;
   out_7560663382806042444[294] = 0.0;
   out_7560663382806042444[295] = 0.0;
   out_7560663382806042444[296] = 0.0;
   out_7560663382806042444[297] = 0.0;
   out_7560663382806042444[298] = 0.0;
   out_7560663382806042444[299] = 0.0;
   out_7560663382806042444[300] = 0.0;
   out_7560663382806042444[301] = 0.0;
   out_7560663382806042444[302] = 0.0;
   out_7560663382806042444[303] = 0.0;
   out_7560663382806042444[304] = 1.0;
   out_7560663382806042444[305] = 0.0;
   out_7560663382806042444[306] = 0.0;
   out_7560663382806042444[307] = 0.0;
   out_7560663382806042444[308] = 0.0;
   out_7560663382806042444[309] = 0.0;
   out_7560663382806042444[310] = 0.0;
   out_7560663382806042444[311] = 0.0;
   out_7560663382806042444[312] = 0.0;
   out_7560663382806042444[313] = 0.0;
   out_7560663382806042444[314] = 0.0;
   out_7560663382806042444[315] = 0.0;
   out_7560663382806042444[316] = 0.0;
   out_7560663382806042444[317] = 0.0;
   out_7560663382806042444[318] = 0.0;
   out_7560663382806042444[319] = 0.0;
   out_7560663382806042444[320] = 0.0;
   out_7560663382806042444[321] = 0.0;
   out_7560663382806042444[322] = 0.0;
   out_7560663382806042444[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_4734719999270252373) {
   out_4734719999270252373[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_4734719999270252373[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_4734719999270252373[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_4734719999270252373[3] = dt*state[12] + state[3];
   out_4734719999270252373[4] = dt*state[13] + state[4];
   out_4734719999270252373[5] = dt*state[14] + state[5];
   out_4734719999270252373[6] = state[6];
   out_4734719999270252373[7] = state[7];
   out_4734719999270252373[8] = state[8];
   out_4734719999270252373[9] = state[9];
   out_4734719999270252373[10] = state[10];
   out_4734719999270252373[11] = state[11];
   out_4734719999270252373[12] = state[12];
   out_4734719999270252373[13] = state[13];
   out_4734719999270252373[14] = state[14];
   out_4734719999270252373[15] = state[15];
   out_4734719999270252373[16] = state[16];
   out_4734719999270252373[17] = state[17];
}
void F_fun(double *state, double dt, double *out_7949110505019825749) {
   out_7949110505019825749[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7949110505019825749[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7949110505019825749[2] = 0;
   out_7949110505019825749[3] = 0;
   out_7949110505019825749[4] = 0;
   out_7949110505019825749[5] = 0;
   out_7949110505019825749[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7949110505019825749[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7949110505019825749[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7949110505019825749[9] = 0;
   out_7949110505019825749[10] = 0;
   out_7949110505019825749[11] = 0;
   out_7949110505019825749[12] = 0;
   out_7949110505019825749[13] = 0;
   out_7949110505019825749[14] = 0;
   out_7949110505019825749[15] = 0;
   out_7949110505019825749[16] = 0;
   out_7949110505019825749[17] = 0;
   out_7949110505019825749[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7949110505019825749[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7949110505019825749[20] = 0;
   out_7949110505019825749[21] = 0;
   out_7949110505019825749[22] = 0;
   out_7949110505019825749[23] = 0;
   out_7949110505019825749[24] = 0;
   out_7949110505019825749[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7949110505019825749[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7949110505019825749[27] = 0;
   out_7949110505019825749[28] = 0;
   out_7949110505019825749[29] = 0;
   out_7949110505019825749[30] = 0;
   out_7949110505019825749[31] = 0;
   out_7949110505019825749[32] = 0;
   out_7949110505019825749[33] = 0;
   out_7949110505019825749[34] = 0;
   out_7949110505019825749[35] = 0;
   out_7949110505019825749[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7949110505019825749[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7949110505019825749[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7949110505019825749[39] = 0;
   out_7949110505019825749[40] = 0;
   out_7949110505019825749[41] = 0;
   out_7949110505019825749[42] = 0;
   out_7949110505019825749[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7949110505019825749[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7949110505019825749[45] = 0;
   out_7949110505019825749[46] = 0;
   out_7949110505019825749[47] = 0;
   out_7949110505019825749[48] = 0;
   out_7949110505019825749[49] = 0;
   out_7949110505019825749[50] = 0;
   out_7949110505019825749[51] = 0;
   out_7949110505019825749[52] = 0;
   out_7949110505019825749[53] = 0;
   out_7949110505019825749[54] = 0;
   out_7949110505019825749[55] = 0;
   out_7949110505019825749[56] = 0;
   out_7949110505019825749[57] = 1;
   out_7949110505019825749[58] = 0;
   out_7949110505019825749[59] = 0;
   out_7949110505019825749[60] = 0;
   out_7949110505019825749[61] = 0;
   out_7949110505019825749[62] = 0;
   out_7949110505019825749[63] = 0;
   out_7949110505019825749[64] = 0;
   out_7949110505019825749[65] = 0;
   out_7949110505019825749[66] = dt;
   out_7949110505019825749[67] = 0;
   out_7949110505019825749[68] = 0;
   out_7949110505019825749[69] = 0;
   out_7949110505019825749[70] = 0;
   out_7949110505019825749[71] = 0;
   out_7949110505019825749[72] = 0;
   out_7949110505019825749[73] = 0;
   out_7949110505019825749[74] = 0;
   out_7949110505019825749[75] = 0;
   out_7949110505019825749[76] = 1;
   out_7949110505019825749[77] = 0;
   out_7949110505019825749[78] = 0;
   out_7949110505019825749[79] = 0;
   out_7949110505019825749[80] = 0;
   out_7949110505019825749[81] = 0;
   out_7949110505019825749[82] = 0;
   out_7949110505019825749[83] = 0;
   out_7949110505019825749[84] = 0;
   out_7949110505019825749[85] = dt;
   out_7949110505019825749[86] = 0;
   out_7949110505019825749[87] = 0;
   out_7949110505019825749[88] = 0;
   out_7949110505019825749[89] = 0;
   out_7949110505019825749[90] = 0;
   out_7949110505019825749[91] = 0;
   out_7949110505019825749[92] = 0;
   out_7949110505019825749[93] = 0;
   out_7949110505019825749[94] = 0;
   out_7949110505019825749[95] = 1;
   out_7949110505019825749[96] = 0;
   out_7949110505019825749[97] = 0;
   out_7949110505019825749[98] = 0;
   out_7949110505019825749[99] = 0;
   out_7949110505019825749[100] = 0;
   out_7949110505019825749[101] = 0;
   out_7949110505019825749[102] = 0;
   out_7949110505019825749[103] = 0;
   out_7949110505019825749[104] = dt;
   out_7949110505019825749[105] = 0;
   out_7949110505019825749[106] = 0;
   out_7949110505019825749[107] = 0;
   out_7949110505019825749[108] = 0;
   out_7949110505019825749[109] = 0;
   out_7949110505019825749[110] = 0;
   out_7949110505019825749[111] = 0;
   out_7949110505019825749[112] = 0;
   out_7949110505019825749[113] = 0;
   out_7949110505019825749[114] = 1;
   out_7949110505019825749[115] = 0;
   out_7949110505019825749[116] = 0;
   out_7949110505019825749[117] = 0;
   out_7949110505019825749[118] = 0;
   out_7949110505019825749[119] = 0;
   out_7949110505019825749[120] = 0;
   out_7949110505019825749[121] = 0;
   out_7949110505019825749[122] = 0;
   out_7949110505019825749[123] = 0;
   out_7949110505019825749[124] = 0;
   out_7949110505019825749[125] = 0;
   out_7949110505019825749[126] = 0;
   out_7949110505019825749[127] = 0;
   out_7949110505019825749[128] = 0;
   out_7949110505019825749[129] = 0;
   out_7949110505019825749[130] = 0;
   out_7949110505019825749[131] = 0;
   out_7949110505019825749[132] = 0;
   out_7949110505019825749[133] = 1;
   out_7949110505019825749[134] = 0;
   out_7949110505019825749[135] = 0;
   out_7949110505019825749[136] = 0;
   out_7949110505019825749[137] = 0;
   out_7949110505019825749[138] = 0;
   out_7949110505019825749[139] = 0;
   out_7949110505019825749[140] = 0;
   out_7949110505019825749[141] = 0;
   out_7949110505019825749[142] = 0;
   out_7949110505019825749[143] = 0;
   out_7949110505019825749[144] = 0;
   out_7949110505019825749[145] = 0;
   out_7949110505019825749[146] = 0;
   out_7949110505019825749[147] = 0;
   out_7949110505019825749[148] = 0;
   out_7949110505019825749[149] = 0;
   out_7949110505019825749[150] = 0;
   out_7949110505019825749[151] = 0;
   out_7949110505019825749[152] = 1;
   out_7949110505019825749[153] = 0;
   out_7949110505019825749[154] = 0;
   out_7949110505019825749[155] = 0;
   out_7949110505019825749[156] = 0;
   out_7949110505019825749[157] = 0;
   out_7949110505019825749[158] = 0;
   out_7949110505019825749[159] = 0;
   out_7949110505019825749[160] = 0;
   out_7949110505019825749[161] = 0;
   out_7949110505019825749[162] = 0;
   out_7949110505019825749[163] = 0;
   out_7949110505019825749[164] = 0;
   out_7949110505019825749[165] = 0;
   out_7949110505019825749[166] = 0;
   out_7949110505019825749[167] = 0;
   out_7949110505019825749[168] = 0;
   out_7949110505019825749[169] = 0;
   out_7949110505019825749[170] = 0;
   out_7949110505019825749[171] = 1;
   out_7949110505019825749[172] = 0;
   out_7949110505019825749[173] = 0;
   out_7949110505019825749[174] = 0;
   out_7949110505019825749[175] = 0;
   out_7949110505019825749[176] = 0;
   out_7949110505019825749[177] = 0;
   out_7949110505019825749[178] = 0;
   out_7949110505019825749[179] = 0;
   out_7949110505019825749[180] = 0;
   out_7949110505019825749[181] = 0;
   out_7949110505019825749[182] = 0;
   out_7949110505019825749[183] = 0;
   out_7949110505019825749[184] = 0;
   out_7949110505019825749[185] = 0;
   out_7949110505019825749[186] = 0;
   out_7949110505019825749[187] = 0;
   out_7949110505019825749[188] = 0;
   out_7949110505019825749[189] = 0;
   out_7949110505019825749[190] = 1;
   out_7949110505019825749[191] = 0;
   out_7949110505019825749[192] = 0;
   out_7949110505019825749[193] = 0;
   out_7949110505019825749[194] = 0;
   out_7949110505019825749[195] = 0;
   out_7949110505019825749[196] = 0;
   out_7949110505019825749[197] = 0;
   out_7949110505019825749[198] = 0;
   out_7949110505019825749[199] = 0;
   out_7949110505019825749[200] = 0;
   out_7949110505019825749[201] = 0;
   out_7949110505019825749[202] = 0;
   out_7949110505019825749[203] = 0;
   out_7949110505019825749[204] = 0;
   out_7949110505019825749[205] = 0;
   out_7949110505019825749[206] = 0;
   out_7949110505019825749[207] = 0;
   out_7949110505019825749[208] = 0;
   out_7949110505019825749[209] = 1;
   out_7949110505019825749[210] = 0;
   out_7949110505019825749[211] = 0;
   out_7949110505019825749[212] = 0;
   out_7949110505019825749[213] = 0;
   out_7949110505019825749[214] = 0;
   out_7949110505019825749[215] = 0;
   out_7949110505019825749[216] = 0;
   out_7949110505019825749[217] = 0;
   out_7949110505019825749[218] = 0;
   out_7949110505019825749[219] = 0;
   out_7949110505019825749[220] = 0;
   out_7949110505019825749[221] = 0;
   out_7949110505019825749[222] = 0;
   out_7949110505019825749[223] = 0;
   out_7949110505019825749[224] = 0;
   out_7949110505019825749[225] = 0;
   out_7949110505019825749[226] = 0;
   out_7949110505019825749[227] = 0;
   out_7949110505019825749[228] = 1;
   out_7949110505019825749[229] = 0;
   out_7949110505019825749[230] = 0;
   out_7949110505019825749[231] = 0;
   out_7949110505019825749[232] = 0;
   out_7949110505019825749[233] = 0;
   out_7949110505019825749[234] = 0;
   out_7949110505019825749[235] = 0;
   out_7949110505019825749[236] = 0;
   out_7949110505019825749[237] = 0;
   out_7949110505019825749[238] = 0;
   out_7949110505019825749[239] = 0;
   out_7949110505019825749[240] = 0;
   out_7949110505019825749[241] = 0;
   out_7949110505019825749[242] = 0;
   out_7949110505019825749[243] = 0;
   out_7949110505019825749[244] = 0;
   out_7949110505019825749[245] = 0;
   out_7949110505019825749[246] = 0;
   out_7949110505019825749[247] = 1;
   out_7949110505019825749[248] = 0;
   out_7949110505019825749[249] = 0;
   out_7949110505019825749[250] = 0;
   out_7949110505019825749[251] = 0;
   out_7949110505019825749[252] = 0;
   out_7949110505019825749[253] = 0;
   out_7949110505019825749[254] = 0;
   out_7949110505019825749[255] = 0;
   out_7949110505019825749[256] = 0;
   out_7949110505019825749[257] = 0;
   out_7949110505019825749[258] = 0;
   out_7949110505019825749[259] = 0;
   out_7949110505019825749[260] = 0;
   out_7949110505019825749[261] = 0;
   out_7949110505019825749[262] = 0;
   out_7949110505019825749[263] = 0;
   out_7949110505019825749[264] = 0;
   out_7949110505019825749[265] = 0;
   out_7949110505019825749[266] = 1;
   out_7949110505019825749[267] = 0;
   out_7949110505019825749[268] = 0;
   out_7949110505019825749[269] = 0;
   out_7949110505019825749[270] = 0;
   out_7949110505019825749[271] = 0;
   out_7949110505019825749[272] = 0;
   out_7949110505019825749[273] = 0;
   out_7949110505019825749[274] = 0;
   out_7949110505019825749[275] = 0;
   out_7949110505019825749[276] = 0;
   out_7949110505019825749[277] = 0;
   out_7949110505019825749[278] = 0;
   out_7949110505019825749[279] = 0;
   out_7949110505019825749[280] = 0;
   out_7949110505019825749[281] = 0;
   out_7949110505019825749[282] = 0;
   out_7949110505019825749[283] = 0;
   out_7949110505019825749[284] = 0;
   out_7949110505019825749[285] = 1;
   out_7949110505019825749[286] = 0;
   out_7949110505019825749[287] = 0;
   out_7949110505019825749[288] = 0;
   out_7949110505019825749[289] = 0;
   out_7949110505019825749[290] = 0;
   out_7949110505019825749[291] = 0;
   out_7949110505019825749[292] = 0;
   out_7949110505019825749[293] = 0;
   out_7949110505019825749[294] = 0;
   out_7949110505019825749[295] = 0;
   out_7949110505019825749[296] = 0;
   out_7949110505019825749[297] = 0;
   out_7949110505019825749[298] = 0;
   out_7949110505019825749[299] = 0;
   out_7949110505019825749[300] = 0;
   out_7949110505019825749[301] = 0;
   out_7949110505019825749[302] = 0;
   out_7949110505019825749[303] = 0;
   out_7949110505019825749[304] = 1;
   out_7949110505019825749[305] = 0;
   out_7949110505019825749[306] = 0;
   out_7949110505019825749[307] = 0;
   out_7949110505019825749[308] = 0;
   out_7949110505019825749[309] = 0;
   out_7949110505019825749[310] = 0;
   out_7949110505019825749[311] = 0;
   out_7949110505019825749[312] = 0;
   out_7949110505019825749[313] = 0;
   out_7949110505019825749[314] = 0;
   out_7949110505019825749[315] = 0;
   out_7949110505019825749[316] = 0;
   out_7949110505019825749[317] = 0;
   out_7949110505019825749[318] = 0;
   out_7949110505019825749[319] = 0;
   out_7949110505019825749[320] = 0;
   out_7949110505019825749[321] = 0;
   out_7949110505019825749[322] = 0;
   out_7949110505019825749[323] = 1;
}
void h_4(double *state, double *unused, double *out_3083781484230159567) {
   out_3083781484230159567[0] = state[6] + state[9];
   out_3083781484230159567[1] = state[7] + state[10];
   out_3083781484230159567[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_4286441304015093191) {
   out_4286441304015093191[0] = 0;
   out_4286441304015093191[1] = 0;
   out_4286441304015093191[2] = 0;
   out_4286441304015093191[3] = 0;
   out_4286441304015093191[4] = 0;
   out_4286441304015093191[5] = 0;
   out_4286441304015093191[6] = 1;
   out_4286441304015093191[7] = 0;
   out_4286441304015093191[8] = 0;
   out_4286441304015093191[9] = 1;
   out_4286441304015093191[10] = 0;
   out_4286441304015093191[11] = 0;
   out_4286441304015093191[12] = 0;
   out_4286441304015093191[13] = 0;
   out_4286441304015093191[14] = 0;
   out_4286441304015093191[15] = 0;
   out_4286441304015093191[16] = 0;
   out_4286441304015093191[17] = 0;
   out_4286441304015093191[18] = 0;
   out_4286441304015093191[19] = 0;
   out_4286441304015093191[20] = 0;
   out_4286441304015093191[21] = 0;
   out_4286441304015093191[22] = 0;
   out_4286441304015093191[23] = 0;
   out_4286441304015093191[24] = 0;
   out_4286441304015093191[25] = 1;
   out_4286441304015093191[26] = 0;
   out_4286441304015093191[27] = 0;
   out_4286441304015093191[28] = 1;
   out_4286441304015093191[29] = 0;
   out_4286441304015093191[30] = 0;
   out_4286441304015093191[31] = 0;
   out_4286441304015093191[32] = 0;
   out_4286441304015093191[33] = 0;
   out_4286441304015093191[34] = 0;
   out_4286441304015093191[35] = 0;
   out_4286441304015093191[36] = 0;
   out_4286441304015093191[37] = 0;
   out_4286441304015093191[38] = 0;
   out_4286441304015093191[39] = 0;
   out_4286441304015093191[40] = 0;
   out_4286441304015093191[41] = 0;
   out_4286441304015093191[42] = 0;
   out_4286441304015093191[43] = 0;
   out_4286441304015093191[44] = 1;
   out_4286441304015093191[45] = 0;
   out_4286441304015093191[46] = 0;
   out_4286441304015093191[47] = 1;
   out_4286441304015093191[48] = 0;
   out_4286441304015093191[49] = 0;
   out_4286441304015093191[50] = 0;
   out_4286441304015093191[51] = 0;
   out_4286441304015093191[52] = 0;
   out_4286441304015093191[53] = 0;
}
void h_10(double *state, double *unused, double *out_1431314934865780613) {
   out_1431314934865780613[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_1431314934865780613[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_1431314934865780613[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_4520041178384004627) {
   out_4520041178384004627[0] = 0;
   out_4520041178384004627[1] = 9.8100000000000005*cos(state[1]);
   out_4520041178384004627[2] = 0;
   out_4520041178384004627[3] = 0;
   out_4520041178384004627[4] = -state[8];
   out_4520041178384004627[5] = state[7];
   out_4520041178384004627[6] = 0;
   out_4520041178384004627[7] = state[5];
   out_4520041178384004627[8] = -state[4];
   out_4520041178384004627[9] = 0;
   out_4520041178384004627[10] = 0;
   out_4520041178384004627[11] = 0;
   out_4520041178384004627[12] = 1;
   out_4520041178384004627[13] = 0;
   out_4520041178384004627[14] = 0;
   out_4520041178384004627[15] = 1;
   out_4520041178384004627[16] = 0;
   out_4520041178384004627[17] = 0;
   out_4520041178384004627[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_4520041178384004627[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_4520041178384004627[20] = 0;
   out_4520041178384004627[21] = state[8];
   out_4520041178384004627[22] = 0;
   out_4520041178384004627[23] = -state[6];
   out_4520041178384004627[24] = -state[5];
   out_4520041178384004627[25] = 0;
   out_4520041178384004627[26] = state[3];
   out_4520041178384004627[27] = 0;
   out_4520041178384004627[28] = 0;
   out_4520041178384004627[29] = 0;
   out_4520041178384004627[30] = 0;
   out_4520041178384004627[31] = 1;
   out_4520041178384004627[32] = 0;
   out_4520041178384004627[33] = 0;
   out_4520041178384004627[34] = 1;
   out_4520041178384004627[35] = 0;
   out_4520041178384004627[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_4520041178384004627[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_4520041178384004627[38] = 0;
   out_4520041178384004627[39] = -state[7];
   out_4520041178384004627[40] = state[6];
   out_4520041178384004627[41] = 0;
   out_4520041178384004627[42] = state[4];
   out_4520041178384004627[43] = -state[3];
   out_4520041178384004627[44] = 0;
   out_4520041178384004627[45] = 0;
   out_4520041178384004627[46] = 0;
   out_4520041178384004627[47] = 0;
   out_4520041178384004627[48] = 0;
   out_4520041178384004627[49] = 0;
   out_4520041178384004627[50] = 1;
   out_4520041178384004627[51] = 0;
   out_4520041178384004627[52] = 0;
   out_4520041178384004627[53] = 1;
}
void h_13(double *state, double *unused, double *out_1948613078602992934) {
   out_1948613078602992934[0] = state[3];
   out_1948613078602992934[1] = state[4];
   out_1948613078602992934[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6549671561377757496) {
   out_6549671561377757496[0] = 0;
   out_6549671561377757496[1] = 0;
   out_6549671561377757496[2] = 0;
   out_6549671561377757496[3] = 1;
   out_6549671561377757496[4] = 0;
   out_6549671561377757496[5] = 0;
   out_6549671561377757496[6] = 0;
   out_6549671561377757496[7] = 0;
   out_6549671561377757496[8] = 0;
   out_6549671561377757496[9] = 0;
   out_6549671561377757496[10] = 0;
   out_6549671561377757496[11] = 0;
   out_6549671561377757496[12] = 0;
   out_6549671561377757496[13] = 0;
   out_6549671561377757496[14] = 0;
   out_6549671561377757496[15] = 0;
   out_6549671561377757496[16] = 0;
   out_6549671561377757496[17] = 0;
   out_6549671561377757496[18] = 0;
   out_6549671561377757496[19] = 0;
   out_6549671561377757496[20] = 0;
   out_6549671561377757496[21] = 0;
   out_6549671561377757496[22] = 1;
   out_6549671561377757496[23] = 0;
   out_6549671561377757496[24] = 0;
   out_6549671561377757496[25] = 0;
   out_6549671561377757496[26] = 0;
   out_6549671561377757496[27] = 0;
   out_6549671561377757496[28] = 0;
   out_6549671561377757496[29] = 0;
   out_6549671561377757496[30] = 0;
   out_6549671561377757496[31] = 0;
   out_6549671561377757496[32] = 0;
   out_6549671561377757496[33] = 0;
   out_6549671561377757496[34] = 0;
   out_6549671561377757496[35] = 0;
   out_6549671561377757496[36] = 0;
   out_6549671561377757496[37] = 0;
   out_6549671561377757496[38] = 0;
   out_6549671561377757496[39] = 0;
   out_6549671561377757496[40] = 0;
   out_6549671561377757496[41] = 1;
   out_6549671561377757496[42] = 0;
   out_6549671561377757496[43] = 0;
   out_6549671561377757496[44] = 0;
   out_6549671561377757496[45] = 0;
   out_6549671561377757496[46] = 0;
   out_6549671561377757496[47] = 0;
   out_6549671561377757496[48] = 0;
   out_6549671561377757496[49] = 0;
   out_6549671561377757496[50] = 0;
   out_6549671561377757496[51] = 0;
   out_6549671561377757496[52] = 0;
   out_6549671561377757496[53] = 0;
}
void h_14(double *state, double *unused, double *out_7821491744597387773) {
   out_7821491744597387773[0] = state[6];
   out_7821491744597387773[1] = state[7];
   out_7821491744597387773[2] = state[8];
}
void H_14(double *state, double *unused, double *out_8249682160354577720) {
   out_8249682160354577720[0] = 0;
   out_8249682160354577720[1] = 0;
   out_8249682160354577720[2] = 0;
   out_8249682160354577720[3] = 0;
   out_8249682160354577720[4] = 0;
   out_8249682160354577720[5] = 0;
   out_8249682160354577720[6] = 1;
   out_8249682160354577720[7] = 0;
   out_8249682160354577720[8] = 0;
   out_8249682160354577720[9] = 0;
   out_8249682160354577720[10] = 0;
   out_8249682160354577720[11] = 0;
   out_8249682160354577720[12] = 0;
   out_8249682160354577720[13] = 0;
   out_8249682160354577720[14] = 0;
   out_8249682160354577720[15] = 0;
   out_8249682160354577720[16] = 0;
   out_8249682160354577720[17] = 0;
   out_8249682160354577720[18] = 0;
   out_8249682160354577720[19] = 0;
   out_8249682160354577720[20] = 0;
   out_8249682160354577720[21] = 0;
   out_8249682160354577720[22] = 0;
   out_8249682160354577720[23] = 0;
   out_8249682160354577720[24] = 0;
   out_8249682160354577720[25] = 1;
   out_8249682160354577720[26] = 0;
   out_8249682160354577720[27] = 0;
   out_8249682160354577720[28] = 0;
   out_8249682160354577720[29] = 0;
   out_8249682160354577720[30] = 0;
   out_8249682160354577720[31] = 0;
   out_8249682160354577720[32] = 0;
   out_8249682160354577720[33] = 0;
   out_8249682160354577720[34] = 0;
   out_8249682160354577720[35] = 0;
   out_8249682160354577720[36] = 0;
   out_8249682160354577720[37] = 0;
   out_8249682160354577720[38] = 0;
   out_8249682160354577720[39] = 0;
   out_8249682160354577720[40] = 0;
   out_8249682160354577720[41] = 0;
   out_8249682160354577720[42] = 0;
   out_8249682160354577720[43] = 0;
   out_8249682160354577720[44] = 1;
   out_8249682160354577720[45] = 0;
   out_8249682160354577720[46] = 0;
   out_8249682160354577720[47] = 0;
   out_8249682160354577720[48] = 0;
   out_8249682160354577720[49] = 0;
   out_8249682160354577720[50] = 0;
   out_8249682160354577720[51] = 0;
   out_8249682160354577720[52] = 0;
   out_8249682160354577720[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_6666156999559179096) {
  err_fun(nom_x, delta_x, out_6666156999559179096);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6383049424053173820) {
  inv_err_fun(nom_x, true_x, out_6383049424053173820);
}
void pose_H_mod_fun(double *state, double *out_7560663382806042444) {
  H_mod_fun(state, out_7560663382806042444);
}
void pose_f_fun(double *state, double dt, double *out_4734719999270252373) {
  f_fun(state,  dt, out_4734719999270252373);
}
void pose_F_fun(double *state, double dt, double *out_7949110505019825749) {
  F_fun(state,  dt, out_7949110505019825749);
}
void pose_h_4(double *state, double *unused, double *out_3083781484230159567) {
  h_4(state, unused, out_3083781484230159567);
}
void pose_H_4(double *state, double *unused, double *out_4286441304015093191) {
  H_4(state, unused, out_4286441304015093191);
}
void pose_h_10(double *state, double *unused, double *out_1431314934865780613) {
  h_10(state, unused, out_1431314934865780613);
}
void pose_H_10(double *state, double *unused, double *out_4520041178384004627) {
  H_10(state, unused, out_4520041178384004627);
}
void pose_h_13(double *state, double *unused, double *out_1948613078602992934) {
  h_13(state, unused, out_1948613078602992934);
}
void pose_H_13(double *state, double *unused, double *out_6549671561377757496) {
  H_13(state, unused, out_6549671561377757496);
}
void pose_h_14(double *state, double *unused, double *out_7821491744597387773) {
  h_14(state, unused, out_7821491744597387773);
}
void pose_H_14(double *state, double *unused, double *out_8249682160354577720) {
  H_14(state, unused, out_8249682160354577720);
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
