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
void err_fun(double *nom_x, double *delta_x, double *out_8365414901728243703) {
   out_8365414901728243703[0] = delta_x[0] + nom_x[0];
   out_8365414901728243703[1] = delta_x[1] + nom_x[1];
   out_8365414901728243703[2] = delta_x[2] + nom_x[2];
   out_8365414901728243703[3] = delta_x[3] + nom_x[3];
   out_8365414901728243703[4] = delta_x[4] + nom_x[4];
   out_8365414901728243703[5] = delta_x[5] + nom_x[5];
   out_8365414901728243703[6] = delta_x[6] + nom_x[6];
   out_8365414901728243703[7] = delta_x[7] + nom_x[7];
   out_8365414901728243703[8] = delta_x[8] + nom_x[8];
   out_8365414901728243703[9] = delta_x[9] + nom_x[9];
   out_8365414901728243703[10] = delta_x[10] + nom_x[10];
   out_8365414901728243703[11] = delta_x[11] + nom_x[11];
   out_8365414901728243703[12] = delta_x[12] + nom_x[12];
   out_8365414901728243703[13] = delta_x[13] + nom_x[13];
   out_8365414901728243703[14] = delta_x[14] + nom_x[14];
   out_8365414901728243703[15] = delta_x[15] + nom_x[15];
   out_8365414901728243703[16] = delta_x[16] + nom_x[16];
   out_8365414901728243703[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2264373635515068274) {
   out_2264373635515068274[0] = -nom_x[0] + true_x[0];
   out_2264373635515068274[1] = -nom_x[1] + true_x[1];
   out_2264373635515068274[2] = -nom_x[2] + true_x[2];
   out_2264373635515068274[3] = -nom_x[3] + true_x[3];
   out_2264373635515068274[4] = -nom_x[4] + true_x[4];
   out_2264373635515068274[5] = -nom_x[5] + true_x[5];
   out_2264373635515068274[6] = -nom_x[6] + true_x[6];
   out_2264373635515068274[7] = -nom_x[7] + true_x[7];
   out_2264373635515068274[8] = -nom_x[8] + true_x[8];
   out_2264373635515068274[9] = -nom_x[9] + true_x[9];
   out_2264373635515068274[10] = -nom_x[10] + true_x[10];
   out_2264373635515068274[11] = -nom_x[11] + true_x[11];
   out_2264373635515068274[12] = -nom_x[12] + true_x[12];
   out_2264373635515068274[13] = -nom_x[13] + true_x[13];
   out_2264373635515068274[14] = -nom_x[14] + true_x[14];
   out_2264373635515068274[15] = -nom_x[15] + true_x[15];
   out_2264373635515068274[16] = -nom_x[16] + true_x[16];
   out_2264373635515068274[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_9199109663248748241) {
   out_9199109663248748241[0] = 1.0;
   out_9199109663248748241[1] = 0.0;
   out_9199109663248748241[2] = 0.0;
   out_9199109663248748241[3] = 0.0;
   out_9199109663248748241[4] = 0.0;
   out_9199109663248748241[5] = 0.0;
   out_9199109663248748241[6] = 0.0;
   out_9199109663248748241[7] = 0.0;
   out_9199109663248748241[8] = 0.0;
   out_9199109663248748241[9] = 0.0;
   out_9199109663248748241[10] = 0.0;
   out_9199109663248748241[11] = 0.0;
   out_9199109663248748241[12] = 0.0;
   out_9199109663248748241[13] = 0.0;
   out_9199109663248748241[14] = 0.0;
   out_9199109663248748241[15] = 0.0;
   out_9199109663248748241[16] = 0.0;
   out_9199109663248748241[17] = 0.0;
   out_9199109663248748241[18] = 0.0;
   out_9199109663248748241[19] = 1.0;
   out_9199109663248748241[20] = 0.0;
   out_9199109663248748241[21] = 0.0;
   out_9199109663248748241[22] = 0.0;
   out_9199109663248748241[23] = 0.0;
   out_9199109663248748241[24] = 0.0;
   out_9199109663248748241[25] = 0.0;
   out_9199109663248748241[26] = 0.0;
   out_9199109663248748241[27] = 0.0;
   out_9199109663248748241[28] = 0.0;
   out_9199109663248748241[29] = 0.0;
   out_9199109663248748241[30] = 0.0;
   out_9199109663248748241[31] = 0.0;
   out_9199109663248748241[32] = 0.0;
   out_9199109663248748241[33] = 0.0;
   out_9199109663248748241[34] = 0.0;
   out_9199109663248748241[35] = 0.0;
   out_9199109663248748241[36] = 0.0;
   out_9199109663248748241[37] = 0.0;
   out_9199109663248748241[38] = 1.0;
   out_9199109663248748241[39] = 0.0;
   out_9199109663248748241[40] = 0.0;
   out_9199109663248748241[41] = 0.0;
   out_9199109663248748241[42] = 0.0;
   out_9199109663248748241[43] = 0.0;
   out_9199109663248748241[44] = 0.0;
   out_9199109663248748241[45] = 0.0;
   out_9199109663248748241[46] = 0.0;
   out_9199109663248748241[47] = 0.0;
   out_9199109663248748241[48] = 0.0;
   out_9199109663248748241[49] = 0.0;
   out_9199109663248748241[50] = 0.0;
   out_9199109663248748241[51] = 0.0;
   out_9199109663248748241[52] = 0.0;
   out_9199109663248748241[53] = 0.0;
   out_9199109663248748241[54] = 0.0;
   out_9199109663248748241[55] = 0.0;
   out_9199109663248748241[56] = 0.0;
   out_9199109663248748241[57] = 1.0;
   out_9199109663248748241[58] = 0.0;
   out_9199109663248748241[59] = 0.0;
   out_9199109663248748241[60] = 0.0;
   out_9199109663248748241[61] = 0.0;
   out_9199109663248748241[62] = 0.0;
   out_9199109663248748241[63] = 0.0;
   out_9199109663248748241[64] = 0.0;
   out_9199109663248748241[65] = 0.0;
   out_9199109663248748241[66] = 0.0;
   out_9199109663248748241[67] = 0.0;
   out_9199109663248748241[68] = 0.0;
   out_9199109663248748241[69] = 0.0;
   out_9199109663248748241[70] = 0.0;
   out_9199109663248748241[71] = 0.0;
   out_9199109663248748241[72] = 0.0;
   out_9199109663248748241[73] = 0.0;
   out_9199109663248748241[74] = 0.0;
   out_9199109663248748241[75] = 0.0;
   out_9199109663248748241[76] = 1.0;
   out_9199109663248748241[77] = 0.0;
   out_9199109663248748241[78] = 0.0;
   out_9199109663248748241[79] = 0.0;
   out_9199109663248748241[80] = 0.0;
   out_9199109663248748241[81] = 0.0;
   out_9199109663248748241[82] = 0.0;
   out_9199109663248748241[83] = 0.0;
   out_9199109663248748241[84] = 0.0;
   out_9199109663248748241[85] = 0.0;
   out_9199109663248748241[86] = 0.0;
   out_9199109663248748241[87] = 0.0;
   out_9199109663248748241[88] = 0.0;
   out_9199109663248748241[89] = 0.0;
   out_9199109663248748241[90] = 0.0;
   out_9199109663248748241[91] = 0.0;
   out_9199109663248748241[92] = 0.0;
   out_9199109663248748241[93] = 0.0;
   out_9199109663248748241[94] = 0.0;
   out_9199109663248748241[95] = 1.0;
   out_9199109663248748241[96] = 0.0;
   out_9199109663248748241[97] = 0.0;
   out_9199109663248748241[98] = 0.0;
   out_9199109663248748241[99] = 0.0;
   out_9199109663248748241[100] = 0.0;
   out_9199109663248748241[101] = 0.0;
   out_9199109663248748241[102] = 0.0;
   out_9199109663248748241[103] = 0.0;
   out_9199109663248748241[104] = 0.0;
   out_9199109663248748241[105] = 0.0;
   out_9199109663248748241[106] = 0.0;
   out_9199109663248748241[107] = 0.0;
   out_9199109663248748241[108] = 0.0;
   out_9199109663248748241[109] = 0.0;
   out_9199109663248748241[110] = 0.0;
   out_9199109663248748241[111] = 0.0;
   out_9199109663248748241[112] = 0.0;
   out_9199109663248748241[113] = 0.0;
   out_9199109663248748241[114] = 1.0;
   out_9199109663248748241[115] = 0.0;
   out_9199109663248748241[116] = 0.0;
   out_9199109663248748241[117] = 0.0;
   out_9199109663248748241[118] = 0.0;
   out_9199109663248748241[119] = 0.0;
   out_9199109663248748241[120] = 0.0;
   out_9199109663248748241[121] = 0.0;
   out_9199109663248748241[122] = 0.0;
   out_9199109663248748241[123] = 0.0;
   out_9199109663248748241[124] = 0.0;
   out_9199109663248748241[125] = 0.0;
   out_9199109663248748241[126] = 0.0;
   out_9199109663248748241[127] = 0.0;
   out_9199109663248748241[128] = 0.0;
   out_9199109663248748241[129] = 0.0;
   out_9199109663248748241[130] = 0.0;
   out_9199109663248748241[131] = 0.0;
   out_9199109663248748241[132] = 0.0;
   out_9199109663248748241[133] = 1.0;
   out_9199109663248748241[134] = 0.0;
   out_9199109663248748241[135] = 0.0;
   out_9199109663248748241[136] = 0.0;
   out_9199109663248748241[137] = 0.0;
   out_9199109663248748241[138] = 0.0;
   out_9199109663248748241[139] = 0.0;
   out_9199109663248748241[140] = 0.0;
   out_9199109663248748241[141] = 0.0;
   out_9199109663248748241[142] = 0.0;
   out_9199109663248748241[143] = 0.0;
   out_9199109663248748241[144] = 0.0;
   out_9199109663248748241[145] = 0.0;
   out_9199109663248748241[146] = 0.0;
   out_9199109663248748241[147] = 0.0;
   out_9199109663248748241[148] = 0.0;
   out_9199109663248748241[149] = 0.0;
   out_9199109663248748241[150] = 0.0;
   out_9199109663248748241[151] = 0.0;
   out_9199109663248748241[152] = 1.0;
   out_9199109663248748241[153] = 0.0;
   out_9199109663248748241[154] = 0.0;
   out_9199109663248748241[155] = 0.0;
   out_9199109663248748241[156] = 0.0;
   out_9199109663248748241[157] = 0.0;
   out_9199109663248748241[158] = 0.0;
   out_9199109663248748241[159] = 0.0;
   out_9199109663248748241[160] = 0.0;
   out_9199109663248748241[161] = 0.0;
   out_9199109663248748241[162] = 0.0;
   out_9199109663248748241[163] = 0.0;
   out_9199109663248748241[164] = 0.0;
   out_9199109663248748241[165] = 0.0;
   out_9199109663248748241[166] = 0.0;
   out_9199109663248748241[167] = 0.0;
   out_9199109663248748241[168] = 0.0;
   out_9199109663248748241[169] = 0.0;
   out_9199109663248748241[170] = 0.0;
   out_9199109663248748241[171] = 1.0;
   out_9199109663248748241[172] = 0.0;
   out_9199109663248748241[173] = 0.0;
   out_9199109663248748241[174] = 0.0;
   out_9199109663248748241[175] = 0.0;
   out_9199109663248748241[176] = 0.0;
   out_9199109663248748241[177] = 0.0;
   out_9199109663248748241[178] = 0.0;
   out_9199109663248748241[179] = 0.0;
   out_9199109663248748241[180] = 0.0;
   out_9199109663248748241[181] = 0.0;
   out_9199109663248748241[182] = 0.0;
   out_9199109663248748241[183] = 0.0;
   out_9199109663248748241[184] = 0.0;
   out_9199109663248748241[185] = 0.0;
   out_9199109663248748241[186] = 0.0;
   out_9199109663248748241[187] = 0.0;
   out_9199109663248748241[188] = 0.0;
   out_9199109663248748241[189] = 0.0;
   out_9199109663248748241[190] = 1.0;
   out_9199109663248748241[191] = 0.0;
   out_9199109663248748241[192] = 0.0;
   out_9199109663248748241[193] = 0.0;
   out_9199109663248748241[194] = 0.0;
   out_9199109663248748241[195] = 0.0;
   out_9199109663248748241[196] = 0.0;
   out_9199109663248748241[197] = 0.0;
   out_9199109663248748241[198] = 0.0;
   out_9199109663248748241[199] = 0.0;
   out_9199109663248748241[200] = 0.0;
   out_9199109663248748241[201] = 0.0;
   out_9199109663248748241[202] = 0.0;
   out_9199109663248748241[203] = 0.0;
   out_9199109663248748241[204] = 0.0;
   out_9199109663248748241[205] = 0.0;
   out_9199109663248748241[206] = 0.0;
   out_9199109663248748241[207] = 0.0;
   out_9199109663248748241[208] = 0.0;
   out_9199109663248748241[209] = 1.0;
   out_9199109663248748241[210] = 0.0;
   out_9199109663248748241[211] = 0.0;
   out_9199109663248748241[212] = 0.0;
   out_9199109663248748241[213] = 0.0;
   out_9199109663248748241[214] = 0.0;
   out_9199109663248748241[215] = 0.0;
   out_9199109663248748241[216] = 0.0;
   out_9199109663248748241[217] = 0.0;
   out_9199109663248748241[218] = 0.0;
   out_9199109663248748241[219] = 0.0;
   out_9199109663248748241[220] = 0.0;
   out_9199109663248748241[221] = 0.0;
   out_9199109663248748241[222] = 0.0;
   out_9199109663248748241[223] = 0.0;
   out_9199109663248748241[224] = 0.0;
   out_9199109663248748241[225] = 0.0;
   out_9199109663248748241[226] = 0.0;
   out_9199109663248748241[227] = 0.0;
   out_9199109663248748241[228] = 1.0;
   out_9199109663248748241[229] = 0.0;
   out_9199109663248748241[230] = 0.0;
   out_9199109663248748241[231] = 0.0;
   out_9199109663248748241[232] = 0.0;
   out_9199109663248748241[233] = 0.0;
   out_9199109663248748241[234] = 0.0;
   out_9199109663248748241[235] = 0.0;
   out_9199109663248748241[236] = 0.0;
   out_9199109663248748241[237] = 0.0;
   out_9199109663248748241[238] = 0.0;
   out_9199109663248748241[239] = 0.0;
   out_9199109663248748241[240] = 0.0;
   out_9199109663248748241[241] = 0.0;
   out_9199109663248748241[242] = 0.0;
   out_9199109663248748241[243] = 0.0;
   out_9199109663248748241[244] = 0.0;
   out_9199109663248748241[245] = 0.0;
   out_9199109663248748241[246] = 0.0;
   out_9199109663248748241[247] = 1.0;
   out_9199109663248748241[248] = 0.0;
   out_9199109663248748241[249] = 0.0;
   out_9199109663248748241[250] = 0.0;
   out_9199109663248748241[251] = 0.0;
   out_9199109663248748241[252] = 0.0;
   out_9199109663248748241[253] = 0.0;
   out_9199109663248748241[254] = 0.0;
   out_9199109663248748241[255] = 0.0;
   out_9199109663248748241[256] = 0.0;
   out_9199109663248748241[257] = 0.0;
   out_9199109663248748241[258] = 0.0;
   out_9199109663248748241[259] = 0.0;
   out_9199109663248748241[260] = 0.0;
   out_9199109663248748241[261] = 0.0;
   out_9199109663248748241[262] = 0.0;
   out_9199109663248748241[263] = 0.0;
   out_9199109663248748241[264] = 0.0;
   out_9199109663248748241[265] = 0.0;
   out_9199109663248748241[266] = 1.0;
   out_9199109663248748241[267] = 0.0;
   out_9199109663248748241[268] = 0.0;
   out_9199109663248748241[269] = 0.0;
   out_9199109663248748241[270] = 0.0;
   out_9199109663248748241[271] = 0.0;
   out_9199109663248748241[272] = 0.0;
   out_9199109663248748241[273] = 0.0;
   out_9199109663248748241[274] = 0.0;
   out_9199109663248748241[275] = 0.0;
   out_9199109663248748241[276] = 0.0;
   out_9199109663248748241[277] = 0.0;
   out_9199109663248748241[278] = 0.0;
   out_9199109663248748241[279] = 0.0;
   out_9199109663248748241[280] = 0.0;
   out_9199109663248748241[281] = 0.0;
   out_9199109663248748241[282] = 0.0;
   out_9199109663248748241[283] = 0.0;
   out_9199109663248748241[284] = 0.0;
   out_9199109663248748241[285] = 1.0;
   out_9199109663248748241[286] = 0.0;
   out_9199109663248748241[287] = 0.0;
   out_9199109663248748241[288] = 0.0;
   out_9199109663248748241[289] = 0.0;
   out_9199109663248748241[290] = 0.0;
   out_9199109663248748241[291] = 0.0;
   out_9199109663248748241[292] = 0.0;
   out_9199109663248748241[293] = 0.0;
   out_9199109663248748241[294] = 0.0;
   out_9199109663248748241[295] = 0.0;
   out_9199109663248748241[296] = 0.0;
   out_9199109663248748241[297] = 0.0;
   out_9199109663248748241[298] = 0.0;
   out_9199109663248748241[299] = 0.0;
   out_9199109663248748241[300] = 0.0;
   out_9199109663248748241[301] = 0.0;
   out_9199109663248748241[302] = 0.0;
   out_9199109663248748241[303] = 0.0;
   out_9199109663248748241[304] = 1.0;
   out_9199109663248748241[305] = 0.0;
   out_9199109663248748241[306] = 0.0;
   out_9199109663248748241[307] = 0.0;
   out_9199109663248748241[308] = 0.0;
   out_9199109663248748241[309] = 0.0;
   out_9199109663248748241[310] = 0.0;
   out_9199109663248748241[311] = 0.0;
   out_9199109663248748241[312] = 0.0;
   out_9199109663248748241[313] = 0.0;
   out_9199109663248748241[314] = 0.0;
   out_9199109663248748241[315] = 0.0;
   out_9199109663248748241[316] = 0.0;
   out_9199109663248748241[317] = 0.0;
   out_9199109663248748241[318] = 0.0;
   out_9199109663248748241[319] = 0.0;
   out_9199109663248748241[320] = 0.0;
   out_9199109663248748241[321] = 0.0;
   out_9199109663248748241[322] = 0.0;
   out_9199109663248748241[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_465988394318989602) {
   out_465988394318989602[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_465988394318989602[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_465988394318989602[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_465988394318989602[3] = dt*state[12] + state[3];
   out_465988394318989602[4] = dt*state[13] + state[4];
   out_465988394318989602[5] = dt*state[14] + state[5];
   out_465988394318989602[6] = state[6];
   out_465988394318989602[7] = state[7];
   out_465988394318989602[8] = state[8];
   out_465988394318989602[9] = state[9];
   out_465988394318989602[10] = state[10];
   out_465988394318989602[11] = state[11];
   out_465988394318989602[12] = state[12];
   out_465988394318989602[13] = state[13];
   out_465988394318989602[14] = state[14];
   out_465988394318989602[15] = state[15];
   out_465988394318989602[16] = state[16];
   out_465988394318989602[17] = state[17];
}
void F_fun(double *state, double dt, double *out_58762011279446305) {
   out_58762011279446305[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_58762011279446305[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_58762011279446305[2] = 0;
   out_58762011279446305[3] = 0;
   out_58762011279446305[4] = 0;
   out_58762011279446305[5] = 0;
   out_58762011279446305[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_58762011279446305[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_58762011279446305[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_58762011279446305[9] = 0;
   out_58762011279446305[10] = 0;
   out_58762011279446305[11] = 0;
   out_58762011279446305[12] = 0;
   out_58762011279446305[13] = 0;
   out_58762011279446305[14] = 0;
   out_58762011279446305[15] = 0;
   out_58762011279446305[16] = 0;
   out_58762011279446305[17] = 0;
   out_58762011279446305[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_58762011279446305[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_58762011279446305[20] = 0;
   out_58762011279446305[21] = 0;
   out_58762011279446305[22] = 0;
   out_58762011279446305[23] = 0;
   out_58762011279446305[24] = 0;
   out_58762011279446305[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_58762011279446305[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_58762011279446305[27] = 0;
   out_58762011279446305[28] = 0;
   out_58762011279446305[29] = 0;
   out_58762011279446305[30] = 0;
   out_58762011279446305[31] = 0;
   out_58762011279446305[32] = 0;
   out_58762011279446305[33] = 0;
   out_58762011279446305[34] = 0;
   out_58762011279446305[35] = 0;
   out_58762011279446305[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_58762011279446305[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_58762011279446305[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_58762011279446305[39] = 0;
   out_58762011279446305[40] = 0;
   out_58762011279446305[41] = 0;
   out_58762011279446305[42] = 0;
   out_58762011279446305[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_58762011279446305[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_58762011279446305[45] = 0;
   out_58762011279446305[46] = 0;
   out_58762011279446305[47] = 0;
   out_58762011279446305[48] = 0;
   out_58762011279446305[49] = 0;
   out_58762011279446305[50] = 0;
   out_58762011279446305[51] = 0;
   out_58762011279446305[52] = 0;
   out_58762011279446305[53] = 0;
   out_58762011279446305[54] = 0;
   out_58762011279446305[55] = 0;
   out_58762011279446305[56] = 0;
   out_58762011279446305[57] = 1;
   out_58762011279446305[58] = 0;
   out_58762011279446305[59] = 0;
   out_58762011279446305[60] = 0;
   out_58762011279446305[61] = 0;
   out_58762011279446305[62] = 0;
   out_58762011279446305[63] = 0;
   out_58762011279446305[64] = 0;
   out_58762011279446305[65] = 0;
   out_58762011279446305[66] = dt;
   out_58762011279446305[67] = 0;
   out_58762011279446305[68] = 0;
   out_58762011279446305[69] = 0;
   out_58762011279446305[70] = 0;
   out_58762011279446305[71] = 0;
   out_58762011279446305[72] = 0;
   out_58762011279446305[73] = 0;
   out_58762011279446305[74] = 0;
   out_58762011279446305[75] = 0;
   out_58762011279446305[76] = 1;
   out_58762011279446305[77] = 0;
   out_58762011279446305[78] = 0;
   out_58762011279446305[79] = 0;
   out_58762011279446305[80] = 0;
   out_58762011279446305[81] = 0;
   out_58762011279446305[82] = 0;
   out_58762011279446305[83] = 0;
   out_58762011279446305[84] = 0;
   out_58762011279446305[85] = dt;
   out_58762011279446305[86] = 0;
   out_58762011279446305[87] = 0;
   out_58762011279446305[88] = 0;
   out_58762011279446305[89] = 0;
   out_58762011279446305[90] = 0;
   out_58762011279446305[91] = 0;
   out_58762011279446305[92] = 0;
   out_58762011279446305[93] = 0;
   out_58762011279446305[94] = 0;
   out_58762011279446305[95] = 1;
   out_58762011279446305[96] = 0;
   out_58762011279446305[97] = 0;
   out_58762011279446305[98] = 0;
   out_58762011279446305[99] = 0;
   out_58762011279446305[100] = 0;
   out_58762011279446305[101] = 0;
   out_58762011279446305[102] = 0;
   out_58762011279446305[103] = 0;
   out_58762011279446305[104] = dt;
   out_58762011279446305[105] = 0;
   out_58762011279446305[106] = 0;
   out_58762011279446305[107] = 0;
   out_58762011279446305[108] = 0;
   out_58762011279446305[109] = 0;
   out_58762011279446305[110] = 0;
   out_58762011279446305[111] = 0;
   out_58762011279446305[112] = 0;
   out_58762011279446305[113] = 0;
   out_58762011279446305[114] = 1;
   out_58762011279446305[115] = 0;
   out_58762011279446305[116] = 0;
   out_58762011279446305[117] = 0;
   out_58762011279446305[118] = 0;
   out_58762011279446305[119] = 0;
   out_58762011279446305[120] = 0;
   out_58762011279446305[121] = 0;
   out_58762011279446305[122] = 0;
   out_58762011279446305[123] = 0;
   out_58762011279446305[124] = 0;
   out_58762011279446305[125] = 0;
   out_58762011279446305[126] = 0;
   out_58762011279446305[127] = 0;
   out_58762011279446305[128] = 0;
   out_58762011279446305[129] = 0;
   out_58762011279446305[130] = 0;
   out_58762011279446305[131] = 0;
   out_58762011279446305[132] = 0;
   out_58762011279446305[133] = 1;
   out_58762011279446305[134] = 0;
   out_58762011279446305[135] = 0;
   out_58762011279446305[136] = 0;
   out_58762011279446305[137] = 0;
   out_58762011279446305[138] = 0;
   out_58762011279446305[139] = 0;
   out_58762011279446305[140] = 0;
   out_58762011279446305[141] = 0;
   out_58762011279446305[142] = 0;
   out_58762011279446305[143] = 0;
   out_58762011279446305[144] = 0;
   out_58762011279446305[145] = 0;
   out_58762011279446305[146] = 0;
   out_58762011279446305[147] = 0;
   out_58762011279446305[148] = 0;
   out_58762011279446305[149] = 0;
   out_58762011279446305[150] = 0;
   out_58762011279446305[151] = 0;
   out_58762011279446305[152] = 1;
   out_58762011279446305[153] = 0;
   out_58762011279446305[154] = 0;
   out_58762011279446305[155] = 0;
   out_58762011279446305[156] = 0;
   out_58762011279446305[157] = 0;
   out_58762011279446305[158] = 0;
   out_58762011279446305[159] = 0;
   out_58762011279446305[160] = 0;
   out_58762011279446305[161] = 0;
   out_58762011279446305[162] = 0;
   out_58762011279446305[163] = 0;
   out_58762011279446305[164] = 0;
   out_58762011279446305[165] = 0;
   out_58762011279446305[166] = 0;
   out_58762011279446305[167] = 0;
   out_58762011279446305[168] = 0;
   out_58762011279446305[169] = 0;
   out_58762011279446305[170] = 0;
   out_58762011279446305[171] = 1;
   out_58762011279446305[172] = 0;
   out_58762011279446305[173] = 0;
   out_58762011279446305[174] = 0;
   out_58762011279446305[175] = 0;
   out_58762011279446305[176] = 0;
   out_58762011279446305[177] = 0;
   out_58762011279446305[178] = 0;
   out_58762011279446305[179] = 0;
   out_58762011279446305[180] = 0;
   out_58762011279446305[181] = 0;
   out_58762011279446305[182] = 0;
   out_58762011279446305[183] = 0;
   out_58762011279446305[184] = 0;
   out_58762011279446305[185] = 0;
   out_58762011279446305[186] = 0;
   out_58762011279446305[187] = 0;
   out_58762011279446305[188] = 0;
   out_58762011279446305[189] = 0;
   out_58762011279446305[190] = 1;
   out_58762011279446305[191] = 0;
   out_58762011279446305[192] = 0;
   out_58762011279446305[193] = 0;
   out_58762011279446305[194] = 0;
   out_58762011279446305[195] = 0;
   out_58762011279446305[196] = 0;
   out_58762011279446305[197] = 0;
   out_58762011279446305[198] = 0;
   out_58762011279446305[199] = 0;
   out_58762011279446305[200] = 0;
   out_58762011279446305[201] = 0;
   out_58762011279446305[202] = 0;
   out_58762011279446305[203] = 0;
   out_58762011279446305[204] = 0;
   out_58762011279446305[205] = 0;
   out_58762011279446305[206] = 0;
   out_58762011279446305[207] = 0;
   out_58762011279446305[208] = 0;
   out_58762011279446305[209] = 1;
   out_58762011279446305[210] = 0;
   out_58762011279446305[211] = 0;
   out_58762011279446305[212] = 0;
   out_58762011279446305[213] = 0;
   out_58762011279446305[214] = 0;
   out_58762011279446305[215] = 0;
   out_58762011279446305[216] = 0;
   out_58762011279446305[217] = 0;
   out_58762011279446305[218] = 0;
   out_58762011279446305[219] = 0;
   out_58762011279446305[220] = 0;
   out_58762011279446305[221] = 0;
   out_58762011279446305[222] = 0;
   out_58762011279446305[223] = 0;
   out_58762011279446305[224] = 0;
   out_58762011279446305[225] = 0;
   out_58762011279446305[226] = 0;
   out_58762011279446305[227] = 0;
   out_58762011279446305[228] = 1;
   out_58762011279446305[229] = 0;
   out_58762011279446305[230] = 0;
   out_58762011279446305[231] = 0;
   out_58762011279446305[232] = 0;
   out_58762011279446305[233] = 0;
   out_58762011279446305[234] = 0;
   out_58762011279446305[235] = 0;
   out_58762011279446305[236] = 0;
   out_58762011279446305[237] = 0;
   out_58762011279446305[238] = 0;
   out_58762011279446305[239] = 0;
   out_58762011279446305[240] = 0;
   out_58762011279446305[241] = 0;
   out_58762011279446305[242] = 0;
   out_58762011279446305[243] = 0;
   out_58762011279446305[244] = 0;
   out_58762011279446305[245] = 0;
   out_58762011279446305[246] = 0;
   out_58762011279446305[247] = 1;
   out_58762011279446305[248] = 0;
   out_58762011279446305[249] = 0;
   out_58762011279446305[250] = 0;
   out_58762011279446305[251] = 0;
   out_58762011279446305[252] = 0;
   out_58762011279446305[253] = 0;
   out_58762011279446305[254] = 0;
   out_58762011279446305[255] = 0;
   out_58762011279446305[256] = 0;
   out_58762011279446305[257] = 0;
   out_58762011279446305[258] = 0;
   out_58762011279446305[259] = 0;
   out_58762011279446305[260] = 0;
   out_58762011279446305[261] = 0;
   out_58762011279446305[262] = 0;
   out_58762011279446305[263] = 0;
   out_58762011279446305[264] = 0;
   out_58762011279446305[265] = 0;
   out_58762011279446305[266] = 1;
   out_58762011279446305[267] = 0;
   out_58762011279446305[268] = 0;
   out_58762011279446305[269] = 0;
   out_58762011279446305[270] = 0;
   out_58762011279446305[271] = 0;
   out_58762011279446305[272] = 0;
   out_58762011279446305[273] = 0;
   out_58762011279446305[274] = 0;
   out_58762011279446305[275] = 0;
   out_58762011279446305[276] = 0;
   out_58762011279446305[277] = 0;
   out_58762011279446305[278] = 0;
   out_58762011279446305[279] = 0;
   out_58762011279446305[280] = 0;
   out_58762011279446305[281] = 0;
   out_58762011279446305[282] = 0;
   out_58762011279446305[283] = 0;
   out_58762011279446305[284] = 0;
   out_58762011279446305[285] = 1;
   out_58762011279446305[286] = 0;
   out_58762011279446305[287] = 0;
   out_58762011279446305[288] = 0;
   out_58762011279446305[289] = 0;
   out_58762011279446305[290] = 0;
   out_58762011279446305[291] = 0;
   out_58762011279446305[292] = 0;
   out_58762011279446305[293] = 0;
   out_58762011279446305[294] = 0;
   out_58762011279446305[295] = 0;
   out_58762011279446305[296] = 0;
   out_58762011279446305[297] = 0;
   out_58762011279446305[298] = 0;
   out_58762011279446305[299] = 0;
   out_58762011279446305[300] = 0;
   out_58762011279446305[301] = 0;
   out_58762011279446305[302] = 0;
   out_58762011279446305[303] = 0;
   out_58762011279446305[304] = 1;
   out_58762011279446305[305] = 0;
   out_58762011279446305[306] = 0;
   out_58762011279446305[307] = 0;
   out_58762011279446305[308] = 0;
   out_58762011279446305[309] = 0;
   out_58762011279446305[310] = 0;
   out_58762011279446305[311] = 0;
   out_58762011279446305[312] = 0;
   out_58762011279446305[313] = 0;
   out_58762011279446305[314] = 0;
   out_58762011279446305[315] = 0;
   out_58762011279446305[316] = 0;
   out_58762011279446305[317] = 0;
   out_58762011279446305[318] = 0;
   out_58762011279446305[319] = 0;
   out_58762011279446305[320] = 0;
   out_58762011279446305[321] = 0;
   out_58762011279446305[322] = 0;
   out_58762011279446305[323] = 1;
}
void h_4(double *state, double *unused, double *out_2902652897630841860) {
   out_2902652897630841860[0] = state[6] + state[9];
   out_2902652897630841860[1] = state[7] + state[10];
   out_2902652897630841860[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_7379410937342451380) {
   out_7379410937342451380[0] = 0;
   out_7379410937342451380[1] = 0;
   out_7379410937342451380[2] = 0;
   out_7379410937342451380[3] = 0;
   out_7379410937342451380[4] = 0;
   out_7379410937342451380[5] = 0;
   out_7379410937342451380[6] = 1;
   out_7379410937342451380[7] = 0;
   out_7379410937342451380[8] = 0;
   out_7379410937342451380[9] = 1;
   out_7379410937342451380[10] = 0;
   out_7379410937342451380[11] = 0;
   out_7379410937342451380[12] = 0;
   out_7379410937342451380[13] = 0;
   out_7379410937342451380[14] = 0;
   out_7379410937342451380[15] = 0;
   out_7379410937342451380[16] = 0;
   out_7379410937342451380[17] = 0;
   out_7379410937342451380[18] = 0;
   out_7379410937342451380[19] = 0;
   out_7379410937342451380[20] = 0;
   out_7379410937342451380[21] = 0;
   out_7379410937342451380[22] = 0;
   out_7379410937342451380[23] = 0;
   out_7379410937342451380[24] = 0;
   out_7379410937342451380[25] = 1;
   out_7379410937342451380[26] = 0;
   out_7379410937342451380[27] = 0;
   out_7379410937342451380[28] = 1;
   out_7379410937342451380[29] = 0;
   out_7379410937342451380[30] = 0;
   out_7379410937342451380[31] = 0;
   out_7379410937342451380[32] = 0;
   out_7379410937342451380[33] = 0;
   out_7379410937342451380[34] = 0;
   out_7379410937342451380[35] = 0;
   out_7379410937342451380[36] = 0;
   out_7379410937342451380[37] = 0;
   out_7379410937342451380[38] = 0;
   out_7379410937342451380[39] = 0;
   out_7379410937342451380[40] = 0;
   out_7379410937342451380[41] = 0;
   out_7379410937342451380[42] = 0;
   out_7379410937342451380[43] = 0;
   out_7379410937342451380[44] = 1;
   out_7379410937342451380[45] = 0;
   out_7379410937342451380[46] = 0;
   out_7379410937342451380[47] = 1;
   out_7379410937342451380[48] = 0;
   out_7379410937342451380[49] = 0;
   out_7379410937342451380[50] = 0;
   out_7379410937342451380[51] = 0;
   out_7379410937342451380[52] = 0;
   out_7379410937342451380[53] = 0;
}
void h_10(double *state, double *unused, double *out_2167868639468176046) {
   out_2167868639468176046[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2167868639468176046[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2167868639468176046[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_476627251863467283) {
   out_476627251863467283[0] = 0;
   out_476627251863467283[1] = 9.8100000000000005*cos(state[1]);
   out_476627251863467283[2] = 0;
   out_476627251863467283[3] = 0;
   out_476627251863467283[4] = -state[8];
   out_476627251863467283[5] = state[7];
   out_476627251863467283[6] = 0;
   out_476627251863467283[7] = state[5];
   out_476627251863467283[8] = -state[4];
   out_476627251863467283[9] = 0;
   out_476627251863467283[10] = 0;
   out_476627251863467283[11] = 0;
   out_476627251863467283[12] = 1;
   out_476627251863467283[13] = 0;
   out_476627251863467283[14] = 0;
   out_476627251863467283[15] = 1;
   out_476627251863467283[16] = 0;
   out_476627251863467283[17] = 0;
   out_476627251863467283[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_476627251863467283[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_476627251863467283[20] = 0;
   out_476627251863467283[21] = state[8];
   out_476627251863467283[22] = 0;
   out_476627251863467283[23] = -state[6];
   out_476627251863467283[24] = -state[5];
   out_476627251863467283[25] = 0;
   out_476627251863467283[26] = state[3];
   out_476627251863467283[27] = 0;
   out_476627251863467283[28] = 0;
   out_476627251863467283[29] = 0;
   out_476627251863467283[30] = 0;
   out_476627251863467283[31] = 1;
   out_476627251863467283[32] = 0;
   out_476627251863467283[33] = 0;
   out_476627251863467283[34] = 1;
   out_476627251863467283[35] = 0;
   out_476627251863467283[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_476627251863467283[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_476627251863467283[38] = 0;
   out_476627251863467283[39] = -state[7];
   out_476627251863467283[40] = state[6];
   out_476627251863467283[41] = 0;
   out_476627251863467283[42] = state[4];
   out_476627251863467283[43] = -state[3];
   out_476627251863467283[44] = 0;
   out_476627251863467283[45] = 0;
   out_476627251863467283[46] = 0;
   out_476627251863467283[47] = 0;
   out_476627251863467283[48] = 0;
   out_476627251863467283[49] = 0;
   out_476627251863467283[50] = 1;
   out_476627251863467283[51] = 0;
   out_476627251863467283[52] = 0;
   out_476627251863467283[53] = 1;
}
void h_13(double *state, double *unused, double *out_3170648018346895167) {
   out_3170648018346895167[0] = state[3];
   out_3170648018346895167[1] = state[4];
   out_3170648018346895167[2] = state[5];
}
void H_13(double *state, double *unused, double *out_7855059311034767435) {
   out_7855059311034767435[0] = 0;
   out_7855059311034767435[1] = 0;
   out_7855059311034767435[2] = 0;
   out_7855059311034767435[3] = 1;
   out_7855059311034767435[4] = 0;
   out_7855059311034767435[5] = 0;
   out_7855059311034767435[6] = 0;
   out_7855059311034767435[7] = 0;
   out_7855059311034767435[8] = 0;
   out_7855059311034767435[9] = 0;
   out_7855059311034767435[10] = 0;
   out_7855059311034767435[11] = 0;
   out_7855059311034767435[12] = 0;
   out_7855059311034767435[13] = 0;
   out_7855059311034767435[14] = 0;
   out_7855059311034767435[15] = 0;
   out_7855059311034767435[16] = 0;
   out_7855059311034767435[17] = 0;
   out_7855059311034767435[18] = 0;
   out_7855059311034767435[19] = 0;
   out_7855059311034767435[20] = 0;
   out_7855059311034767435[21] = 0;
   out_7855059311034767435[22] = 1;
   out_7855059311034767435[23] = 0;
   out_7855059311034767435[24] = 0;
   out_7855059311034767435[25] = 0;
   out_7855059311034767435[26] = 0;
   out_7855059311034767435[27] = 0;
   out_7855059311034767435[28] = 0;
   out_7855059311034767435[29] = 0;
   out_7855059311034767435[30] = 0;
   out_7855059311034767435[31] = 0;
   out_7855059311034767435[32] = 0;
   out_7855059311034767435[33] = 0;
   out_7855059311034767435[34] = 0;
   out_7855059311034767435[35] = 0;
   out_7855059311034767435[36] = 0;
   out_7855059311034767435[37] = 0;
   out_7855059311034767435[38] = 0;
   out_7855059311034767435[39] = 0;
   out_7855059311034767435[40] = 0;
   out_7855059311034767435[41] = 1;
   out_7855059311034767435[42] = 0;
   out_7855059311034767435[43] = 0;
   out_7855059311034767435[44] = 0;
   out_7855059311034767435[45] = 0;
   out_7855059311034767435[46] = 0;
   out_7855059311034767435[47] = 0;
   out_7855059311034767435[48] = 0;
   out_7855059311034767435[49] = 0;
   out_7855059311034767435[50] = 0;
   out_7855059311034767435[51] = 0;
   out_7855059311034767435[52] = 0;
   out_7855059311034767435[53] = 0;
}
void h_14(double *state, double *unused, double *out_5521880719452377363) {
   out_5521880719452377363[0] = state[6];
   out_5521880719452377363[1] = state[7];
   out_5521880719452377363[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7104092280027615707) {
   out_7104092280027615707[0] = 0;
   out_7104092280027615707[1] = 0;
   out_7104092280027615707[2] = 0;
   out_7104092280027615707[3] = 0;
   out_7104092280027615707[4] = 0;
   out_7104092280027615707[5] = 0;
   out_7104092280027615707[6] = 1;
   out_7104092280027615707[7] = 0;
   out_7104092280027615707[8] = 0;
   out_7104092280027615707[9] = 0;
   out_7104092280027615707[10] = 0;
   out_7104092280027615707[11] = 0;
   out_7104092280027615707[12] = 0;
   out_7104092280027615707[13] = 0;
   out_7104092280027615707[14] = 0;
   out_7104092280027615707[15] = 0;
   out_7104092280027615707[16] = 0;
   out_7104092280027615707[17] = 0;
   out_7104092280027615707[18] = 0;
   out_7104092280027615707[19] = 0;
   out_7104092280027615707[20] = 0;
   out_7104092280027615707[21] = 0;
   out_7104092280027615707[22] = 0;
   out_7104092280027615707[23] = 0;
   out_7104092280027615707[24] = 0;
   out_7104092280027615707[25] = 1;
   out_7104092280027615707[26] = 0;
   out_7104092280027615707[27] = 0;
   out_7104092280027615707[28] = 0;
   out_7104092280027615707[29] = 0;
   out_7104092280027615707[30] = 0;
   out_7104092280027615707[31] = 0;
   out_7104092280027615707[32] = 0;
   out_7104092280027615707[33] = 0;
   out_7104092280027615707[34] = 0;
   out_7104092280027615707[35] = 0;
   out_7104092280027615707[36] = 0;
   out_7104092280027615707[37] = 0;
   out_7104092280027615707[38] = 0;
   out_7104092280027615707[39] = 0;
   out_7104092280027615707[40] = 0;
   out_7104092280027615707[41] = 0;
   out_7104092280027615707[42] = 0;
   out_7104092280027615707[43] = 0;
   out_7104092280027615707[44] = 1;
   out_7104092280027615707[45] = 0;
   out_7104092280027615707[46] = 0;
   out_7104092280027615707[47] = 0;
   out_7104092280027615707[48] = 0;
   out_7104092280027615707[49] = 0;
   out_7104092280027615707[50] = 0;
   out_7104092280027615707[51] = 0;
   out_7104092280027615707[52] = 0;
   out_7104092280027615707[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_8365414901728243703) {
  err_fun(nom_x, delta_x, out_8365414901728243703);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2264373635515068274) {
  inv_err_fun(nom_x, true_x, out_2264373635515068274);
}
void pose_H_mod_fun(double *state, double *out_9199109663248748241) {
  H_mod_fun(state, out_9199109663248748241);
}
void pose_f_fun(double *state, double dt, double *out_465988394318989602) {
  f_fun(state,  dt, out_465988394318989602);
}
void pose_F_fun(double *state, double dt, double *out_58762011279446305) {
  F_fun(state,  dt, out_58762011279446305);
}
void pose_h_4(double *state, double *unused, double *out_2902652897630841860) {
  h_4(state, unused, out_2902652897630841860);
}
void pose_H_4(double *state, double *unused, double *out_7379410937342451380) {
  H_4(state, unused, out_7379410937342451380);
}
void pose_h_10(double *state, double *unused, double *out_2167868639468176046) {
  h_10(state, unused, out_2167868639468176046);
}
void pose_H_10(double *state, double *unused, double *out_476627251863467283) {
  H_10(state, unused, out_476627251863467283);
}
void pose_h_13(double *state, double *unused, double *out_3170648018346895167) {
  h_13(state, unused, out_3170648018346895167);
}
void pose_H_13(double *state, double *unused, double *out_7855059311034767435) {
  H_13(state, unused, out_7855059311034767435);
}
void pose_h_14(double *state, double *unused, double *out_5521880719452377363) {
  h_14(state, unused, out_5521880719452377363);
}
void pose_H_14(double *state, double *unused, double *out_7104092280027615707) {
  H_14(state, unused, out_7104092280027615707);
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
