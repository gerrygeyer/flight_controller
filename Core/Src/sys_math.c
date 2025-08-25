/*
 * sys_math.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Gerry Geyer
 */

#include <sys_math.h>
#include <main.h>
#include <stdbool.h>
//#include "arm_math.h"
uint32_t time;


void multiply_matrix_with_scalar(float scalar, float in_matrix[4][4], float out_matrix[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            out_matrix[i][j] = scalar * in_matrix[i][j];
        }
    }
}



void multiply_4x4_with_vector(float in_matrix[4][4], float in_vector[4][1], float out_vector[4][1]) {
    for (int i = 0; i < 4; i++) {
        out_vector[i][0] = 0;
        for (int j = 0; j < 4; j++) {
            out_vector[i][0] += in_matrix[i][j] * in_vector[j][0];
        }
    }
}

at_angl_f degree_to_rad(at_angl_f input){
	at_angl_f Output;

	Output.pitch = input.pitch * DEGREE_TO_RAD;
	Output.roll = input.roll * DEGREE_TO_RAD;
	Output.yaw = input.yaw * DEGREE_TO_RAD;

	return (Output);
}


void inverse_matrix_3x3_f(float in_matrix[3][3], float out_matrix[3][3]) {
    float det_matrix, det_positive_1, det_positive_2, det_positive_3, det_negative_1, det_negative_2, det_negative_3;
    float a_11, a_12, a_13, a_21, a_22, a_23, a_31, a_32, a_33;

    det_positive_1 = in_matrix[0][0] * in_matrix[1][1] * in_matrix[2][2];
    det_positive_2 = in_matrix[1][0] * in_matrix[2][1] * in_matrix[0][2];
    det_positive_3 = in_matrix[2][0] * in_matrix[0][1] * in_matrix[1][2];

    det_negative_1 = in_matrix[2][0] * in_matrix[1][1] * in_matrix[0][2];
    det_negative_2 = in_matrix[1][0] * in_matrix[0][1] * in_matrix[2][2];
    det_negative_3 = in_matrix[0][0] * in_matrix[2][1] * in_matrix[1][2];

    det_matrix = det_positive_1 + det_positive_2 + det_positive_3 - det_negative_1 - det_negative_2 - det_negative_3;

    if (det_matrix == 0) {
        // Die Matrix ist singulär und nicht invertierbar
        return;
    }

    a_11 = +(in_matrix[1][1] * in_matrix[2][2]) - (in_matrix[2][1] * in_matrix[1][2]);
    a_12 = -(in_matrix[0][1] * in_matrix[2][2]) + (in_matrix[2][1] * in_matrix[0][2]);
    a_13 = +(in_matrix[0][1] * in_matrix[1][2]) - (in_matrix[1][1] * in_matrix[0][2]);

    a_21 = -(in_matrix[1][0] * in_matrix[2][2]) + (in_matrix[2][0] * in_matrix[1][2]);
    a_22 = +(in_matrix[0][0] * in_matrix[2][2]) - (in_matrix[2][0] * in_matrix[0][2]);
    a_23 = -(in_matrix[0][0] * in_matrix[1][2]) + (in_matrix[1][0] * in_matrix[0][2]);

    a_31 = +(in_matrix[1][0] * in_matrix[2][1]) - (in_matrix[2][0] * in_matrix[1][1]);
    a_32 = -(in_matrix[0][0] * in_matrix[2][1]) + (in_matrix[2][0] * in_matrix[0][1]);
    a_33 = +(in_matrix[0][0] * in_matrix[1][1]) - (in_matrix[1][0] * in_matrix[0][1]);

    out_matrix[0][0] = (a_11 / det_matrix);
    out_matrix[0][1] = (a_12 / det_matrix);
    out_matrix[0][2] = (a_13 / det_matrix);

    out_matrix[1][0] = (a_21 / det_matrix);
    out_matrix[1][1] = (a_22 / det_matrix);
    out_matrix[1][2] = (a_23 / det_matrix);

    out_matrix[2][0] = (a_31 / det_matrix);
    out_matrix[2][1] = (a_32 / det_matrix);
    out_matrix[2][2] = (a_33 / det_matrix);
}

void inverse_matrix_4x4_f(float in_matrix[4][4], float out_matrix[4][4]) {
    float det_matrix;
    float minors[4][4], cofactors[4][4], adjugate[4][4];

    // Berechne die Determinante der Matrix
    det_matrix = 0.0f;
    for (int col = 0; col < 4; col++) {
        float sub_matrix[3][3];
        // Submatrix extrahieren
        for (int i = 1; i < 4; i++) {
            int sub_col = 0;
            for (int j = 0; j < 4; j++) {
                if (j == col) continue;
                sub_matrix[i - 1][sub_col++] = in_matrix[i][j];
            }
        }
        // Berechne den Minor (Determinante der Submatrix)
        float det_minor =
            sub_matrix[0][0] * (sub_matrix[1][1] * sub_matrix[2][2] - sub_matrix[2][1] * sub_matrix[1][2]) -
            sub_matrix[0][1] * (sub_matrix[1][0] * sub_matrix[2][2] - sub_matrix[2][0] * sub_matrix[1][2]) +
            sub_matrix[0][2] * (sub_matrix[1][0] * sub_matrix[2][1] - sub_matrix[2][0] * sub_matrix[1][1]);

        // Addiere/ Subtrahiere den Beitrag zur Determinante
        det_matrix += ((col % 2 == 0 ? 1 : -1) * in_matrix[0][col] * det_minor);
    }

    if (det_matrix == 0.0f) {
        // Die Matrix ist singulär und nicht invertierbar
        return;
    }

    // Berechne die Minoren, Cofaktoren und die Adjunkte
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            float sub_matrix[3][3];
            int sub_row = 0;
            for (int i = 0; i < 4; i++) {
                if (i == row) continue;
                int sub_col = 0;
                for (int j = 0; j < 4; j++) {
                    if (j == col) continue;
                    sub_matrix[sub_row][sub_col++] = in_matrix[i][j];
                }
                sub_row++;
            }

            // Berechne den Minor (Determinante der Submatrix)
            minors[row][col] =
                sub_matrix[0][0] * (sub_matrix[1][1] * sub_matrix[2][2] - sub_matrix[2][1] * sub_matrix[1][2]) -
                sub_matrix[0][1] * (sub_matrix[1][0] * sub_matrix[2][2] - sub_matrix[2][0] * sub_matrix[1][2]) +
                sub_matrix[0][2] * (sub_matrix[1][0] * sub_matrix[2][1] - sub_matrix[2][0] * sub_matrix[1][1]);

            // Cofaktor mit alternierenden Vorzeichen
            cofactors[row][col] = ((row + col) % 2 == 0 ? 1 : -1) * minors[row][col];
        }
    }

    // Transponiere die Cofaktormatrix (Adjunkte)
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            adjugate[col][row] = cofactors[row][col];
        }
    }

    // Skaliere mit der Determinante, um die Inverse zu erhalten
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            out_matrix[row][col] = adjugate[row][col] / det_matrix;
        }
    }
}


// #############





int16_t rpm2rad_sQ15_scaled(int16_t rpm){
	int32_t Output;
	Output = (rpm * RPM_TO_RAD_Q15)/MAX_SPEED_MOTOR_RAD; // note: can also pre-calculate RPM_TO_RAD_Q15/MAX_SPEED_MOTOR_RAD

	return CLAMP_INT32_TO_INT16(Output);
}


/**
 * @brief       Sine lookup table for the interval [0, π/2] in Q15 format.
 *
 * @details     This table contains 1024 precomputed values of the sine function
 *              between 0 and π/2, scaled to Q15 format (0 … 32767).
 *              It is used to accelerate sine evaluations by avoiding runtime trigonometric computations,
 *              especially on embedded systems without floating-point support.
 *
 * @note
 * 				- Resolution: 1024 steps over [0, π/2] → step size ≈ 0.001534 rad
 * 				- Range: \f$ \text{sin}(x) \cdot 32767 \f$ for \f$ x \in [0, \frac{\pi}{2}] \f$
 * 				- Symmetry can be used to compute values over [0, 2π] using mirror rules.
 *
 * @see         Q15 fixed-point format, cosine symmetry: \f$ \cos(x) = \sin(x + \frac{\pi}{2}) \f$
 */
extern const int16_t sineLookupTable[1024];
const int16_t sineLookupTable[] = {
		0,50,101,151,201,252,302,352,402,453,503,553,604,654,704,755,805,855,906,956,1006,
		1056,1107,1157,1207,1258,1308,1358,1408,1459,1509,1559,1609,1660,1710,1760,1810,1861,
		1911,1961,2011,2061,2112,2162,2212,2262,2312,2363,2413,2463,2513,2563,2614,2664,2714,
		2764,2814,2864,2914,2964,3015,3065,3115,3165,3215,3265,3315,3365,3415,3465,3515,3565,
		3615,3665,3715,3765,3815,3865,3915,3965,4015,4065,4115,4165,4215,4264,4314,4364,4414,
		4464,4514,4564,4613,4663,4713,4763,4813,4862,4912,4962,5012,5061,5111,5161,5210,5260,
		5310,5359,5409,5459,5508,5558,5607,5657,5706,5756,5806,5855,5905,5954,6003,6053,6102,
		6152,6201,6251,6300,6349,6399,6448,6497,6547,6596,6645,6694,6744,6793,6842,6891,6941,
		6990,7039,7088,7137,7186,7235,7284,7333,7382,7431,7480,7529,7578,7627,7676,7725,7774,
		7823,7872,7921,7969,8018,8067,8116,8164,8213,8262,8311,8359,8408,8456,8505,8554,8602,
		8651,8699,8748,8796,8845,8893,8941,8990,9038,9087,9135,9183,9232,9280,9328,9376,9424,
		9473,9521,9569,9617,9665,9713,9761,9809,9857,9905,9953,10001,10049,10097,10145,10193,
		10240,10288,10336,10384,10431,10479,10527,10574,10622,10669,10717,10765,10812,10860,
		10907,10954,11002,11049,11097,11144,11191,11238,11286,11333,11380,11427,11474,11522,
		11569,11616,11663,11710,11757,11804,11851,11897,11944,11991,12038,12085,12132,12178,
		12225,12272,12318,12365,12411,12458,12505,12551,12597,12644,12690,12737,12783,12829,
		12876,12922,12968,13014,13060,13107,13153,13199,13245,13291,13337,13383,13429,13474,
		13520,13566,13612,13658,13703,13749,13795,13840,13886,13931,13977,14022,14068,14113,
		14159,14204,14249,14295,14340,14385,14430,14476,14521,14566,14611,14656,14701,14746,
		14791,14836,14880,14925,14970,15015,15059,15104,15149,15193,15238,15282,15327,15371,
		15416,15460,15504,15549,15593,15637,15681,15726,15770,15814,15858,15902,15946,15990,
		16034,16078,16121,16165,16209,16253,16296,16340,16383,16427,16471,16514,16557,16601,
		16644,16688,16731,16774,16817,16860,16904,16947,16990,17033,17076,17119,17161,17204,
		17247,17290,17333,17375,17418,17460,17503,17546,17588,17630,17673,17715,17757,17800,
		17842,17884,17926,17968,18010,18052,18094,18136,18178,18220,18262,18304,18345,18387,
		18429,18470,18512,18553,18595,18636,18677,18719,18760,18801,18842,18884,18925,18966,
		19007,19048,19089,19129,19170,19211,19252,19293,19333,19374,19414,19455,19495,19536,
		19576,19616,19657,19697,19737,19777,19817,19857,19897,19937,19977,20017,20057,20097,
		20136,20176,20216,20255,20295,20334,20374,20413,20452,20492,20531,20570,20609,20648,
		20687,20726,20765,20804,20843,20882,20921,20959,20998,21037,21075,21114,21152,21190,
		21229,21267,21305,21344,21382,21420,21458,21496,21534,21572,21610,21647,21685,21723,
		21760,21798,21836,21873,21910,21948,21985,22022,22060,22097,22134,22171,22208,22245,
		22282,22319,22356,22392,22429,22466,22502,22539,22575,22612,22648,22685,22721,22757,
		22793,22829,22865,22901,22937,22973,23009,23045,23081,23116,23152,23188,23223,23259,
		23294,23329,23365,23400,23435,23470,23505,23540,23575,23610,23645,23680,23715,23749,
		23784,23819,23853,23887,23922,23956,23991,24025,24059,24093,24127,24161,24195,24229,
		24263,24297,24330,24364,24398,24431,24465,24498,24532,24565,24598,24631,24665,24698,
		24731,24764,24797,24829,24862,24895,24928,24960,24993,25025,25058,25090,25123,25155,
		25187,25219,25251,25283,25315,25347,25379,25411,25443,25474,25506,25537,25569,25600,
		25632,25663,25694,25725,25757,25788,25819,25850,25881,25911,25942,25973,26003,26034,
		26065,26095,26125,26156,26186,26216,26246,26276,26307,26336,26366,26396,26426,26456,
		26485,26515,26545,26574,26603,26633,26662,26691,26720,26749,26778,26807,26836,26865,
		26894,26923,26951,26980,27008,27037,27065,27094,27122,27150,27178,27206,27234,27262,
		27290,27318,27346,27373,27401,27429,27456,27483,27511,27538,27565,27593,27620,27647,
		27674,27701,27727,27754,27781,27808,27834,27861,27887,27913,27940,27966,27992,28018,
		28044,28070,28096,28122,28148,28174,28199,28225,28250,28276,28301,28327,28352,28377,
		28402,28427,28452,28477,28502,28527,28552,28576,28601,28625,28650,28674,28698,28723,
		28747,28771,28795,28819,28843,28867,28890,28914,28938,28961,28985,29008,29032,29055,
		29078,29101,29124,29147,29170,29193,29216,29239,29262,29284,29307,29329,29352,29374,
		29396,29418,29440,29463,29485,29506,29528,29550,29572,29593,29615,29636,29658,29679,
		29701,29722,29743,29764,29785,29806,29827,29848,29868,29889,29910,29930,29950,29971,
		29991,30011,30032,30052,30072,30092,30111,30131,30151,30171,30190,30210,30229,30249,
		30268,30287,30306,30325,30344,30363,30382,30401,30420,30438,30457,30476,30494,30512,
		30531,30549,30567,30585,30603,30621,30639,30657,30675,30692,30710,30727,30745,30762,
		30779,30797,30814,30831,30848,30865,30882,30898,30915,30932,30948,30965,30981,30998,
		31014,31030,31046,31062,31078,31094,31110,31126,31141,31157,31173,31188,31203,31219,
		31234,31249,31264,31279,31294,31309,31324,31339,31353,31368,31382,31397,31411,31425,
		31440,31454,31468,31482,31496,31510,31523,31537,31551,31564,31578,31591,31604,31618,
		31631,31644,31657,31670,31683,31696,31708,31721,31734,31746,31758,31771,31783,31795,
		31807,31819,31831,31843,31855,31867,31879,31890,31902,31913,31925,31936,31947,31958,
		31969,31980,31991,32002,32013,32024,32034,32045,32055,32066,32076,32086,32096,32106,
		32116,32126,32136,32146,32156,32165,32175,32184,32194,32203,32212,32222,32231,32240,
		32249,32257,32266,32275,32284,32292,32301,32309,32317,32326,32334,32342,32350,32358,
		32366,32374,32381,32389,32397,32404,32412,32419,32426,32433,32441,32448,32455,32462,
		32468,32475,32482,32488,32495,32501,32508,32514,32520,32526,32532,32538,32544,32550,
		32556,32561,32567,32572,32578,32583,32589,32594,32599,32604,32609,32614,32619,32623,
		32628,32633,32637,32642,32646,32650,32654,32659,32663,32667,32670,32674,32678,32682,
		32685,32689,32692,32696,32699,32702,32705,32708,32711,32714,32717,32720,32722,32725,
		32727,32730,32732,32735,32737,32739,32741,32743,32745,32747,32748,32750,32752,32753,
		32754,32756,32757,32758,32759,32760,32761,32762,32763,32764,32765,32765,32766,32766,
		32766,32767,32767,32767,32767
};


int16_t sin_i(int16_t y){
	int16_t Output;

	int32_t x_i = ((int32_t)(y * 2048)/ INT16_MAX_VALUE);

	if(x_i < 0){
		x_i = x_i * (-1);
		if(x_i > 1024){
			Output = -sineLookupTable[(2048 - x_i)];
		}else{
			Output = -sineLookupTable[x_i];
		}
	}else{
		if(x_i > 1024){
			Output = sineLookupTable[(2048 - x_i)];
		}else{
			Output = sineLookupTable[x_i];
		}
	}
	return Output;
}

int16_t cos_i(int16_t y){
	return sin_i(y + INT16_HALF_VALUE);
}

/*
 * kein beispiel bitte
 *% Anzahl der LUT-Einträge
N = 256;

% Skalierung: asin ∈ [0, pi/2] → [0, 32767]
SCALE_ASIN = 32767 / (pi/2);  % ≈ 20860.75

% Eingangsbereich: x ∈ [0, 1]
x = linspace(0, 1, N);
y = asin(x);  % rad

% Ausgabe skalieren auf int16_t Bereich
y_q15 = round(y * SCALE_ASIN);  % jetzt ∈ [0, 32767]

% Ausgabe als C-Array
fprintf('const int16_t asin_q15_lut[%d] = {\n', N);
for i = 1:N
    fprintf('%6d', y_q15(i));
    if i < N
        fprintf(', ');
    end
    if mod(i, 8) == 0
        fprintf('\n');
    end
end
fprintf('\n};\n');
 */

/**
 * @brief       Lookup table for arcsine function (asin) in Q15 format.
 *
 * @details     Precomputed values of \f$ \arcsin(x) \f$ for \f$ x \in [0, 1] \f$ in 256 steps,
 *              scaled to Q15 format (−32768 ≙ −π, +32767 ≙ +π).
 *              The table allows fast fixed-point evaluation of asin without using floating-point math.
 *
 * @note
 * 			- Input domain: \f$ x \in [0, 32767] \f$ (Q15 positive half)
 * 			- Output: \f$ \text{asin}(x) \cdot \frac{32767}{\pi} \f$ in Q15
 * 			- For negative inputs, use symmetry: \f$ \arcsin(-x) = -\arcsin(x) \f$
 * 			- Interpolation (optional) improves precision between steps
 *
 * @see         q15_asin(), Q15 fixed-point format
 */
const int16_t asin_q15_lut[256] = {
     0,     82,    164,    245,    327,    409,    491,    573,
   655,    736,    818,    900,    982,   1064,   1146,   1228,
  1310,   1392,   1474,   1556,   1638,   1720,   1802,   1884,
  1966,   2048,   2131,   2213,   2295,   2377,   2460,   2542,
  2625,   2707,   2790,   2872,   2955,   3037,   3120,   3203,
  3286,   3369,   3452,   3534,   3617,   3701,   3784,   3867,
  3950,   4034,   4117,   4200,   4284,   4367,   4451,   4535,
  4619,   4703,   4787,   4871,   4955,   5039,   5123,   5208,
  5292,   5377,   5461,   5546,   5631,   5716,   5801,   5886,
  5971,   6056,   6142,   6227,   6313,   6399,   6485,   6571,
  6657,   6743,   6829,   6916,   7002,   7089,   7176,   7263,
  7350,   7437,   7525,   7612,   7700,   7787,   7875,   7963,
  8052,   8140,   8229,   8317,   8406,   8495,   8584,   8674,
  8763,   8853,   8943,   9033,   9123,   9213,   9304,   9395,
  9486,   9577,   9668,   9760,   9851,   9943,  10036,  10128,
 10221,  10313,  10406,  10500,  10593,  10687,  10781,  10875,
 10970,  11064,  11159,  11254,  11350,  11446,  11542,  11638,
 11735,  11831,  11929,  12026,  12124,  12222,  12320,  12419,
 12518,  12617,  12717,  12817,  12917,  13017,  13118,  13220,
 13321,  13424,  13526,  13629,  13732,  13836,  13940,  14044,
 14149,  14254,  14360,  14466,  14573,  14680,  14787,  14895,
 15004,  15113,  15222,  15332,  15443,  15554,  15665,  15778,
 15890,  16004,  16118,  16232,  16347,  16463,  16580,  16697,
 16815,  16933,  17052,  17172,  17293,  17414,  17537,  17660,
 17784,  17908,  18034,  18161,  18288,  18416,  18546,  18676,
 18807,  18940,  19073,  19208,  19343,  19480,  19619,  19758,
 19899,  20041,  20184,  20329,  20476,  20624,  20773,  20925,
 21078,  21232,  21389,  21548,  21709,  21871,  22037,  22204,
 22374,  22546,  22722,  22900,  23081,  23265,  23453,  23644,
 23839,  24038,  24242,  24450,  24663,  24882,  25107,  25338,
 25576,  25823,  26077,  26342,  26618,  26906,  27208,  27528,
 27868,  28233,  28629,  29067,  29564,  30153,  30919,  32767

};


// x ∈ [-32768, +32767]  ⇒ Bereich [-1, +1]
int16_t q15_asin(int16_t x) {
	bool sign = 0;
    if (x < 0) {
        x = -x;
        sign = 1;
    }

    uint16_t z = ((uint16_t)x) << 1; // upscalint to uint16_t
    uint16_t index = z >> 8;  // divide by 256
    uint16_t next = (index < 255) ? index + 1 : 255;

    // Lineare Interpolation
    int16_t y0 = asin_q15_lut[index];
    int16_t y1 = asin_q15_lut[next];

    uint16_t x0 = index << 7;              // x0 = index * 128
    int16_t dx = x - x0;
    int16_t dy = y1 - y0;

    int32_t interp = y0 + ((int32_t)dy * dx >> 7);  // / 128

    return sign ? -interp : interp;
}

/**
 * @brief  Lookup table für acos(1 - t) in Q15-Winkelskalierung (Q15/π), t ∈ [0,1], 256 Schritte.
 *         Start: t=0 ⇒ acos(1)=0; Ende: t=1 ⇒ acos(0)=π/2 (→ 16384).
 */
static const int16_t acos_q15_lut_t[256] = {
     0,    924,   1307,   1602,   1850,   2069,   2267,   2450,
  2620,   2779,   2931,   3075,   3213,   3345,   3472,   3595,
  3715,   3830,   3942,   4052,   4159,   4263,   4364,   4464,
  4562,   4657,   4751,   4843,   4934,   5023,   5110,   5197,
  5282,   5365,   5448,   5529,   5610,   5689,   5767,   5845,
  5921,   5997,   6072,   6146,   6219,   6292,   6363,   6434,
  6505,   6574,   6643,   6712,   6780,   6847,   6914,   6980,
  7046,   7111,   7176,   7240,   7303,   7367,   7430,   7492,
  7554,   7615,   7676,   7737,   7798,   7858,   7917,   7976,
  8035,   8094,   8152,   8210,   8268,   8325,   8382,   8439,
  8495,   8551,   8607,   8662,   8718,   8773,   8827,   8882,
  8936,   8990,   9044,   9097,   9151,   9204,   9257,   9309,
  9362,   9414,   9466,   9518,   9569,   9621,   9672,   9723,
  9774,   9825,   9875,   9925,   9976,  10026,  10075,  10125,
 10174,  10224,  10273,  10322,  10371,  10420,  10468,  10517,
 10565,  10613,  10661,  10709,  10757,  10804,  10852,  10899,
 10946,  10993,  11040,  11087,  11134,  11181,  11227,  11274,
 11320,  11366,  11412,  11458,  11504,  11550,  11595,  11641,
 11687,  11732,  11777,  11822,  11868,  11913,  11957,  12002,
 12047,  12092,  12136,  12181,  12225,  12270,  12314,  12358,
 12402,  12446,  12490,  12534,  12578,  12622,  12665,  12709,
 12752,  12796,  12839,  12883,  12926,  12969,  13012,  13056,
 13099,  13142,  13184,  13227,  13270,  13313,  13356,  13398,
 13441,  13484,  13526,  13568,  13611,  13653,  13696,  13738,
 13780,  13822,  13864,  13907,  13949,  13991,  14033,  14075,
 14116,  14158,  14200,  14242,  14284,  14325,  14367,  14409,
 14450,  14492,  14534,  14575,  14617,  14658,  14700,  14741,
 14782,  14824,  14865,  14907,  14948,  14989,  15030,  15072,
 15113,  15154,  15195,  15236,  15278,  15319,  15360,  15401,
 15442,  15483,  15524,  15565,  15606,  15647,  15688,  15729,
 15770,  15811,  15852,  15893,  15934,  15975,  16016,  16057,
 16098,  16139,  16179,  16220,  16261,  16302,  16343,  16384
};

/**
 * @brief  Präziser acos in Q15 mittels t-LUT (t = 1 - |x|), linear interpoliert.
 * @param  x  Q15 in [-32768, 32767] ⇒ real [-1, 1]
 * @return Winkel in Q15 (−π..+π Skala, Wertebereich [0..π])
 * @note   Symmetrie: acos(−x) = π − acos(x). Für x≥0: direkt aus LUT.
 */
int16_t q15_acos(int16_t x)
{
    // Betrag und Vorzeichen (Symmetrie später)
    uint16_t ux = (x < 0) ? (uint16_t)(-x) : (uint16_t)x;

    // t = 1 - |x|  (Q15)
    uint16_t t_q15 = (uint16_t)Q15_ONE - ux;   // 0..32767

    // Index/Fraktion für 256er LUT auf t∈[0,1]:
    // i = floor(t*255), frac in 0..127 (Q7)
    uint16_t i     = t_q15 >> 7;               // /128  ⇒ 0..255
    if (i > 255) i = 255;
    uint16_t frac  = t_q15 & 0x7F;             // Rest für lin. Interp (Q7)

    int16_t y0 = acos_q15_lut_t[i];
    int16_t y1 = (i < 255) ? acos_q15_lut_t[i+1] : y0;
    int16_t dy = (int16_t)(y1 - y0);

    // Linear: y = y0 + dy*(frac/128)
    int32_t y  = (int32_t)y0 + (((int32_t)dy * (int32_t)frac) >> 7);

    // Symmetrie für negatives x: acos(-|x|) = π - acos(|x|)
    if (x < 0) {
        y = (int32_t)Q15 - y;
        if (y < 0) y = 0;           // numerische Sicherheit
    }

    // Sättigen in int16
    if (y > 32767)  y = 32767;
    if (y < -32768) y = -32768;
    return (int16_t)y;
}
//int16_t q15_acos(int16_t x){
//
//    return PI_OVER_2_Q15 - q15_asin(x);
//}

/**
 * @brief       Lookup table for arctangent approximation (atan2-style) in Q15 format.
 *
 * @details     This table contains 256 precomputed values of the arctangent function
 *              for the ratio \f$ y/x \in [0, 1] \f$, mapped to Q15 format representing radians.
 *              Used for fast fixed-point implementation of `atan2(y, x)` via symmetry and scaling.
 *
 * @note
 * 				- Input domain: \f$ \frac{y}{x} \in [0, 1] \f$
 * 				- Output: \f$ \arctan\left(\frac{y}{x}\right) \cdot \frac{32767}{\pi} \f$ in Q15
 * 				- Covers only the first octant (0°…45°) → full circle can be reconstructed using quadrant logic
 * 				- Resolution: 256 steps → step size ≈ 1/256
 * @see         q15_atan2(), Q15 fixed-point format, trig symmetry
 */
const int16_t atan2_q15_lut[256] = {
      0,     41,     82,    123,    164,    204,    245,    286,
    327,    368,    409,    450,    490,    531,    572,    613,
    654,    694,    735,    776,    816,    857,    898,    938,
    979,   1019,   1060,   1100,   1141,   1181,   1221,   1262,
   1302,   1342,   1383,   1423,   1463,   1503,   1543,   1583,
   1623,   1663,   1703,   1742,   1782,   1822,   1861,   1901,
   1941,   1980,   2019,   2059,   2098,   2137,   2177,   2216,
   2255,   2294,   2333,   2371,   2410,   2449,   2488,   2526,
   2565,   2603,   2642,   2680,   2718,   2756,   2794,   2832,
   2870,   2908,   2946,   2984,   3021,   3059,   3096,   3133,
   3171,   3208,   3245,   3282,   3319,   3356,   3393,   3429,
   3466,   3502,   3539,   3575,   3611,   3648,   3684,   3720,
   3755,   3791,   3827,   3862,   3898,   3933,   3969,   4004,
   4039,   4074,   4109,   4144,   4179,   4213,   4248,   4282,
   4316,   4351,   4385,   4419,   4453,   4487,   4520,   4554,
   4588,   4621,   4654,   4688,   4721,   4754,   4787,   4819,
   4852,   4885,   4917,   4950,   4982,   5014,   5046,   5078,
   5110,   5142,   5174,   5205,   5237,   5268,   5299,   5331,
   5362,   5393,   5424,   5454,   5485,   5515,   5546,   5576,
   5606,   5637,   5667,   5697,   5726,   5756,   5786,   5815,
   5845,   5874,   5903,   5932,   5961,   5990,   6019,   6047,
   6076,   6105,   6133,   6161,   6189,   6217,   6245,   6273,
   6301,   6329,   6356,   6384,   6411,   6438,   6465,   6492,
   6519,   6546,   6573,   6600,   6626,   6653,   6679,   6705,
   6731,   6757,   6783,   6809,   6835,   6861,   6886,   6912,
   6937,   6962,   6988,   7013,   7038,   7062,   7087,   7112,
   7137,   7161,   7186,   7210,   7234,   7258,   7282,   7306,
   7330,   7354,   7378,   7401,   7425,   7448,   7471,   7495,
   7518,   7541,   7564,   7587,   7609,   7632,   7655,   7677,
   7700,   7722,   7744,   7766,   7788,   7810,   7832,   7854,
   7876,   7897,   7919,   7940,   7962,   7983,   8004,   8026,
   8047,   8068,   8088,   8109,   8130,   8151,   8171,   8192

};

int16_t q15_atan2(int16_t y, int16_t x) {
    if (x == 0 && y == 0)
        return 0;  // undefined, fallback auf 0

    int abs_y = abs(y);
    int abs_x = abs(x);

    // Verhältnis r = min/ max in Q15 → z ∈ [0, 32767]
    uint16_t ratio;
    int y_greater = abs_y > abs_x;
    if (y_greater) {
        ratio = ((uint32_t)abs_x << 15) / abs_y;
    } else {
        ratio = ((uint32_t)abs_y << 15) / abs_x;
    }

    // Skalieren auf 256 LUT-Einträge (index ∈ [0, 255])
    uint16_t z = ratio << 1;
    uint16_t index = z >> 8;
    uint16_t next = (index < 255) ? index + 1 : 255;

    // Interpolieren
    int16_t theta0 = atan2_q15_lut[index];
    int16_t theta1 = atan2_q15_lut[next];
    uint16_t x0 = index << 7;  // *128
    int16_t dx = (ratio - x0);
    int16_t dtheta = theta1 - theta0;
    int32_t theta = theta0 + ((int32_t)dtheta * dx >> 7);  // /128

    // Basiswinkel theta ∈ [0, π/4]

    // Korrektur je nach Quadrant
    if (x >= 0 && y >= 0) {
        return y_greater ? (16384 - theta) : theta;
    } else if (x < 0 && y >= 0) {
        return y_greater ? (16384 + theta) : (32767 - theta);
    } else if (x < 0 && y < 0) {
        return y_greater ? -(16384 + theta) : -(32767 - theta);
    } else { // x >= 0 && y < 0
        return y_greater ? -(16384 - theta) : -theta;
    }
}

int check_overflow(int32_t a, int32_t b) {
    // Prüfen auf Überlauf
    if (a > 0 && b > 0 && a > INT32_MAX / b) {
        return NOT_OK; // Überlauf
    }
    if (a > 0 && b < 0 && b < INT32_MIN / a) {
        return NOT_OK; // Überlauf
    }
    if (a < 0 && b > 0 && a < INT32_MIN / b) {
        return NOT_OK; // Überlauf
    }
    if (a < 0 && b < 0 && a < INT32_MAX / b) {
        return NOT_OK; // Überlauf
    }

    return OK;
}

void multiply_Matrix3x3(int32_t mat1[3][3],int32_t mat2[3][3], int32_t result[3][3]){
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++) {
            	if(check_overflow(mat1[i][k],mat2[k][j]) == OK){
            		result[i][j] += mat1[i][k] * mat2[k][j];
            	}else{
            		result[i][j] += INT32_MAX;
            	}
            }
        }
    }
}

/*
 *  example_matrix[4][5] : 4 Zeilen und 5 Spalten
 */


/**
 * @brief       Computes the cross product of two 3D vectors in Q15 fixed-point format.
 *
 * @details     This function calculates the cross product of two input vectors @p v1 and @p v2,
 *              both represented in Q15 fixed-point format. Intermediate results are calculated
 *              using 32-bit integers to prevent overflow and then scaled back to Q15 with rounding.
 *
 * @param[in]   v1     Pointer to the first vector (array of 3 Q15 values).
 * @param[in]   v2     Pointer to the second vector (array of 3 Q15 values).
 * @param[out]  cross  Pointer to the output vector (array of 3 Q15 values).
 *
 * @note        The function uses 32-bit intermediate arithmetic and rounding
 *              with `(1 << 13)` before shifting back to Q15.
 * @warning     Input values must be within the valid Q15 range ([-1, 1) scaled to int16_t).
 *
 * @see         dotproduct_3x3_Q15()
 */
void crossproduct_3x3_Q15(const int16_t *v1, const int16_t *v2, int16_t *cross){
	int32_t x;

	x = (((int32_t)v1[1] * (int32_t)v2[2]) >> 1) - (((int32_t)v1[2] * (int32_t)v2[1]) >> 1);
	x = ((x + ( 1 << 13 )) >> 14); // back to Q15
	cross[0] = CLAMP_INT32_TO_INT16(x);

	x = (((int32_t)v1[2] * (int32_t)v2[0]) >> 1) - (((int32_t)v1[0] * (int32_t)v2[2]) >> 1);
	x = ((x + ( 1 << 13 )) >> 14); // back to Q15
	cross[1] = CLAMP_INT32_TO_INT16(x);

	x = (((int32_t)v1[0] * (int32_t)v2[1]) >> 1) - (((int32_t)v1[1] * (int32_t)v2[0]) >> 1);
	x = ((x + ( 1 << 13 )) >> 14); // back to Q15
	cross[2] = CLAMP_INT32_TO_INT16(x);

}

void dotporduct_3x3_Q15(const int16_t *v1, const int16_t *v2, int16_t *dot){
	int32_t x;
	x = ((int32_t)v1[0] * (int32_t)v2[0]) >> 1; // Q29
	x += ((int32_t)v1[1] * (int32_t)v2[1]) >> 1; // Q29
	x += ((int32_t)v1[2] * (int32_t)v2[2]) >> 1; // Q29

	x = ((x + (1 << 13)) >> 14); // Q29 - Q14 = Q15
	x = CLAMP_INT32_TO_INT16(x);
	dot = x;
}


// function to start, stop the time the control need

void start_time_measurement(void){
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_CEN;
}
// return the operation time in us (uint32_t)
uint32_t stopp_time_measurement(void){

	uint32_t Output;

	TIM2->CR1 &= ~TIM_CR1_CEN;
	uint32_t ticks = TIM2->CNT;
	TIM2->CNT = 0;

	Output = ticks/240;
	time = Output;
	return (Output);

}


uint32_t sqrt_fast_uint(uint32_t n) {
    if (n == 0) return 0;

    uint8_t b = 31 - __builtin_clz(n);
    uint32_t x = 1 << (b >> 1); // Initial guess: 2^(b/2)

    x = (x + n / x) >> 1;
    x = (x + n / x) >> 1;
    x = (x + n / x) >> 1;

    if ((uint64_t)x * x > n) x--;

    return x;
}

/**
 * @brief       Multiplies two Q15 fixed-point numbers with rounding.
 *
 * @details     Performs a 16-bit × 16-bit multiplication with intermediate 32-bit precision,
 *              adds 0.5 (1 << 14) for rounding, and shifts the result back to Q15 format.
 *
 * @param       a       First Q15 operand.
 * @param       b       Second Q15 operand.
 *
 * @return      Rounded Q15 result of the multiplication.
 *
 * @note        This function is `static inline` for performance and can be used in tight control loops.
 *              Result is clamped implicitly by casting to int16_t.
 */
static inline int16_t q15_mul(int16_t a, int16_t b) {
    return (int16_t)(((int32_t)a * b + (1 << 14)) >> 15); // mit Rundung
}

/**
 * @brief       Multiplies two Q15 values and doubles the result, with clamping.
 *
 * @details     Computes \f$ 2 \cdot (a \cdot b) \f$ where both inputs are in Q15 fixed-point format.
 *              The intermediate result is in Q30, scaled back to Q15 with rounding and clamped to int16_t range.
 *
 * @param       a   First Q15 operand (int16_t).
 * @param       b   Second Q15 operand (int16_t).
 *
 * @return      Result of \f$ 2 \cdot a \cdot b \f$ in Q15 format, clamped to \f$ [-32768, 32767] \f$.
 *
 * @note        Rounding is applied before shifting: adds \f$ 2^{13} \f$ before right-shift by 14.
 */
static inline int16_t q15_mul_2(int16_t a, int16_t b) {
	int32_t x = (((int32_t)a * b + (1 << 13)) >> 14);
    return CLAMP_INT32_TO_INT16(x);
}

void norm_2d_vector_q15(int16_t *v){
	int32_t x = v[0];
	int32_t y = v[1];

	uint32_t mag = (uint32_t)(x*x) + (uint32_t)(y*y);
	if (mag < 5){		// its too small
		v[0] = 0;
		v[1] = 0;
		return;
	}
	if (mag == 1) return; // be happy, we done

	mag = sqrt_fast_uint(mag);

	uint32_t scale_q15 = (32767UL << 15) / mag;
	// 3. norm[i] = (accel[i] * scale_q15) >> 15
	v[0] = CLAMP_INT32_TO_INT16((((int32_t)x * scale_q15) >> 15));
	v[1] = CLAMP_INT32_TO_INT16((((int32_t)y * scale_q15) >> 15));
}

void norm_3d_vector(int16_t *input, int16_t *norm_out){
	int32_t x = input[0];
	int32_t y = input[1];
	int32_t z = input[2];


	uint32_t mag = (uint32_t)(x*x) + (uint32_t)(y*y) + (uint32_t)(z*z);

	mag = sqrt_fast_uint(mag);

	if(mag == 0){
		norm_out[0] = 0;
		norm_out[1] = 0;
		norm_out[2] = 0;
		return;
	}

    // 2. Skalenfaktor vorbereiten in Q15: scale = 32767 / |v|
    // Wir rechnen: scale = (32767 << 15) / mag
    uint32_t scale_q15 = (32767UL << 15) / mag;

    // 3. norm[i] = (accel[i] * scale_q15) >> 15
    norm_out[0] = (int16_t)(((int64_t)x * scale_q15) >> 15);
    norm_out[1] = (int16_t)(((int64_t)y * scale_q15) >> 15);
    norm_out[2] = (int16_t)(((int64_t)z * scale_q15) >> 15);

}


// ########### QUATERNION MATH ############

/**
 * @brief       Multiplies two Q15 values and right-shifts the result by 1.
 *
 * @details     Performs a 16-bit × 16-bit multiplication with 32-bit intermediate result.
 *              The result is not scaled back to Q15 (i.e., no >>15 shift), but only shifted
 *              by 1 bit, typically used for special cases like symmetric expressions
 *              or energy/power terms.
 *
 * @param       a       First operand in Q15 format (int16_t).
 * @param       b       Second operand in Q15 format (int16_t).
 *
 * @return      31-bit result (int32_t), effectively: \f$ \frac{a \cdot b}{2} \f$
 *
 * @note        No rounding is applied. Use when half-scale product is intended.
 */
#define Q15_MUL_HALF(a, b) (((int32_t)(a) * (int32_t)(b)) >> 1)

void NormalizeQuaternionQ15(const int16_t *q, int16_t *q_out) {
	uint32_t minimal_mag_value = 2;
    int32_t qw = q[0];
    int32_t qx = q[1];
    int32_t qy = q[2];
    int32_t qz = q[3];

    // Betrag berechnen: |q| = sqrt(w² + x² + y² + z²)
    uint64_t mag_sq =
        (int32_t)qw * qw +
        (int32_t)qx * qx +
        (int32_t)qy * qy +
        (int32_t)qz * qz;

    if (mag_sq < minimal_mag_value) {
        q_out[0] = 32767;  // Default-Einheitsquat: w = 1.0
        q_out[1] = q_out[2] = q_out[3] = 0;
        return;
    }
    if (mag_sq <= 0x7FFFFFFF) {
        // passt in int32_t → schneller Pfad
        uint32_t mag = sqrt_fast_uint((uint32_t)mag_sq);
        // Q15: scale = (32767 << 15) / mag
        uint32_t scale_q15 = (32767UL << 15) / mag;

        q_out[0] = (int16_t)((qw * scale_q15) >> 15);
        q_out[1] = (int16_t)((qx * scale_q15) >> 15);
        q_out[2] = (int16_t)((qy * scale_q15) >> 15);
        q_out[3] = (int16_t)((qz * scale_q15) >> 15);

        // → verwende scale_q15 etc.
    } else {
        uint8_t shift = 64 - __builtin_clzll(mag_sq);
        mag_sq >>= shift;

        uint32_t mag = sqrt_fast_uint((uint32_t)mag_sq);
        uint32_t scale_q15 = (32767UL << 15) / mag;

        shift >>= 1;
        qw >>= shift;
        qx >>= shift;
        qy >>= shift;
        qz >>= shift;

        q_out[0] = (int16_t)(((int64_t)qw * scale_q15) >> 15);
        q_out[1] = (int16_t)(((int64_t)qx * scale_q15) >> 15);
        q_out[2] = (int16_t)(((int64_t)qy * scale_q15) >> 15);
        q_out[3] = (int16_t)(((int64_t)qz * scale_q15) >> 15);
    }

}

// its the same, but here the default setting are [0;0;0;0]
void Normalize4DvectorQ15(int16_t *in, int16_t *out){
	uint32_t minimal_mag_value = 4; // if the mag smaller than that, we say its noise
    int32_t qw = in[0];
    int32_t qx = in[1];
    int32_t qy = in[2];
    int32_t qz = in[3];

    // Betrag berechnen: |q| = sqrt(w² + x² + y² + z²)
    uint64_t mag_sq =
        (int32_t)qw * qw +
        (int32_t)qx * qx +
        (int32_t)qy * qy +
        (int32_t)qz * qz;

    if (mag_sq < minimal_mag_value) {
        out[0] = out[1] = out[2] = out[3] = 0;
        return;
    }
    if (mag_sq <= 0x7FFFFFFF) {
        // passt in int32_t → schneller Pfad
        uint32_t mag = sqrt_fast_uint((uint32_t)mag_sq);
        // Q15: scale = (32767 << 15) / mag
        uint32_t scale_q15 = (32767UL << 15) / mag;

        out[0] = (int16_t)((qw * scale_q15) >> 15);
        out[1] = (int16_t)((qx * scale_q15) >> 15);
        out[2] = (int16_t)((qy * scale_q15) >> 15);
        out[3] = (int16_t)((qz * scale_q15) >> 15);

        // → verwende scale_q15 etc.
    } else {
        uint8_t shift = 64 - __builtin_clzll(mag_sq);
        mag_sq >>= shift;

        uint32_t mag = sqrt_fast_uint((uint32_t)mag_sq);
        uint32_t scale_q15 = (32767UL << 15) / mag;

        shift >>= 1;
        qw >>= shift;
        qx >>= shift;
        qy >>= shift;
        qz >>= shift;

        out[0] = (int16_t)(((int64_t)qw * scale_q15) >> 15);
        out[1] = (int16_t)(((int64_t)qx * scale_q15) >> 15);
        out[2] = (int16_t)(((int64_t)qy * scale_q15) >> 15);
        out[3] = (int16_t)(((int64_t)qz * scale_q15) >> 15);
    }
}

void q_t_conj_function(int16_t *q){
	q[1] = -q[1];
	q[2] = -q[2];
	q[3] = -q[3];
}
void q_t_conj_function_in_out_q15(const int16_t *q_in, int16_t *q_out){
	q_out[0] = q_in[0];
	q_out[1] = -q_in[1];
	q_out[2] = -q_in[2];
	q_out[3] = -q_in[3];
}

void q_t_flipp(int16_t *q){
	q[0] = -q[0];
	q[1] = -q[1];
	q[2] = -q[2];
	q[3] = -q[3];
}


/*
 * Matlab function:
 * function q = quat_mult(q1, q2)
w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);

q = [ w1*w2 - x1*x2 - y1*y2 - z1*z2;
      w1*x2 + x1*w2 + y1*z2 - z1*y2;
      w1*y2 - x1*z2 + y1*w2 + z1*x2;
      w1*z2 + x1*y2 - y1*x2 + z1*w2 ];
end
 */
void multiplicateQuaternionQ15(const int16_t *q1, const int16_t *q2, int16_t *q_out){
    int32_t w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    int32_t w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];

    // Q29 Berechnung mit Shifts direkt nach jeder Multiplikation
    int32_t qw = Q15_MUL_HALF(w1, w2) - Q15_MUL_HALF(x1, x2)
               - Q15_MUL_HALF(y1, y2) - Q15_MUL_HALF(z1, z2);

    int32_t qx = Q15_MUL_HALF(w1, x2) + Q15_MUL_HALF(x1, w2)
               + Q15_MUL_HALF(y1, z2) - Q15_MUL_HALF(z1, y2);

    int32_t qy = Q15_MUL_HALF(w1, y2) - Q15_MUL_HALF(x1, z2)
               + Q15_MUL_HALF(y1, w2) + Q15_MUL_HALF(z1, x2);

    int32_t qz = Q15_MUL_HALF(w1, z2) + Q15_MUL_HALF(x1, y2)
               - Q15_MUL_HALF(y1, x2) + Q15_MUL_HALF(z1, w2);

    // Q29 → Q15 (inkl. Rundung)
    q_out[0] = CLAMP_INT32_TO_INT16((qw + (1 << 13)) >> 14);
    q_out[1] = CLAMP_INT32_TO_INT16((qx + (1 << 13)) >> 14);
    q_out[2] = CLAMP_INT32_TO_INT16((qy + (1 << 13)) >> 14);
    q_out[3] = CLAMP_INT32_TO_INT16((qz + (1 << 13)) >> 14);
}



/*
 * quat_to_euler_q15(): input q and output euler[] with
 * 			euler[0] = roll;
 * 			euler]1| = pitch;
 * 			euler[2] = yaw
 */
void quat_to_euler_q15(const int16_t q[4], int16_t euler[3]) {
    // Roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2))
    int32_t q0q1 = q15_mul(q[0], q[1]);
    int32_t q2q3 = q15_mul(q[2], q[3]);
    int32_t roll_num = (q0q1 + q2q3) << 1;

    int32_t q1_sq = q15_mul(q[1], q[1]);
    int32_t q2_sq = q15_mul(q[2], q[2]);
    int32_t roll_den = ((1 << 15) - 2 * (q1_sq + q2_sq));

    euler[0] = q15_atan2((int16_t)(roll_num >> 0), (int16_t)(roll_den >> 0));

    // Pitch = asin(2*(q0*q2 - q3*q1))
    int32_t q0q2 = q15_mul(q[0], q[2]);
    int32_t q3q1 = q15_mul(q[3], q[1]);
    int32_t pitch_arg = (q0q2 - q3q1) << 1;

    euler[1] = q15_asin((int16_t)(pitch_arg >> 0));

    // Yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2))
    int32_t q0q3 = q15_mul(q[0], q[3]);
    int32_t q1q2 = q15_mul(q[1], q[2]);
    int32_t yaw_num = (q0q3 + q1q2) << 1;

    int32_t q2_sq2 = q15_mul(q[2], q[2]);
    int32_t q3_sq = q15_mul(q[3], q[3]);
    int32_t yaw_den = ((1 << 15) - 2 * (q2_sq2 + q3_sq));

    euler[2] = q15_atan2((int16_t)(yaw_num >> 0), (int16_t)(yaw_den >> 0));
}

void vector2quaternion_q15(const int16_t *v, int16_t *q){
	q[0] = 0;
	q[1] = v[0];
	q[2] = v[1];
	q[3] = v[2];
}

void rotate_quat_sandwich_q15(const int16_t *q1, const int16_t *v_q, const int16_t *q2, int16_t *v_q_out){
	int16_t q_x[4];
	multiplicateQuaternionQ15(q1,v_q,q_x);
	multiplicateQuaternionQ15(q_x,q2,v_q_out);
}

//\#########################################################


void ln_q15_unit_quaternions_multiplicate_2(const int16_t *q_in, int16_t *ln_out){

	int16_t v[3], theta_pi;
	int32_t theta_q15;

	v[0] 	= q_in[1];
	v[1] 	= q_in[2];
	v[2] 	= q_in[3];
	norm_3d_vector(v, v);
	if(v[0] == 0 && v[1] == 0 && v[2] == 0){
		ln_out[0] = 0;
		ln_out[1] = 0;
		ln_out[2] = 0;
		return;
	}
	theta_pi = (int16_t)q15_acos(q_in[0]); // Output Q15 -> pi
	theta_q15 = ((int32_t)theta_pi * PI_Q13 + (1<<12)) >> 13;
	theta_q15 = CLAMP_INT32_TO_INT16(theta_q15);

	ln_out[0] = q15_mul_2(v[0],theta_q15);
	ln_out[1] = q15_mul_2(v[1],theta_q15);
	ln_out[2] = q15_mul_2(v[2],theta_q15);
}

void minimal_rotation(const int16_t *a, const int16_t *b, int16_t *q_out){
	int16_t v[3], c;
	int32_t x_i;
	uint32_t x_u;

	crossproduct_3x3_Q15(a,b,v);
	dotporduct_3x3_Q15(a,b,&c);

	if(c < -(Q15 - 20)){

	}else{
		x_u = ((uint32_t)((int32_t)c + Q15)) >> 16; // Q15^2 * 2x -> sqrt(.. ) = Q15*sqrt(2x)
		x_u = sqrt_fast_uint(x_u);
		x_i = CLAMP_INT32_TO_INT16((int32_t)x_u);
		if(x_i > 5){
			q_out[0] = x_i >> 1;
			q_out[1] = v[0]/x_i;
			q_out[2] = v[1]/x_i;
			q_out[3] = v[2]/x_i;
		}else{
			q_out[0] = Q15;
			q_out[1] = 0;
			q_out[2] = 0;
			q_out[3] = 0;
		}
	}
}

