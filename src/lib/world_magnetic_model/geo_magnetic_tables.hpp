/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>

static constexpr float SAMPLING_RES = 10;
static constexpr float SAMPLING_MIN_LAT = -90;
static constexpr float SAMPLING_MAX_LAT = 90;
static constexpr float SAMPLING_MIN_LON = -180;
static constexpr float SAMPLING_MAX_LON = 180;

static constexpr int LAT_DIM = 19;
static constexpr int LON_DIM = 37;


// *INDENT-OFF*
// Magnetic declination data in radians * 10^-4
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2021.6438,
static constexpr const int16_t declination_table[19][37] {
	//    LONGITUDE:   -180,  -170,  -160,  -150,  -140,  -130,  -120,  -110,  -100,   -90,   -80,   -70,   -60,   -50,   -40,   -30,   -20,   -10,     0,    10,    20,    30,    40,    50,    60,    70,    80,    90,   100,   110,   120,   130,   140,   150,   160,   170,   180,
	/* LAT: -90 */ {  26010, 24264, 22519, 20773, 19028, 17283, 15537, 13792, 12047, 10302,  8556,  6811,  5066,  3320,  1575,  -170, -1915, -3661, -5406, -7151, -8897,-10642,-12387,-14133,-15878,-17623,-19369,-21114,-22859,-24605,-26350,-28095,-29841, 31246, 29500, 27755, 26010, },
	/* LAT: -80 */ {  22578, 20444, 18501, 16725, 15083, 13543, 12079, 10667,  9291,  7940,  6608,  5289,  3980,  2675,  1368,    49, -1291, -2661, -4069, -5519, -7012, -8546,-10122,-11739,-13402,-15118,-16902,-18773,-20753,-22870,-25143,-27576,-30145, 30045, 27423, 24913, 22578, },
	/* LAT: -70 */ {  14969, 13573, 12449, 11490, 10625,  9796,  8956,  8071,  7119,  6101,  5031,  3937,  2850,  1793,   770,  -240, -1276, -2378, -3575, -4870, -6247, -7673, -9118,-10558,-11983,-13398,-14824,-16307,-17925,-19830,-22336,-26112, 30802, 24183, 19627, 16842, 14969, },
	/* LAT: -60 */ {   8388,  8149,  7874,  7606,  7358,  7109,  6805,  6376,  5765,  4948,  3949,  2838,  1715,   683,  -204,  -967, -1696, -2515, -3510, -4698, -6021, -7386, -8704, -9915,-10980,-11881,-12599,-13095,-13266,-12817,-10727, -3581,  4803,  7588,  8378,  8515,  8388, },
	/* LAT: -50 */ {   5460,  5499,  5448,  5363,  5294,  5261,  5230,  5107,  4768,  4109,  3104,  1834,   490,  -697, -1579, -2155, -2557, -2994, -3667, -4664, -5895, -7165, -8305, -9211, -9816,-10064, -9879, -9130, -7629, -5274, -2374,   372,  2481,  3905,  4779,  5254,  5460, },
	/* LAT: -40 */ {   3936,  4030,  4042,  4002,  3946,  3916,  3923,  3915,  3746,  3218,  2203,   766,  -800, -2124, -3002, -3467, -3661, -3726, -3885, -4431, -5386, -6454, -7342, -7890, -8010, -7646, -6768, -5390, -3676, -1968,  -504,   718,  1760,  2623,  3276,  3705,  3936, },
	/* LAT: -30 */ {   2967,  3055,  3089,  3080,  3027,  2952,  2895,  2863,  2739,  2268,  1243,  -268, -1877, -3143, -3897, -4256, -4347, -4151, -3721, -3476, -3809, -4544, -5248, -5611, -5500, -4925, -3974, -2765, -1549,  -597,    84,   686,  1307,  1905,  2411,  2770,  2967, },
	/* LAT: -20 */ {   2325,  2372,  2395,  2403,  2366,  2276,  2172,  2097,  1954,  1464,   420, -1060, -2542, -3616, -4166, -4297, -4108, -3564, -2708, -1896, -1594, -1931, -2579, -3057, -3105, -2756, -2125, -1306,  -522,   -26,   241,   544,   982,  1454,  1871,  2175,  2325, },
	/* LAT: -10 */ {   1931,  1927,  1911,  1914,  1891,  1812,  1707,  1619,  1436,   891,  -161, -1533, -2812, -3656, -3943, -3732, -3167, -2392, -1545,  -783,  -305,  -322,  -783, -1292, -1513, -1430, -1119,  -618,  -115,   132,   186,   348,   715,  1145,  1533,  1815,  1931, },
	/* LAT:   0 */ {   1719,  1687,  1636,  1633,  1627,  1564,  1465,  1356,  1104,   487,  -547, -1773, -2835, -3437, -3456, -2970, -2208, -1425,  -768,  -228,   191,   313,    43,  -374,  -634,  -693,  -594,  -330,   -36,    58,     2,    93,   430,   865,  1278,  1593,  1719, },
	/* LAT:  10 */ {   1586,  1595,  1557,  1578,  1608,  1565,  1450,  1268,   895,   179,  -839, -1918, -2754, -3110, -2922, -2326, -1552,  -836,  -315,    69,   396,   551,   397,    76,  -162,  -270,  -291,  -208,  -100,  -138,  -272,  -244,    54,   501,   976,  1378,  1586, },
	/* LAT:  20 */ {   1407,  1556,  1620,  1716,  1806,  1790,  1641,  1344,   802,   -54, -1098, -2055, -2664, -2787, -2470, -1875, -1165,  -512,   -46,   267,   528,   678,   592,   350,   148,    31,   -57,  -117,  -194,  -380,  -614,  -672,  -442,     0,   537,  1052,  1407, },
	/* LAT:  30 */ {   1113,  1481,  1742,  1968,  2130,  2143,  1960,  1542,   816,  -213, -1336, -2222, -2649, -2598, -2206, -1630,  -976,  -355,   112,   421,   655,   805,   790,   642,   491,   367,   215,    10,  -268,  -640, -1008, -1169, -1012,  -591,   -17,   592,  1113, },
	/* LAT:  40 */ {    764,  1352,  1846,  2238,  2488,  2533,  2320,  1789,   879,  -352, -1600, -2478, -2810, -2666, -2224, -1633,  -980,  -348,   166,   534,   808,  1010,  1109,  1100,  1024,   877,   614,   211,  -322,  -926, -1449, -1696, -1584, -1170,  -571,   103,   764, },
	/* LAT:  50 */ {    484,  1233,  1914,  2469,  2834,  2940,  2711,  2056,   912,  -594, -2029, -2957, -3259, -3072, -2584, -1938, -1227,  -527,    92,   602,  1024,  1383,  1668,  1842,  1864,  1681,  1244,   545,  -342, -1244, -1928, -2221, -2100, -1659, -1021,  -284,   484, },
	/* LAT:  60 */ {    303,  1159,  1964,  2657,  3158,  3367,  3149,  2331,   803, -1176, -2910, -3900, -4155, -3894, -3320, -2573, -1744,  -900,   -89,   666,  1362,  1996,  2543,  2944,  3110,  2932,  2306,  1203,  -204, -1527, -2405, -2721, -2558, -2061, -1361,  -553,   303, },
	/* LAT:  70 */ {     96,  1036,  1935,  2733,  3345,  3632,  3368,  2208,   -78, -2838, -4804, -5624, -5623, -5130, -4352, -3411, -2381, -1310,  -231,   833,  1861,  2831,  3703,  4415,  4870,  4913,  4322,  2873,   698, -1411, -2719, -3169, -3010, -2474, -1716,  -838,    96, },
	/* LAT:  80 */ {   -520,   406,  1263,  1957,  2339,  2136,   871, -1889, -5232, -7361, -8071, -7904, -7247, -6310, -5204, -3993, -2718, -1406,   -75,  1256,  2574,  3858,  5087,  6227,  7223,  7985,  8333,  7907,  6044,  2394, -1149, -2885, -3275, -2961, -2292, -1443,  -520, },
	/* LAT:  90 */ { -30178,-28433,-26687,-24942,-23196,-21451,-19706,-17960,-16215,-14470,-12724,-10979, -9234, -7489, -5744, -3999, -2253,  -508,  1237,  2982,  4727,  6473,  8218,  9963, 11709, 13454, 15199, 16945, 18690, 20435, 22181, 23926, 25672, 27417, 29163, 30908,-30178, },
};

// Magnetic inclination data in radians * 10^-4
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2021.6438,
static constexpr const int16_t inclination_table[19][37] {
	//    LONGITUDE:   -180,  -170,  -160,  -150,  -140,  -130,  -120,  -110,  -100,   -90,   -80,   -70,   -60,   -50,   -40,   -30,   -20,   -10,     0,    10,    20,    30,    40,    50,    60,    70,    80,    90,   100,   110,   120,   130,   140,   150,   160,   170,   180,
	/* LAT: -90 */ { -12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577,-12577, },
	/* LAT: -80 */ { -13663,-13530,-13369,-13189,-12995,-12793,-12589,-12388,-12197,-12020,-11863,-11728,-11618,-11532,-11468,-11427,-11407,-11410,-11437,-11491,-11575,-11689,-11833,-12006,-12204,-12421,-12650,-12883,-13111,-13324,-13511,-13661,-13766,-13818,-13815,-13761,-13663, },
	/* LAT: -70 */ { -14113,-13795,-13475,-13152,-12820,-12475,-12120,-11762,-11417,-11108,-10854,-10669,-10556,-10503,-10489,-10494,-10502,-10513,-10538,-10594,-10702,-10875,-11119,-11432,-11804,-12220,-12668,-13131,-13598,-14050,-14469,-14816,-15008,-14956,-14726,-14429,-14113, },
	/* LAT: -60 */ { -13523,-13170,-12833,-12500,-12157,-11784,-11369,-10913,-10444,-10012, -9679, -9500, -9494, -9630, -9835,-10030,-10160,-10208,-10199,-10186,-10232,-10388,-10673,-11078,-11574,-12128,-12713,-13309,-13898,-14459,-14956,-15246,-15075,-14692,-14287,-13895,-13523, },
	/* LAT: -50 */ { -12498,-12157,-11827,-11505,-11182,-10835,-10436, -9964, -9433, -8909, -8515, -8383, -8577, -9035, -9599,-10115,-10482,-10654,-10632,-10486,-10343,-10344,-10557,-10967,-11505,-12096,-12683,-13223,-13671,-13970,-14079,-14007,-13804,-13521,-13193,-12846,-12498, },
	/* LAT: -40 */ { -11240,-10894,-10548,-10204, -9864, -9525, -9164, -8737, -8215, -7647, -7215, -7163, -7612, -8430, -9353,-10187,-10853,-11297,-11450,-11294,-10957,-10680,-10663,-10936,-11393,-11897,-12345,-12671,-12833,-12839,-12749,-12611,-12430,-12198,-11912,-11585,-11240, },
	/* LAT: -30 */ {  -9601, -9226, -8850, -8461, -8068, -7692, -7333, -6940, -6426, -5813, -5354, -5421, -6174, -7368, -8621, -9722,-10643,-11362,-11775,-11781,-11420,-10913,-10571,-10562,-10809,-11130,-11390,-11507,-11448,-11272,-11096,-10958,-10810,-10602,-10321, -9976, -9601, },
	/* LAT: -20 */ {  -7370, -6936, -6524, -6098, -5653, -5221, -4828, -4411, -3842, -3154, -2697, -2937, -4027, -5629, -7260, -8639, -9715,-10502,-10954,-11003,-10651,-10043, -9482, -9230, -9274, -9435, -9579, -9601, -9431, -9158, -8961, -8869, -8760, -8550, -8233, -7823, -7370, },
	/* LAT: -10 */ {  -4413, -3886, -3439, -3006, -2548, -2097, -1679, -1219,  -587,   121,   495,    90, -1217, -3123, -5102, -6725, -7838, -8486, -8762, -8710, -8312, -7635, -6971, -6620, -6576, -6669, -6788, -6815, -6629, -6332, -6173, -6174, -6126, -5906, -5523, -4998, -4413, },
	/* LAT:   0 */ {   -903,  -291,   166,   565,   982,  1400,  1794,  2245,  2837,  3422,  3636,  3158,  1883,   -15, -2061, -3728, -4756, -5200, -5268, -5105, -4674, -3967, -3262, -2886, -2824, -2895, -3022, -3097, -2968, -2731, -2671, -2804, -2856, -2665, -2241, -1616,  -903, },
	/* LAT:  10 */ {   2564,  3180,  3607,  3945,  4299,  4666,  5022,  5416,  5876,  6254,  6299,  5824,  4756,  3201,  1510,   122,  -700,  -961,  -878,  -648,  -244,   388,  1020,  1361,  1423,  1375,  1273,  1181,  1225,  1330,  1260,  1004,   822,   900,  1251,  1849,  2564, },
	/* LAT:  20 */ {   5418,  5939,  6314,  6607,  6919,  7263,  7609,  7962,  8302,  8509,  8428,  7975,  7148,  6058,  4932,  4015,  3475,  3346,  3489,  3728,  4055,  4515,  4975,  5231,  5286,  5265,  5215,  5158,  5148,  5133,  4964,  4639,  4349,  4263,  4435,  4854,  5418, },
	/* LAT:  30 */ {   7569,  7939,  8254,  8535,  8844,  9192,  9552,  9897, 10179, 10297, 10158,  9742,  9117,  8407,  7745,  7229,  6931,  6883,  7020,  7228,  7472,  7769,  8056,  8230,  8285,  8295,  8296,  8286,  8266,  8187,  7969,  7621,  7270,  7052,  7036,  7228,  7569, },
	/* LAT:  40 */ {   9266,  9486,  9741, 10026, 10353, 10715, 11085, 11427, 11682, 11768, 11622, 11265, 10793, 10318,  9920,  9634,  9481,  9473,  9580,  9739,  9912, 10094, 10265, 10390, 10467, 10526, 10579, 10612, 10597, 10490, 10251,  9904,  9539,  9255,  9112,  9124,  9266, },
	/* LAT:  50 */ {  10801, 10923, 11125, 11395, 11719, 12075, 12431, 12749, 12974, 13035, 12898, 12600, 12235, 11888, 11611, 11423, 11327, 11321, 11385, 11485, 11598, 11712, 11828, 11943, 12060, 12181, 12293, 12366, 12359, 12237, 11994, 11668, 11329, 11044, 10854, 10774, 10801, },
	/* LAT:  60 */ {  12318, 12393, 12544, 12764, 13036, 13337, 13641, 13909, 14086, 14114, 13978, 13728, 13438, 13166, 12944, 12786, 12693, 12660, 12674, 12720, 12787, 12871, 12976, 13107, 13265, 13440, 13604, 13714, 13723, 13606, 13382, 13104, 12825, 12588, 12417, 12326, 12318, },
	/* LAT:  70 */ {  13759, 13803, 13900, 14044, 14225, 14430, 14640, 14822, 14926, 14905, 14765, 14562, 14343, 14138, 13964, 13828, 13733, 13677, 13657, 13669, 13710, 13781, 13884, 14019, 14185, 14371, 14555, 14697, 14746, 14676, 14516, 14317, 14122, 13958, 13837, 13770, 13759, },
	/* LAT:  80 */ {  15001, 15015, 15054, 15115, 15193, 15280, 15360, 15404, 15383, 15301, 15186, 15060, 14936, 14822, 14722, 14639, 14577, 14536, 14517, 14520, 14546, 14595, 14666, 14758, 14869, 14996, 15130, 15262, 15369, 15414, 15378, 15294, 15200, 15116, 15053, 15014, 15001, },
	/* LAT:  90 */ {  15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, 15392, },
};

// Magnetic strength data in milli-Gauss * 10
// Model: WMM-2020,
// Version: 0.5.1.11,
// Date: 2021.6438,
static constexpr const int16_t strength_table[19][37] {
	//    LONGITUDE:  -180, -170, -160, -150, -140, -130, -120, -110, -100,  -90,  -80,  -70,  -60,  -50,  -40,  -30,  -20,  -10,    0,   10,   20,   30,   40,   50,   60,   70,   80,   90,  100,  110,  120,  130,  140,  150,  160,  170,  180,
	/* LAT: -90 */ {  5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, 5457, },
	/* LAT: -80 */ {  6063, 6000, 5922, 5830, 5728, 5616, 5498, 5377, 5254, 5134, 5020, 4915, 4821, 4741, 4677, 4632, 4608, 4605, 4626, 4672, 4742, 4835, 4949, 5081, 5224, 5374, 5523, 5666, 5797, 5911, 6004, 6073, 6118, 6138, 6134, 6108, 6063, },
	/* LAT: -70 */ {  6307, 6175, 6026, 5862, 5684, 5492, 5287, 5073, 4853, 4638, 4435, 4252, 4094, 3963, 3858, 3781, 3734, 3721, 3748, 3823, 3950, 4130, 4357, 4624, 4918, 5225, 5529, 5813, 6062, 6267, 6418, 6515, 6557, 6550, 6501, 6418, 6307, },
	/* LAT: -60 */ {  6192, 6001, 5801, 5593, 5375, 5141, 4885, 4606, 4314, 4024, 3756, 3528, 3348, 3213, 3113, 3040, 2989, 2969, 2994, 3083, 3250, 3500, 3825, 4209, 4631, 5065, 5486, 5870, 6196, 6445, 6609, 6689, 6692, 6630, 6517, 6367, 6192, },
	/* LAT: -50 */ {  5848, 5620, 5389, 5159, 4927, 4683, 4413, 4110, 3781, 3449, 3146, 2905, 2742, 2650, 2602, 2568, 2535, 2509, 2514, 2585, 2756, 3043, 3436, 3905, 4411, 4917, 5393, 5812, 6152, 6395, 6534, 6576, 6534, 6425, 6264, 6067, 5848, },
	/* LAT: -40 */ {  5396, 5152, 4908, 4669, 4435, 4199, 3945, 3661, 3347, 3021, 2721, 2497, 2378, 2352, 2371, 2393, 2399, 2389, 2377, 2405, 2531, 2802, 3215, 3726, 4273, 4798, 5266, 5656, 5949, 6137, 6227, 6230, 6162, 6032, 5851, 5634, 5396, },
	/* LAT: -30 */ {  4880, 4641, 4403, 4169, 3944, 3725, 3507, 3276, 3020, 2745, 2487, 2303, 2231, 2254, 2321, 2393, 2460, 2512, 2536, 2548, 2613, 2806, 3162, 3646, 4177, 4675, 5096, 5416, 5620, 5721, 5748, 5721, 5642, 5511, 5332, 5116, 4880, },
	/* LAT: -20 */ {  4322, 4111, 3903, 3699, 3503, 3321, 3153, 2990, 2813, 2615, 2424, 2289, 2246, 2286, 2375, 2486, 2614, 2744, 2836, 2874, 2898, 2989, 3223, 3603, 4051, 4479, 4830, 5068, 5175, 5183, 5153, 5106, 5024, 4897, 4731, 4534, 4322, },
	/* LAT: -10 */ {  3791, 3631, 3479, 3333, 3198, 3078, 2975, 2884, 2788, 2674, 2553, 2452, 2404, 2425, 2509, 2637, 2793, 2953, 3080, 3145, 3160, 3184, 3303, 3551, 3874, 4196, 4463, 4629, 4666, 4614, 4547, 4483, 4394, 4269, 4120, 3957, 3791, },
	/* LAT:   0 */ {  3412, 3320, 3237, 3165, 3110, 3072, 3047, 3029, 3007, 2960, 2881, 2786, 2704, 2669, 2708, 2809, 2941, 3077, 3194, 3271, 3303, 3324, 3396, 3550, 3757, 3970, 4151, 4259, 4267, 4200, 4112, 4019, 3907, 3776, 3642, 3519, 3412, },
	/* LAT:  10 */ {  3283, 3253, 3233, 3230, 3255, 3303, 3360, 3415, 3452, 3443, 3374, 3259, 3131, 3034, 3004, 3043, 3123, 3221, 3321, 3407, 3471, 3533, 3619, 3736, 3870, 4007, 4127, 4198, 4202, 4141, 4032, 3889, 3728, 3569, 3434, 3338, 3283, },
	/* LAT:  20 */ {  3400, 3404, 3431, 3486, 3579, 3701, 3831, 3950, 4032, 4045, 3972, 3830, 3662, 3519, 3440, 3426, 3459, 3531, 3626, 3723, 3814, 3911, 4021, 4132, 4240, 4350, 4451, 4518, 4530, 4473, 4337, 4136, 3909, 3700, 3538, 3438, 3400, },
	/* LAT:  30 */ {  3723, 3732, 3788, 3889, 4033, 4205, 4382, 4539, 4648, 4675, 4603, 4446, 4256, 4089, 3980, 3932, 3934, 3983, 4069, 4166, 4262, 4367, 4483, 4601, 4720, 4845, 4965, 5051, 5079, 5023, 4868, 4630, 4356, 4102, 3904, 3776, 3723, },
	/* LAT:  40 */ {  4222, 4223, 4290, 4416, 4584, 4772, 4957, 5114, 5219, 5244, 5176, 5026, 4837, 4661, 4531, 4453, 4425, 4445, 4504, 4583, 4668, 4764, 4879, 5012, 5163, 5324, 5474, 5585, 5626, 5573, 5419, 5182, 4908, 4649, 4439, 4294, 4222, },
	/* LAT:  50 */ {  4832, 4827, 4885, 4998, 5145, 5305, 5456, 5576, 5650, 5658, 5594, 5466, 5302, 5136, 4997, 4898, 4842, 4828, 4852, 4901, 4969, 5059, 5178, 5327, 5502, 5686, 5853, 5972, 6018, 5976, 5846, 5650, 5425, 5208, 5028, 4900, 4832, },
	/* LAT:  60 */ {  5392, 5382, 5413, 5478, 5565, 5660, 5746, 5812, 5844, 5834, 5779, 5683, 5562, 5432, 5312, 5215, 5147, 5112, 5109, 5136, 5192, 5278, 5395, 5542, 5708, 5877, 6025, 6130, 6176, 6155, 6072, 5946, 5799, 5655, 5533, 5443, 5392, },
	/* LAT:  70 */ {  5726, 5708, 5707, 5720, 5743, 5770, 5793, 5807, 5806, 5785, 5745, 5687, 5615, 5538, 5464, 5399, 5350, 5322, 5317, 5336, 5380, 5449, 5541, 5649, 5767, 5881, 5981, 6055, 6095, 6098, 6070, 6016, 5949, 5878, 5814, 5762, 5726, },
	/* LAT:  80 */ {  5789, 5772, 5758, 5747, 5737, 5728, 5718, 5706, 5691, 5672, 5649, 5623, 5596, 5568, 5543, 5522, 5508, 5502, 5507, 5522, 5547, 5582, 5625, 5673, 5723, 5771, 5814, 5848, 5873, 5886, 5888, 5882, 5868, 5849, 5829, 5808, 5789, },
	/* LAT:  90 */ {  5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, 5680, },
};
