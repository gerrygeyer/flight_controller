/*
 * axis_aligned_filter.h
 *
 *  Created on: Sep 18, 2025
 *      Author: gerrygeyer
 */
#include <parameter.h>


//#define Q30_MUL(a,b)   ((int64_t)(a) * (int64_t)(b) >> 30)
#define MAX3(a,b,c) ((a) > (b) ? ((a) > (c) ? (a) : (c)) : ((b) > (c) ? (b) : (c)))

#define SENSOR_FUSION_FREQ	1000 // Hz

#define ACC_MAX_VALUE	16		// g
#define GYRO_MAX_VALUE	2000 	// Degree / s
#define GRAD2RAD_GYRO_MAX_VALUE_Q15 	30039 //939 // (1/2000'°/s')* (2*pi/360°) -> Gyro * 938.7341; max output = 34,9 rad/s


void axis_aligned_init(void);
void vqf_init(void);
void mahony_filter_init(void);
void axis_aligned_filter(sensor_fusion *pHandle_sf, int16_t *q_out,const bool acc_on,const bool mag_on);
void vqf_filter(const sensor_fusion *pHandle_sf,int16_t *q_out, const bool mag_on, uint8_t reset);
void mahony_filter(const sensor_fusion *pHandle_sf,int16_t *q_out, const bool mag_on, uint8_t reset);


/**
 * @brief StructDescription
 * @note  Optionaler Hinweis zur Verwendung
 * @see   ReferenzOderModulname
 */
typedef struct
{
	int32_t b0_Q30; /**< FieldDesc1 */
	int32_t b1_Q30; /**< FieldDesc2 */
	int32_t b2_Q30; /**< FieldDesc3 */
	int32_t a1_Q30; /**< FieldDesc3 */
	int32_t a2_Q30; /**< FieldDesc3 */
} butterworth_coefficients;

/**
 * @brief StructDescription
 * @note  Optionaler Hinweis zur Verwendung
 * @see   ReferenzOderModulname
 */
typedef struct
{
	int16_t max; /**< FieldDesc1 */
	int16_t min; /**< FieldDesc2 */
	int16_t c; /**< FieldDesc3 */
	int16_t result; /**< FieldDesc3 */
	int16_t theta;
} adapt_coefficients;
