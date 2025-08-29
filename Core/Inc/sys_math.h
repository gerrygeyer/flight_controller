/*
 * sys_math.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Gerry
 */

#ifndef INC_SYS_MATH_H_
#define INC_SYS_MATH_H_

#include <parameter.h>
#include <math.h>

#define CLAMP_INT32_TO_INT16(x) ((x) > INT16_MAX ? INT16_MAX : ((x) < INT16_MIN ? INT16_MIN : (int16_t)(x)))
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define INT16_HALF_VALUE	0x3FFF
#define INT16_MAX_VALUE		0x7FFF
#define Q15_ONE				0x7FFF
#define PI					(float)3.14159265358979
#define PI_MULTIPLY_2		(float)6.283185307179586
#define RAD_TO_RPM			(float)9.549296585513721
#define RPM_TO_RAD			(float)(1.0f/RAD_TO_RPM)
#define RAD_TO_RPM_SQRT		(float)3.090193616185517
#define DEGREE_TO_RAD		(float)0.017453292519943
#define RAD_TO_DEGREE		(float)57.29577951308232


#define PI_Q13				25736
#define PI_OVER_2_Q15		16384
#define SQRT_2_OVER_2_Q15	23170
#define RPM_TO_RAD_Q15		3432
#define RAD2RPM_2_Q8 		23344 // (60(2pi)^2 = 91.18...-> 91.18*Q8
#define RADQ5_TO_RPMQ13_Q15	1222 // //(60/(2*pi)) * (2^5)/((2^13))



#define Q1					2
#define Q4					16
#define Q10					1024
#define Q11					2048
#define Q12					4096
#define Q13					8192
#define Q14					16384
#define Q15					32768
#define Q16					65536
#define Q18					262144
#define Q19					524288
#define Q20					1048576
#define Q21					2097152
#define Q22					4194304
#define Q29					536870912
#define Q30					1073741824


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

/**
 * @brief   Q15 rounding right-shift with sign-dependent bias.
 *
 * This macro performs a fixed-point rounding operation when shifting
 * a 32-bit integer value down by 15 bits (Q15 scaling).
 *
 * The rounding offset (1 << 14) is added or subtracted depending
 * on the sign of the input value, ensuring correct rounding for
 * both positive and negative numbers.
 *
 * Formula:
 *   result = (x + (x >= 0 ? +16384 : -16384)) >> 15
 *
 * - For positive values: adds +16384 before shifting → rounds up.
 * - For negative values: adds -16384 before shifting → rounds down.
 *
 * @param x  Input value in int32_t to be rounded and shifted.
 * @return   int32_t Rounded result of (x / 32768) with sign-aware rounding.
 *
 * @note This is a "round to nearest, tie away from zero" implementation.
 */
#define Q15_SHIFT_ROUND(x)   (((x) + (((x) >= 0) ? (1 << 14) : -(1 << 14))) >> 15)
#define Q14_SHIFT_ROUND(x)   (((x) + (((x) >= 0) ? (1 << 13) : -(1 << 13))) >> 14)

/**
 * @brief       Converts motor speed from RPM to angular velocity in rad/s (Q15 scaled).
 *
 * @details     Computes the angular velocity from a given motor speed in revolutions per minute (RPM)
 *              using a fixed-point Q15 conversion factor.
 *              The result is normalized by the maximum motor speed (`MAX_SPEED_MOTOR_RAD`).
 *
 * @param[in]   rpm     Motor speed in revolutions per minute (int16_t).
 *
 * @return      Angular velocity in rad/s, represented in Q15 fixed-point format.
 *
 * @note
 * - `RPM_TO_RAD_Q15` must represent the factor \f$ \frac{2\pi}{60} \f$ in Q15 format.
 * - `MAX_SPEED_MOTOR_RAD` must represent the maximum speed in rad/s.
 * - Result is clamped to the valid int16_t range via `CLAMP_INT32_TO_INT16()`.
 *
 * @see         CLAMP_INT32_TO_INT16()
 */
int16_t rpm2rad_sQ15_scaled(int16_t rpm);

/**
 * @defgroup TimeMeas Time Measurement Functions
 * @brief Utility functions to measure code execution time using TIM2.
 * @{
 */

/**
 * @brief       Starts the time measurement using TIM2.
 *
 * @details     Resets the TIM2 counter and enables the timer. Used to measure the execution time
 *              of control code or other critical sections.
 *
 * @note        Assumes TIM2 is configured with a 240 MHz clock (1 tick = 1/240 µs).
 */
void start_time_measurement(void);

/**
 * @brief       Stops the time measurement and returns the elapsed time in microseconds.
 *
 * @details     Disables TIM2 and reads the number of elapsed ticks since `start_time_measurement()`.
 *              Converts ticks to microseconds based on a 240 MHz timer clock.
 *
 * @return      Elapsed time in microseconds (uint32_t).
 *
 * @note        Assumes TIM2 is clocked at 240 MHz. Stores result in global `time` variable.
 * @warning     Not reentrant. Do not use in interrupt context without protection.
 */
uint32_t stopp_time_measurement(void);

/** @} */  // end of TimeMeas group

/**
 * @defgroup TrigQ15 Trigonometric Functions (Q15)
 * @brief Fixed-point trigonometric functions (sine, cosine, asin, atan2) using lookup tables.
 * @{
 */

/**
 * @brief       Computes the sine of an angle in Q15 format using a lookup table.
 *
 * @details     Approximates \f$ \sin(y) \f$ using a 1024-point sine lookup table (`sineLookupTable`)
 *              for the range \f$ [0, \pi/2] \f$, exploiting symmetry for full range support.
 *              The input `y` is in Q15 format representing angles in radians scaled to \f$ [-\pi, +\pi] \f$.
 *
 * @param       y       Angle in Q15 format (−32768 ≙ −π, +32767 ≙ +π).
 *
 * @return      Sine of the input angle, in Q15 format (range: −32768 … +32767).
 *
 * @note
 * 				- Internally maps the input angle to an index between 0 and 2047.
 * 				- Uses symmetry:
 *   			- sin(−x) = −sin(x)
 *   			- sin(π − x) = sin(x)
 *		 		- Table contains 1024 entries for \f$ [0, \frac{\pi}{2}] \f$.
 *
 * @see         sineLookupTable, Q15 fixed-point format
 */
int16_t sin_i(int16_t y);

/**
 * @brief       Computes the cosine of an angle in Q15 format using a sine lookup table.
 *
 * @details     Approximates \f$ \cos(y) \f$ by shifting the input angle by \f$ \frac{\pi}{2} \f$
 *              and evaluating the sine using `sin_i()`.
 *              This uses the identity:
 *              \f$ \cos(y) = \sin(y + \frac{\pi}{2}) \f$
 *
 * @param       y       Angle in Q15 format (−32768 ≙ −π, +32767 ≙ +π).
 *
 * @return      Cosine of the input angle, in Q15 format (range: −32768 … +32767).
 *
 * @note
 * - Internally calls `sin_i()`.
 * - `INT16_HALF_VALUE` must represent \f$ \frac{\pi}{2} \f$ in Q15 (i.e., 16384).
 *
 * @see         sin_i(), sineLookupTable
 */
int16_t cos_i(int16_t y);

/**
 * @brief       Approximates the arcsine (asin) of a Q15 input using a lookup table and linear interpolation.
 *
 * @details     Computes \f$ \arcsin(x) \f$ for \f$ x \in [-1, +1] \f$ (Q15 format: −32768 … +32767)
 *              using the `asin_q15_lut[]` table (covering [0, 1]) and symmetric extension for negative values.
 *              Linear interpolation is applied between table entries to improve accuracy.
 *
 * @param       x       Input value in Q15 format representing a value in the range [−1, +1].
 *
 * @return      Arcsine of the input in Q15 format, scaled such that \f$ \pi \approx 32767 \f$.
 *
 * @note
 * 				- Uses symmetry: \f$ \arcsin(-x) = -\arcsin(x) \f$
 * 				- Table resolution: 256 entries over [0, 1], each step ≈ 128 in Q15
 * 				- Output range: \f$ [-\frac{\pi}{2}, +\frac{\pi}{2}] \f$ mapped to Q15
 *
 * @see         asin_q15_lut, q15_atan2(), Q15 fixed-point format
 */
int16_t q15_asin(int16_t x);

/**
 * @brief       Approximates the arccosine (acos) of a Q15 input using q15_asin().
 *
 * @details     Computes \f$ \arccos(x) = \frac{\pi}{2} - \arcsin(x) \f$
 *              for \f$ x \in [-1, +1] \f$ in Q15 format.
 *
 * @param       x       Input value in Q15 format representing a value in the range [−1, +1].
 *
 * @return      Arccosine of the input in Q15 format, scaled such that \f$ \pi \approx 32767 \f$.
 *
 * @see			q15_asin()
 */
int16_t q15_acos(int16_t x);

/**
 * @brief       Approximates atan2(y, x) using a Q15 lookup table with interpolation.
 *
 * @details     Computes the arctangent of \f$ \frac{y}{x} \f$ in radians (Q15 format),
 *              correctly handling all four quadrants.
 *              The core angle is calculated using a 256-entry LUT for \f$ [0, \frac{\pi}{4}] \f$
 *              and mirrored accordingly based on the quadrant.
 *              Result is scaled so that \f$ \pi \approx 32767 \f$.
 *
 * @param       y       Y-component (int16_t, Q15).
 * @param       x       X-component (int16_t, Q15).
 *
 * @return      Angle in Q15 fixed-point format, range \f$ [-32767, +32767] \f$ ≙ \f$ [-\pi, +\pi] \f$.
 *
 * @note
 * - Uses symmetry: computes \f$ \theta \in [0, \frac{\pi}{4}] \f$ and mirrors based on quadrant.
 * - Falls back to `0` if both `x == 0 && y == 0` (undefined case).
 * - Interpolation improves accuracy between LUT entries.
 * - Assumes input is already scaled to Q15 (e.g., normalized vector components).
 *
 * @see         atan2_q15_lut, q15_asin(), sineLookupTable
 */
int16_t q15_atan2(int16_t y, int16_t x);




void crossproduct_3x3_Q15(const int16_t *v1, const int16_t *v2, int16_t *cross);
void dotporduct_3x3_Q15(const int16_t *v1, const int16_t *v2, int16_t *dot);

/** @} */


void multiply_matrix_with_scalar(float scalar, float in_matrix[4][4], float out_matrix[4][4]);
void inverse_matrix_3x3_f(float in_matrix[3][3], float out_matrix[3][3]);
void inverse_matrix_4x4_f(float in_matrix[4][4], float out_matrix[4][4]);
void multiply_4x4_with_vector(float in_matrix[4][4], float in_vector[4][1], float out_vector[4][1]);
at_angl_f degree_to_rad(at_angl_f input);



/**
 * @brief       Normalizes a 2D vector in Q15 fixed-point format.
 *
 * @details     This function normalizes the 2D vector @p v to unit length
 *              (magnitude = 1.0 in Q15). The vector is represented in Q15
 *              fixed-point format, where 32767 corresponds to 1.0.
 *
 *              The normalization is skipped if:
 *              - The magnitude is less than 5 (vector too small → set to zero).
 *              - The magnitude is exactly 1 (already normalized).
 *
 *
 * @param[in,out] v   Pointer to a 2-element vector (Q15 format) to be normalized.
 *
 * @note        Uses 32-bit intermediate calculations to prevent overflow.
 *              Magnitude calculation uses `sqrt_fast_uint()` for speed.
 * @warning     If the magnitude is very small, the output will be set to zero.
 *
 * @see         sqrt_fast_uint(), CLAMP_INT32_TO_INT16()
 */
void norm_2d_vector_q15(int16_t *v);

/**
 * @defgroup QuaternionMathQ15 Quaternion Math Functions (Q15)
 * @brief Fixed-point quaternion operations and vector normalization in Q15 format.
 * @{
 */

/**
 * @brief       Fast approximation of the integer square root.
 *
 * @details     Computes the floor of the square root of an unsigned 32-bit integer `n` using:
 *              - Bitwise leading-zero detection for an initial guess
 *              - Three Newton-Raphson refinement iterations
 *              - Final correction to ensure \f$x^2 \leq n\f$
 *
 *              This function avoids floating-point operations and is optimized for speed
 *              on embedded platforms.
 *
 * @param       n       Unsigned 32-bit input value.
 *
 * @return      Floor of the square root of `n` as a 32-bit unsigned integer.
 *
 * @note
 * - Uses `__builtin_clz()` to count leading zeros (GCC/Clang built-in).
 * - Runs in constant time for a given bit-width and does not use recursion or division in a loop.
 *
 * @warning
 * - Accuracy is guaranteed only for non-negative 32-bit integers (i.e., `n ≥ 0`).
 * - Requires a compiler that supports `__builtin_clz()`.
 *
 * @see         https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Integer_square_root
 */
uint32_t sqrt_fast_uint(uint32_t n);


/**
 * @brief       In-place conjugation of a quaternion in Q15 format.
 *
 * @details     Negates the vector part of the quaternion \f$ q = [w, x, y, z] \f$,
 *              resulting in the conjugate:
 *              \f$ q^* = [w, -x, -y, -z] \f$.
 *
 * @param       q       Pointer to the quaternion (int16_t[4]). The result overwrites the input.
 *
 * @note        Only the vector part (indices 1–3) is modified; the scalar part remains unchanged.
 * @see         q_t_conj_function_in_out_q15()
 */
void q_t_conj_function(int16_t *q);

/**
 * @brief       Computes the conjugate of a quaternion in Q15 format.
 *
 * @details     Copies the scalar part unchanged and negates the vector part:
 *              \f$ q^* = [q_0, -q_1, -q_2, -q_3] \f$
 *
 * @param       q_in        Pointer to input quaternion [q0, q1, q2, q3] in Q15.
 * @param       q_out       Pointer to output quaternion (conjugated) in Q15.
 *
 * @note        Input and output must be arrays of 4 elements.
 * @see         Quaternion multiplication, normalization
 */
void q_t_conj_function_in_out_q15(const int16_t *q_in, int16_t *q_out);

/**
 * @brief		Multiply the quaternionion with -1
 */
void q_t_flipp(int16_t *q);

/**
 * @brief       Normalizes a 3D vector to unit length in Q15 format.
 *
 * @details     Computes the Euclidean norm of the 3D vector and scales each component
 *              so that the output vector has a magnitude of ~32767 (Q15 equivalent of 1.0).
 *              If the input vector has zero magnitude, the output is set to zero.
 *
 * @param       input       Pointer to the input vector [x, y, z] as int16_t[3].
 * @param       norm_out    Pointer to the output normalized vector in Q15 format (int16_t[3]).
 *
 * @note
 * - Internally uses `sqrt_fast_uint()` for fast magnitude calculation.
 * - The scaling factor is computed as:
 *   \f$ \text{scale} = \frac{32767 \ll 15}{|v|} \f$
 *   and each component is scaled accordingly.
 *
 * @warning
 * - If the input vector is zero, normalization is skipped to avoid division by zero,
 *   and the output is set to [0, 0, 0].
 *
 * @see         sqrt_fast_uint()
 */
void norm_3d_vector(int16_t *input, int16_t *norm_out);

/**
 * @brief       Normalizes a 4D vector (typically a quaternion) in Q15 format.
 *
 * @details     Computes the magnitude of a 4D vector \f$ q = [w, x, y, z] \f$ and scales each component
 *              so that the result lies on the unit hypersphere (length ≈ 32767 in Q15).
 *              Handles both small and large magnitudes to avoid overflow or underflow.
 *
 * @param       in      Pointer to input vector [w, x, y, z] in Q15 format (int16_t[4]).
 * @param       out     Pointer to output normalized vector in Q15 format (int16_t[4]).
 *
 * @note
 * - For small vectors (\f$ |q|^2 < 128 \f$), the output is set to [0, 0, 0, 0] (treated as noise).
 * - Uses fast square root approximation (`sqrt_fast_uint()`).
 * - If the magnitude exceeds int32_t range, the input is downscaled before normalization.
 * - The scaling factor is computed as:
 *   \f$ \text{scale} = \frac{32767 \ll 15}{|q|} \f$
 *   and each component is scaled accordingly.
 *
 * @warning
 * - Input must be a valid 4-element array.
 * - No rounding is applied; accuracy depends on Q15 resolution and approximation quality.
 *
 * @see         sqrt_fast_uint(), norm_3d_vector()
 */
void Normalize4DvectorQ15(int16_t *in, int16_t *out);

/**
 * @brief       Normalizes a quaternion in Q15 fixed-point format.
 *
 * @details     Computes the magnitude of a 4D quaternion \f$ q = [w, x, y, z] \f$ and scales each component
 *              to produce a unit quaternion (i.e., \f$ \|q\| \approx 1.0 \f$ in Q15).
 *              Handles both low and high magnitude ranges to avoid over-/underflows.
 *              If the magnitude is near zero, a default identity quaternion \f$ [1, 0, 0, 0] \f$ is returned.
 *
 * @param       q           Pointer to input quaternion in Q15 format (int16_t[4]).
 * @param       q_out       Pointer to output normalized quaternion in Q15 format (int16_t[4]).
 *
 * @note
 * - Uses `sqrt_fast_uint()` for fast magnitude approximation.
 * - Scaling factor is computed as:
 *   \f$ \text{scale} = \frac{32767 \ll 15}{|q|} \f$
 *   and applied to all components.
 * - For large magnitudes, input is downscaled (bit-shifted) before normalization.
 *
 * @warning
 * - If magnitude squared is below 2, the result is forcibly set to `[32767, 0, 0, 0]`.
 * - Ensure input points to a valid 4-element array.
 *
 * @see         sqrt_fast_uint(), Normalize4DvectorQ15()
 */
void NormalizeQuaternionQ15(const int16_t *q, int16_t *q_out);


/**
 * @brief       Multiplies two quaternions in Q15 format.
 *
 * @details     Performs quaternion multiplication:
 *              \f$ q_{\text{out}} = q_1 \otimes q_2 \f$
 *              using fixed-point arithmetic (Q15). Intermediate results use Q29 precision and are
 *              downscaled to Q15 with rounding. The result is clamped to int16_t range.
 *
 * @param       q1          Pointer to the first quaternion [w, x, y, z] (Q15, int16_t[4]).
 * @param       q2          Pointer to the second quaternion [w, x, y, z] (Q15, int16_t[4]).
 * @param       q_out       Pointer to the output quaternion [w, x, y, z] (Q15, int16_t[4]).
 *
 * @note
 * - Uses `Q15_MUL_HALF` for intermediate multiplications to maintain precision.
 * - Final shift is `>> 14` (Q29 → Q15) with rounding added (`+ (1 << 13)`).
 * - Output is clamped to int16_t range using `CLAMP_INT32_TO_INT16`.
 *
 * @see         Q15_MUL_HALF, CLAMP_INT32_TO_INT16
 */
void multiplicateQuaternionQ15(const int16_t *q1, const int16_t *q2, int16_t *q_out);


/**
 * @brief       Converts a quaternion to Euler angles (roll, pitch, yaw) in Q15 format.
 *
 * @details     Converts a normalized quaternion \f$ q = [q_0, q_1, q_2, q_3] \f$ into
 *              Euler angles using standard Tait-Bryan angles (ZYX order):
 *              - Roll  (x-axis rotation): \f$ \text{atan2}(2(q_0 q_1 + q_2 q_3), 1 - 2(q_1^2 + q_2^2)) \f$
 *              - Pitch (y-axis rotation): \f$ \text{asin}(2(q_0 q_2 - q_3 q_1)) \f$
 *              - Yaw   (z-axis rotation): \f$ \text{atan2}(2(q_0 q_3 + q_1 q_2), 1 - 2(q_2^2 + q_3^2)) \f$
 *
 * @param       q           Pointer to input quaternion [q0, q1, q2, q3] in Q15 format.
 * @param       euler       Pointer to output Euler angles [roll, pitch, yaw] in Q15 format (int16_t[3]).
 *
 * @note
 * - Input quaternion must be normalized (length ≈ 32767 in Q15).
 * - The result is returned in Q15 format, corresponding to angles in radians scaled to \f$ \pm\pi \approx \pm32767 \f$.
 * - Internally uses `q15_mul()`, `q15_atan2()` and `q15_asin()` for fixed-point trigonometry.
 *
 * @see         q15_atan2(), q15_asin(), NormalizeQuaternionQ15()
 */
void quat_to_euler_q15(const int16_t q[4], int16_t euler[3]);


/**
 * @brief       Computes the logarithm of a unit quaternion in Q15 format.
 *
 * @details     Given a unit quaternion \f$ q = [w, x, y, z] \f$, the logarithmic map is:
 *              \f$ \ln(q) = \theta \cdot \hat{v} \f$,
 *              where \f$ \theta = \arccos(w) \f$ and \f$ \hat{v} = [x, y, z] / \|[x, y, z]\| \f$.
 *              This maps the quaternion to a 3D vector in the tangent space.
 *
 *              All operations are performed in fixed-point Q15 format.
 *
 * @param[in]   q_in        Input unit quaternion [w, x, y, z] in Q15 format (int16_t[4]).
 * @param[out]  ln_out      Output logarithm vector [x, y, z] in Q15 format (int16_t[3]).
 *
 * @note
 * - The input quaternion must be normalized (i.e., unit length).
 * - If the vector part is zero, the result is [0, 0, 0].
 * - Internally uses `norm_3d_vector()`, `q15_acos()` and `q15_mul_2()`.
 *
 * @see         norm_3d_vector(), q15_acos(), q15_mul_2()
 */
void ln_q15_unit_quaternions_multiplicate_2(const int16_t *q_in, int16_t *ln_out);



void rotate_quat_sandwich_q15(const int16_t *q1, const int16_t *v_q, const int16_t *q2, int16_t *v_q_out);
void rotate_vector_Q15(const int16_t *q, const int16_t *v, int16_t *v_out);
void vector2quaternion_q15(const int16_t *v, int16_t *q);


void nLERP_quaternion_Q15(const int16_t *q1, const int16_t *q2, const int16_t beta, int16_t *q_out);
/** @} */  // end of QuaternionMathQ15 group



#endif /* INC_SYS_MATH_H_ */
