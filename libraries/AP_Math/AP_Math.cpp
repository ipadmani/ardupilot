#include "AP_Math.h"

#include <float.h>

#include <AP_InternalError/AP_InternalError.h>

/*
 * is_equal(): Integer implementation, provided for convenience and
 * compatibility with old code. Expands to the same as comparing the values
 * directly
 */
template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_integral<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value ,bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2)
{
    typedef typename std::common_type<Arithmetic1, Arithmetic2>::type common_type;
    return static_cast<common_type>(v_1) == static_cast<common_type>(v_2);
}

/*
 * is_equal(): double/float implementation - takes into account
 * std::numeric_limits<T>::epsilon() to return if 2 values are equal.
 */
template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_floating_point<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value, bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2)
{
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
    typedef typename std::common_type<Arithmetic1, Arithmetic2>::type common_type;
    typedef typename std::remove_cv<common_type>::type common_type_nonconst;
    if (std::is_same<double, common_type_nonconst>::value) {
        return fabs(v_1 - v_2) < std::numeric_limits<double>::epsilon();
    }
#endif
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wabsolute-value"
    // clang doesn't realise we catch the double case above and warns
    // about loss of precision here.
    return fabsf(v_1 - v_2) < std::numeric_limits<float>::epsilon();
#pragma clang diagnostic pop
}

template bool is_equal<int>(const int v_1, const int v_2);
template bool is_equal<short>(const short v_1, const short v_2);
template bool is_equal<long>(const long v_1, const long v_2);
template bool is_equal<float>(const float v_1, const float v_2);
template bool is_equal<double>(const double v_1, const double v_2);

template <typename T>
float safe_asin(const T v)
{
    const float f = static_cast<const float>(v);
    if (isnan(f)) {
        return 0.0f;
    }
    if (f >= 1.0f) {
        return static_cast<float>(M_PI_2);
    }
    if (f <= -1.0f) {
        return static_cast<float>(-M_PI_2);
    }
    return asinf(f);
}

template float safe_asin<int>(const int v);
template float safe_asin<short>(const short v);
template float safe_asin<float>(const float v);
template float safe_asin<double>(const double v);

template <typename T>
float safe_sqrt(const T v)
{
    float ret = sqrtf(static_cast<float>(v));
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

template float safe_sqrt<int>(const int v);
template float safe_sqrt<short>(const short v);
template float safe_sqrt<float>(const float v);
template float safe_sqrt<double>(const double v);

/*
  replacement for std::swap() needed for STM32
 */
static void swap_float(float &f1, float &f2)
{
    float tmp = f1;
    f1 = f2;
    f2 = tmp;
}

/*
 * linear interpolation based on a variable in a range
 */
float linear_interpolate(float low_output, float high_output,
                         float var_value,
                         float var_low, float var_high)
{
    if (var_low > var_high) {
        // support either polarity
        swap_float(var_low, var_high);
        swap_float(low_output, high_output);
    }
    if (var_value <= var_low) {
        return low_output;
    }
    if (var_value >= var_high) {
        return high_output;
    }
    float p = (var_value - var_low) / (var_high - var_low);
    return low_output + p * (high_output - low_output);
}

/* cubic "expo" curve generator
 * alpha range: [0,1] min to max expo
 * input range: [-1,1]
 */
float expo_curve(float alpha, float x)
{
    return (1.0f - alpha) * x + alpha * x * x * x;
}

/* throttle curve generator
 * thr_mid: output at mid stick
 * alpha: expo coefficient
 * thr_in: [0-1]
 */
float throttle_curve(float thr_mid, float alpha, float thr_in)
{
    float alpha2 = alpha + 1.25 * (1.0f - alpha) * (0.5f - thr_mid) / 0.5f;
    alpha2 = constrain_float(alpha2, 0.0f, 1.0f);
    float thr_out = 0.0f;
    if (thr_in < 0.5f) {
        float t = linear_interpolate(-1.0f, 0.0f, thr_in, 0.0f, 0.5f);
        thr_out = linear_interpolate(0.0f, thr_mid, expo_curve(alpha, t), -1.0f, 0.0f);
    } else {
        float t = linear_interpolate(0.0f, 1.0f, thr_in, 0.5f, 1.0f);
        thr_out = linear_interpolate(thr_mid, 1.0f, expo_curve(alpha2, t), 0.0f, 1.0f);
    }
    return thr_out;
}

template <typename T>
T wrap_180(const T angle)
{
    auto res = wrap_360(angle);
    if (res > T(180)) {
        res -= T(360);
    }
    return res;
}

template <typename T>
T wrap_180_cd(const T angle)
{
    auto res = wrap_360_cd(angle);
    if (res > T(18000)) {
        res -= T(36000);
    }
    return res;
}

template int wrap_180<int>(const int angle);
template short wrap_180<short>(const short angle);
template float wrap_180<float>(const float angle);
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
template double wrap_180<double>(const double angle);
#endif

template int wrap_180_cd<int>(const int angle);
template long wrap_180_cd<long>(const long angle);
template short wrap_180_cd<short>(const short angle);
template float wrap_180_cd<float>(const float angle);
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
template double wrap_180_cd<double>(const double angle);
#endif

float wrap_360(const float angle)
{
    float res = fmodf(angle, 360.0f);
    if (res < 0) {
        res += 360.0f;
    }
    return res;
}

#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
double wrap_360(const double angle)
{
    double res = fmod(angle, 360.0);
    if (res < 0) {
        res += 360.0;
    }
    return res;
}
#endif

int wrap_360(const int angle)
{
    int res = angle % 360;
    if (res < 0) {
        res += 360;
    }
    return res;
}

float wrap_360_cd(const float angle)
{
    float res = fmodf(angle, 36000.0f);
    if (res < 0) {
        res += 36000.0f;
    }
    return res;
}

#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
double wrap_360_cd(const double angle)
{
    double res = fmod(angle, 36000.0);
    if (res < 0) {
        res += 36000.0;
    }
    return res;
}
#endif

int wrap_360_cd(const int angle)
{
    int res = angle % 36000;
    if (res < 0) {
        res += 36000;
    }
    return res;
}

long wrap_360_cd(const long angle)
{
    long res = angle % 36000;
    if (res < 0) {
        res += 36000;
    }
    return res;
}

ftype wrap_PI(const ftype radian)
{
    ftype res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}

ftype wrap_2PI(const ftype radian)
{
    ftype res = fmodF(radian, M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

template <typename T>
T constrain_value_line(const T amt, const T low, const T high, uint32_t line)
{
    // the check for NaN as a float prevents propagation of floating point
    // errors through any function that uses constrain_value(). The normal
    // float semantics already handle -Inf and +Inf
    if (isnan(amt)) {
        AP::internalerror().error(AP_InternalError::error_t::constraining_nan, line);
        return (low + high) / 2;
    }

    if (amt < low) {
        return low;
    }

    if (amt > high) {
        return high;
    }

    return amt;
}

template float constrain_value_line<float>(const float amt, const float low, const float high, uint32_t line);
template double constrain_value_line<double>(const double amt, const double low, const double high, uint32_t line);

template <typename T>
T constrain_value(const T amt, const T low, const T high)
{
    // the check for NaN as a float prevents propagation of floating point
    // errors through any function that uses constrain_value(). The normal
    // float semantics already handle -Inf and +Inf
    if (std::is_floating_point<T>::value) {
        if (isnan(amt)) {
            INTERNAL_ERROR(AP_InternalError::error_t::constraining_nan);
            return (low + high) / 2;
        }
    }

    if (amt < low) {
        return low;
    }

    if (amt > high) {
        return high;
    }

    return amt;
}

template int constrain_value<int>(const int amt, const int low, const int high);
template long constrain_value<long>(const long amt, const long low, const long high);
template long long constrain_value<long long>(const long long amt, const long long low, const long long high);
template short constrain_value<short>(const short amt, const short low, const short high);
template float constrain_value<float>(const float amt, const float low, const float high);
template double constrain_value<double>(const double amt, const double low, const double high);


/*
  simple 16 bit random number generator
 */
uint16_t get_random16(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 0xFFFFu) + (m_z >> 16);
    m_w = 18000 * (m_w & 0xFFFFu) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xFFFF;
}


#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// generate a random float between -1 and 1, for use in SITL
float rand_float(void)
{
    return ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
}

Vector3f rand_vec3f(void)
{
    Vector3f v = Vector3f(rand_float(),
                          rand_float(),
                          rand_float());
    if (!is_zero(v.length())) {
        v.normalize();
    }
    return v;
}
#endif

/*
  return true if two rotations are equivalent
  This copes with the fact that we have some duplicates, like ROLL_180_YAW_90 and PITCH_180_YAW_270
 */
bool rotation_equal(enum Rotation r1, enum Rotation r2)
{
    if (r1 == r2) {
        return true;
    }
    Vector3f v(1,2,3);
    Vector3f v1 = v;
    Vector3f v2 = v;
    v1.rotate(r1);
    v2.rotate(r2);
    return (v1 - v2).length() < 0.001;
}

/*
 * return a velocity correction (in m/s in NED) for a sensor's position given it's position offsets
 * this correction should be added to the sensor NED measurement
 * sensor_offset_bf is in meters in body frame (Foward, Right, Down)
 * rot_ef_to_bf is a rotation matrix to rotate from earth-frame (NED) to body frame
 * angular_rate is rad/sec
 */
Vector3F get_vel_correction_for_sensor_offset(const Vector3F &sensor_offset_bf, const Matrix3F &rot_ef_to_bf, const Vector3F &angular_rate)
{
    if (sensor_offset_bf.is_zero()) {
        return Vector3F();
    }

    // correct velocity
    const Vector3F vel_offset_body = angular_rate % sensor_offset_bf;
    return rot_ef_to_bf.mul_transpose(vel_offset_body) * -1.0;
}

/*
  calculate a low pass filter alpha value
 */
float calc_lowpass_alpha_dt(float dt, float cutoff_freq)
{
    if (dt <= 0.0f || cutoff_freq <= 0.0f) {
        return 1.0;
    }
    float rc = 1.0f/(M_2PI*cutoff_freq);
    return constrain_float(dt/(dt+rc), 0.0f, 1.0f);
}

#ifndef AP_MATH_FILL_NANF_USE_MEMCPY
#define AP_MATH_FILL_NANF_USE_MEMCPY (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// fill an array of float with NaN, used to invalidate memory in SITL
void fill_nanf(float *f, uint16_t count)
{
#if AP_MATH_FILL_NANF_USE_MEMCPY
    static bool created;
    static float many_nanfs[2048];
    if (!created) {
        for (uint16_t i=0; i<ARRAY_SIZE(many_nanfs); i++) {
            created = true;
            many_nanfs[i] = std::numeric_limits<float>::signaling_NaN();
        }
    }
    if (count > ARRAY_SIZE(many_nanfs)) {
        AP_HAL::panic("Too big an area to fill");
    }
    memcpy(f, many_nanfs, count*sizeof(many_nanfs[0]));
#else
    const float n = std::numeric_limits<float>::signaling_NaN();
    while (count--) {
        *f++ = n;
    }
#endif
}

void fill_nanf(double *f, uint16_t count)
{
#if AP_MATH_FILL_NANF_USE_MEMCPY
    static bool created;
    static double many_nanfs[2048];
    if (!created) {
        for (uint16_t i=0; i<ARRAY_SIZE(many_nanfs); i++) {
            created = true;
            many_nanfs[i] = std::numeric_limits<double>::signaling_NaN();
        }
    }
    if (count > ARRAY_SIZE(many_nanfs)) {
        AP_HAL::panic("Too big an area to fill");
    }
    memcpy(f, many_nanfs, count*sizeof(many_nanfs[0]));
#else
    while (count--) {
        *f++ = std::numeric_limits<double>::signaling_NaN();
    }
#endif
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL

// Convert 16-bit fixed-point to float
float fixed2float(const uint16_t input, const uint8_t fractional_bits)
{
    return ((float)input / (float)(1U << fractional_bits));
}

// Convert float to 16-bit fixed-point
uint16_t float2fixed(const float input, const uint8_t fractional_bits)
{
    return (uint16_t)(roundf(input * (1U << fractional_bits)));
}

/*
  calculate turn rate in deg/sec given a bank angle and airspeed for a
  fixed wing aircraft
 */
float fixedwing_turn_rate(float bank_angle_deg, float airspeed)
{
    bank_angle_deg = constrain_float(bank_angle_deg, -80, 80);
    return degrees(GRAVITY_MSS*tanf(radians(bank_angle_deg))/MAX(airspeed,1));
}

// convert degrees farenheight to Kelvin
float degF_to_Kelvin(float temp_f)
{
    return (temp_f + 459.67) * 0.55556;
}


float freq_msine[] = {0.077777778,0.072222222,0.05,0.066666667,0.055555556,0.061111111,0.083333333,0.088888889,0.094444444,0.1,0.105555556,0.111111111,0.116666667,0.122222222,0.127777778,0.133333333,0.138888889,0.144444444,
0.177777778,0.172222222,0.15,0.166666667,0.155555556,0.161111111,0.183333333,0.188888889,0.194444444,0.2,0.205555556,0.211111111,0.216666667,0.222222222,0.227777778,0.233333333,0.238888889,0.244444444,
0.277777778,0.272222222,0.25,0.266666667,0.255555556,0.261111111,0.283333333,0.288888889,0.294444444,0.3,0.305555556,0.311111111,0.316666667,0.322222222,0.327777778,0.333333333,0.338888889,0.344444444,
0.377777778,0.372222222,0.35,0.366666667,0.355555556,0.361111111,0.383333333,0.388888889,0.394444444,0.4,0.405555556,0.411111111,0.416666667,0.422222222,0.427777778,0.433333333,0.438888889,0.444444444,
0.477777778,0.472222222,0.45,0.466666667,0.455555556,0.461111111,0.483333333,0.488888889,0.494444444,0.5,0.505555556,0.511111111,0.516666667,0.522222222,0.527777778,0.533333333,0.538888889,0.544444444,
0.577777778,0.572222222,0.55,0.566666667,0.555555556,0.561111111,0.583333333,0.588888889,0.594444444,0.6,0.605555556,0.611111111,0.616666667,0.622222222,0.627777778,0.633333333,0.638888889,0.644444444,
0.677777778,0.672222222,0.65,0.666666667,0.655555556,0.661111111,0.683333333,0.688888889,0.694444444,0.7,0.705555556,0.711111111,0.716666667,0.722222222,0.727777778,0.733333333,0.738888889,0.744444444,
0.777777778,0.772222222,0.75,0.766666667,0.755555556,0.761111111,0.783333333,0.788888889,0.794444444,0.8,0.805555556,0.811111111,0.816666667,0.822222222,0.827777778,0.833333333,0.838888889,0.844444444,
0.877777778,0.872222222,0.85,0.866666667,0.855555556,0.861111111,0.883333333,0.888888889,0.894444444,0.9,0.905555556,0.911111111,0.916666667,0.922222222,0.927777778,0.933333333,0.938888889,0.944444444,
0.977777778,0.972222222,0.95,0.966666667,0.955555556,0.961111111,0.983333333,0.988888889,0.994444444,1,1.005555556,1.011111111,1.016666667,1.022222222,1.027777778,1.033333333,1.038888889,1.044444444,
1.077777778,1.072222222,1.05,1.066666667,1.055555556,1.061111111,1.083333333,1.088888889,1.094444444,1.1,1.105555556,1.111111111,1.116666667,1.122222222,1.127777778,1.133333333,1.138888889,1.144444444,
1.177777778,1.172222222,1.15,1.166666667,1.155555556,1.161111111,1.183333333,1.188888889,1.194444444,1.2,1.205555556,1.211111111,1.216666667,1.222222222,1.227777778,1.233333333,1.238888889,1.244444444,
1.277777778,1.272222222,1.25,1.266666667,1.255555556,1.261111111,1.283333333,1.288888889,1.294444444,1.3,1.305555556,1.311111111,1.316666667,1.322222222,1.327777778,1.333333333,1.338888889,1.344444444,
1.377777778,1.372222222,1.35,1.366666667,1.355555556,1.361111111,1.383333333,1.388888889,1.394444444,1.4,1.405555556,1.411111111,1.416666667,1.422222222,1.427777778,1.433333333,1.438888889,1.444444444,
1.477777778,1.472222222,1.45,1.466666667,1.455555556,1.461111111,0,0,0,0,1.483333333,1.488888889,1.494444444,1.5,1.505555556,1.511111111,1.516666667,1.522222222,
1.555555556,1.55,1.527777778,1.544444444,1.533333333,1.538888889,0,0,0,0,1.561111111,1.566666667,1.572222222,1.577777778,1.583333333,1.588888889,1.594444444,1.6,
1.633333333,1.627777778,1.605555556,1.622222222,1.611111111,1.616666667,0,0,0,0,1.638888889,1.644444444,1.65,1.655555556,1.661111111,1.666666667,1.672222222,1.677777778,
1.711111111,1.705555556,1.683333333,1.7,1.688888889,1.694444444,0,0,0,0,1.716666667,1.722222222,1.727777778,1.733333333,1.738888889,1.744444444,1.75,1.755555556,
1.788888889,1.783333333,1.761111111,1.777777778,1.766666667,1.772222222,0,0,0,0,1.794444444,1.8,1.805555556,1.811111111,1.816666667,1.822222222,1.827777778,1.833333333
};


float ph_msine[] = {0.391343246,-1.125313627,2.892947723,-1.6556454,-0.118312441,0.644567341,-0.59020296,-0.281174093,-3.024682014,-1.432657054,2.695816705,0.373976714,-1.419652848,0.327691779,-1.3551188,-1.098811683,-2.836659307,-2.006203691,
-2.613310393,1.080178503,-1.834023488,-2.797848589,-1.321631672,2.825007927,-0.561044297,-1.95465509,1.675521173,-2.326625644,1.015541547,0.506303652,1.920467178,2.053797632,0.751276253,1.386606887,0.076720902,0.204139372,
3.108833158,-0.338472019,-1.677128834,2.417696355,2.048750738,2.34282916,-2.16905264,2.666946363,-0.134900119,-2.231160814,0.031160923,1.631265631,-2.80895197,-0.550734307,0.913411957,0.595183903,-0.156467662,0.5603953,
-1.211800066,0.982465803,1.208255313,0.928840754,2.200073169,-2.903471731,-1.264531051,1.051027489,-1.408447259,2.947812717,-2.61288763,-0.803278423,2.46417841,-0.332716814,2.554633429,1.238216244,0.298989236,2.288713663,
3.128576497,-0.140778988,-2.331086167,-1.398635624,-0.146563911,2.797258669,1.451167872,-1.530900131,-1.840144593,1.358383576,-1.544390439,-2.848830697,-2.934112326,2.756181367,1.071368712,-2.76463853,-3.056683662,-1.745245367,
1.173334941,0.341290678,2.86762146,2.887858545,3.139085294,0.539051945,-1.359657863,1.044690258,-0.575065633,0.675628984,1.29507152,2.485689252,1.641612477,-2.323421378,-2.252515221,2.284683443,3.114578637,-2.604264169,
-2.968770047,2.467639696,-2.899098419,-2.021390868,1.103377604,-3.098955721,1.793645924,-2.505824562,-3.137424776,-2.743726806,0.912596442,-2.342772408,1.653686947,1.971734147,2.131491943,-0.510172614,1.560769701,2.437255131,
-1.824569471,-2.169969591,1.417353736,-2.388279278,-2.813993415,0.898032995,0.43666459,-2.654702265,-0.083746505,-0.194092367,-2.615547848,-1.994294152,-2.8471649,-1.043571117,0.110972158,1.375795696,-0.349024415,-3.019878124,
1.921268347,-1.52396817,0.532765156,-0.643625016,-1.848492875,-0.581149316,0.431089549,-0.337500409,1.076819345,-0.073806498,-0.211243924,1.777059749,0.90342137,2.573340842,-0.382927905,-1.841090411,1.355568846,0.692752917,
2.942272773,0.533315235,1.299304558,1.757042281,-0.966837197,-1.460785437,1.728887872,1.209385484,2.330080726,-2.804595887,-1.048035647,2.109339474,-1.686001586,2.083825907,-2.298033405,-2.101710805,-1.949803575,-2.637380233,
-2.639444952,-2.26118047,-2.389828751,0.423197661,-2.915449382,-2.693889579,2.598545968,-0.316112157,-1.552738122,-0.198074965,-0.686057225,-1.321036982,-2.267767469,3.063611772,2.84228276,2.380337664,-0.272518701,0.738348921,
-2.778285629,2.030329707,2.61459737,1.754068737,-1.270776723,0.110435186,-2.343304588,0.66904582,0.566017374,0.772555457,0.280871314,3.024432199,-2.628234166,-2.66176905,-2.489782828,-2.397545657,-0.226975015,0.459717666,
2.81214091,0.913000765,3.04763128,2.775430155,2.628581765,0.819194201,0.693774599,-0.164091456,-0.303865058,0.236038132,-1.208447938,-1.694783444,0.325065728,0.28872029,0.777255933,2.647614868,-1.893639388,-0.888941002,
-0.151929344,-0.970302972,-1.652686864,0.012014648,2.799250961,-2.109525837,-0.400914457,0.33326598,1.487843729,1.846497738,1.453953949,1.720153077,0.518724686,-0.999075187,2.706776465,1.563884251,-0.394540293,1.887785604,
0.386529389,-1.380244891,-2.707137255,1.910716869,-2.504117647,-0.620173641,0,0,0,0,-2.487558351,-2.401394082,2.40608376,2.802659939,-2.173336985,-0.737438804,0.400659374,0.831983452,
-0.231518182,0.934514365,2.859694518,-2.95670992,3.047337429,3.099166132,0,0,0,0,-2.906564179,2.626046538,-1.462696987,-2.91694498,3.041780765,-1.690769529,2.938947541,-0.587540556,
-1.978812101,-1.437525524,1.573684535,-1.977732281,0.435814346,-1.71407041,0,0,0,0,2.337339226,-2.370738297,0.099234608,-1.927515531,0.163580995,-2.120584483,-1.288571896,3.061517517,
1.462318594,0.443529253,2.925007184,1.463799227,2.316426636,-2.66211408,0,0,0,0,-0.585102648,-1.961737705,-1.518964974,1.145653728,-2.842407976,-0.005867231,2.301730659,-1.467369569,
2.579405716,0.734256146,-1.355341887,-0.915887778,2.681127907,1.599732917,0,0,0,0,1.424889165,1.506499881,3.01763669,-0.660981055,-1.901076747,2.748106126,1.095845501,-0.331405312,
};


float pwr_msine[] = {0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.071428571,0.071428571,0.071428571,0.071428571,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0,0,0,0,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0,0,0,0,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0,0,0,0,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0,0,0,0,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0,0,0,0,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,0.052631579,
};


float msine(int i, float t) //generate multisine signal
{
    float ret = 0.0f;
    float time = (float) t;
    time = time/1000.0f;
    for (uint8_t j=0; j<19; j++) {
        //ret += 125.0f*sinf(2.0f*3.141592f*freq[j]*time+ph[j]) ;
        ret += sqrtf(pwr_msine[i + j*18])*sinf(2.0f*3.141592f*freq_msine[i +j*18]*time+ph_msine[i + j*18]) ;
    }
    return ret;
}



