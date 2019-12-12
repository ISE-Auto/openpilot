// trimmed down version of safety.h that will run on a STM32F205 board

struct sample_t {
  int values[6];
  int min;
  int max;
} sample_t_default = {{0}, 0, 0};


// safety code requires floats
struct lookup_t {
  float x[3];
  float y[3];
};

// limits
const int TRQ_MAX = 2000;
// rate based torque limit + stay within actually applied
// packet is sent at 100hz, so this limit is 1000/sec
const int MAX_RATE = 150;        // ramp up
const int MAX_TORQUE_ERROR = 400;  // max torque cmd in excess of torque motor

// real time torque limit to prevent controls spamming
// the real time limit is 1500/sec
const int MAX_RT_DELTA = 400;      // max delta torque allowed for real time checks
const uint32_t RT_INTERVAL = 250000;    // 250ms between real time checks


// states
int desired_torque_last_1 = 0;       // last desired steer torque for DAC0
int desired_torque_last_2 = 0;       // last desired steer torque for DAC1
int rt_torque_last_1 = 0;            // last desired torque for real time check
int rt_torque_last_2 = 0;            // last desired torque for real time check
uint32_t ts_last = 0;
struct sample_t torque_meas_1;
struct sample_t torque_meas_2;

// compute the time elapsed (in microseconds) from 2 counter samples
// case where ts < ts_last is ok: overflow is properly re-casted into uint32_t
uint32_t get_ts_elapsed(uint32_t ts, uint32_t ts_last) {
  return ts - ts_last;
}

// convert a trimmed integer to signed 32 bit int
int to_signed(int d, int bits) {
  int d_signed = d;
  if (d >= (1 << MAX((bits - 1), 0))) {
    d_signed = d - (1 << MAX(bits, 0));
  }
  return d_signed;
}

// given a new sample, update the smaple_t struct
void update_sample(struct sample_t *sample, int sample_new) {
  int sample_size = sizeof(sample->values) / sizeof(sample->values[0]);
  for (int i = sample_size - 1; i > 0; i--) {
    sample->values[i] = sample->values[i-1];
  }
  sample->values[0] = sample_new;

  // get the minimum and maximum measured samples
  sample->min = sample->values[0];
  sample->max = sample->values[0];
  for (int i = 1; i < sample_size; i++) {
    if (sample->values[i] < sample->min) {
      sample->min = sample->values[i];
    }
    if (sample->values[i] > sample->max) {
      sample->max = sample->values[i];
    }
  }
}

bool max_limit_check(int val, const int MAX_VAL, const int MIN_VAL) {
  return (val > MAX_VAL) || (val < MIN_VAL);
}

// check that commanded value isn't too far from measured
// bool dist_to_meas_check(int val, int val_last, struct sample_t *val_meas,
//   const int MAX_RATE_UP, const int MAX_RATE_DOWN, const int MAX_ERROR) {

//   // *** val rate limit check ***
//   int highest_allowed_rl = MAX(val_last, 0) + MAX_RATE_UP;
//   int lowest_allowed_rl = MIN(val_last, 0) - MAX_RATE_UP;

//   // if we've exceeded the meas val, we must start moving toward 0
//   int highest_allowed = MIN(highest_allowed_rl, MAX(val_last - MAX_RATE_DOWN, MAX(val_meas->max, 0) + MAX_ERROR));
//   int lowest_allowed = MAX(lowest_allowed_rl, MIN(val_last + MAX_RATE_DOWN, MIN(val_meas->min, 0) - MAX_ERROR));

//   // check for violation
//   return (val < lowest_allowed) || (val > highest_allowed);
// }

// check that commanded value isn't fighting against driver
bool driver_limit_check(int val, int val_last, struct sample_t *val_driver,
  const int MAX_VAL, const int MAX_RATE_UP, const int MAX_RATE_DOWN,
  const int MAX_ALLOWANCE, const int DRIVER_FACTOR) {

  int highest_allowed_rl = MAX(val_last, 0) + MAX_RATE_UP;
  int lowest_allowed_rl = MIN(val_last, 0) - MAX_RATE_UP;

  int driver_max_limit = MAX_VAL + (MAX_ALLOWANCE + val_driver->max) * DRIVER_FACTOR;
  int driver_min_limit = -MAX_VAL + (-MAX_ALLOWANCE + val_driver->min) * DRIVER_FACTOR;

  // if we've exceeded the applied torque, we must start moving toward 0
  int highest_allowed = MIN(highest_allowed_rl, MAX(val_last - MAX_RATE_DOWN,
                                             MAX(driver_max_limit, 0)));
  int lowest_allowed = MAX(lowest_allowed_rl, MIN(val_last + MAX_RATE_DOWN,
                                           MIN(driver_min_limit, 0)));

  // check for violation
  return (val < lowest_allowed) || (val > highest_allowed);
}


// real time check, mainly used for steer torque rate limiter
bool rt_rate_limit_check(int val, int val_last, const int MAX_RT_DELTA) {

  // *** torque real time rate limit check ***
  int highest_val = MAX(val_last, 0) + MAX_RT_DELTA;
  int lowest_val = MIN(val_last, 0) - MAX_RT_DELTA;

  // check for violation
  return (val < lowest_val) || (val > highest_val);
}


// // interp function that holds extreme values
// float interpolate(struct lookup_t xy, float x) {

//   int size = sizeof(xy.x) / sizeof(xy.x[0]);
//   float ret = xy.y[size - 1];  // default output is last point

//   // x is lower than the first point in the x array. Return the first point
//   if (x <= xy.x[0]) {
//     ret = xy.y[0];

//   } else {
//     // find the index such that (xy.x[i] <= x < xy.x[i+1]) and linearly interp
//     for (int i=0; i < (size - 1); i++) {
//       if (x < xy.x[i+1]) {
//         float x0 = xy.x[i];
//         float y0 = xy.y[i];
//         float dx = xy.x[i+1] - x0;
//         float dy = xy.y[i+1] - y0;
//         // dx should not be zero as xy.x is supposed ot be monotonic
//         if (dx <= 0.) {
//           dx = 0.0001;
//         }
//         ret = (dy * (x - x0) / dx) + y0;
//         break;
//       }
//     }
//   }
//   return ret;
// }
