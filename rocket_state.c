#include "rocket_state.h"
#include <string.h>
#include <assert.h>

#define TICK_RATE 1000
#define SAMPLE_RATE 40

void init_state(struct rocket_state * state)
{
      memset(state, 0, sizeof(struct rocket_state));
      init_interp(&state->baro.alt_interp, TICK_RATE/SAMPLE_RATE);
      init_interp(&state->baro.field_alt_interp, TICK_RATE/2);
      init_fir_filter(&state->baro.alt_filter, 20);
      init_fir_filter(&state->baro.field_alt_filter, 60);
      init_fir_filter(&state->baro.apogee_detect_filter, 80);
}

int update_baro(struct rocket_state * state, int32_t raw_alt, uint64_t tick)
{
      int phase = state->phase;

      /* Init case */
      if (!state->baro.valid) {
            state->baro.valid = 1;
            state->baro.alt = raw_alt;
            state->baro.alt2 = raw_alt;
            state->baro.alt_raw = raw_alt;
            state->baro.alt_field = raw_alt;
            state->baro.alt_max = raw_alt;
            state->baro.tick = tick;
            phase = ROCKET_PHASE_IDLE;
      }

      /* Signal processing */
      feed_interp(&state->baro.alt_interp, raw_alt, tick);

      /* Advance filters */
      int32_t alt = state->baro.alt;
      int32_t alt2 = state->baro.alt2;
      int32_t alt_i;
      uint64_t tick_i;
      while (get_interp(&state->baro.alt_interp, &alt_i, &tick_i)) {
            int32_t old_alt = alt;
            alt = run_fir_filter(&state->baro.alt_filter, alt_i);
            state->baro.vspeed = (alt - old_alt) * SAMPLE_RATE;

            alt2 = run_fir_filter(&state->baro.apogee_detect_filter, alt);
            if (alt2 > state->baro.alt_max) {
                  state->baro.alt_max = alt2;
            }
      }

      if ((state->phase == ROCKET_PHASE_IDLE) ||
                  (state->phase == ROCKET_PHASE_ARMED)) {
            feed_interp(&state->baro.field_alt_interp, raw_alt, tick);
            while (get_interp(&state->baro.field_alt_interp, &alt_i, &tick_i)) {
                  state->baro.alt_field = 
                        run_fir_filter(&state->baro.field_alt_filter, alt_i);
            }
      }

      /* Update first-order data */
      state->baro.alt = alt;
      state->baro.alt2 = alt2;
      state->baro.alt_raw = raw_alt;
      state->baro.tick = tick;

      /* Update maximums */
      if (raw_alt > state->baro.alt_raw_max) {
            state->baro.alt_raw_max = raw_alt;
      }

      if (state->baro.vspeed > state->baro.vspeed_max) {
            state->baro.vspeed_max = state->baro.vspeed;
      }

      /* Detect phase changes */

      if (phase == ROCKET_PHASE_ARMED) {
            /* 25 m/s */
            if (state->baro.vspeed >= 25000) {
                  phase = ROCKET_PHASE_ASCENT;
            }
            /* 100m above field */
            if ((state->baro.alt - state->baro.alt_field) > 100000) {
                  phase = ROCKET_PHASE_ASCENT;
            }
      }

      if (phase == ROCKET_PHASE_ASCENT) {
            /* 10m below max alt */
            if ((state->baro.alt_max - state->baro.alt) > 10000) {
                  phase = ROCKET_PHASE_DESCENT;
            }
      }

      if (phase == ROCKET_PHASE_DESCENT) {
            /* Initialize landing tick */
            if (state->baro.landing_tick == 0) {
                  state->baro.landing_tick = tick;
            }

            /* abs(vspeed) < 100mm/s */
            if ((state->baro.vspeed < 100) && (state->baro.vspeed > -100)) {
                  /* Sustained for 5 seconds */
                  if ((tick - state->baro.landing_tick) > 5000) {
                        phase = ROCKET_PHASE_LANDED;
                  }
            } else {
                  state->baro.landing_tick = tick;
            }
      }

      return phase;
}

void init_interp(struct interpolator * interp, int period)
{
      memset(interp, 0, sizeof(struct interpolator));
      interp->period = period;
}

void feed_interp(struct interpolator * interp, int32_t sample, uint64_t tick)
{
      if (!interp->valid) {
            interp->real_last = sample;
            interp->real_last_tick = tick;
            interp->next_tick = tick;
            interp->valid = 1;
      } else {
            interp->real_last = interp->current;
            interp->real_last_tick = interp->current_tick;
      }

      interp->current = sample;
      interp->current_tick = tick;
}

int get_interp(struct interpolator * interp, int32_t * sample, uint64_t * tick)
{
      if (!interp->valid) {
            return 0;
      }

      /* Tick of next generated sample is in the future  */
      if (interp->next_tick > interp->current_tick) {
            return 0;
      }

      /* Handle special case of exactl lined up */
      if (interp->next_tick == interp->current_tick) {
            *sample = interp->current;
            *tick = interp->current_tick;
            interp->next_tick += interp->period;
            return 1;
      }

      /* Interpolate between last real and current latest */
      int32_t delta = interp->current - interp->real_last;
      int64_t tdelta = interp->current_tick - interp->real_last_tick;
      int64_t progress = interp->next_tick - interp->real_last_tick;

      /* Interpolated = last + ((progress/tdelta) * delta )
       * Rearraged to minimize loss of precision
       */

      *sample = interp->real_last + ((progress * delta) / tdelta);
      *tick = interp->next_tick;
      interp->next_tick += interp->period;

      return 1;
}

void init_fir_filter(struct fir_filter * filter, int ntaps)
{
      memset(filter, 0, sizeof(struct fir_filter));
      filter->ntaps = ntaps;
      int32_t window_tap = ((1 << 24) / ntaps);
      for (int i = 0; i < ntaps; i++) {
            filter->c[i] = window_tap;
      }
}

int32_t run_fir_filter(struct fir_filter * filter, int32_t sample)
{
      int64_t sum = 0;
      sample = sample << 4;

      if (!filter->valid) {
            for (int i = 0; i < filter->ntaps; i++) {
                  filter->taps[i] = sample;
            }
            filter->valid = 1;
      }

      for (int i = 0; i < filter->ntaps; i++) {
            int64_t c = filter->c[i];
            sum += (((int64_t) sample) * c);

            int32_t t = filter->taps[i];
            filter->taps[i] = sample;
            sample = t;
      }

      return sum >> 28;
}

int32_t gps_deg_to_fixed(int32_t ipart, int32_t fpart, int32_t fbase)
{
      int64_t ret = ipart << 23;
      assert(fpart < fbase);
      if (fbase > 10000000) {
            int scale = fbase / 10000000;
            fbase = fbase / scale;
            fpart = fpart / scale;
      }
      ret |= (((int64_t) fpart) * ((int64_t) (1 << 23))) / ((int64_t) fbase);
      return (int32_t) ret;
}

int32_t gps_fixed_to_deg_ipart(int32_t deg)
{
      return deg >> 23;
}

int32_t gps_fixed_to_deg_fpart(int32_t deg, int32_t fbase)
{
      int64_t fpart = deg & 0x7fffff;
      return (int) ((fpart * ((int64_t) fbase)) / ((int64_t) 1 << 23));
}
