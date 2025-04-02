#ifndef _rocket_state_h
#define _rocket_state_h

#include <stdint.h>

enum phase {
	ROCKET_PHASE_IDLE,
	ROCKET_PHASE_ARMED,
	ROCKET_PHASE_ASCENT,
	ROCKET_PHASE_DESCENT,
	ROCKET_PHASE_LANDED
};

struct interpolator {
	uint32_t period;

	uint64_t next_tick;

	int32_t real_last;
	uint64_t real_last_tick;

	int32_t current;
	uint64_t current_tick;

	int valid;
};

/* Coefficients are 8.24 fixed point
 *
 * Taps internally are 28.4 signed 
 *
 * Max value ~ 134_217_728
 *
 */
struct fir_filter {
	int ntaps;
	int valid;
	int32_t c[100];
	int32_t taps[100];
};

struct gps_data {
    /* Format: Fixed-point 9.23
     *
	 */
	int32_t lat;
	int32_t lon;

	/* mm */
	int32_t alt;
	int32_t alt_max;

	/* Time of UTC day milliseconds  */
	uint32_t time;

	uint64_t tick;

	int valid;
};

struct baro_data {
	/* mm */
	int32_t alt;
	int32_t alt2;
	int32_t alt_max;
	int32_t alt_raw;
	int32_t alt_raw_max;
	int32_t alt_field;

	/* monotonic ms */
	uint64_t tick;

	/* mm/s */
	int32_t vspeed;
	int32_t vspeed_max;

	/* for landing detection */
	uint64_t landing_tick;

	/* Processing */
	struct interpolator alt_interp;
	struct fir_filter alt_filter;
	struct interpolator field_alt_interp;
	struct fir_filter field_alt_filter;
	struct fir_filter apogee_detect_filter;

	int valid;
};

struct rocket_state {
	int phase;

	struct gps_data gps;
	struct baro_data baro;
};

/* Update baro. Returns phase of flight */
int update_baro(struct rocket_state * state, int32_t alt, uint64_t tick);

void init_state(struct rocket_state * state);

void init_interp(struct interpolator * interp, int period);
void feed_interp(struct interpolator * interp, int32_t sample, uint64_t tick);
int get_interp(struct interpolator * interp, int32_t * sample, uint64_t * tick);

void init_fir_filter(struct fir_filter *, int ntaps);
int32_t run_fir_filter(struct fir_filter * filter, int32_t sample);

int32_t gps_deg_to_fixed(int ipart, int fpart, int fbase);
int gps_fixed_to_deg_ipart(int32_t fixed);
int gps_fixed_to_deg_fpart(int32_t fixed, int fbase);
#endif
