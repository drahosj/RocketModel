#ifndef __nmea_h
#define __nmea_h

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*nmea_callback)(char * str, void * ctx);

enum nmea_state {
    NMEA_STATE_WAITING,
    NMEA_STATE_RECEIVING
};

struct nmea_builder {
    int state;
    char buffer[83];
    int i;
    nmea_callback callback;
	void * callback_context;
	uint32_t completions;
	uint32_t overruns;
	uint32_t restarts;
};

#define NMEA_RINGBUF_SLOTS 8
struct nmea_ringbuf {
	_Atomic uint_fast32_t nput;
	_Atomic uint_fast32_t nget;
	char slots[NMEA_RINGBUF_SLOTS][83];
};

int nmea_ringbuf_put(struct nmea_ringbuf * rb, char * sentence);
int nmea_ringbuf_get(struct nmea_ringbuf * rb, char * sentence);
void nmea_ringbuf_init(struct nmea_ringbuf * rb);

void init_nmea_builder(struct nmea_builder * nmea);
int run_nmea_builder(struct nmea_builder * nmea, char c);

void set_nmea_builder_callback(
		struct nmea_builder * nmea,
		nmea_callback cb, 
		void * ctx);

#ifdef __cplusplus
}
#endif
#endif
