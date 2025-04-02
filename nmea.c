#include "nmea.h"
#include <string.h>
#include <assert.h>
#include <stdlib.h>

void init_nmea_builder(struct nmea_builder * nmea)
{
	memset(nmea, 0, sizeof(struct nmea_builder));
	nmea->state = NMEA_STATE_WAITING;
}

void set_nmea_builder_callback(
		struct nmea_builder * nmea,
		nmea_callback cb, 
		void * ctx)
{
      nmea->callback = cb;
      nmea->callback_context = ctx;
}

int run_nmea_builder(struct nmea_builder * nmea, char c)
{
	if (nmea->state == NMEA_STATE_WAITING) {
		if ((c == '$') || (c == '!')) {
			nmea->state = NMEA_STATE_RECEIVING;
                  nmea->buffer[0] = c;
                  nmea->i = 1;
		}
	} else if (nmea->state == NMEA_STATE_RECEIVING) {
            nmea->buffer[nmea->i] = c;
            nmea->i++;
            if (c == '\n') {
                  nmea->completions++;
                  nmea->buffer[nmea->i] = '\0';     
                  if (nmea->callback) {
                        nmea->callback(nmea->buffer, nmea->callback_context);
                  }
                  nmea->state = NMEA_STATE_WAITING;
                  return 1;
            } else if ((c == '!') || (c == '$')) {
                  nmea->restarts++;
                  nmea->buffer[0] = c;
                  nmea->i = 1;
            } else if (nmea->i > 82) {
                  nmea->overruns++;
                  nmea->state = NMEA_STATE_WAITING;
            }
      }
      return 0;
}

void nmea_ringbuf_init(struct nmea_ringbuf * rb)
{
      memset(rb, 0, sizeof(struct nmea_ringbuf));
}

int nmea_ringbuf_put(struct nmea_ringbuf * rb, char * sentence)
{
      int i = (rb->nput) % NMEA_RINGBUF_SLOTS;

      /* Full buffer case */
      if ((i == (rb->nget % NMEA_RINGBUF_SLOTS)) && (rb->nput != rb->nget)) {
            return 0;
      }

      assert(strlen(sentence) < 83);
      strcpy(rb->slots[i], sentence);
      rb->nput++;
      return 1;
}

int nmea_ringbuf_get(struct nmea_ringbuf * rb, char * sentence)
{
      if (rb->nget == rb->nput) {
            return 0;
      }
      strcpy(sentence, rb->slots[rb->nget % NMEA_RINGBUF_SLOTS]);
      rb->nget++;
      return 1;
}
