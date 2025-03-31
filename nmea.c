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

int nmea_parse(char * sentence, 
            uint32_t * lat, uint32_t * lon, uint32_t * time, int32_t * alt)
{
      /* TODO checksum validation */
      char * tok;
      tok = strtok(sentence, ",");

      if (strcmp("$GPGGA", tok)) {
            return -2;
      }

      char tmp[12];

      /* Time */
      tok = strtok(NULL, ",");

      /* HH */
      bzero(tmp, sizeof(tmp));
      strncpy(tmp, tok, 2);
      *time = atoi(tmp) * 60 * 60 * 1000;

      /* MM */
      bzero(tmp, sizeof(tmp));
      strncpy(tmp, tok + 2, 2);
      *time += atoi(tmp) * 60 * 1000;

      /* SS */
      bzero(tmp, sizeof(tmp));
      strncpy(tmp, tok + 4, 2);
      *time += atoi(tmp) * 1000;

      /* ms if present */
      if (strlen(tok) >= 8) {
            bzero(tmp, sizeof(tmp));
            strncpy(tmp, tok + 7, 3);
            int f = atoi(tmp);
            for (int i = 0; i < (3 - strlen(tmp)); i++) {
                  f = f * 10;
            }
            *time += f;
      }

      /* Lat */
      tok = strtok(NULL, ",");
      assert(tok != NULL);

      /* DD */
      bzero(tmp, sizeof(tmp));
      strncpy(tmp, tok, 2);
      *lat = atoi(tmp) * 60 * 100000;

      /* MM */
      bzero(tmp, sizeof(tmp));
      strncpy(tmp, tok + 2, 2);
      *lat += atoi(tmp) * 100000;

      /* .mm if present */
      if (strlen(tok) >= 6) {
            bzero(tmp, sizeof(tmp));
            strncpy(tmp, tok + 5, 5);
            int f = atoi(tmp);
            for (int i = 0; i < 5 - strlen(tmp); i++) {
                  f = f * 10;
            }
            *lat += f;
      }

      /* Name */
      tok = strtok(NULL, ",");
      assert(tok != NULL);
      if (tok[0] == 'S') {
            *lat = *lat * -1;
      } else {
            assert(tok[0] == 'N');
      }

      /* Lon */
      tok = strtok(NULL, ",");
      assert(tok != NULL);

      /* DDD */
      bzero(tmp, sizeof(tmp));
      strncpy(tmp, tok, 3);
      *lon = atoi(tmp) * 60 * 100000;

      /* MM */
      bzero(tmp, sizeof(tmp));
      strncpy(tmp, tok + 3, 2);
      *lon += atoi(tmp) * 100000;

      /* .mm if present */
      if (strlen(tok) >= 7) {
            bzero(tmp, sizeof(tmp));
            strncpy(tmp, tok + 6, 5);
            int f = atoi(tmp);
            for (int i = 0; i < (5 - strlen(tmp)); i++) {
                  f = f * 10;
            }
            *lon += f;
      }

      /* Name */
      tok = strtok(NULL, ",");
      assert(tok != NULL);
      if (tok[0] == 'W') {
            *lon = *lon * -1;
      } else {
            assert(tok[0] == 'E');
      }

      /* Quality */
      assert(strtok(NULL, ",") != NULL);

      /* Nsat */
      assert(strtok(NULL, ",") != NULL);

      /* HDOP */
      assert(strtok(NULL, ",") != NULL);

      /* Alt */
      tok = strtok(NULL, ",");
      *alt = 0;
      assert(tok != NULL);
      char * fpart = strstr(tok, ".");
      if ((fpart != NULL) && (strlen(fpart) >= 2)) {
            bzero(tmp, sizeof(tmp));
            strncpy(tmp, fpart + 1, 3);
            int f = atoi(tmp);
            for (int i = 0; i < (3 - strlen(tmp)); i++) {
                  f = f * 10;
            }
            *alt += f;
            fpart[0] = '\0';
      }

      /* non-fractional */
      bzero(tmp, sizeof(tmp));
      strncpy(tmp, tok, 10);
      *alt += atoi(tmp) * 1000;

      /* Units */
      tok = strtok(NULL, ",");
      assert(tok != NULL);
      if (tok[0] == 'F') {
            *alt = (*alt * 10) / 33;
      } else {
            assert(tok[0] == 'M');
      }

      /* Geoidal sep */
      int32_t geoidal = 0;
      tok = strtok(NULL, ",");
      assert(tok != NULL);
      fpart = strstr(tok, ".");
      if ((fpart != NULL) && (strlen(fpart) >= 2)) {
            bzero(tmp, sizeof(tmp));
            strncpy(tmp, fpart + 1, 3);
            int f = atoi(tmp);
            for (int i = 0; i < 3 - strlen(tmp); i++) {
                  f = f * 10;
            }
            geoidal = f;
            fpart[0] = '\0';
      }

      /* non-fractional */
      bzero(tmp, sizeof(tmp));
      strncpy(tmp, tok, 10);
      geoidal += atoi(tmp) * 1000;

      /* Units */
      tok = strtok(NULL, ",");
      assert(tok != NULL);
      if (tok[0] == 'F') {
            geoidal = (geoidal * 10) / 33;
      } else {
            assert(tok[0] == 'M');
      }

      *alt -= geoidal;
      return 0;
}
