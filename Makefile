all: librocket.so

export CFLAGS=-Wall -Werror -fPIC -g

librocket.so: rocket_state.o nmea.o
	gcc -o librocket.so -shared rocket_state.o nmea.o

clean:
	rm -f *.o *.so
