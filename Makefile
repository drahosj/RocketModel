all: librocket.so

librocket.so: rocket_state.o
	gcc -o librocket.so -shared rocket_state.o

clean:
	rm -f *.o *.so
