
all: test


test: test.c motor-dshot.c 
	gcc -O2 -S -o motor-dshot.s motor-dshot.c
	gcc -O2 -Wall -o test test.c motor-dshot.c



