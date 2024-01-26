build:
	gcc src/*.c -std=c99 -lmodbus -lws2_32 -llua -o main
run:
	./main