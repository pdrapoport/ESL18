
CC=gcc
CFLAGS = -g -Wall -lm
EXEC = ./pc-terminal

all:
	$(CC) $(CFLAGS) pc_terminal.c msgprocess.c crc16.c -o $(EXEC)

run: all
	$(EXEC)