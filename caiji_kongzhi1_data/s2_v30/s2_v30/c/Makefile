CC ?= cc
CFLAGS = -O2 -std=c99 -Wall -Wextra -Werror -ffp-contract=off
all: ../build/example ../build/libs2.so
../build:
	mkdir -p $@
../build/example: s2_deploy.c s2_deploy.h main.c | ../build
	$(CC) $(CFLAGS) s2_deploy.c main.c -lm -o $@
../build/libs2.so: s2_deploy.c s2_deploy.h | ../build
	$(CC) $(CFLAGS) -fPIC -shared s2_deploy.c -lm -o $@
