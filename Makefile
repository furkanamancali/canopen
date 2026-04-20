CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -pedantic -O2

.PHONY: test clean

test: tests_host
	./tests_host

tests_host: tests_host.c canopen.c cia402.c canopen.h cia402.h
	$(CC) $(CFLAGS) tests_host.c canopen.c cia402.c -o $@

clean:
	rm -f tests_host
