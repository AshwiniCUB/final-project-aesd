ifeq ($(CC),)
	CC=$(CROSS_COMPILE)gcc
endif

ifeq ($(CFLAGS),)
	CFLAGS= -g -Wall -Werror
endif

ifeq ($(LDFLAGS),)
	LDFLAGS = -pthread
endif

all: bme280
default: bme280

bme280 :bme280.o
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -o bme280 bme280.o $(LDFLAGS)

bme280.o : bme280.c i2c_utils.h
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -c bme280.c $(LDFLAGS)

clean :
	rm -f bme280 *.o
