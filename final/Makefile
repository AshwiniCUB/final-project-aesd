ifeq ($(CC),)
	CC=$(CROSS_COMPILE)gcc
endif

ifeq ($(CFLAGS),)
	CFLAGS= -g -Wall -Werror
endif

ifeq ($(LDFLAGS),)
	LDFLAGS = -pthread
endif

all: final_code
default: final_code

final_code :final_code.o
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -o final_code final_code.o $(LDFLAGS)

final_code.o : final_code.c i2c_utils.h
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -c final_code.c $(LDFLAGS)

clean :
	rm -f final_code *.o
