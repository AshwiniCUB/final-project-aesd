CFLAGS = -g -Wall -Wextra -pedantic -std=c11 -Wno-implicit-function-declaration
LDFLAGS = -lrt

TARGET = bme280
SRCS = bme280.c

all: $(TARGET)

$(TARGET): $(SRCS)
	gcc $(CFLAGS) -o $(TARGET) $(SRCS) $(LDFLAGS)

clean:
	rm -f $(TARGET) *.o

