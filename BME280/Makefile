bme280:	bme280.o 
	gcc -Wall $^ -o $@ -li2c

%.o:	%.c %.h
	gcc -Wall -O2 -c $< -o $@

clean:
	rm -f *.o bme280


