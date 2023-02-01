CROSS_COMPILE = aarch64-linux-gnu-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc


lcd_pwm_set: main.o uart.o
	$(CC) $^ -o $@ 

%.o:%.c
	$(CC) $< -c


clean:
	rm lcd_pwm_set
