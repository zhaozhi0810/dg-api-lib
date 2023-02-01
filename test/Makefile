CROSS_COMPILE = aarch64-linux-gnu-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc

SRCS = $(wildcard ./*.c)
OBJS = $(patsubst %.c,%.o,$(SRCS))

CFLAGS = -c -Wall
CFLAGS += -I./
CFLAGS += -I../include

LDFLAGS = -L../lib
LDFLAGS += -lpthread
LDFLAGS += -ldrv722

TARGET = drv722test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(LD) $(OBJS) -o $@ $(LDFLAGS)

$(OBJS): %.o:%.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf $(OBJS)
	