CROSS_COMPILE = aarch64-linux-gnu-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc

TARGET = libdrv722.so

SRCS = $(wildcard ./common/*.c)
SRCS += $(wildcard ./mcu/*.c)
SRCS += $(wildcard ./audio/*.c)
SRCS += $(wildcard ./misc/*.c)
SRCS += $(wildcard ./panel/*.c)
SRCS += $(wildcard ./watchdog/*.c)
OBJS += $(patsubst %.c,%.o,$(SRCS))

CLFAGS = -c -Wall
CLFAGS += -fPIC
CLFAGS += -I../include
CLFAGS += -I./common
CLFAGS += -I./mcu
CLFAGS += -I./panel
CLFAGS += -I./misc

LDFLAGS = -shared
#LDFLAGS += -lpthread

all: $(TARGET)

$(TARGET): $(OBJS)
	$(LD) $(LDFLAGS) $(OBJS) -o $@

$(OBJS): %.o:%.c
	$(CC) $(CLFAGS) $< -o $@

clean:
	rm -rf $(OBJS)
