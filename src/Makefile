CC = gcc
STD_CFLAGS = -Wall -Wno-multichar -std=gnu99 -c -O2

# Enable ARM-specific options only on ARM, and compilation of the app only on ARM
UNAME := $(shell uname -m)
# PCPUI := $(shell cat /proc/cpuinfo | grep Revision | cut -c16-)
LSCPU := $(shell lscpu | grep "Model name" | awk '{print $$3}')

# Determine the hardware platform. Below, pi1 stands for the RaspberryPi 1 (the original one),
# and pi2 stands for both the RaspberryPi 2 and 3.
# Update pi4 settings 
ifeq ($(UNAME), armv6l)
	CFLAGS = $(STD_CFLAGS) -march=armv6 -mtune=arm1176jzf-s -mfloat-abi=hard -mfpu=vfp -ffast-math -DRASPI=1
	TARGET = pi1
else ifeq ($(UNAME), armv7l)
	CFLAGS = $(STD_CFLAGS) -march=armv7-a -mtune=arm1176jzf-s -mfloat-abi=hard -mfpu=vfp -ffast-math -DRASPI=2
	TARGET = pi2
else ifeq ($(UNAME), aarch64)
	ifeq ($(LSCPU), Cortex-A72)
		CFLAGS = $(STD_CFLAGS) -march=armv8-a -mtune=cortex-a72 -pipe -fstack-protector-strong -fno-plt -ffast-math -DRASPI=4
		TARGET = pi4
	endif
else
	CFLAGS = $(STD_CFLAGS)
	TARGET = other
endif

ifneq ($(TARGET), other)

app: rds.o waveforms.o pi_fm_adv.o fm_mpx.o control_pipe.o mailbox.o
	$(CC) -o pi_fm_adv rds.o waveforms.o mailbox.o pi_fm_adv.o fm_mpx.o control_pipe.o -lm -lsndfile -lsoxr

endif


rds_wav: rds.o waveforms.o rds_wav.o fm_mpx.o
	$(CC) -o rds_wav rds_wav.o rds.o waveforms.o fm_mpx.o -lm -lsndfile

rds.o: rds.c waveforms.h
	$(CC) $(CFLAGS) rds.c

control_pipe.o: control_pipe.c control_pipe.h rds.h
	$(CC) $(CFLAGS) control_pipe.c

waveforms.o: waveforms.c waveforms.h
	$(CC) $(CFLAGS) waveforms.c

mailbox.o: mailbox.c mailbox.h
	$(CC) $(CFLAGS) mailbox.c

pi_fm_rds.o: pi_fm_adv.c control_pipe.h fm_mpx.h rds.h mailbox.h
	$(CC) $(CFLAGS) pi_fm_adv.c

rds_wav.o: rds_wav.c
	$(CC) $(CFLAGS) rds_wav.c

fm_mpx.o: fm_mpx.c fm_mpx.h
	$(CC) $(CFLAGS) fm_mpx.c

clean:
	rm -f *.o
