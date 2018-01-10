/*
    PiFmAdv - Advanced FM transmitter for the Raspberry Pi
    Copyright (C) 2017 Miegl

    See https://github.com/Miegl/PiFmAdv
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sndfile.h>

#include "rds.h"                    // RDS
#include "fm_mpx.h"                 // FM MPX signal
#include "control_pipe.h"           // FIFO
#include "mailbox.h"

#define MBFILE                      DEVICE_FILE_NAME // From mailbox.h

#if (RASPI) == 1                    // Original Raspberry Pi 1
#define PERIPH_VIRT_BASE            0x20000000
#define PERIPH_PHYS_BASE            0x7e000000
#define DRAM_PHYS_BASE              0x40000000
#define MEM_FLAG                    0x0c
#elif (RASPI) == 2                  // Raspberry Pi 2 & 3
#define PERIPH_VIRT_BASE            0x3f000000
#define PERIPH_PHYS_BASE            0x7e000000
#define DRAM_PHYS_BASE              0xc0000000
#define MEM_FLAG                    0x04
#else
#error Unknown Raspberry Pi version (variable RASPI)
#endif

#define NUM_SAMPLES                 64000
#define NUM_CBS                     (NUM_SAMPLES * 2)

#define BCM2708_DMA_NO_WIDE_BURSTS  (1<<26)
#define BCM2708_DMA_WAIT_RESP       (1<<3)
#define BCM2708_DMA_D_DREQ          (1<<6)
#define BCM2708_DMA_PER_MAP(x)      ((x)<<16)
#define BCM2708_DMA_END             (1<<1)
#define BCM2708_DMA_RESET           (1<<31)
#define BCM2708_DMA_INT             (1<<2)

#define DMA_CS                      (0x00/4)
#define DMA_CONBLK_AD               (0x04/4)
#define DMA_DEBUG                   (0x20/4)

#define DMA_NUMBER                  5 // DMA channel 5 appears to be unused
#define DMA_BASE_OFFSET             0x00007000
#define DMA_LEN                     0xe24

#define PWM_BASE_OFFSET             0x0020C000
#define PWM_LEN                     0x28
#define CLK_BASE_OFFSET             0x00101000
#define CLK_LEN                     0x1300
#define GPIO_BASE_OFFSET            0x00200000
#define GPIO_LEN                    0x100
#define PAD_BASE_OFFSET             0x00100000
#define PAD_LEN                     0x64

#define DMA_VIRT_BASE               (PERIPH_VIRT_BASE + DMA_BASE_OFFSET)
#define PWM_VIRT_BASE               (PERIPH_VIRT_BASE + PWM_BASE_OFFSET)
#define CLK_VIRT_BASE               (PERIPH_VIRT_BASE + CLK_BASE_OFFSET)
#define GPIO_VIRT_BASE              (PERIPH_VIRT_BASE + GPIO_BASE_OFFSET)
#define PAD_VIRT_BASE               (PERIPH_VIRT_BASE + PAD_BASE_OFFSET)
#define PCM_VIRT_BASE               (PERIPH_VIRT_BASE + PCM_BASE_OFFSET)

#define PWM_PHYS_BASE               (PERIPH_PHYS_BASE + PWM_BASE_OFFSET)
#define PCM_PHYS_BASE               (PERIPH_PHYS_BASE + PCM_BASE_OFFSET)
#define GPIO_PHYS_BASE              (PERIPH_PHYS_BASE + GPIO_BASE_OFFSET)

#define GPIO_PAD_0_27               (0x2C/4) // 0x7e10002c
#define GPIO_PAD_28_45              (0x30/4) // 0x7e100030
#define GPIO_PAD_46_52              (0x34/4) // 0x7e100034

#define PWM_CTL                     (0x00/4)
#define PWM_DMAC                    (0x08/4)
#define PWM_RNG1                    (0x10/4)
#define PWM_FIFO                    (0x18/4)

#define PWMCLK_CNTL                 40
#define PWMCLK_DIV                  41

#define CM_GP0DIV                   (0x7e101074)
#define CM_PLLCFRAC                 (0x7e102220)

#define CORECLK_CNTL                (0x08/4)
#define CORECLK_DIV                 (0x0c/4)
#define GPCLK_CNTL                  (0x70/4)
#define GPCLK_DIV                   (0x74/4)
#define EMMCCLK_CNTL                (0x1C0/4)
#define EMMCCLK_DIV                 (0x1C4/4)

// PLLs
#define PLLA_CTRL                   (0x1100/4)
#define PLLA_FRAC                   (0x1200/4)
#define PLLC_CTRL                   (0x1120/4)
#define PLLC_FRAC                   (0x1220/4)
#define PLLD_CTRL                   (0x1140/4)
#define PLLD_FRAC                   (0x1240/4)

#define PWMCTL_MODE1                (1<<1)
#define PWMCTL_PWEN1                (1<<0)
#define PWMCTL_CLRF                 (1<<6)
#define PWMCTL_USEF1                (1<<5)

#define PWMDMAC_ENAB                (1<<31)
#define PWMDMAC_THRSHLD             ((15<<8)|(15<<0))

#define GPFSEL0                     (0x00/4)

#define BUS_TO_PHYS(x)              ((x)&~0xC0000000)

#define PAGE_SIZE                   4096
#define PAGE_SHIFT                  12
#define NUM_PAGES                   ((sizeof(struct control_data_s) + PAGE_SIZE - 1) >> PAGE_SHIFT)
#define SUBSIZE                     1
#define DATA_SIZE                   5000

typedef struct {
    uint32_t info, src, dst, length,
         stride, next, pad[2];
} dma_cb_t;

static struct {
    int handle;                     /* From mbox_open() */
    unsigned mem_ref;               /* From mem_alloc() */
    unsigned bus_addr;              /* From mem_lock() */
    uint8_t *virt_addr;             /* From mapmem() */
} mbox;

struct control_data_s {
    dma_cb_t cb[NUM_CBS];
    uint32_t sample[NUM_SAMPLES];
};

static struct control_data_s *ctl;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;
static volatile uint32_t *pad_reg;

static void udelay(int us)
{
    struct timespec ts = { 0, us * 1000 };
    nanosleep(&ts, NULL);
}

static void terminate(int num)
{
    // Stop outputting and generating the clock.
    if (clk_reg && gpio_reg && mbox.virt_addr) {
        // Set GPIO4 to be an output (instead of ALT FUNC 0, which is the clock).
        gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (1 << 12);

        // Disable the clock generator.
        clk_reg[GPCLK_CNTL] = 0x5A;
    }

    if (dma_reg && mbox.virt_addr) {
        dma_reg[DMA_CS] = BCM2708_DMA_RESET;
        udelay(10);
    }

    fm_mpx_close();
    close_control_pipe();

    if (mbox.virt_addr != NULL) {
        unmapmem(mbox.virt_addr, NUM_PAGES * 4096);
        mem_unlock(mbox.handle, mbox.mem_ref);
        mem_free(mbox.handle, mbox.mem_ref);
    }

    printf("Terminating: cleanly deactivated the DMA engine and killed the carrier.\n");

    exit(num);
}

static void fatal(char *fmt, ...)
{
    va_list ap;
    fprintf(stderr,"ERROR: ");
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    terminate(0);
}

static void warn(char *fmt, ...)
{
    va_list ap;
    fprintf(stderr,"WARNING: ");
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

static uint32_t mem_virt_to_phys(void *virt)
{
    uint32_t offset = (uint8_t *)virt - mbox.virt_addr;

    return mbox.bus_addr + offset;
}

static uint32_t mem_phys_to_virt(uint32_t phys)
{
    return phys - (uint32_t)mbox.bus_addr + (uint32_t)mbox.virt_addr;
}

static void *map_peripheral(uint32_t base, uint32_t len)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    void * vaddr;

    if (fd < 0)
        fatal("Failed to open /dev/mem: %m.\n");
    vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
    if (vaddr == MAP_FAILED)
        fatal("Failed to map peripheral at 0x%08x: %m.\n", base);
    close(fd);

    return vaddr;
}



int tx(uint32_t carrier_freq, uint32_t divider, char *audio_file, int rds, uint16_t pi, char *ps, char *rt, float ppm, float deviation, float mpx, float cutoff, float preemphasis_cutoff, char *control_pipe, int pty, int power) {
	// Catch only important signals
	for (int i = 0; i < 25; i++) {
		struct sigaction sa;

		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = terminate;
		sigaction(i, &sa, NULL);
	}

	dma_reg = map_peripheral(DMA_VIRT_BASE, DMA_LEN);
	dma_reg = dma_reg+((0x100/sizeof(int))*(DMA_NUMBER));
	pwm_reg = map_peripheral(PWM_VIRT_BASE, PWM_LEN);
	clk_reg = map_peripheral(CLK_VIRT_BASE, CLK_LEN);
	gpio_reg = map_peripheral(GPIO_VIRT_BASE, GPIO_LEN);
	pad_reg = map_peripheral(PAD_VIRT_BASE, PAD_LEN);
	uint32_t freq_ctl;

	// Use the mailbox interface to the VC to ask for physical memory.
	mbox.handle = mbox_open();
	if (mbox.handle < 0)
		fatal("Failed to open mailbox. Check kernel support for vcio / BCM2708 mailbox.\n");
	printf("Allocating physical memory: size = %d, ", NUM_PAGES * 4096);
	if(!(mbox.mem_ref = mem_alloc(mbox.handle, NUM_PAGES * 4096, 4096, MEM_FLAG))) {
		fatal("Could not allocate memory.\n");
	}
	printf("mem_ref = %u, ", mbox.mem_ref);
	if(!(mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref))) {
		fatal("Could not lock memory.\n");
	}
	printf("bus_addr = %x, ", mbox.bus_addr);
	if(!(mbox.virt_addr = mapmem(BUS_TO_PHYS(mbox.bus_addr), NUM_PAGES * 4096))) {
		fatal("Could not map memory.\n");
	}
	printf("virt_addr = %p\n", mbox.virt_addr);

	// Switch the core over to PLLA
	int clktmp;
	clktmp = clk_reg[CORECLK_CNTL];

	//clk_reg[CORECLK_CNTL] = (0xF0F&clktmp) | (0x5a<<24); // Clear run
	//udelay(100);
	//clk_reg[CORECLK_DIV]  = (0x5a<<24) | (2<<12);
	//udelay(100);
	clk_reg[CORECLK_CNTL] = (0x5a<<24) | (4); // Source = PLLA (4)
	udelay(100);
	clk_reg[CORECLK_CNTL] = (0x5a<<24) | (1<<4) | (4); // Run, Source = PLLA (4)
	udelay(100);

	// Switch EEMC over to PLLD
	clktmp = clk_reg[EMMCCLK_CNTL];
	clk_reg[EMMCCLK_CNTL] = (0xF0F&clktmp) | (0x5a<<24); // Clear run
	udelay(100);
	//clk_reg[EMMCCLK_DIV]  = (0x5a<<24) | (2<<12);
        //udelay(100);
	clk_reg[EMMCCLK_CNTL] = (0xF0F&clktmp) | (0x5a<<24) | (6); // Source = PLLD (6)
	udelay(100);
	clk_reg[EMMCCLK_CNTL] = (0xF0F&clktmp) | (0x5a<<24) | (1<<4) | (6); // Run, Source = PLLD (6)
	udelay(100);

    /*
	printf("PPM: %f\n", (1.+ppm/1.e6));
	printf("Real freq: %f\n", (((((carrier_freq*divider)/19.2e6*(1.+ppm/1.e6))*19.2e6)/divider)/1e6));
	printf("To achieve: %f\n", ((carrier_freq+((ppm* -1)*1e2))+((10000-(carrier_freq*1e-4))*(ppm*1e-2)))/1e6);
    */

	// Adjust PLLC frequency
	clktmp = clk_reg[PLLC_CTRL];
	//clk_reg[PLLC_CTRL] = (0xF0F&clktmp) | (0x5a<<24); // Clear run
	freq_ctl = (unsigned int)(((carrier_freq*divider)/19.2e6*((double)(1<<20))));
	clk_reg[PLLC_CTRL] = (0x5a<<24) | (0x21<<12) | (freq_ctl>>20 ); // Integer part
	freq_ctl&=0xFFFFF;
	clk_reg[PLLC_FRAC] = (0x5a<<24) | (freq_ctl&0xFFFFC); // Fractional part
	udelay(100);

	// Program GPCLK integer division
	clktmp = clk_reg[GPCLK_CNTL];
	clk_reg[GPCLK_CNTL] = (0xF0F&clktmp) | (0x5a<<24); // Clear run
	udelay(100);
	clk_reg[GPCLK_DIV]  = (0x5a<<24) | (divider<<12);
	udelay(100);
	clk_reg[GPCLK_CNTL] = (0x5a<<24) | (5); // Source = PLLC (5)
	udelay(100);
	clk_reg[GPCLK_CNTL] = (0x5a<<24) | (1<<4) | (5); // Run, Source = PLLC (5)
	udelay(100);

	// Drive Strength: 0 = 2mA, 7 = 16mA. Ref: https://www.scribd.com/doc/101830961/GPIO-Pads-Control2
	pad_reg[GPIO_PAD_0_27] = 0x5a000018 + power;
	udelay(100);

	// GPIO4 needs to be ALT FUNC 0 to output the clock
	gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (4 << 12);
	udelay(100);

	ctl = (struct control_data_s *) mbox.virt_addr;
	dma_cb_t *cbp = ctl->cb;
	uint32_t phys_sample_dst = divider ? CM_PLLCFRAC : CM_GP0DIV;
	uint32_t phys_pwm_fifo_addr = PWM_PHYS_BASE + 0x18;

	for (int i = 0; i < NUM_SAMPLES; i++) {
		ctl->sample[i] = 0x5a << 24 | freq_ctl; // Silence
		// Write a frequency sample
		cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
		cbp->src = mem_virt_to_phys(ctl->sample + i);
		cbp->dst = phys_sample_dst;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
		// Delay
		cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP | BCM2708_DMA_D_DREQ | BCM2708_DMA_PER_MAP(5);
		cbp->src = mem_virt_to_phys(mbox.virt_addr);
		cbp->dst = phys_pwm_fifo_addr;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
	}
	cbp--;
	cbp->next = mem_virt_to_phys(mbox.virt_addr);

	// Here we define the rate at which we want to update the GPCLK control register
	float srdivider = (((double)carrier_freq*divider/1e3)/(2*228*(1.+ppm/1.e6)));
	uint32_t idivider = (uint32_t) srdivider;
	uint32_t fdivider = (uint32_t) ((srdivider - idivider)*pow(2, 12));

	printf("PPM correction is %.4f, divider is %.4f (%d + %d*2^-12).\n", ppm, srdivider, idivider, fdivider);

	pwm_reg[PWM_CTL] = 0;
	udelay(100);
	clk_reg[PWMCLK_CNTL] = 0x5A000005; // Source = PLLC & disable
	udelay(100);
	clk_reg[PWMCLK_DIV] = 0x5A000000 | (idivider<<12) | fdivider;
	udelay(100);
	clk_reg[PWMCLK_CNTL] = 0x5A000015; // Source = PLLC, enable, MASH setting 0
	udelay(100);
	pwm_reg[PWM_RNG1] = 2;
	udelay(100);
	pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
	udelay(100);
	pwm_reg[PWM_CTL] = PWMCTL_CLRF;
	udelay(100);
	pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
	udelay(100);

	// Initialise the DMA
	dma_reg[DMA_CS] = BCM2708_DMA_RESET;
	udelay(100);
	dma_reg[DMA_CS] = BCM2708_DMA_INT | BCM2708_DMA_END;
	dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(ctl->cb);
	dma_reg[DMA_DEBUG] = 7; // clear debug error flags
	dma_reg[DMA_CS] = 0x10880001; // go, mid priority, wait for outstanding writes


	uint32_t last_cb = (uint32_t)ctl->cb;

	// Data structures for baseband data
	float data[DATA_SIZE];
	float rdsdata[DATA_SIZE];
	int data_len = 0;
	int data_index = 0;

	// Initialize the baseband generator
	if(fm_mpx_open(audio_file, DATA_SIZE, mpx, cutoff, preemphasis_cutoff, rds) < 0) return 1;

	// Initialize the RDS modulator
	char myps[9] = {0};
	set_rds_pi(pi);
	set_rds_rt(rt);
	set_rds_pty(pty);
	uint16_t count = 0;
	uint16_t count2 = 0;
	int varying_ps = 0;

    printf("RDS Options:\n");

    if(rds) {
        printf("RDS: %i, ", rds);
        if(ps) {
    		set_rds_ps(ps);
    		printf("PI: %04X, PS: \"%s\", PTY: %i\n", pi, ps, pty);
    	} else {
    		printf("PI: %04X, PS: <Varying>, PTY: %i\n", pi, pty);
    		varying_ps = 1;
    	}
    	printf("RT: \"%s\"\n", rt);
    }
    else {
        printf("RDS: %i\n", rds);
    }

	// Initialize the control pipe reader
	if(control_pipe) {
		if(open_control_pipe(control_pipe) == 0) {
			printf("Reading control commands on %s.\n", control_pipe);
		} else {
			printf("Failed to open control pipe: %s.\n", control_pipe);
			control_pipe = NULL;
		}
	}

	printf("Starting to transmit on %3.1f MHz.\n", carrier_freq/1e6);

	float deviation_scale_factor =  0.1 * (divider*(deviation*1000)/(19.2e6/((double)(1<<20))) ); //TODO: PPM

	for (;;) { // Loop
		// Default (varying) PS
		if(varying_ps) {
			if(count == 2048) {
				snprintf(myps, 9, "%08d", count2);
				set_rds_ps(myps); // Show PI
				count2++;
			}
			if(count == 4096) {
				set_rds_ps("PiFmAdv");
				count = 0;
			}
			count++;
		}

		if(control_pipe && poll_control_pipe() == CONTROL_PIPE_PS_SET) {
			varying_ps = 0; // Disable varying PS when control pipe is set.
		}

		usleep(2500);

		uint32_t cur_cb = mem_phys_to_virt(dma_reg[DMA_CONBLK_AD]);
		int last_sample = (last_cb - (uint32_t)mbox.virt_addr) / (sizeof(dma_cb_t) * 2);
		int this_sample = (cur_cb - (uint32_t)mbox.virt_addr) / (sizeof(dma_cb_t) * 2);
		int free_slots = this_sample - last_sample;

		if (free_slots < 0)
			free_slots += NUM_SAMPLES;

		while (free_slots >= SUBSIZE) {
			// Get more baseband samples if necessary
			if(data_len == 0) {
				if( fm_mpx_get_samples(data, rdsdata) < 0 ) {
					return 0;
				}
				data_len = DATA_SIZE;
				data_index = 0;
			}

			float dval = data[data_index]*deviation_scale_factor;
			int intval = ((int)((floor)(dval)) & ~0x3);
			data_index++;
			data_len--;

			ctl->sample[last_sample++] = (0x5A << 24 | freq_ctl) + intval;
			if (last_sample == NUM_SAMPLES)
				last_sample = 0;

			free_slots -= SUBSIZE;
		}
		last_cb = (uint32_t)mbox.virt_addr + last_sample * sizeof(dma_cb_t) * 2;
	}

	return 0;
}

int main(int argc, char **argv) {
	char *audio_file = NULL;
	char *control_pipe = NULL;
	uint32_t carrier_freq = 107900000;
    	int rds = 1;
	char *ps = NULL;
	char *rt = "PiFmAdv: Advanced FM transmitter for the Raspberry Pi";
	uint16_t pi = 0x1234;
	float ppm = 0;
	float deviation = 75.0;
	float cutoff = 16400;
	float preemphasis_cutoff = 3185;
	int pty = 15;
	int divc = 0;
	int power = 7;
	float mpx = 30;

	// Parse command-line arguments
	for(int i=1; i<argc; i++) {
		char *arg = argv[i];
		char *param = NULL;

		if(arg[0] == '-' && i+1 < argc) param = argv[i+1];

		if((strcmp("-audio", arg)==0) && param != NULL) {
			i++;
			audio_file = param;
		} else if(strcmp("-freq", arg)==0 && param != NULL) {
			i++;
			carrier_freq = 1e6 * atof(param);
			if(carrier_freq < 76e6 || carrier_freq > 108e6)
				warn("Frequency should be in megahertz between 76.0 and 108.0, but is %f MHz\n", atof(param));
        } else if(strcmp("-rds", arg)==0 && param != NULL) {
    		i++;
    		rds = atof(param);
		} else if(strcmp("-pi", arg)==0 && param != NULL) {
			i++;
			pi = (uint16_t) strtol(param, NULL, 16);
		} else if(strcmp("-ps", arg)==0 && param != NULL) {
			i++;
			ps = param;
		} else if(strcmp("-rt", arg)==0 && param != NULL) {
			i++;
			rt = param;
		} else if(strcmp("-pty", arg)==0 && param != NULL) {
			i++;
			pty = atof(param);
		} else if(strcmp("-dev", arg)==0 && param != NULL) {
			i++;
			deviation = atof(param);
		} else if(strcmp("-ppm", arg)==0 && param != NULL) {
			i++;
			ppm = atof(param);
		} else if(strcmp("-preemph", arg)==0 && param != NULL) {
			i++;
			if(strcmp("eu", param)==0) {
				preemphasis_cutoff = 3185;
			} else if(strcmp("us", param)==0) {
				preemphasis_cutoff = 2120;
			}
			else {
				preemphasis_cutoff = atof(param);
			}
		} else if(strcmp("-cutoff", arg)==0 && param != NULL) {
			i++;
			cutoff = atof(param);
		} else if(strcmp("-mpx", arg)==0 && param != NULL) {
                        i++; 
                        mpx = atof(param);
		} else if(strcmp("-ctl", arg)==0 && param != NULL) {
			i++;
			control_pipe = param;
		} else if(strcmp("-div", arg)==0 && param != NULL) {
			i++;
			divc = atof(param);
		} else if(strcmp("-power", arg)==0 && param != NULL) {
			i++;
			power = atof(param);
			if(power < 0 || power >= 8)
				fatal("Output power must be set in range of 0 - 7\n");
		} else {
			fatal("Unrecognised argument: %s.\n"
			      "Syntax: pi_fm_adv [-freq freq] [-div divider] [-audio file] [-ppm ppm_error] [-pi pi_code]\n"
			      "                  [-ps ps_text] [-rt rt_text] [-pty program_type] [-dev deviation]\n"
			      "                  [-cutoff cutoff_freq] [-preemph preemphasis] [-ctl control_pipe] [-rds rds]\n", arg);
		}
	}

	double xtal_freq_recip=1.0/19.2e6;
	int divider, best_divider = 0;
	int min_int_multiplier, max_int_multiplier;
	int int_multiplier;
	double frac_multiplier;
	int fom, best_fom = 0;
	int solution_count = 0;
	for(divider = 2; divider < 50; divider += 1)
	{
		if(carrier_freq * divider > 1400e6) break;

		max_int_multiplier=((int)((double)(carrier_freq + 10 + (deviation * 1000)) * divider * xtal_freq_recip));
		min_int_multiplier=((int)((double)(carrier_freq - 10 - (deviation * 1000)) * divider * xtal_freq_recip));
		if(min_int_multiplier != max_int_multiplier) continue;

		solution_count++;
		fom = 0;

		if(carrier_freq * divider >  900e6) fom++; // Prefer frequencies close to 1.0 Ghz
		if(carrier_freq * divider < 1100e6) fom++;

		if(carrier_freq * divider >  800e6) fom++;
		if(carrier_freq * divider < 1200e6) fom++;

		frac_multiplier = ((double)(carrier_freq) * divider * xtal_freq_recip);
		int_multiplier = (int)frac_multiplier;
		frac_multiplier = frac_multiplier - int_multiplier;
		if((frac_multiplier > 0.2) && (frac_multiplier < 0.8)) fom++; // Prefer mulipliers away from integer boundaries

		if(fom > best_fom) // Best match so far
		{
			best_fom = fom;
			best_divider = divider;
		}
	}

	if(divc) {
		best_divider = divc;
	}
	else if(!solution_count & !best_divider) {
		fatal("No tuning solution found. You can specify the divider manually by setting the -div parameter.\n");
	}

	printf("Carrier: %3.2f Mhz, VCO: %4.1f MHz, Multiplier: %f, Divider: %d\n", carrier_freq/1e6, (double)carrier_freq * best_divider / 1e6, carrier_freq * best_divider * xtal_freq_recip, best_divider);

	int errcode = tx(carrier_freq, best_divider, audio_file, rds, pi, ps, rt, ppm, deviation, mpx, cutoff, preemphasis_cutoff, control_pipe, pty, power);

	terminate(errcode);
}
