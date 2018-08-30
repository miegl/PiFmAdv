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
#include <getopt.h>

#include "rds.h"
#include "fm_mpx.h"
#include "control_pipe.h"
#include "mailbox.h"

#define MBFILE                          DEVICE_FILE_NAME // From mailbox.h

#if (RASPI) == 1                        // Original Raspberry Pi 1
#define PERIPH_VIRT_BASE                0x20000000
#define PERIPH_PHYS_BASE                0x7e000000
#define DRAM_PHYS_BASE                  0x40000000
#define MEM_FLAG                        0x0c
#elif (RASPI) == 2                      // Raspberry Pi 2 & 3
#define PERIPH_VIRT_BASE                0x3f000000
#define PERIPH_PHYS_BASE                0x7e000000
#define DRAM_PHYS_BASE                  0xc0000000
#define MEM_FLAG                        0x04
#else
#error Unknown Raspberry Pi version (variable RASPI)
#endif

#define DMA_BASE_OFFSET                 0x00007000
#define PWM_BASE_OFFSET                 0x0020C000
#define PWM_LEN                         0x28
#define CLK_BASE_OFFSET                 0x00101000
#define CLK_LEN                         0x1300
#define GPIO_BASE_OFFSET                0x00200000
#define GPIO_LEN                        0x100
#define PCM_BASE_OFFSET                 0x00203000
#define PCM_LEN                         0x24
#define PAD_BASE_OFFSET                 0x00100000
#define PAD_LEN                         (0x40/4) //0x64

#define DMA_VIRT_BASE                   (PERIPH_VIRT_BASE + DMA_BASE_OFFSET)
#define PWM_VIRT_BASE                   (PERIPH_VIRT_BASE + PWM_BASE_OFFSET)
#define CLK_VIRT_BASE                   (PERIPH_VIRT_BASE + CLK_BASE_OFFSET)
#define GPIO_VIRT_BASE                  (PERIPH_VIRT_BASE + GPIO_BASE_OFFSET)
#define PAD_VIRT_BASE                   (PERIPH_VIRT_BASE + PAD_BASE_OFFSET)
#define PCM_VIRT_BASE                   (PERIPH_VIRT_BASE + PCM_BASE_OFFSET)

#define PWM_PHYS_BASE                   (PERIPH_PHYS_BASE + PWM_BASE_OFFSET)
#define PCM_PHYS_BASE                   (PERIPH_PHYS_BASE + PCM_BASE_OFFSET)
#define GPIO_PHYS_BASE                  (PERIPH_PHYS_BASE + GPIO_BASE_OFFSET)

// GPIO
#define GPFSEL0                         (0x00/4)
#define GPFSEL1                         (0x04/4)
#define GPFSEL2                         (0x08/4)
#define GPPUD                           (0x94/4)
#define GPPUDCLK0                       (0x98/4)
#define GPPUDCLK1                       (0x9C/4)

#define CORECLK_CNTL                    (0x08/4)
#define CORECLK_DIV                     (0x0c/4)
#define GPCLK_CNTL                      (0x70/4)
#define GPCLK_DIV                       (0x74/4)
#define EMMCCLK_CNTL                    (0x1C0/4)
#define EMMCCLK_DIV                     (0x1C4/4)

#define CM_LOCK                         (0x114/4)
#define CM_LOCK_FLOCKA                  (1<<8)
#define CM_LOCK_FLOCKB                  (1<<9)
#define CM_LOCK_FLOCKC                  (1<<10)
#define CM_LOCK_FLOCKD                  (1<<11)
#define CM_LOCK_FLOCKH                  (1<<12)

#define CM_PLLA                         (0x104/4)
#define CM_PLLC                         (0x108/4)
#define CM_PLLD                         (0x10c/4)
#define CM_PLLH                         (0x110/4)
#define CM_PLLB                         (0x170/4)

#define A2W_PLLA_ANA0                   (0x1010/4)
#define A2W_PLLC_ANA0                   (0x1030/4)
#define A2W_PLLD_ANA0                   (0x1050/4)
#define A2W_PLLH_ANA0                   (0x1070/4)
#define A2W_PLLB_ANA0                   (0x10f0/4)
#define A2W_PLL_KA_SHIFT                7
#define A2W_PLL_KI_SHIFT                19
#define A2W_PLL_KP_SHIFT                15

#define PLLA_CTRL                       (0x1100/4)
#define PLLA_FRAC                       (0x1200/4)
#define PLLA_DSI0                       (0x1300/4)
#define PLLA_CORE                       (0x1400/4)
#define PLLA_PER                        (0x1500/4)
#define PLLA_CCP2                       (0x1600/4)

#define PLLB_CTRL                       (0x11e0/4)
#define PLLB_FRAC                       (0x12e0/4)
#define PLLB_ARM                        (0x13e0/4)
#define PLLB_SP0                        (0x14e0/4)
#define PLLB_SP1                        (0x15e0/4)
#define PLLB_SP2                        (0x16e0/4)

#define PLLC_CTRL                       (0x1120/4)
#define PLLC_FRAC                       (0x1220/4)
#define PLLC_CORE2                      (0x1320/4)
#define PLLC_CORE1                      (0x1420/4)
#define PLLC_PER                        (0x1520/4)
#define PLLC_CORE0                      (0x1620/4)

#define PLLD_CTRL                       (0x1140/4)
#define PLLD_FRAC                       (0x1240/4)
#define PLLD_DSI0                       (0x1340/4)
#define PLLD_CORE                       (0x1440/4)
#define PLLD_PER                        (0x1540/4)
#define PLLD_DSI1                       (0x1640/4)

#define PLLH_CTRL                       (0x1160/4)
#define PLLH_FRAC                       (0x1260/4)
#define PLLH_AUX                        (0x1360/4)
#define PLLH_RCAL                       (0x1460/4)
#define PLLH_PIX                        (0x1560/4)
#define PLLH_STS                        (0x1660/4)

// PWM
#define PWM_CTL                         (0x00/4)
#define PWM_DMAC                        (0x08/4)
#define PWM_RNG1                        (0x10/4)
#define PWM_RNG2                        (0x20/4)
#define PWM_FIFO                        (0x18/4)

#define PWMCLK_CNTL                     40
#define PWMCLK_DIV                      41

#define PWMCTL_PWEN1                    (1<<0)
#define PWMCTL_MODE1                    (1<<1)
#define PWMCTL_RPTL1                    (1<<2)
#define PWMCTL_POLA1                    (1<<4)
#define PWMCTL_USEF1                    (1<<5)
#define PWMCTL_CLRF                     (1<<6)
#define PWMCTL_MSEN1                    (1<<7)
#define PWMCTL_PWEN2                    (1<<8)
#define PWMCTL_MODE2                    (1<<9)
#define PWMCTL_RPTL2                    (1<<10)
#define PWMCTL_USEF2                    (1<<13)
#define PWMCTL_MSEN2                    (1<<15)

#define PWMDMAC_ENAB                    (1<<31)
#define PWMDMAC_THRSHLD                 ((15<<8)|(15<<0))

// PCM
#define PCM_CS_A                        (0x00/4)
#define PCM_FIFO_A                      (0x04/4)
#define PCM_MODE_A                      (0x08/4)
#define PCM_RXC_A                       (0x0c/4)
#define PCM_TXC_A                       (0x10/4)
#define PCM_DREQ_A                      (0x14/4)
#define PCM_INTEN_A                     (0x18/4)
#define PCM_INT_STC_A                   (0x1c/4)
#define PCM_GRAY                        (0x20/4)

#define PCMCLK_CNTL                     38
#define PCMCLK_DIV                      39

// PAD
#define GPIO_PAD_0_27                   (0x2C/4)
#define GPIO_PAD_28_45                  (0x30/4)
#define GPIO_PAD_46_52                  (0x34/4)

// DMA
#define DMA_CHANNEL                     14
#define DMA_CHANNEL_MAX                 14
#define DMA_CHANNEL_SIZE                0x100

#define BCM2708_DMA_ACTIVE              (1<<0)
#define BCM2708_DMA_END                 (1<<1)
#define BCM2708_DMA_INT                 (1<<2)
#define BCM2708_DMA_WAIT_RESP           (1<<3)
#define BCM2708_DMA_D_DREQ              (1<<6)
#define BCM2708_DMA_DST_IGNOR           (1<<7)
#define BCM2708_DMA_SRC_INC             (1<<8)
#define BCM2708_DMA_SRC_IGNOR           (1<<11)
#define BCM2708_DMA_NO_WIDE_BURSTS      (1<<26)
#define BCM2708_DMA_DISDEBUG            (1<<28)
#define BCM2708_DMA_ABORT               (1<<30)
#define BCM2708_DMA_RESET               (1<<31)
#define BCM2708_DMA_PER_MAP(x)          ((x)<<16)
#define BCM2708_DMA_PRIORITY(x)         ((x)&0xf << 16)
#define BCM2708_DMA_PANIC_PRIORITY(x)   ((x)&0xf << 20)

#define DMA_CS                          (0x00/4)
#define DMA_CONBLK_AD                   (0x04/4)
#define DMA_DEBUG                       (0x20/4)

#define DMA_CS_RESET			(1<<31)
#define DMA_CS_ABORT			(1<<30)
#define DMA_CS_DISDEBUG			(1<<29)
#define DMA_CS_WAIT_FOR_OUTSTANDING_WRITES (1<<28)
#define DMA_CS_INT			(1<<2)
#define DMA_CS_END			(1<<1)
#define DMA_CS_ACTIVE			(1<<0)
#define DMA_CS_PRIORITY(x)		((x)&0xf << 16)
#define DMA_CS_PANIC_PRIORITY(x)	((x)&0xf << 20)

#define DREQ_PCM_TX                     2
#define DREQ_PCM_RX                     3
#define DREQ_SMI                        4
#define DREQ_PWM                        5
#define DREQ_SPI_TX                     6
#define DREQ_SPI_RX                     7
#define DREQ_SPI_SLAVE_TX               8
#define DREQ_SPI_SLAVE_RX               9

#define MEM_FLAG_DISCARDABLE            (1 << 0) /* can be resized to 0 at any time. Use for cached data */
#define MEM_FLAG_NORMAL                 (0 << 2) /* normal allocating alias. Don't use from ARM */
#define MEM_FLAG_DIRECT                 (1 << 2) /* 0xC alias uncached */
#define MEM_FLAG_COHERENT               (2 << 2) /* 0x8 alias. Non-allocating in L2 but coherent */
#define MEM_FLAG_L1_NONALLOCATING       (MEM_FLAG_DIRECT | MEM_FLAG_COHERENT) /* Allocating in L2 */
#define MEM_FLAG_ZERO                   (1 << 4)  /* initialise buffer to all zeros */
#define MEM_FLAG_NO_INIT                (1 << 5) /* don't initialise (default is initialise to all ones */
#define MEM_FLAG_HINT_PERMALOCK         (1 << 6) /* Likely to be locked for long periods of time. */

#define BUS_TO_PHYS(x)                  ((x)&~0xC0000000)

#define PAGE_SIZE                       4096
#define PAGE_SHIFT                      12
#define NUM_PAGES                       ((sizeof(struct control_data_s) + PAGE_SIZE - 1) >> PAGE_SHIFT)

#define NUM_SAMPLES                     64000
#define NUM_CBS                         (NUM_SAMPLES * 2)

#define SUBSIZE                         1
#define DATA_SIZE                       1000



typedef struct {
    uint32_t info, src, dst, length, stride, next, pad[2];
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
static volatile uint32_t *pcm_reg;
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
        // Set GPIOs to be an output (instead of ALT FUNC 0, which is the clock).
        gpio_reg[0] = (gpio_reg[0] & ~(7 << 12)) | (1 << 12); //GPIO4
	udelay(10);
	gpio_reg[2] = (gpio_reg[2] & ~(7 << 0)) | (1 << 0); //GPIO20
	udelay(10);
	gpio_reg[3] = (gpio_reg[3] & ~(7 << 6)) | (1 << 6); //GPIO32
	udelay(10);
	gpio_reg[3] = (gpio_reg[3] & ~(7 << 12)) | (1 << 12); //GPIO34
	udelay(10);

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
        unmapmem(mbox.virt_addr, NUM_PAGES * PAGE_SIZE);
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



int tx(uint32_t carrier_freq, int divider, char *audio_file, int rds, uint16_t pi, char *ps, char *rt, int *af_array, float ppm, float deviation, float mpx, int cutoff, int preemphasis_cutoff, char *control_pipe, int pty, int tp, int power, int gpio, int wait) {
	// Catch only important signals
	for (int i = 0; i < 25; i++) {
		struct sigaction sa;

		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = terminate;
		sigaction(i, &sa, NULL);
	}

	dma_reg = map_peripheral(DMA_VIRT_BASE, (DMA_CHANNEL_SIZE * (DMA_CHANNEL_MAX + 1)));
	dma_reg = dma_reg + ((DMA_CHANNEL_SIZE / sizeof(int)) * (DMA_CHANNEL));
	pwm_reg = map_peripheral(PWM_VIRT_BASE, PWM_LEN);
	clk_reg = map_peripheral(CLK_VIRT_BASE, CLK_LEN);
	gpio_reg = map_peripheral(GPIO_VIRT_BASE, GPIO_LEN);
	pcm_reg = map_peripheral(PCM_VIRT_BASE, PCM_LEN);
	pad_reg = map_peripheral(PAD_VIRT_BASE, PAD_LEN);
	uint32_t freq_ctl;

	// Use the mailbox interface to the VC to ask for physical memory.
	mbox.handle = mbox_open();
	if (mbox.handle < 0)
		fatal("Failed to open mailbox. Check kernel support for vcio / BCM2708 mailbox.\n");
	printf("Allocating physical memory: size = %d, ", NUM_PAGES * PAGE_SIZE);
	if(!(mbox.mem_ref = mem_alloc(mbox.handle, NUM_PAGES * PAGE_SIZE, PAGE_SIZE, MEM_FLAG))) {
		fatal("Could not allocate memory.\n");
	}
	printf("mem_ref = %u, ", mbox.mem_ref);
	if(!(mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref))) {
		fatal("Could not lock memory.\n");
	}
	printf("bus_addr = %x, ", mbox.bus_addr);
	if(!(mbox.virt_addr = mapmem(BUS_TO_PHYS(mbox.bus_addr), NUM_PAGES * PAGE_SIZE))) {
		fatal("Could not map memory.\n");
	}
	printf("virt_addr = %p\n", mbox.virt_addr);

	clk_reg[GPCLK_CNTL] = (0x5a<<24) | (1<<4) | (4);
	udelay(100);

	clk_reg[CM_PLLA] = 0x5A00022A; // Enable PLLA_PER
	udelay(100);
	
	int ana[4];
	for (int i = 3; i >= 0; i--)
	{
		ana[i] = clk_reg[(A2W_PLLA_ANA0) + i];
	}

	ana[1]&=~(1<<14);
	for (int i = 3; i >= 0; i--)
	{
		clk_reg[(A2W_PLLA_ANA0) + i] = (0x5A << 24) | ana[i];
	}
	udelay(100);

	clk_reg[PLLA_CORE] = (0x5a<<24) | (1<<8); // Disable
	clk_reg[PLLA_PER] = 0x5A000001; // Div
	udelay(100);


	// Adjust PLLA frequency
	freq_ctl = (unsigned int)(((carrier_freq*divider)/19.2e6*((double)(1<<20))));
	clk_reg[PLLA_CTRL] = (0x5a<<24) | (0x21<<12) | (freq_ctl>>20); // Integer part
	freq_ctl&=0xFFFFF;
	clk_reg[PLLA_FRAC] = (0x5a<<24) | (freq_ctl&0xFFFFC); // Fractional part
	udelay(100);

	if ((clk_reg[CM_LOCK] & CM_LOCK_FLOCKA) > 0)
		printf("Master PLLA Locked\n");
	else
		printf("Warning: Master PLLA NOT Locked\n");

	// Program GPCLK integer division
	//int clktmp;
	//clktmp = clk_reg[GPCLK_CNTL];
	//clk_reg[GPCLK_CNTL] = (0xF0F&clktmp) | (0x5a<<24); // Clear run
	//udelay(100);
	clk_reg[GPCLK_DIV]  = (0x5a<<24) | (divider<<12);
	udelay(100);
	clk_reg[GPCLK_CNTL] = (0x5a<<24) | (4); // Source = PLLA (4)
	udelay(100);
	clk_reg[GPCLK_CNTL] = (0x5a<<24) | (1<<4) | (4); // Run, Source = PLLA (4)
	udelay(100);

	// Drive Strength: 0 = 2mA, 7 = 16mA. Ref: https://www.scribd.com/doc/101830961/GPIO-Pads-Control2
	pad_reg[GPIO_PAD_0_27] = 0x5a000018 + power;
	pad_reg[GPIO_PAD_28_45] = 0x5a000018 + power;
	udelay(100);

	int reg = gpio / 10;
	int shift = (gpio % 10) * 3;
	int mode;
	if(gpio == 20) {
		mode = 2;
	} else {
		mode = 4;
	}

	// GPIO needs to be ALT FUNC 0 to output the clock
	gpio_reg[reg] = (gpio_reg[reg] & ~(7 << shift)) | (mode << shift);
	udelay(100);

	ctl = (struct control_data_s *) mbox.virt_addr;
	dma_cb_t *cbp = ctl->cb;

	for (int i = 0; i < NUM_SAMPLES; i++) {
		ctl->sample[i] = 0x5a << 24 | freq_ctl; // Silence
		// Write a frequency sample
		cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
		cbp->src = mem_virt_to_phys(ctl->sample + i);
		cbp->dst = PERIPH_PHYS_BASE + (PLLA_FRAC<<2) + CLK_BASE_OFFSET;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
		// Delay
		cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP | BCM2708_DMA_D_DREQ | BCM2708_DMA_PER_MAP(5);
		cbp->src = mem_virt_to_phys(mbox.virt_addr);
		cbp->dst = PERIPH_PHYS_BASE + (PWM_FIFO<<2) + PWM_BASE_OFFSET;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
	}
	cbp--;
	cbp->next = mem_virt_to_phys(mbox.virt_addr);

	// Here we define the rate at which we want to update the GPCLK control register
	double srdivider = (((double)carrier_freq*divider/1e3)/(2*228*(1.+ppm/1.e6)));
	uint32_t idivider = (uint32_t)srdivider;
	uint32_t fdivider = (uint32_t)((srdivider - idivider)*pow(2, 12));

	printf("PPM correction is %.4f, divider is %.4f (%d + %d*2^-12).\n", ppm, srdivider, idivider, fdivider);

	pwm_reg[PWM_CTL] = 0;
	udelay(100);
	clk_reg[PWMCLK_CNTL] = (0x5a<<24) | (4); // Source = PLLA & disable
	udelay(100);
	clk_reg[PWMCLK_DIV] = (0x5a<<24) | (idivider<<12) | fdivider;
	udelay(100);
	clk_reg[PWMCLK_CNTL] = (0x5a<<24) | (1<<9) | (1<<4) | (4); // Source = PLLA, enable, MASH setting 1
	udelay(100);
	pwm_reg[PWM_RNG1] = 2;
	udelay(100);
	pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
	udelay(100);
	pwm_reg[PWM_CTL] = PWMCTL_CLRF;
	udelay(100);
	pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_MODE1 | PWMCTL_PWEN1 | PWMCTL_MSEN1;
	//pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
	udelay(100);

	// Initialise the DMA
	dma_reg[DMA_CS] = BCM2708_DMA_RESET;
	udelay(100);
	dma_reg[DMA_CS] = BCM2708_DMA_INT | BCM2708_DMA_END;
	dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(ctl->cb);
	dma_reg[DMA_DEBUG] = 7; // clear debug error flags
	dma_reg[DMA_CS] = DMA_CS_PRIORITY(15) | DMA_CS_PANIC_PRIORITY(15) | DMA_CS_DISDEBUG | DMA_CS_ACTIVE;


	uint32_t last_cb = (uint32_t)ctl->cb;

	// Data structures for baseband data
	double data[DATA_SIZE];
	double rds_buffer[DATA_SIZE];
	int data_len = 0;
	int data_index = 0;

	// Initialize the baseband generator
	if(fm_mpx_open(audio_file, DATA_SIZE, cutoff, preemphasis_cutoff) < 0) return 1;

	// Initialize the RDS modulator
	set_rds_pi(pi);
	set_rds_ps(ps);
	set_rds_rt(rt);
	set_rds_pty(pty);
	set_rds_tp(tp);
	set_rds_ms(1);
	set_rds_ab(0);

	printf("RDS Options:\n");

	if(rds) {
		printf("RDS: %i, ", rds);
		printf("PI: %04X, PS: \"%s\", PTY: %i\n", pi, ps, pty);
		printf("RT: \"%s\"\n", rt);
		if(af_array[0]) {
			set_rds_af(af_array);
			printf("AF: ");
			int f;
			for(f = 1; f < af_array[0]+1; f++) {
				printf("%f Mhz ", (float)(af_array[f]+875)/10);
			}
			printf("\n");
		}
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

	double deviation_scale_factor =  0.1 * (divider*(deviation*1000)/(19.2e6/((double)(1<<20))));

	for (;;) {

		if(control_pipe) poll_control_pipe();

		uint32_t cur_cb = (int)mem_phys_to_virt(dma_reg[DMA_CONBLK_AD]);
		int last_sample = (last_cb - (int)mbox.virt_addr) / (sizeof(dma_cb_t) * 2);
		int this_sample = (cur_cb - (int)mbox.virt_addr) / (sizeof(dma_cb_t) * 2);
		int free_slots = this_sample - last_sample;

		if (free_slots < 0)
			free_slots += NUM_SAMPLES;

		while (free_slots >= SUBSIZE) {
			// Get more baseband samples if necessary
			if(data_len == 0) {
				if(fm_mpx_get_samples(data, rds_buffer, mpx, rds, wait) < 0 ) {
					return 0;
				}
				data_len = DATA_SIZE;
				data_index = 0;
			}

			double dval = data[data_index]*deviation_scale_factor;
			//int intval = ((int)(dval)); //((int)((dval)) & ~0x3);
			data_index++;
			data_len--;

			ctl->sample[last_sample++] = (0x5A << 24 | freq_ctl) + dval;
			if (last_sample == NUM_SAMPLES)
				last_sample = 0;

			free_slots -= SUBSIZE;
		}
		last_cb = (uint32_t)mbox.virt_addr + last_sample * sizeof(dma_cb_t) * 2;

		usleep(1000);
	}

	return 0;
}

int main(int argc, char **argv) {
	int opt;

	char *audio_file = NULL;
	char *control_pipe = NULL;
	uint32_t carrier_freq = 87600000;
    	int rds = 1;
	int alternative_freq[100] = {};
	int af_size = 0;
	char *ps = "PiFmAdv";
	char *rt = "PiFmAdv: Advanced FM transmitter for the Raspberry Pi";
	uint16_t pi = 0x1234;
	float ppm = 0;
	float deviation = 50;
	int cutoff = 15000;
	int preemphasis_cutoff = 3185;
	int pty = 15;
	int tp = 1;
	int divc = 0;
	int power = 7;
	int gpio = 4;
	float mpx = 40;
	int wait = 1;

	const char    	*short_opt = "a:f:d:p:c:P:D:m:w:W:C:h";
	struct option   long_opt[] =
	{
		{"audio", 	required_argument, NULL, 'a'},
		{"freq", 	required_argument, NULL, 'f'},
		{"dev", 	required_argument, NULL, 'd'},
		{"ppm", 	required_argument, NULL, 'p'},
		{"cutoff", 	required_argument, NULL, 'c'},
		{"preemph", 	required_argument, NULL, 'P'},
		{"div", 	required_argument, NULL, 'D'},
		{"mpx", 	required_argument, NULL, 'm'},
		{"power", 	required_argument, NULL, 'w'},
		{"gpio",	required_argument, NULL, 'g'},
		{"wait",	required_argument, NULL, 'W'},

		{"rds", 	required_argument, NULL, 'rds'},
		{"pi", 		required_argument, NULL, 'pi'},
		{"ps", 		required_argument, NULL, 'ps'},
		{"rt", 		required_argument, NULL, 'rt'},
		{"pty", 	required_argument, NULL, 'pty'},
		{"tp",		required_argument, NULL, 'tp'},
		{"af", 		required_argument, NULL, 'af'},
		{"ctl", 	required_argument, NULL, 'C'},

		{"help",	no_argument, NULL, 'h'},
		{ 0, 		0, 		   0,    0 }
	};

	while((opt = getopt_long(argc, argv, short_opt, long_opt, NULL)) != -1)
	{
		switch(opt)
		{
			case -1:
			case 0:
			break;

			case 'a': //audio
				audio_file = optarg;
				break;

			case 'f': //freq
				carrier_freq = 1e6 * atof(optarg);
				if(carrier_freq < 76e6 || carrier_freq > 108e6)
					warn("Frequency should be in megahertz between 76.0 and 108.0, but is %f MHz\n", atof(optarg));
				break;

			case 'd': //dev
				deviation = atof(optarg);
				break;

			case 'p': //ppm
				ppm = atof(optarg);
				break;

			case 'c': //cutoff
				cutoff = atoi(optarg);
				break;

			case 'P': //preemph
				if(strcmp("eu", optarg)==0) {
					preemphasis_cutoff = 3185;
				} else if(strcmp("us", optarg)==0) {
					preemphasis_cutoff = 2120;
				}
				else {
					preemphasis_cutoff = atoi(optarg);
				}
				break;

			case 'D': //div
				divc = atoi(optarg);
				break;

			case 'm': //mpx
				mpx = atof(optarg);
				break;

			case 'w': //power
				power = atoi(optarg);
				if(power < 0 || power > 7)
					fatal("Output power has to be set in range of 0 - 7\n");
				break;

			case 'g': //gpio
                                gpio = atoi(optarg); 
                                if(gpio != 4 && gpio != 20 && gpio != 32 && gpio != 34)
                                        fatal("Available GPIO pins: 4,20,32,34\n");
                                break;

			case 'W': //wait
                                wait = atoi(optarg);
                                break;

			case 'rds': //rds
				rds = atoi(optarg);
				break;

			case 'pi': //pi
				pi = (uint16_t) strtol(optarg, NULL, 16);
				break;

			case 'ps': //ps
				ps = optarg;
				break;

			case 'rt': //rt
				rt = optarg;
				break;

			case 'pty': //pty
				pty = atoi(optarg);
				break;

			case 'tp': //tp
                                tp = atoi(optarg);
                                break;

			case 'af': //af
				af_size++;
				alternative_freq[af_size] = (int)(10*atof(optarg))-875;
				if(alternative_freq[af_size] < 1 || alternative_freq[af_size] > 204)
					fatal("Alternative Frequency has to be set in range of 87.6 Mhz - 107.9 Mhz\n");
				break;

			case 'C': //ctl
				control_pipe = optarg;
				break;

			case 'h': //help
				fatal("Help:\n"
				      "Syntax: pi_fm_adv [--audio (-a) file] [--freq (-f) frequency] [--dev (-d) deviation] [--ppm (-p) ppm-error]\n"
				      "                  [--cutoff (-c) cutoff-freq] [--preemph (-P) preemphasis] [--div (-D) divider] \n"
				      "                  [--mpx (-m) mpx-power] [--power (-w) output-power] [--gpio (-g) gpio-pin] [--wait (-W) wait-switch]\n"
				      "                  [--rds rds-switch] [--pi pi-code] [--ps ps-text] [--rt radiotext] [--tp traffic-program]\n"
				      "                  [--pty program-type] [--af alternative-freq] [--ctl (-C) control-pipe]\n");

				break;

			case ':':
				fatal("%s: option '-%c' requires an argument\n", argv[0], optopt);
				break;

			case '?':
			default:
				fatal("%s: option '-%c' is invalid. See -h (--help)\n", argv[0], optopt);
				break;
		}
	}

	alternative_freq[0] = af_size;

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
	
	int errcode = tx(carrier_freq, best_divider, audio_file, rds, pi, ps, rt, alternative_freq, ppm, deviation, mpx, cutoff, preemphasis_cutoff, control_pipe, pty, tp, power, gpio, wait);

	terminate(errcode);
}
