/*
Code to generate DSHOT protocol signals from a Raspberry Pi GPIO
output.  Original code comes from
https://github.com/dmrlawson/raspberrypi-dshot.  This code is enhanced
by the possibility to broadcast mutliple dshot frames on multiple pins
at once and by some autocalibration which may or may not work on your
computer.  The assembly busy-loop delay, wait_cycles, was inspired by
ElderBug's answer here:
https://stackoverflow.com/questions/32719767/cycles-per-instruction-in-delay-loop-on-arm
I used this to help make the assembly code work:
https://www.cl.cam.ac.uk/projects/raspberrypi/tutorials/os/troubleshooting.html#immediate
The method of controlling the GPIO pins directly was taken from here:
https://elinux.org/RPi_GPIO_Code_Samples#Direct_register_access
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <assert.h>
#include <stdint.h>
#include <sched.h>
#include <errno.h>
#include <string.h>

// #include "../motor-common.h"

//////////////////////////////////////////////////////////////////////
// select the dshot version you want to use by uncommenting the
// corresponding two timings.  On raspberry pi zero 2, dshot 150 is
// unusable because OS scheduler tick seems to be 100us and the
// process is always interrupted during the transmission.  The best
// choice seems to be dshot300. Dshot600 is noisy.

// DSHOT_150
//#define DSHOT_T0H_us 2.50
//#define DSHOT_BIT_us 6.67

// DSHOT_300
#define DSHOT_T0H_us 1.25
#define DSHOT_BIT_us 3.33

// DSHOT_600
//#define DSHOT_T0H_us 0.625
//#define DSHOT_BIT_us 1.67

// DSHOT_1200
//#define DSHOT_T0H_us 0.313
//#define DSHOT_BIT_us 0.83

//////////////////////////////////////////////////////////////////////

#define DSHOT_T1L_us 			(DSHOT_BIT_us - 2*DSHOT_T0H_us)
#define DSHOT_FRAME_ns 			(DSHOT_BIT_us * 1000.0 * 16)

//#define AUTOCALIBRATION_CLOCK		CLOCK_REALTIME
#define AUTOCALIBRATION_CLOCK		CLOCK_MONOTONIC_RAW 
#define USLEEP_BEFORE_BROADCAST		100
#define AUTOCALIBRATION_TESTS 		10

// how many times test is repeated during calibration. Too large value
// will cause the test to be interrupted by the scheduler. The value
// shall be such that the ETA does not exceed scheduler tick.
#define AUTOCALIBRATION_EMTY_LOOP_N 	10000
#define AUTOCALIBRATION_GPIO_WRITE_N 	1000


#define TIMESPEC_TO_INT(tt) 		(tt.tv_sec * 1000000000LL + tt.tv_nsec)
#define ABS(x) 				((x)<0?-(x):(x))

/////////

#define BCM2708_PERI_BASE       0x3F000000
#define GPIO_BASE               (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define PAGE_SIZE 		(4*1024)
#define BLOCK_SIZE 		(4*1024)

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) 		(*(dshotGpio+((g)/10)) &= ~(7<<(((g)%10)*3)))
#define OUT_GPIO(g) 		(*(dshotGpio+((g)/10)) |=  (1<<(((g)%10)*3)))

#define GPIO_SET 		(*(dshotGpio+7))
#define GPIO_CLR 		(*(dshotGpio+10))

#define NUM_PINS 		27

    
double dshotLatencyOfGetNanosecondsNs;
double dshotLatencyOfGpioWriteNs;
double dshotLatencyOfSingleEmptyLoopNs;

double dshotGpioWriteTestEtaNs;
double dshotEmptyLoopTestEtaNs;

int dshotBusyLoopIterationsForBitPart1 = 1;
int dshotBusyLoopIterationsForBitPart2 = 1;
int dshotBusyLoopIterationsForBitPart3 = 1;
int dshotAutoCorrection = 0;

uint32_t dshotAllMotorsPinsMaskForTests;

// I/O access
void 			*dshotGpioMap;
volatile uint32_t 	*dshotGpio;


inline void wait_cycles( int l ) {
#if defined(__ARM_ARCH)
    asm volatile( "0:" "SUBS %[count], #1;" "BNE 0b;" :[count]"+r"(l) );
#else
    int i;
    for(i=0; i<l; i++) asm volatile("");
#endif    
}

static inline uint64_t dshotGetNanoseconds() {
#if 0 && defined(__ARM_ARCH)
  unsigned cc;
  asm volatile ("mrc p15, 0, %0, c15, c12, 1" : "=r" (cc));
  // ??? how to translate to nanoseconds?
  return (cc / ticksPerNanosecond);
#else
  struct timespec tt;
  clock_gettime(AUTOCALIBRATION_CLOCK, &tt);
  return(TIMESPEC_TO_INT(tt));
#endif
}


int dshotAddChecksumAndTelemetry(int packet, int telem) {
    int packet_telemetry = (packet << 1) | (telem & 1);
    int i;
    int csum = 0;
    int csum_data = packet_telemetry;
    for (i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    csum &= 0xf;
    // csum = 0;
    // append checksum
    int packet_telemetry_checksum = (packet_telemetry << 4) | csum;

    return packet_telemetry_checksum;
}

void dshotSendFrames(uint32_t allMotorsPinMask, uint32_t *clearMasks) {
    int 		i;
    volatile unsigned	*gpioset;
    volatile unsigned	*gpioclear;

    // prepare addresses
    gpioset = &GPIO_SET;
    gpioclear = &GPIO_CLR;
    
    // send dshot frame bits
    for(i=0; i<16; i++) {
	*gpioset = allMotorsPinMask;
	wait_cycles(dshotBusyLoopIterationsForBitPart1);
	*gpioclear = clearMasks[i];
	wait_cycles(dshotBusyLoopIterationsForBitPart2);
	*gpioclear = allMotorsPinMask;
	wait_cycles(dshotBusyLoopIterationsForBitPart3);
    }
}

//
// Set up a memory regions to access GPIO
//
void dshotSetupIo() {
    int  mem_fd;

    /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("debug can't open /dev/mem \n");
      exit(-1);
   }

   /* mmap GPIO */
   dshotGpioMap = mmap(
      NULL,             // Any adddress in our space will do
      BLOCK_SIZE,       // Map length
      PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       // Shared with other processes
      mem_fd,           // File to map
      GPIO_BASE         // Offset to GPIO peripheral
   );

   close(mem_fd); //No need to keep mem_fd open after mmap

   if (dshotGpioMap == MAP_FAILED) {
      printf("debug mmap error %p\n", dshotGpioMap);//errno also set!
      exit(-1);
   }

   // Always use volatile pointer!
   dshotGpio = (volatile unsigned *)dshotGpioMap;

}


/////////////////////////////////////////////////////////////////////////////////////////////////
// calibration stuff

void dshotUpdateConfigurationForSendFrame() {
    double tick;

    // This is computing numbers for busy waits loops in
    // "dshotSendFrames".  My assembly code generated for
    // dshotSendFrames do following instructions between busy loops:
    
    // 1.) before setting pins low for zero bits:
    //     one register move + memory load + register move + gpio store
    // 2.) before setting all pins low at the end of the bit:
    //     one gpio store
    // 3.) before starting another bit and setting all pins high:
    //     one register move + compare + branch jump  + gpio store
    
    // In my (very) simplified model of ARM I suppose following timing:
    // - 1 tick for register move and compare
    // - 2 ticks for branchig instructions
    // - 2 ticks for memory access instructions

    // "dshotAutoCorrection" is an integer incremented/decremented
    // after each broadcast which was shorter/longer that it should
    // be.
    
    // From that I get:
    tick = dshotLatencyOfSingleEmptyLoopNs / 3;
    dshotBusyLoopIterationsForBitPart1 = (DSHOT_T0H_us * 1000 - dshotLatencyOfGpioWriteNs - 4*tick + dshotAutoCorrection) / dshotLatencyOfSingleEmptyLoopNs;
    dshotBusyLoopIterationsForBitPart2 = (DSHOT_T0H_us * 1000 - dshotLatencyOfGpioWriteNs + dshotAutoCorrection) / dshotLatencyOfSingleEmptyLoopNs;
    dshotBusyLoopIterationsForBitPart3 = (DSHOT_T1L_us * 1000 - dshotLatencyOfGpioWriteNs - 4*tick + dshotAutoCorrection*DSHOT_T1L_us/DSHOT_T0H_us) / dshotLatencyOfSingleEmptyLoopNs;
    
    // printf("debug TBIT0, TBIT1, TBIT2 == %d, %d, %d\n", dshotBusyLoopIterationsForBitPart1, dshotBusyLoopIterationsForBitPart2, dshotBusyLoopIterationsForBitPart3); fflush(stdout);
}

int64_t dshotTestTheTimingOfaFunction(void (*fun)()) {
    long long 	t, t1, t2, res;
    int 	i;
    
    // compute how much time gets the function fun
    res = (1LL<<60);
    for(i=0; i<AUTOCALIBRATION_TESTS; i++) {
	usleep(USLEEP_BEFORE_BROADCAST);
	t1 = dshotGetNanoseconds();
	fun();
	t2 = dshotGetNanoseconds();
	t = t2 - t1;
	// take the smallest value, i.e. the one which took whole CPU.
	if (t < res) res = t;
    }
    return(res);
}

void dshotTestFunctionNothing() {
}
void dshotTestFunctionEmptyLoop() {
    int k;
    for(k=0; k<AUTOCALIBRATION_EMTY_LOOP_N; k++) {
	asm("");
    }
}
void dshotTestFunctionGpioWrite() {
    int k;
    for(k=0; k<AUTOCALIBRATION_GPIO_WRITE_N; k++) {
	asm("");
	GPIO_CLR = dshotAllMotorsPinsMaskForTests;
    } 
}

void dshotCalibrateGetNanoseconds() {
    long long 	t;

    t = dshotTestTheTimingOfaFunction(dshotTestFunctionNothing);
    dshotLatencyOfGetNanosecondsNs = t;
    printf("debug dshotLatencyOfClockGettime == %gns\n", dshotLatencyOfGetNanosecondsNs); fflush(stdout);
}

void dshotCalibrateEmptyLoop() {
    long long 	t;

    t = dshotTestTheTimingOfaFunction(dshotTestFunctionEmptyLoop);
    dshotEmptyLoopTestEtaNs = t - dshotLatencyOfGetNanosecondsNs;
    dshotLatencyOfSingleEmptyLoopNs = dshotEmptyLoopTestEtaNs / AUTOCALIBRATION_EMTY_LOOP_N;
    printf("debug Empty loop test ETA: %3.0fus; dshotLatencyOfSinglePassInEmptyLoopNs == %gns\n", dshotEmptyLoopTestEtaNs/1000.0, dshotLatencyOfSingleEmptyLoopNs); fflush(stdout);
}

void dshotCalibrateGpioWrite() {
    long long 	t;

    t = dshotTestTheTimingOfaFunction(dshotTestFunctionGpioWrite);
    dshotGpioWriteTestEtaNs = (t - dshotLatencyOfSingleEmptyLoopNs * AUTOCALIBRATION_GPIO_WRITE_N - dshotLatencyOfGetNanosecondsNs);
    dshotLatencyOfGpioWriteNs = dshotGpioWriteTestEtaNs / AUTOCALIBRATION_GPIO_WRITE_N;
    printf("debug Gpio write test ETA: %3.0fus; dshotLatencyOfGpioWrite == %gns\n", dshotGpioWriteTestEtaNs/1000.0, dshotLatencyOfGpioWriteNs); fflush(stdout);
}

uint32_t dshotGetAllMotorsPinMask(int motorPins[], int motorMax) {
    int 	i;
    uint32_t 	allMotorsPinsMask;

    // compute masks
    allMotorsPinsMask = 0;
    for(i=0; i<motorMax; i++) allMotorsPinsMask |= (1<<motorPins[i]);
    return(allMotorsPinsMask);
}

void dshotInitialCalibration(int motorPins[], int motorMax) {

    dshotAllMotorsPinsMaskForTests = dshotGetAllMotorsPinMask(motorPins, motorMax);
    dshotCalibrateGetNanoseconds();
    dshotCalibrateEmptyLoop();
    dshotCalibrateGpioWrite();
    dshotUpdateConfigurationForSendFrame();
}

void dshotMaybePrintFrameDebugInfo(int64_t t) {
    static int filter = 1;
    static int goodFrames = 0;
    static int allFrames = 0;
    
    allFrames ++;
    // Consider the frame as good if timing was within 1%
    if (ABS(t-DSHOT_FRAME_ns) < DSHOT_FRAME_ns/100.0) goodFrames ++;
    
    // Do not print debug info for each frame, only from time to time
    if (filter ++ % 100 != 0) return;
    
    printf("debug last dshot frame sent in %g usec. Expected %g\n", t / 1000.0, DSHOT_FRAME_ns / 1000);
    printf("debug Good frames: %d; All frames: %d;  ratio == %5.2f %%\n", goodFrames, allFrames, 100.0*goodFrames/allFrames);
    fflush(stdout);
    
    // reset values after each info line
    goodFrames = allFrames = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// raspilot motor instance implementation

void motorImplementationInitialize(int motorPins[], int motorMax) {
    int i, pin;
    
    dshotSetupIo();
    
    for(i=0; i<motorMax; i++) {
	pin = motorPins[i];
	INP_GPIO(pin); // must use INP_GPIO before we can use OUT_GPIO
	OUT_GPIO(pin);
	GPIO_CLR = 1<<pin;
    }

    dshotInitialCalibration(motorPins, motorMax);
}

void motorImplementationFinalize(int motorPins[], int motorMax) {
    munmap(dshotGpioMap, BLOCK_SIZE);
}

void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) {
    int		repeat, i, bi;
    unsigned	frame[NUM_PINS+1];
    unsigned	bit;
    uint32_t	msk, allMotorsMask;
    uint32_t	clearMasks[16];
    int64_t	t, t1, t2;

    assert(motorMax < NUM_PINS);

    allMotorsMask = dshotGetAllMotorsPinMask(motorPins, motorMax);

    // translate double throttles ranging <0, 1> to dshot frames.
    for(i=0; i<motorMax; i++) frame[i] = dshotAddChecksumAndTelemetry(motorThrottle[i] * 1999 + 48, 0);

    // compute masks for zero bits in all frames
    for(bi=0; bi<16; bi++) {
	msk = 0;
	bit = (0x8000 >> bi);
	for(i=0; i<motorMax; i++) {
	    if ((frame[i] & bit) == 0) msk |= (1<<motorPins[i]);
	}
	clearMasks[bi] = msk;
    }

    // Everything is ready, we can broadcast the messages and meassure
    // the broadcasting time. Eventually send the frames up to 5 times
    // if the previous timing was wrong.
    repeat = 0; 
    for(repeat = 0; repeat < 5; repeat++) {

	// relax to OS for a small period of time, it reduces the probability that we will
	// be interrupted during broadcasting.
	usleep(USLEEP_BEFORE_BROADCAST);
	
	t1 = dshotGetNanoseconds();
	// send all dshot frames at once
	dshotSendFrames(allMotorsMask, clearMasks);
	t2 = dshotGetNanoseconds();
	t = t2 - t1 - dshotLatencyOfGetNanosecondsNs;

	dshotMaybePrintFrameDebugInfo(t);
	
	// If better than 1% difference, we are done. It also depends on how tolerant the ESC is.
	if (ABS(t - DSHOT_FRAME_ns) < DSHOT_FRAME_ns*1/100) break;

	// Frame was sent broken, adjust the calibration in the right
	// direction Do not adjust for times completely out of
	// range. Those are due to OS interrupts or something.
	if (t < DSHOT_FRAME_ns && t > DSHOT_FRAME_ns/2) dshotAutoCorrection ++;
	else if (t > DSHOT_FRAME_ns && t < DSHOT_FRAME_ns*2) dshotAutoCorrection --;
	dshotUpdateConfigurationForSendFrame();
    }
}


