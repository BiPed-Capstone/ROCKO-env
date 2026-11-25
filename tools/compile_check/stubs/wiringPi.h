/*
 * STUB FOR COMPILE CHECK ONLY. This is NOT WiringPi.
 *
 * It declares just the WiringPi names Motor12VoltQuadEncoder.cpp uses, with the real signatures
 * and the real constant values, so that g++ can type-check that file off a Raspberry Pi. It
 * defines nothing, so nothing that includes it can be linked into a working program. The real
 * header is the one on the Pi (and the copy in rocko_env/rocko_env/hardware/WiringPi/).
 *
 * Used only by check_motor_plugin_compiles.sh, which passes -DROCKO_COMPILE_CHECK_ONLY. If this
 * file is ever found on a real build's include path, the #error below stops that build.
 */
#ifndef ROCKO_STUB_WIRINGPI_H
#define ROCKO_STUB_WIRINGPI_H

#ifndef ROCKO_COMPILE_CHECK_ONLY
#error "tools/compile_check/stubs/wiringPi.h is a stub for check_motor_plugin_compiles.sh only"
#endif

// Pin modes, same values as the real wiringPi.h
#define INPUT 0
#define OUTPUT 1
#define SOFT_PWM_OUTPUT 4

// Pin levels and interrupt edges, same values as the real header (enums there, WiringPi 3.20)
#define LOW 0
#define HIGH 1
#define INT_EDGE_BOTH 3

#ifdef __cplusplus
extern "C" {
#endif

// The interrupt record WiringPi passes an ISR, as in the real header (V3.16 and later)
struct WPIWfiStatus {
    int statusOK;
    unsigned int pinBCM;
    int edge;
    long long int timeStamp_us;
};

extern int wiringPiSetup(void);
extern void pinMode(int pin, int mode);
extern void digitalWrite(int pin, int value);
extern int digitalRead(int pin);
extern int wiringPiISR2(int pin, int edgeMode, void (*function)(struct WPIWfiStatus wfiStatus, void* userdata),
                        unsigned long debounce_period_us, void* userdata);
extern int wiringPiISRStop(int pin);

#ifdef __cplusplus
}
#endif

#endif  // ROCKO_STUB_WIRINGPI_H
