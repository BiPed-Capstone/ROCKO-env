/*
 * STUB FOR COMPILE CHECK ONLY. This is NOT WiringPi's softPwm.
 *
 * Declares just the software-PWM functions Motor12VoltQuadEncoder.cpp calls, with the real
 * signatures. Declarations only: nothing is defined, so nothing can be linked or run. See
 * wiringPi.h in this folder.
 */
#ifndef ROCKO_STUB_SOFTPWM_H
#define ROCKO_STUB_SOFTPWM_H

#ifndef ROCKO_COMPILE_CHECK_ONLY
#error "tools/compile_check/stubs/softPwm.h is a stub for check_motor_plugin_compiles.sh only"
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern int softPwmCreate(int pin, int initialValue, int pwmRange);
extern void softPwmWrite(int pin, int value);
extern void softPwmStop(int pin);

#ifdef __cplusplus
}
#endif

#endif  // ROCKO_STUB_SOFTPWM_H
