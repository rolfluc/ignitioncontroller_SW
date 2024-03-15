/*
 * settings.h
 *
 *  Created on: 20.11.2020
 *      Author: hs
 */

#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_


//=================================================================
// Global definitions


//=================================================================
// test cases



//=================================================================
// Smack application to drive a motor stepwise:
// First, the HB cap is charged to max. voltage
// Then, the  motor is switched on for a short period until the cap voltage drops to
// a level where the motor may stall. Thus, the motor is switched off to charge the
// cap again to full voltage. This repeats a couple of times to drive the motor to the
// intended stop position.
//
// duration of the three phases of operation, given in milliseconds (probably; tbd)

// There are different methods to control operation of the motor.
// The simplest is a static, timer controlled approach, where assumptions are made
// how long it takes to charge the HB cap and how fast the motor will drain the cap.
// This approach does not adjust for variations in energy harvesting and motor load.
// Another approach uses measurements of the HB cap voltage to decide about motor
// operation. This approach may be improved by using timer control for some of the
// phases.

// timer controlled operation: all three phases are controlled by fixed timings:
// 1st initial charge of the HB cap, then in a loop: 2nd the motor on phase, then 3rd
// a recharge phase where motor is off.
#define STEPWISE_TIMER_CONTROLLED       1

// voltage controlled operation with hysteresis: motor is switched on when the HB cap
// reaches "on" voltage and switched off when reaching "off" voltage.
#define STEPWISE_VOLTAGE_CONTROLLED     2


// pick the desired strategy from options listed above:
#define STEPWISE_METHOD         STEPWISE_VOLTAGE_CONTROLLED


//-----------------------------------------------------------------
// Global settings

// total time the motor shall run
// number of cycles will be calculated from configured timing or from actual measurement (e.g. when using voltage controlled strategy)
#define TOTAL_MOTOR_RUNTIME     2000

// correction for higher current drawn by motor drawn in spin up phase
// every time the motor is switched on this amount is added to the total run time
// remark: currently only supported with voltage controlled method
#define MOTOR_START_CORRECTION  10


//-----------------------------------------------------------------
// Settings for voltage controlled operations

// start with a grace period for initial an initial charge of HB cap
#define DELAY_INITIAL_CHARGE_2  500

// wait a bit before switching on the motor after reaching the upper limit of CA capacitor voltage
// upper voltage must be configured slightly below the HW limit, and this delay will allow to charge the remaining bit as well
// set to 0 to disable this option
#define DELAY_ADDITIONAL_CHARGE 50

// specify voltage levels in millivolts (add about 2% or 3% for a better match of the prescaler, and calculate a safety margin for tolerances)
// clamping voltage is 3.3V
#define VOLTAGE_ON              3100
#define VOLTAGE_OFF             2200


//-----------------------------------------------------------------
// Settings for timer controlled operation

// initial delay to charge HB capacitor
#define DELAY_INITIAL_CHARGE    1500

// run motor until capacitor is drained a bit
#define DELAY_MOTOR_RUN         500

// switch off motor and let capacitor charge again
#define DELAY_MOTOR_OFF         300

// define # of loops or total motor runtime
//#define LOOP_COUNT              1
//see above: #define TOTAL_MOTOR_RUNTIME     2000

#ifdef LOOP_COUNT               // put priority on loop count
#define LOOP_COUNT_COMPUTED     LOOP_COUNT
#else
#define LOOP_COUNT_COMPUTED    (((TOTAL_MOTOR_RUNTIME) + (DELAY_MOTOR_RUN) - 1U) / (DELAY_MOTOR_RUN))
#endif


#endif /* INC_SETTINGS_H_ */
