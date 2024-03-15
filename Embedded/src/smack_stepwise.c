/* ============================================================================
** Copyright (c) 2021 Infineon Technologies AG
**               All rights reserved.
**               www.infineon.com
** ============================================================================
**
** ============================================================================
** Redistribution and use of this software only permitted to the extent
** expressly agreed with Infineon Technologies AG.
** ============================================================================
*
*/

/** @file     smack_stepwise.c
 *  @brief    Smack stepwise motor operation
 *
 *  This example shows how to operate a motor which needs more power that can be harvested from the NFC field step
 *  by step without the need to provide one big and expensive capacitor that can store the total energy needed to
 *  operate the motor. Instead, a smaller and cheaper capacitor can be used to store a part of this energy, move
 *  the motor a small step, and repeat this until the desired movement has been completed.
 *
 *  There are two different examples in this file, selectable and configurable in the file "settings.h". One is
 *  using a simple, timer based scheme to perform the stepwise movement. As the timer settings are set before
 *  compile time, this scheme is suitable for situations with a guaranteed NFC field stength, or applications
 *  where external circuitry (e.g. voltage comparator) can be used to initiate the next step.
 *  The second scheme uses internal comparator functionality to observe the voltage of the external capacitor
 *  which provides the energy to drive the motor. By comparing the capacitor voltage, this scheme speeds up
 *  operation where more power can be harvested from strong NFC fields and ensures proper operation in weaker
 *  fields by prolonging the charge cycles of the capacitor.
 */

// standard libs
// included by core_cm0.h: #include <stdint.h>
#include "core_cm0.h"
#include <stdbool.h>

// Smack ROM lib
#include "rom_lib.h"

// Smack NVM lib
#include "sys_tim_lib.h"
#include "shc_lib.h"

// Smack stepwise project
#include "settings.h"
#include "smack_stepwise.h"


//#undef wait_about_1ms
//#define wait_about_1ms  (XTAL / 1000U)          // calculated from oscillator; more exact than rough estimate
#define ms2ticks(ms)    ((ms) * WAIT_ABOUT_1MS)

/** The functions which drive the motor are using timers in a different manner to estimate if the operationis completed.
 *  The timer channels to be used are defined here.
 */
#define TIMER_SINGLE    1       // timer # used for single shot delays
#define TIMER_CLOCK     5       // timer # configured together with the preceeding timer as a free running 32 bit timer to be used for timing measurements

// interrupt number used in system timer calls
#define SYSTIM_IRQ      9



/** _nvm_start() is the main() routine of the application code:
 * - when building the image (rom or ram), it is called by Reset_Handler() (see startup_smack.c) after SystemInit().
 * - when building and running the unit tests, it is not called, as far as I know, it is not subject to unit testing.
 * - when building and running integration and/or system tests, it is called by sc_main() upon simulation start.
 * This is also the reason of why it is called _start() and not main(): The VP has a higher layer main() which
 * calls sc_main() and _start(). Having two main's will fail when linking the VP executable.
 *
 * @note _start() cannot be made static, it is referenced by startup_smack.c and also by the interface
 * to the Virtual Prototype. But linting does not know about such external references. The default
 * approach to solve is to add '//lint -e765'. Our linter is plain old, we don't get a new one thanks to
 * the application owner from IT and it has a bug which renders -e765 useless. The only option we had
 * is to suppress the warning in au_misra2_fixes.lnt
 *
 * @return nothing
 */

// prototypes
void _nvm_start(void);

#if STEPWISE_METHOD == STEPWISE_TIMER_CONTROLLED
static void drive_motor_timer_controlled(void);
#endif

#if STEPWISE_METHOD == STEPWISE_VOLTAGE_CONTROLLED
static void drive_motor_voltage_controlled(void);
#endif


/** @brief main function
 *  This function performs initialization and a background task after motor operation is completed.
 *  The motor operation is done in a dedicated function as configured by the user.
 */
void _nvm_start(void)
{

    // ******************* Test of hb_ctrl *********************

    set_hb_eventctrl(false);


#if STEPWISE_METHOD == STEPWISE_TIMER_CONTROLLED
    drive_motor_timer_controlled();
#elif STEPWISE_METHOD == STEPWISE_VOLTAGE_CONTROLLED
    drive_motor_voltage_controlled();
#else
#error Please select STEPWISE_METHOD
#endif


    // background task is just an endless
    while (true)
    {
        asm("WFI");
    }

}


#if STEPWISE_METHOD == STEPWISE_TIMER_CONTROLLED

/** @brief Simple timer based stepwise motor operation
 *
 *  This function charges an external capacitor on the VCCHB pin for a fixed amount of time to store energy.
 *  Then, the H bridge is switched on for a fixed amount of time to operate a motor from the stored energy.
 *  This scheme is repeated until the motor has reached its desired movement, e.g. until the configured number
 *  of steps was done.
 *
 *  The charging time of the capacitor as well as the running time of the motor are defined before compiling
 *  the firmware. They are estimated from tests, calculations or simulations and assume are based on an
 *  assumption regarding the power delivery through the NFC field. If the NFC field and the harvested power
 *  is lower, then this scheme may show less motor movement than necessary for a given device.
 *
 *  With a simple external charge detector circuitry, this scheme can be enhanced to dynamically adjust to
 *  different NFC harvesting levels. Just replace the timer based recharge delay with a loop that waits for
 *  the detection circuit to report a fully charged capacitor. After that, the well know amount of stored
 *  energy will operate the motor for a certain amount of time which can be predetermined from tests.
 *  Also, if you do not want to preset the number of steps, you may add a detection circuitry for the motor
 *  movement (switch, optical sensor).
 */
static void drive_motor_timer_controlled(void)
{
    uint32_t count;
    uint32_t i;

    /* In every step, the motor performs a well known movement. After a given number of steps, the desired
     * total movement was performed. Set the loop count here to the configured value.
     */
    count = LOOP_COUNT_COMPUTED;

    /* When an NFC reader (e.g. smartphone) connects to the device, we assume that the external capacitor
     * on the VCCHB pin is empty and needs a longer time to fully charge than in the following cycles. So
     * we wait for the time configured for the initial charge.
     */
    sys_tim_singleshot_32(TIMER_SINGLE, wait_about_1ms * (DELAY_INITIAL_CHARGE), SYSTIM_IRQ);

    /* Now we perform a loop to perform a movement step by step:
     * - operate the motor with energy from the capacitor
     * - stop the motor and recharge the capacitor
     */
    i = 0;

    while (i < count)
    {
        /* Here we start the motor. This is done in two steps: first connect one side of the motor to the
         * positive voltage, then the other side to the negative voltage.
         */
        set_hb_switch(true, false, false, false);
        set_hb_switch(true, false, false, true);

        /* Now the motor is running. Before switching it off, we wait for a fixed amount of time, and let
         * the motor perform one step of the movement.
         */
        sys_tim_singleshot_32(TIMER_SINGLE, wait_about_1ms * (DELAY_MOTOR_RUN), SYSTIM_IRQ);

        /* Now one step of the movement is done, and the H bridge outputs are switched off to stop the motor.
         */
        set_hb_switch(false, false, false, false);

        /* The operation of the motor has drained the capacitor. Now the motor is switched off, and the
         * harvested energy flows into the capacitor and recharges it. Just wait until the capacitor is
         * fully charged again.
         * Note: After the last motor movement, this delay can be skipped.
         */
        sys_tim_singleshot_32(TIMER_SINGLE, wait_about_1ms * (DELAY_MOTOR_OFF), SYSTIM_IRQ);

        /* Now the capacitor is fully charged again, and we can continue with the next step - or finish
         * if all steps are done.
         */
        i++;
    }

    /* All done now. Just make sure the H bridge is switched off.
     */
    set_hb_switch(false, false, false, false);
}

#endif


#if STEPWISE_METHOD == STEPWISE_VOLTAGE_CONTROLLED

/** @brief Stepwise motor operation with observation of capacitor state
 *
 *  This function charges an external capacitor on the VCCHB pin to store energy for motor operation until
 *  fully charged. Then, the H bridge is switched on to operate a motor from the stored energy until the
 *  voltage level reaches a level where the motor may stall. This scheme is repeated until the motor has
 *  reached its desired movement, e.g. until the configured runtime which is needed to perform this movement
 *  has been reached.
 *
 *  In contrast to the timer based example above, this scheme takes into account how much energy can be
 *  harvested from the NFC field. If the device is exposed to an NFC reader which delivers a strong field,
 *  charging times will be faster, and the operation of the device will be finished in a shorter time. With
 *  a weak field, charging times will be longer, and the motor runtime during one step will be shorter, but
 *  the device still will ensure that the operation is completed by doing more steps of motor movement and
 *  wait longer for a proper recharge of the VCCHB capacitor.
 *
 *  This diagram shows the voltage on the VCCHB capacitor and the control of the motor operation by the
 *  firmware depending on what the comparator reports about theVCCHB voltage. Please note that between
 *  the second and the third step there is a situation shown with a worse energy harvesting:
 *
 *                              ___        ___             ___
 *  Motor on:    ______________/   \______/   \___________/   \______
 *
 *  VCCHB:
 *                             _          _               _
 *  VOLTAGE_ON -             _/ \       _/ \          ___/ \        _
 *                         _/    \    _/    \     ___/      \    __/
 *  VOLTAGE_OFF-         _/       \__/       \___/           \__/
 *                     _/
 *                   _/
 *            0-  __/
 *
 */
static void drive_motor_voltage_controlled(void)
{
    uint32_t total_on, timestamp_on, timestamp_off, target_off;
    bool state, run, cmp;

    /* Set initial state:
     * - remember that motor is switched off (state = false)
     * - ensure that H bridge is switched off (set_hb_switch())
     */
    set_hb_switch(false, false, false, false);
    state = false;

    /* The duration of each step of motor operation as well as the recharge steps are adjusted dynamically
     * and not known yet. To judge the motor movement that was performed we keep track of the time that the
     * motor was switched on. For this, we need some kind of a clock.
     * The system timer unit provides several timers with a width of 16 bits which can be concatenated. Clocked
     * with 28 MHz, a single timer will overflow after about 2ms. The Smack NVM library provides a function that
     * concatenates two of the timers to build a 32 bit timer which can be used to measure time distances of more
     * than 2 minutes. This function is used to start a background clock which will be queried on state changes,
     * and the results will be used to calculate time distances between those queries.
     */
    // start free running timer as a clock for time measurements; use ticks as a basis as on all other timers
    sys_tim_cyclic_cascaded(TIMER_CLOCK, 0xffff, 0xffff);

    /* As in the simple timer example above, we assume that the capacitor on the VCCHB pin is empty when the device
     * is exposed to an NFC reader, so we perform an initial charge for a fixed amount of time.
     * However, it is allowed that the capacitor is only partially charged after this time has passed. When entering
     * the loop below, a voltage comparator will ensure that the capacitor will be fully charged before motor opeation
     * starts.
     */
    sys_tim_singleshot_32(TIMER_SINGLE, WAIT_ABOUT_1MS * (DELAY_INITIAL_CHARGE), SYSTIM_IRQ);

    /* The voltage comparator cannot see the voltage on the VCCHB pin but the voltage on one of hte H bridge pins.
     * By closing the top switch of the H bridge, we connect the VCCHB voltage to the MA pin of the H bridge, and
     * via this trick, the voltage on the VCCHB pin can be fed into the comparator.
     * Whenever the comparator is used to detect thresholds on the VCCHB pin and the external capacitor connected to
     * it, one of the top switches of the H bridge must be in the closed state, and the proper H bridge output must
     * be selected as an input for comparison. Care must be taken about the low side switches inside the H bridge.
     * Both low side switches shall be switched off unless motor operation is intended, and if the motor shall be
     * operated, the comparator input must be selected upon the H bridge settings needed for the motor operation.
     */
    set_hb_switch(true, false, false, false);

    // todo: time delay after on voltage, total motor runtime

    /* Assumption: The amount of motor movement depends on the time the motor is switched on, and the motor movement
     * is the same if the motor runs for this time in one big step, or if this time is divided into multiple smaller
     * steps.
     * So we sum up the motor run time in "total_on", and when the desired runtime was summed up, the motor movement
     * was completed.
     */
    total_on = 0;
    timestamp_on = 0;
    run = true;

    /* The comaprator circuitry requires initialization through shc_init(). This function must be called before we can
     * call shc_compare().
     */
    shc_init();

    /* The following loop implements a two point regulation. It waits in the "off" until the VCCHB capacitor is fully
     * charged (e.g. upper threshold reached), then it switches to the "on" state.
     * In the "on" state, the motor is switched on, and the comparator is queried if the VCCHB voltage drops to a
     * threshold where the motor may stall.
     * If the VCCHB voltage has dropped below this lower threshold, the motor is stopped, and we are in the "off" state
     * agein, and we start at the beginning again.
     * This loop runs until the preconfigured total motor runtime is reached.
     */
    while (run)
    {
        if (state)
        {
            /* Here we are in the "on" state. The motor is switched on, and we observe the voltage of our power source,
             * the capacitor on the VCCHB pin, if it drops below a threshold that may be insufficiant for proper motor
             * operation.
             */
            cmp = shc_compare(shc_channel_ma, VOLTAGE_OFF);

            if (!cmp || ((int32_t)(sys_tim_cyclic_cascaded_get_combined(TIMER_CLOCK) - target_off) > 0))
            {
                /* Now the voltage of the capacitor has dropped below the threshold where the motor will operate properly.
                 * The capacitor on the VCCHB pin needs to be recharged to sotre energy for the next step of motor movement.
                 * So, we switch off the power sink (motor) and let the available energy flow into the capacitor.
                 * Other reason we may drop in here: motor movement is finished -> job done.
                 */

                /* First, we remember the time when the motor is switched off in order to calculate the motor runtime. Then
                 * we switch off the motor and remember the new state.
                 */
                timestamp_off = sys_tim_cyclic_cascaded_get_combined(TIMER_CLOCK);
                set_hb_switch(true, false, false, false);
                state = false;

                /* We also remembered the time when we switched on the motor. Here, we can calculate the difference, e.g. the
                 * motor runtime in this step, and sum it up in "total_on".
                 * Remember that we started a cascaded timer pair from the system timer block to act as a free running 32 bit
                 * in background. We can calculate time differences simply be "now - then".
                 */
                total_on += timestamp_off - timestamp_on;

                /* If total motor runtime was reached (e.g. movement done), leave loop.
                 */
                if (total_on >= ms2ticks(TOTAL_MOTOR_RUNTIME))
                {
                    run = false;
                    break;
                }
            }
        }
        else
        {
            /* This is the branch for the "off" state. We observe the voltage on the VCCHB capacitor for reaching a "full"
             * threshold. Due to possible variances between comparator and clamping circuit, we actually have to compare
             * against a threshold that is somwhat lower than the "full charged" voltage. the remainder of the charging phase
             * then is realized as a timer based charging step.
             */
            cmp = shc_compare(shc_channel_ma, VOLTAGE_ON);

            if (cmp)
            {
                /* The comparator reported that the capacitor on the VCCHB pin is almost full, and we may continue with the
                 * next step of motor movement.
                 *
                 * First, if configured, charge for some additional time to ensure that the capacitor is really full.
                 */
#if defined DELAY_ADDITIONAL_CHARGE && DELAY_ADDITIONAL_CHARGE
                sys_tim_singleshot_32(TIMER_SINGLE, ms2ticks(DELAY_ADDITIONAL_CHARGE), SYSTIM_IRQ);
#endif

                /* When the motor is switched on, for a short period it draws a higher startup current, e.g. builds up
                 * some kinetic energy in its rotating parts that will result in some further movement after the motor
                 * has been switched off. To account for this additional movement, you may configure a correction value
                 * that add a "motor on" time equivalent for this further movement to the "total_on" variable.
                 */
#ifdef MOTOR_START_CORRECTION
                total_on += ms2ticks(MOTOR_START_CORRECTION);
#endif

                /* Remember the time when we started the motor. Needed later to calculate the runtime during the next motor
                 * movement step.
                 */
                timestamp_on = sys_tim_cyclic_cascaded_get_combined(TIMER_CLOCK);

                /* The "on" state shall be left when the total motor movement was done, e.g. the total motor runtime has been
                 * reached. To make it easier in the "on" state, we calculate a time when to switch off the motor here, so we
                 * only need to compare the current time against this target in the "on" state rather than performing some
                 * calculations on every check then.
                 */
                target_off = timestamp_on + (ms2ticks(TOTAL_MOTOR_RUNTIME) - total_on);

                /* Actually start the motor and remember the state
                 */
                set_hb_switch(true, false, false, true);
                state = true;
            }
        }

        /* Between each loop, enter a low power sleep for a short period of time.
         * The single_shot_systick() function sets up the system ticker and then puts the CPU to a low power mode by executing
         * a WFI instruction. The WFI completes, if the system ticker expires and issues an interrupt, but also on any other
         * interrupt. If other interrupts are active, the actual delay may be shorter than requested. The NVM library provides
         * other delay functions which are also using WFI to reduce power consumption but check the timer status in order to
         * produce a fixed delay, e. g. sys_tim_singleshot_32().
         */
        single_shot_systick(ms2ticks(10));
    }

    /* Motor operation done -> ensure that H bridge is switched off
     */
    set_hb_switch(false, false, false, false);

    /* Switch off the peripherals used in this function in order to conserve some power:
     * - switch off the comparator module (e.g. reset analog paths, stop clocks)
     * - switch off system timer in two steps: halt running timer, then stop module
     */
    shc_close();
    sys_tim_cyclic_cascaded_stop(TIMER_CLOCK);
    sys_tim_close();
}

#endif


// In case of a Hardfault, spin in a loop for a while before resetting so that a debugger may connect
void hardfault_handler(void)
{
    uint32_t cnt = 30000000;

    while (cnt--)
    {
        __NOP();
    }

}
