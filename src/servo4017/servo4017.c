#include "servo4017/servo4017.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/cm3/nvic.h>

void servo4017_setServoValue(servo4017_t *dev, uint8_t servoIdx, uint16_t pulseLength_ms)
{

    const uint16_t defaultTickCount = ((1e9) / dev->updateRate_Hz) / 8 / dev->tickLength_ns;

    // Determine how many ticks to set it high for
    dev->pulseLengths_ticks[servoIdx] = 1000 * 1000 * pulseLength_ms / dev->tickLength_ns;

    // Figure out
    dev->extraTime_ticks = 0;
    for (int sIdx = 0; sIdx < 8; sIdx++)
    {
        dev->extraTime_ticks += dev->pulseLengths_ticks[sIdx] - defaultTickCount;
    }

    // apply extra for fixing the update rate
    if (dev->extraTime_ticks >= 65535)
    {
        dev->pulseLengths_ticks[8] = 65535;
    }
    else if (dev->extraTime_ticks < 0)
    {
        dev->pulseLengths_ticks[8] = 0;
    }
    else
    {
        dev->pulseLengths_ticks[8] += dev->extraTime_ticks;
    }
}

void servo4017_initialiseDevice(servo4017_t *dev)
{
    for (uint8_t servoIdx = 0; servoIdx < 8; servoIdx++)
    {
        servo4017_setServoValue(dev, servoIdx, 1500);
    }
}

void servo4017_setup(servo4017_t *dev)
{

    // Set default values for the commanded values
    servo4017_initialiseDevice(dev);

    // Uses a single 4017 and a timer to drive multiple servos
    //	With thanks to Rue Mohr
    //
    //	CLOCK: PA6, TIM3_CH1
    //	RESET: PA4, GPIO

    // First configure reset line
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);

    // Then configure the clock line
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6);

    // Then configure timer
    rcc_periph_clock_enable(RCC_TIM3);
    nvic_enable_irq(NVIC_TIM3_IRQ);
    rcc_periph_reset_pulse(RST_TIM3);

    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(TIM3, 65535);
    timer_set_prescaler(TIM3, 2);
    timer_disable_preload(TIM3);

    timer_continuous_mode(TIM3);

    timer_enable_oc_output(TIM3, TIM_OC1);
    timer_disable_oc_clear(TIM3, TIM_OC1);            //     as per OC1CE page 328
    timer_disable_oc_preload(TIM3, TIM_OC1);          //     as per OC1PE page 329
    timer_set_oc_slow_mode(TIM3, TIM_OC1);            //     as per OC1FE page 329
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_ACTIVE); //     as per OC1M  page 329
    timer_set_oc_value(TIM3, TIM_OC1, 30000);
    timer_set_oc_polarity_high(TIM3, TIM_OC1); //     as per CC1P page 333
    timer_enable_oc_output(TIM3, TIM_OC1);     //     as per CC1E page 333

    timer_enable_counter(TIM3);

    nvic_enable_irq(NVIC_TIM3_IRQ);
    timer_enable_irq(TIM3, TIM_DIER_CC1IE); // compare 1 interrupt
}