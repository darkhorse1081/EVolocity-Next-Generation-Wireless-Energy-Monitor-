#include "soc/rtc_cntl_reg.h"
#include "soc/soc_ulp.h"
#include "soc/sens_reg.h"

    .set adc_channel, 7    /* GPIO35 - current channel */
    .set adc_oversampling_factor_log, 2
    .set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

    .bss
    .global high_thr
high_thr:
    .long 0

    .global last_result
last_result:
    .long 0

    .text
    .global entry
entry:
    /* Power up SAR ADC */
    WRITE_RTC_REG(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR_S, 2, SENS_FORCE_XPD_SAR_PU)

    wait 1000

    move r0, 0
    stage_rst

measure:
    adc r1, 0, adc_channel + 1
    add r0, r0, r1
    stage_inc 1
    jumps measure, adc_oversampling_factor, lt

    rsh r0, r0, adc_oversampling_factor_log

    move r3, last_result
    st r0, r3, 0

    /* Power down SAR ADC */
    WRITE_RTC_REG(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR_S, 2, SENS_FORCE_XPD_SAR_PD)

    /* threshold - reading: overflow means reading > threshold */
    move r3, high_thr
    ld r3, r3, 0
    sub r2, r3, r0
    jump wake_up, ov

    halt

    .global wake_up
wake_up:
    READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
    and r0, r0, 1
    jump halt_retry, eq
    wake
    WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
    halt

halt_retry:
    halt