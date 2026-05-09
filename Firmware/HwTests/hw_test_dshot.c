#include "hw_test_dshot.h"
#include "hw_test_runner.h"

#include "hal.h"
#include "mc/dshot.h"

#include <stdint.h>
#include <stdbool.h>

/*
 * Hardware loopback: PC6 (TIM8_CH1 DShot output) → PA6 (TIM3_CH1 input capture).
 * Connect with a jumper wire on the Nucleo board before running.
 *
 * TIM3 PWM-input capture mode:
 *   CH1 (rising edge, slave reset) captures bit period  → TIM3->CCR1
 *   CH2 (falling edge, indirect)  captures high time    → TIM3->CCR2
 *
 * Timer clock: 64 MHz (HSI, APB1 prescaler /2 → timer clock = APB1 × 2 = 64 MHz).
 * Expected values:
 *   Bit period  = 427 ticks  (≈ 6.67 µs)
 *   '1' high    = 320 ticks  (≈ 5.00 µs)
 *   '0' high    = 160 ticks  (≈ 2.50 µs)
 * Tolerance: 5% → ±21 ticks for period, ±16 ticks for high time.
 */

#define DSHOT_PERIOD_TICKS   427U
#define DSHOT_TICKS_1        320U
#define DSHOT_TICKS_0        160U
#define PERIOD_TOL           ((DSHOT_PERIOD_TICKS * 5U) / 100U)   /* ±21 */
#define PULSE_TOL            ((DSHOT_TICKS_1 * 5U) / 100U)        /* ±16 */

/* Throttle values chosen so the last data bit (bit 0 of packet) is predictable:
 *   value=0  → packet=0x0000 → last bit=0 → high=160
 *   value=8  → packet=0x0101 → last bit=1 → high=320
 * (See DShotPreparePacket: checksum bit 0 = bit3(value) ^ bit7(value))
 */
#define THROTTLE_ALL_ZEROS   0U
#define THROTTLE_LAST_BIT_1  8U

static TIM_HandleTypeDef s_htim3 = { 0 };

static void SetupInputCapture(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA6 → TIM3_CH1 (AF2) */
    GPIO_InitTypeDef gpio = { 0 };
    gpio.Pin       = GPIO_PIN_6;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* TIM3 base: free-running, no prescaler, ARR=0xFFFF */
    s_htim3.Instance               = TIM3;
    s_htim3.Init.Prescaler         = 0;
    s_htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim3.Init.Period            = 0xFFFF;
    s_htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_IC_Init(&s_htim3);

    /* Slave mode: reset on TI1FP1 (CH1 rising edge) to get period in CCR1 */
    TIM_SlaveConfigTypeDef slave = { 0 };
    slave.SlaveMode    = TIM_SLAVEMODE_RESET;
    slave.InputTrigger = TIM_TS_TI1FP1;
    HAL_TIM_SlaveConfigSynchro(&s_htim3, &slave);

    /* CH1: capture period on rising edge (direct) */
    TIM_IC_InitTypeDef ic = { 0 };
    ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&s_htim3, &ic, TIM_CHANNEL_1);

    /* CH2: capture high time on falling edge (indirect from CH1 input) */
    ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
    ic.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&s_htim3, &ic, TIM_CHANNEL_2);

    HAL_TIM_IC_Start(&s_htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&s_htim3, TIM_CHANNEL_2);
}

/* Send one DShot frame and read back the last captured CCR1/CCR2. */
static void SendAndCapture(uint16_t throttle, uint32_t* out_period, uint32_t* out_high) {
    /* Clear stale flags */
    s_htim3.Instance->SR = 0;

    uint16_t vals[BRD_MOTOR_COUNT] = { throttle };
    DShotBB_Write(vals);

    /* Read CCR values after the frame.  PWM-input mode has already latched
     * the last complete bit's period (CCR1) and high time (CCR2). */
    *out_period = s_htim3.Instance->CCR1;
    *out_high   = s_htim3.Instance->CCR2;
}

void HwTestDShot_Run(void) {
    /* --- Test 1: Init --- */
    eSTATUS_t initStatus = DShotBB_Init();
    if (initStatus == eSTATUS_SUCCESS) {
        HwTest_ReportPass("hw_dshot_init");
    } else {
        HwTest_ReportFail("hw_dshot_init", "DShotBB_Init returned FAILURE");
        HwTest_Printf("RESULTS:%d/%d %s\r\n",
                      g_hw_tests_passed, g_hw_tests_total,
                      g_hw_tests_passed == g_hw_tests_total ? "PASS" : "FAIL");
        return;
    }

    SetupInputCapture();

    /* Small settle delay */
    HAL_Delay(5);

    /* --- Test 2: Bit period (any frame, all bits have the same ARR) --- */
    uint32_t period, high;
    SendAndCapture(THROTTLE_ALL_ZEROS, &period, &high);
    HW_ASSERT_WITHIN("hw_dshot_bit_period", DSHOT_PERIOD_TICKS, period, PERIOD_TOL);

    /* --- Test 3: '0' bit high-pulse width --- */
    HW_ASSERT_WITHIN("hw_dshot_bit0_pulse_width", DSHOT_TICKS_0, high, PULSE_TOL);

    /* --- Test 4: '1' bit high-pulse width --- */
    SendAndCapture(THROTTLE_LAST_BIT_1, &period, &high);
    HW_ASSERT_WITHIN("hw_dshot_bit1_pulse_width", DSHOT_TICKS_1, high, PULSE_TOL);

    /* --- Test 5: Frame length (18 bits × 427 ticks measured via TIM3 CNT) ---
     * Restart TIM3 counter, send a frame, then read CNT right after.
     * 18 × 427 = 7686 ticks.  Allow ±2% = ±154 ticks. */
    s_htim3.Instance->CNT = 0;
    uint16_t vals[BRD_MOTOR_COUNT] = { DSHOT_MIN_THROTTLE };
    DShotBB_Write(vals);
    uint32_t frame_ticks = s_htim3.Instance->CNT;
    HW_ASSERT_WITHIN("hw_dshot_frame_length",
                     DSHOT_DMA_BUFFER_SIZE * DSHOT_PERIOD_TICKS,
                     frame_ticks,
                     (DSHOT_DMA_BUFFER_SIZE * DSHOT_PERIOD_TICKS * 2U) / 100U);

    HwTest_Printf("RESULTS:%d/%d %s\r\n",
                  g_hw_tests_passed, g_hw_tests_total,
                  g_hw_tests_passed == g_hw_tests_total ? "PASS" : "FAIL");
}
