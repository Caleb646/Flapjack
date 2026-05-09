#include "test_dshot.h"

#include "hal.h"
#include "mc/dshot.h"
#include "target.h"
#include "unity.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Hardware loopback: PC6 (TIM8_CH1 DShot output) → PA6 (TIM3_CH1 input capture).
 * Connect with a jumper wire on the Nucleo board before running.
 *
 * TIM3 PWM-input capture mode:
 *   CH1 (rising edge, slave reset) captures bit period  → TIM3->CCR1
 *   CH2 (falling edge, indirect)  captures high time    → TIM3->CCR2
 *
 * Timer clock: 64 MHz (HSI, APB1 /2 → timer clock = APB1 × 2 = 64 MHz).
 * Expected values:
 *   Bit period  = 427 ticks  (≈ 6.67 µs)
 *   '1' high    = 320 ticks  (≈ 5.00 µs)
 *   '0' high    = 160 ticks  (≈ 2.50 µs)
 * Tolerance: 5% → ±21 ticks for period, ±16 ticks for high time.
 */

#define DSHOT_PERIOD_TICKS  427U
#define DSHOT_TICKS_1       320U
#define DSHOT_TICKS_0       160U
#define PERIOD_TOL          ((DSHOT_PERIOD_TICKS * 5U) / 100U)  /* ±21 */
#define PULSE_TOL           ((DSHOT_TICKS_1 * 5U) / 100U)       /* ±16 */

/* Throttle values chosen so the last data bit is predictable:
 *   value=0  → packet=0x0000 → last bit=0 → high=160
 *   value=8  → packet=0x0101 → last bit=1 → high=320 */
#define THROTTLE_ALL_ZEROS   0U
#define THROTTLE_LAST_BIT_1  8U

static TIM_HandleTypeDef s_htim3  = { 0 };
static bool              s_ready  = false;

void setUp(void)    {}
void tearDown(void) {}

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

    /* Slave mode: reset on TI1FP1 (CH1 rising edge) to capture period in CCR1 */
    TIM_SlaveConfigTypeDef slave = { 0 };
    slave.SlaveMode    = TIM_SLAVEMODE_RESET;
    slave.InputTrigger = TIM_TS_TI1FP1;
    HAL_TIM_SlaveConfigSynchro(&s_htim3, &slave);

    /* CH1: period on rising edge (direct) */
    TIM_IC_InitTypeDef ic = { 0 };
    ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&s_htim3, &ic, TIM_CHANNEL_1);

    /* CH2: high time on falling edge (indirect from CH1 input) */
    ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
    ic.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&s_htim3, &ic, TIM_CHANNEL_2);

    HAL_TIM_IC_Start(&s_htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&s_htim3, TIM_CHANNEL_2);
}

static void SendAndCapture(uint16_t throttle, uint32_t *out_period, uint32_t *out_high) {
    s_htim3.Instance->SR = 0;
    uint16_t vals[BRD_MOTOR_COUNT] = { throttle };
    DShotBB_Write(vals);
    *out_period = s_htim3.Instance->CCR1;
    *out_high   = s_htim3.Instance->CCR2;
}

/* ── Tests ────────────────────────────────────────────────────────────────── */

void test_hil_dshot_init(void) {
    TEST_ASSERT_EQUAL(eSTATUS_SUCCESS, DShotBB_Init());
    SetupInputCapture();
    HAL_Delay(5);
    s_ready = true;
}

void test_hil_dshot_bit_period(void) {
    TEST_ASSERT_TRUE_MESSAGE(s_ready, "skipped: DShot not initialized");
    uint32_t period, high;
    SendAndCapture(THROTTLE_ALL_ZEROS, &period, &high);
    TEST_ASSERT_UINT32_WITHIN(PERIOD_TOL, DSHOT_PERIOD_TICKS, period);
}

void test_hil_dshot_bit0_pulse_width(void) {
    TEST_ASSERT_TRUE_MESSAGE(s_ready, "skipped: DShot not initialized");
    uint32_t period, high;
    SendAndCapture(THROTTLE_ALL_ZEROS, &period, &high);
    TEST_ASSERT_UINT32_WITHIN(PULSE_TOL, DSHOT_TICKS_0, high);
}

void test_hil_dshot_bit1_pulse_width(void) {
    TEST_ASSERT_TRUE_MESSAGE(s_ready, "skipped: DShot not initialized");
    uint32_t period, high;
    SendAndCapture(THROTTLE_LAST_BIT_1, &period, &high);
    TEST_ASSERT_UINT32_WITHIN(PULSE_TOL, DSHOT_TICKS_1, high);
}

void test_hil_dshot_frame_length(void) {
    TEST_ASSERT_TRUE_MESSAGE(s_ready, "skipped: DShot not initialized");
    /* 18 bits × 427 ticks = 7686. Allow ±2% = ±154 ticks. */
    uint32_t expected = DSHOT_DMA_BUFFER_SIZE * DSHOT_PERIOD_TICKS;
    uint32_t delta    = (expected * 2U) / 100U;
    s_htim3.Instance->CNT = 0;
    uint16_t vals[BRD_MOTOR_COUNT] = { DSHOT_MIN_THROTTLE };
    DShotBB_Write(vals);
    TEST_ASSERT_UINT32_WITHIN(delta, expected, s_htim3.Instance->CNT);
}
