#include "test_dshot.h"

#include "drivers/dma.h"
#include "hal.h"
#include "mc/dshot.h"
#include "target.h"
#include "unity.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*
 * Hardware loopback: PC6 (DShot output) → PC7 (TIM3_CH2 input capture).
 * Connect with a jumper wire on the Nucleo board before running.
 *
 * TIM3 free-running counter (no slave reset).
 *   CH2 (both edges, direct, DMA) → s_captures[32]: [rise0, fall0, rise1, fall1, ...]
 *
 * Timer clock: 64 MHz.  Per-bit timing (8 samples × 53 ticks = 424 ticks/bit):
 *   Bit period  ≈ 424 ticks  (inter-rising-edge time)
 *   '1' high    = 6 × 53 = 318 ticks
 *   '0' high    = 3 × 53 = 159 ticks
 * Tolerance: 5% → ±21 ticks.
 */

#define DSHOT_PERIOD_TICKS   424U
#define DSHOT_TICKS_1        318U
#define DSHOT_TICKS_0        159U
#define PERIOD_TOL           ((DSHOT_PERIOD_TICKS * 5U) / 100U)
#define PULSE_TOL            ((DSHOT_TICKS_1 * 5U) / 100U)

/* value=0  → packet=0x0000 → all 16 bits are '0'
 * value=8  → packet=0x0101 → bits 7 and 15 (MSB-first) are '1' */
#define THROTTLE_ALL_ZEROS   0U
#define THROTTLE_LAST_BIT_1  8U

static TIM_HandleTypeDef s_htim3 = { 0 };
static DmaHandle_t* s_pDma       = NULL;
static uint32_t s_captures[32] = { 0 }; /* interleaved: rise[i]=s_captures[2i], fall[i]=s_captures[2i+1] */
static volatile bool s_captureDone = false;
static bool s_ready                = false;

void setUp(void)    {}
void tearDown(void) {}

/* Override weak HAL callback — fires from DMA TC ISR via TIM_DMACaptureCplt */
void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef* htim) {
    if (htim != &s_htim3) {
        return;
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        s_captureDone = true;
    }
}

static void ArmCaptures (void) {
    memset (s_captures, 0, sizeof (s_captures));
    s_captureDone = false;

    DMA_HandleTypeDef* pHdma = &s_pDma->plat;
    __HAL_LINKDMA (&s_htim3, hdma[TIM_DMA_ID_CC2], *pHdma);
    HAL_TIM_IC_Start_DMA (&s_htim3, TIM_CHANNEL_2, s_captures, 32);
}

static void SendAndCapture (uint16_t throttle, uint32_t captures[32]) {
    ArmCaptures ();
    uint16_t vals[BRD_MOTOR_COUNT] = { throttle };
    DShotBB_Write (vals);
    while (!g_DShotBB.txDone || !s_captureDone) {
    }
    HAL_TIM_IC_Stop_DMA (&s_htim3, TIM_CHANNEL_2);
    for (uint32_t i = 0; i < 16U; i += 2) {
        captures[i]     = s_captures[i];
        captures[i + 1] = s_captures[i + 1U];
    }
}

static void InitCaptureDma (DmaHandle_t** ppOut, uint32_t request) {
    DmaHandle_t* p                   = DmaResource_Alloc ();
    p->plat.Init.Request             = request;
    p->plat.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    p->plat.Init.PeriphInc           = DMA_PINC_DISABLE;
    p->plat.Init.MemInc              = DMA_MINC_ENABLE;
    p->plat.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    p->plat.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    p->plat.Init.Mode                = DMA_NORMAL;
    p->plat.Init.Priority            = DMA_PRIORITY_HIGH;
    p->plat.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    Dma_Init (p);
    *ppOut = p;
}


static void SetupInputCapture (void) {
    __HAL_RCC_TIM3_CLK_ENABLE ();
    __HAL_RCC_GPIOC_CLK_ENABLE ();

    /* PC7 → TIM3_CH2 (AF2) */
    GPIO_InitTypeDef gpio = { 0 };
    gpio.Pin              = GPIO_PIN_7;
    gpio.Mode             = GPIO_MODE_AF_PP;
    gpio.Pull             = GPIO_NOPULL;
    gpio.Speed            = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate        = GPIO_AF2_TIM3;
    HAL_GPIO_Init (GPIOC, &gpio);

    /* TIM3: free-running, no slave mode, 16-bit counter */
    s_htim3.Instance               = TIM3;
    s_htim3.Init.Prescaler         = 0;
    s_htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim3.Init.Period            = 0xFFFF;
    s_htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_IC_Init (&s_htim3);

    /* CH2: capture both edges (direct from TI2) */
    TIM_IC_InitTypeDef ic = { 0 };
    ic.ICPolarity         = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    ic.ICSelection        = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler        = TIM_ICPSC_DIV1;
    ic.ICFilter           = 0;
    HAL_TIM_IC_ConfigChannel (&s_htim3, &ic, TIM_CHANNEL_2);

    /* Single DMA for TIM3 CH2 — captures all edge timestamps into s_captures */
    InitCaptureDma (&s_pDma, DMA_REQUEST_TIM3_CH2);
}

/* ── Tests ────────────────────────────────────────────────────────────────── */

void test_hil_dshot_init (void) {
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, DShotBB_Init ());
    SetupInputCapture ();
    HAL_Delay (5);
    s_ready = true;
}

void test_hil_dshot_bit_period (void) {

    TEST_ASSERT_TRUE_MESSAGE (s_ready, "skipped: DShot not initialized");
    uint32_t expectedDiffs[16] = {};
    uint32_t actualDiffs[16]   = {};
    uint32_t captures[32]      = {};
    SendAndCapture (THROTTLE_ALL_ZEROS, captures);

    for (uint32_t i = 0; i < 16U; ++i) {
        expectedDiffs[i] = 160U;
        actualDiffs[i]   = captures[i * 2U + 1U] - captures[i * 2U];
    }
    LOG_INFO ("%u %u %u %u", actualDiffs[0], actualDiffs[1], actualDiffs[2], actualDiffs[3]);
    LOG_INFO ("%u %u %u %u", captures[0], captures[1], captures[2], captures[3]);
    TEST_ASSERT_UINT_ARRAY_WITHIN (5, expectedDiffs, actualDiffs, 16);
}

// void test_hil_dshot_bit0_pulse_width (void) {
//     TEST_ASSERT_TRUE_MESSAGE (s_ready, "skipped: DShot not initialized");
//     /* THROTTLE_ALL_ZEROS → packet=0x0000, all 16 bits are '0' */
//     uint32_t rise[16], fall[16];
//     SendAndCapture (THROTTLE_ALL_ZEROS, rise, fall);
//     for (uint8_t i = 0; i < 16U; ++i) {
//         uint32_t high = fall[i] - rise[i];
//         TEST_ASSERT_UINT32_WITHIN (PULSE_TOL, DSHOT_TICKS_0, high);
//     }
// }

// void test_hil_dshot_bit1_pulse_width (void) {
//     TEST_ASSERT_TRUE_MESSAGE (s_ready, "skipped: DShot not initialized");
//     /* THROTTLE_LAST_BIT_1 (value=8) → packet=0x0101.
//      * Transmitted MSB-first: bits 7 and 15 (0-indexed) are '1'. */
//     uint32_t rise[16], fall[16];
//     SendAndCapture (THROTTLE_LAST_BIT_1, rise, fall);
//     TEST_ASSERT_UINT32_WITHIN (PULSE_TOL, DSHOT_TICKS_1, fall[7] - rise[7]);
//     TEST_ASSERT_UINT32_WITHIN (PULSE_TOL, DSHOT_TICKS_1, fall[15] - rise[15]);
// }

// void test_hil_dshot_frame_length (void) {
//     TEST_ASSERT_TRUE_MESSAGE (s_ready, "skipped: DShot not initialized");
//     /* Span of 16 rising edges = 15 inter-bit periods ≈ 15 × 424 = 6360 ticks */
//     uint32_t rise[16], fall[16];
//     SendAndCapture (DSHOT_MIN_THROTTLE, rise, fall);
//     uint32_t span     = rise[15] - rise[0];
//     uint32_t expected = 15U * DSHOT_PERIOD_TICKS;
//     uint32_t delta    = (expected * 5U) / 100U;
//     TEST_ASSERT_UINT32_WITHIN (delta, expected, span);
// }
