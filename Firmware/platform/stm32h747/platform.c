#include "platform.h"

#include "hal.h"

#include <stdint.h>
#include <string.h>

/* Every hardware semaphore this firmware uses is allocated here. */
#define HSEM_ID_0    (0U) /* CM4 boot release */
#define HSEM_ID_LOCK (1U) /* Platform_LockTake */

extern void __SHARED_MEM_BSS_START__;
extern void __SHARED_MEM_BSS_END__;
extern void __SHARED_MEM_DATA_START__;
extern void __SHARED_MEM_DATA_END__;
extern void __SHARED_MEM_DATA_FLASH_START__;

static int SystemClock_Config(void);
static int PeriphClock_Config(void);
static void MicrosecondTimer_Config(void);
static void MPU_Config(void);

int Platform_Init(void) {

    /* Otherwise all three escalate to HardFault and Platform_OnFault could not tell them apart. */
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk;

#if defined(CORE_CM7)
    // NOTE: disable mpu for now
    // MPU_Config();

    int32_t timeout = 0xFFFF;
    while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
    if ( timeout < 0 ) {
        return -1;
    }

    uint32_t const bssStart = (uint32_t)&__SHARED_MEM_BSS_START__;
    uint32_t const bssEnd   = (uint32_t)&__SHARED_MEM_BSS_END__;
    memset ((void*)bssStart, 0, bssEnd - bssStart);

    uint32_t const dataStart      = (uint32_t)&__SHARED_MEM_DATA_START__;
    uint32_t const dataEnd        = (uint32_t)&__SHARED_MEM_DATA_END__;
    uint32_t const dataFlashStart = (uint32_t)&__SHARED_MEM_DATA_FLASH_START__;
    memcpy ((void*)dataStart, (void*)dataFlashStart, dataEnd - dataStart);

    HAL_Init();
    if(SystemClock_Config() != 0) {
        return -1;
    }
    if(PeriphClock_Config() != 0) {
        return -1;
    }

    // Enable core debug access and trace unit
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Enable DWT cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    MicrosecondTimer_Config();

#if !defined(SINGLE_CORE)
    /* Wake CM4 and wait for the D2 domain to come up. Skipped in single-core
     * builds: there is no CM4 to release, and RCC_FLAG_D2CKRDY never asserts. */
    __HAL_RCC_HSEM_CLK_ENABLE();
    HAL_HSEM_FastTake(HSEM_ID_0);
    HAL_HSEM_Release(HSEM_ID_0, 0);
    timeout = 0xFFFF;
    while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
    if ( timeout < 0 ) {
        return -1;
    }
#endif /* !SINGLE_CORE */
#endif /* CORE_CM7 */

#if defined(CORE_CM4)

    __HAL_RCC_HSEM_CLK_ENABLE();
    HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
    HAL_PWREx_ClearPendingEvent();
    HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
    __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

    HAL_Init();

    // DWT is per-core: CM7's enable above does not reach this one
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

#endif /* CORE_CM4 */

    return 0;
}

/* Each core has its own copy of the reset flags, so each reads and clears its own. */
bool Platform_TakePowerOnReset(void) {

#if defined(CORE_CM4)
    bool const isPowerOn = __HAL_RCC_C2_GET_FLAG(RCC_FLAG_PORRST) || __HAL_RCC_C2_GET_FLAG(RCC_FLAG_BORRST);
    __HAL_RCC_C2_CLEAR_RESET_FLAGS();
#else
    bool const isPowerOn = __HAL_RCC_C1_GET_FLAG(RCC_FLAG_PORRST) || __HAL_RCC_C1_GET_FLAG(RCC_FLAG_BORRST);
    __HAL_RCC_C1_CLEAR_RESET_FLAGS();
#endif
    return isPowerOn;
}

uint32_t Platform_CurrentException(void) {
    return __get_IPSR() & IPSR_ISR_Msk;
}

void Platform_Reset(void) {

    /* Without a debugger attached BKPT does not halt, it escalates to HardFault. */
    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U) {
        __BKPT(1);
    }
    NVIC_SystemReset();
}

__attribute__ ((used, noreturn)) static void Platform_Fault(uint32_t const* pFrame) {

    PlatformFault_t fault = { 0 };

    switch (Platform_CurrentException()) {
    case MemoryManagement_IRQn + 16: fault.fault = ePLATFORM_FAULT_MEMMANAGE; break;
    case BusFault_IRQn + 16: fault.fault = ePLATFORM_FAULT_BUS; break;
    case UsageFault_IRQn + 16: fault.fault = ePLATFORM_FAULT_USAGE; break;
    default: fault.fault = ePLATFORM_FAULT_HARD; break;
    }
    fault.exception = Platform_CurrentException();
    fault.cfsr      = SCB->CFSR;
    fault.hfsr      = SCB->HFSR;
    fault.mmfar     = SCB->MMFAR;
    fault.bfar      = SCB->BFAR;

    /* A stacking error means the frame was never written - and the HardFault
     * handler faulting on a bad read locks the core up instead of resetting. */
    if ((fault.cfsr & (SCB_CFSR_MSTKERR_Msk | SCB_CFSR_STKERR_Msk)) == 0U) {
        fault.lr        = pFrame[5];
        fault.pc        = pFrame[6];
        fault.exception = pFrame[7] & IPSR_ISR_Msk;
    }

    Platform_OnFault(&fault);
}

/*
 * Every fault vector lands here. Naked because a C prologue would push onto the
 * stack being inspected and overwrite LR, whose EXC_RETURN bit 2 says which
 * stack - MSP or PSP - holds the faulting context's frame.
 */
__attribute__ ((naked)) static void Platform_FaultEntry(void) {

    __asm volatile ("tst   lr, #4          \n"
                    "ite   eq              \n"
                    "mrseq r0, msp         \n"
                    "mrsne r0, psp         \n"
                    "b     Platform_Fault  \n");
}

void HardFault_Handler(void) __attribute__ ((alias ("Platform_FaultEntry")));
void MemManage_Handler(void) __attribute__ ((alias ("Platform_FaultEntry")));
void BusFault_Handler(void) __attribute__ ((alias ("Platform_FaultEntry")));
void UsageFault_Handler(void) __attribute__ ((alias ("Platform_FaultEntry")));

int SystemClock_Config (void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

#if defined(USE_PWR_DIRECT_SMPS_SUPPLY)
#warning "Using SMPS power supply. Ensure that the board is configured for SMPS and not LDO!"
    HAL_PWREx_ConfigSupply (PWR_DIRECT_SMPS_SUPPLY);
#endif /* USE_PWR_DIRECT_SMPS_SUPPLY && SMPS */

#if defined(USE_PWR_LDO_SUPPLY)
#warning "Using LDO power supply. Ensure that the board is configured for LDO and not SMPS!"
    HAL_PWREx_ConfigSupply (PWR_LDO_SUPPLY);
#endif /* USE_PWR_LDO_SUPPLY */

    /* Configure the main internal regulator output voltage */
    __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE1);
    while (!__HAL_PWR_GET_FLAG (PWR_FLAG_VOSRDY)) {
    }

    /** Initializes the RCC Oscillators according to the specified
     * parameters in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState            = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM            = 5;
    RCC_OscInitStruct.PLL.PLLN            = 48;
    RCC_OscInitStruct.PLL.PLLP            = 2;
    RCC_OscInitStruct.PLL.PLLQ            = 5;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    RCC_OscInitStruct.PLL.PLLRGE          = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL       = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN        = 0;
#if HSE_VALUE == 0U
#warning "HSE_VALUE is not defined or set to 0. HSE oscillator will be disabled. Ensure that HSE_VALUE is set correctly if using an external crystal."
#endif
    if (HSE_VALUE == 0U) {
        RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
        RCC_OscInitStruct.OscillatorType &= ~RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    }
    if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK) {
        return -1;
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;

    // If HSE is not used HSI (64MHz) is used as the HCLK. APB1 and APB2 max is 50MHz, so set dividers to 2 (32MHz)
    if (HSE_VALUE == 0U) {
        RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    }

    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        return -1;
    }
    return 0;
}

/*
 * Kernel clock source for each SPI and U(S)ART. The drivers enable the bus
 * clocks and set up the peripherals; which clock feeds them is chosen here,
 * next to the clock tree it depends on.
 */
static int PeriphClock_Config (void) {

    static RCC_PeriphCLKInitTypeDef const clocks[] = {
        { .PeriphClockSelection = RCC_PERIPHCLK_SPI1, .Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL },
        { .PeriphClockSelection = RCC_PERIPHCLK_SPI2, .Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL },
        { .PeriphClockSelection = RCC_PERIPHCLK_SPI3, .Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL },
        { .PeriphClockSelection = RCC_PERIPHCLK_SPI4, .Spi45ClockSelection = RCC_SPI45CLKSOURCE_PCLK2 },
        { .PeriphClockSelection = RCC_PERIPHCLK_SPI5, .Spi45ClockSelection = RCC_SPI45CLKSOURCE_PCLK2 },
        { .PeriphClockSelection = RCC_PERIPHCLK_USART1, .Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2 },
        { .PeriphClockSelection = RCC_PERIPHCLK_USART2, .Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1 },
        { .PeriphClockSelection = RCC_PERIPHCLK_USART3, .Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1 },
        { .PeriphClockSelection = RCC_PERIPHCLK_UART4, .Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1 },
        { .PeriphClockSelection = RCC_PERIPHCLK_UART5, .Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1 },
    };

    for (uint32_t i = 0; i < sizeof (clocks) / sizeof (clocks[0]); ++i) {
        RCC_PeriphCLKInitTypeDef clock = clocks[i];
        if (HAL_RCCEx_PeriphCLKConfig (&clock) != HAL_OK) {
            return -1;
        }
    }
    return 0;
}

/*
 * TIM2 because Platform_GetMicroseconds needs a 32-bit counter. CM7 only: CM4
 * reads the same counter, and reconfiguring it would reset CM7's clock.
 */
static void MicrosecondTimer_Config (void) {

    /* With TIMPRE clear, APB1 timers run at PCLK1 when the APB1 prescaler is 1
     * and at 2x PCLK1 otherwise (RM0399, kernel clock distribution). */
    uint32_t const pclk1    = HAL_RCC_GetPCLK1Freq ();
    uint32_t const timClock = ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) == RCC_APB1_DIV1) ? pclk1 : (2U * pclk1);

    __HAL_RCC_TIM2_CLK_ENABLE ();
    TIM2->CR1 = 0U;
    TIM2->PSC = (timClock / 1000000U) - 1U;
    TIM2->ARR = 0xFFFFFFFFU;
    TIM2->CNT = 0U;
    TIM2->EGR = TIM_EGR_UG; /* PSC is preloaded; latch it now, not at the first overflow */
    TIM2->CR1 = TIM_CR1_CEN;
}

uint32_t Platform_GetMicroseconds (void) {
    return TIM2->CNT;
}

void Platform_NotifyOtherCore (void) {
    /* The barrier makes the mailbox write visible before the event lands. */
    __DSB ();
    __SEV ();
}

bool Platform_LockTake (void) {

    while (HAL_HSEM_FastTake (HSEM_ID_LOCK) != HAL_OK) {
    }
    return true;
}

void Platform_LockRelease (void) {

    HAL_HSEM_Release (HSEM_ID_LOCK, 0);
}

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM16 || htim->Instance == TIM17) {
        HAL_IncTick();
    }
}
