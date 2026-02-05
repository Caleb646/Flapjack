#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "cfg/cfg.h"

#include "core/core.h"

#include "drivers/dma.h"
#include "drivers/dma_defs.h"
#include "drivers/motor.h"
#include "drivers/tim.h"
#include "drivers/tim_defs.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

#include "platform/platform.h"

#include "targets/target.h"

FJ_DEFINE_SHARED (MotorsCfg_t, e_MotorsCfg) = {
    .gpios    = {
#if TARG_MOTOR_ENABLED(1) 
        [MOTOR_ID_TO_INDEX(eMOTOR_1_ID)] = GPIO_ID_MAKE(TARG_MOTOR_1_PIN),
#endif
#if TARG_MOTOR_ENABLED(2) 
        [MOTOR_ID_TO_INDEX(eMOTOR_2_ID)] = GPIO_ID_MAKE(TARG_MOTOR_2_PIN),
#endif
#if TARG_MOTOR_ENABLED(3)
        [MOTOR_ID_TO_INDEX(eMOTOR_3_ID)] = GPIO_ID_MAKE(TARG_MOTOR_3_PIN),
#endif
#if TARG_MOTOR_ENABLED(4)
        [MOTOR_ID_TO_INDEX(eMOTOR_4_ID)] = GPIO_ID_MAKE(TARG_MOTOR_4_PIN),
#endif
    },
    .protType = MOTOR_PROT_MAKE (TARG_MOTOR_PROT),
    .nMotors  = TARG_MAX_MOTORS,
};

FJ_DEFINE_SHARED (ServosCfg_t, e_ServosCfg) = {
    .gpios    = {
#if TARG_SERVO_ENABLED(1)
        [SERVO_ID_TO_INDEX(eSERVO_1_ID)] = GPIO_ID_MAKE(TARG_SERVO_1_PIN),
#endif
#if TARG_SERVO_ENABLED(2)
        [SERVO_ID_TO_INDEX(eSERVO_2_ID)] = GPIO_ID_MAKE(TARG_SERVO_2_PIN),
#endif
#if TARG_SERVO_ENABLED(3)
        [SERVO_ID_TO_INDEX(eSERVO_3_ID)] = GPIO_ID_MAKE(TARG_SERVO_3_PIN),
#endif
#if TARG_SERVO_ENABLED(4)
        [SERVO_ID_TO_INDEX(eSERVO_4_ID)] = GPIO_ID_MAKE(TARG_SERVO_4_PIN),
#endif
#if TARG_SERVO_ENABLED(5)
        [SERVO_ID_TO_INDEX(eSERVO_5_ID)] = GPIO_ID_MAKE(TARG_SERVO_5_PIN),
#endif
#if TARG_SERVO_ENABLED(6)
        [SERVO_ID_TO_INDEX(eSERVO_6_ID)] = GPIO_ID_MAKE(TARG_SERVO_6_PIN),
#endif
#if TARG_SERVO_ENABLED(7)
        [SERVO_ID_TO_INDEX(eSERVO_7_ID)] = GPIO_ID_MAKE(TARG_SERVO_7_PIN),
#endif
#if TARG_SERVO_ENABLED(8)
        [SERVO_ID_TO_INDEX(eSERVO_8_ID)] = GPIO_ID_MAKE(TARG_SERVO_8_PIN),
#endif
},
    .nServos  = TARG_MAX_SERVOS,
};
FJ_DEFINE_SHARED (TimChannel_t, e_TimChans[TARG_MAX_SERVOS]);
FJ_DEFINE_SHARED (MotorsDevice_t, e_MotorsDevice);

extern eSTATUS_t Plat_Dshot_Init (MotorsCfg_t const* pCfg, MotorsDevice_t* pOutMotors);

static eSTATUS_t Dshot_Init (MotorsCfg_t const* pCfg, MotorsDevice_t* pOutMotors) {
    return Plat_Dshot_Init (pCfg, pOutMotors);
}

eSTATUS_t Motors_Init (void) {

    eSTATUS_t status        = eSTATUS_FAIL;
    MotorsDevice_t* pMotors = &e_MotorsDevice;
    MotorsCfg_t const* pCfg = &e_MotorsCfg;

    if (MOTOR_PROT_IS_DSHOT (pCfg->protType)) {
        status = Dshot_Init (pCfg, pMotors);
    }
    return status;
}

eSTATUS_t Motors_Arm (void) {
    // TODO
    return eSTATUS_FAIL;
}

eSTATUS_t Motors_Update (float const* pThrottles, uint32_t nThrottles) {

    MotorsDevice_t* pMotors = &e_MotorsDevice;
    if (!pThrottles || !nThrottles || !pMotors->fnUpdateMotors) {
        return eSTATUS_FAIL;
    }
    return pMotors->fnUpdateMotors (pThrottles, nThrottles);
}

FJ_TESTABLE eSTATUS_t Servos_Init_ (ServosCfg_t const* pCfg, TimChannel_t outTimChans[TARG_MAX_SERVOS]) {

    eSTATUS_t status = eSTATUS_OK;
    uint32_t nChans  = pCfg->nServos;
    for (uint32_t i = 0; i < nChans; ++i) {
        status = TimChan_InitCC (pCfg->gpios[i], &outTimChans[i]);
        TimDev_SetPWMPeriod (outTimChans[i].pTimBaseDev, ONE_MHZ, SERVO_DEF_PERIOD_HZ);
        TimChan_SetCC (&outTimChans[i], 0U);
        TimChan_Start (&outTimChans[i], NULL, 0U);
        RETURN_IF (FJ_FAIL (status), status, "Failed to initialize TIM channel for servo %u", i);
    }

    return status;
}

eSTATUS_t Servos_Init (void) {
    return Servos_Init_ (&e_ServosCfg, e_TimChans);
}

eSTATUS_t Servos_Update (uint16_t const* pOutputs, uint32_t nOutputs) {

    if (!pOutputs || !nOutputs) {
        return eSTATUS_FAIL;
    }

    for (uint32_t i = 0; i < nOutputs; ++i) {
        TimChan_SetCC (&e_TimChans[i], pOutputs[i]);
    }
    return eSTATUS_OK;
}