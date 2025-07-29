#include "control.h"
#include "common.h"
#include "hal.h"
#include "log.h"
#include "mem/mem.h"
#include "mem/queue.h"


// #if not defined(UNIT_TEST) && defined(CORE_M4)

#define RAW_COMMAND_QUEUE_CAPACITY 8U
// A local only buffer to store raw commands during the interrupt handler
static Queue gRawCommandQueue = { 0 };
static uint8_t gRawCommandQueueBuffer[RAW_COMMAND_QUEUE_CAPACITY * sizeof (EmptyCommand)] = { 0 };
static uint8_t gUartInterruptBuffer[sizeof (EmptyCommand)] = { 0 };

/*
 * Global UART recv complete callback. The reason this can work is because
 * their is only 1 uart that is set to recv. If there are multiple receiving uarts, this will not work.
 */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef* huart) {
    // LOG_INFO ("Interrupt");
    if (QueueIsFull (&gRawCommandQueue)) {
        // LOG_ERROR ("queue full");
        return;
    }
    QueueEnqueue (&gRawCommandQueue, (void*)gUartInterruptBuffer);
    HAL_UART_Receive_IT (huart, gUartInterruptBuffer, sizeof (EmptyCommand));
}

STATIC_TESTABLE_DECL STATUS_TYPE ControlInit_CM4 (void) {
    if (QueueInit (&gRawCommandQueue, gRawCommandQueueBuffer, RAW_COMMAND_QUEUE_CAPACITY, sizeof (EmptyCommand)) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize raw command queue");
        return eSTATUS_FAILURE;
    }

    HAL_NVIC_SetPriority (USART1_IRQn, 5, 5);
    HAL_NVIC_EnableIRQ (USART1_IRQn);

    return eSTATUS_SUCCESS;
}

// #endif

static Queue* gpSharedCommandQueue = NULL;
static FCState* gpFlightState      = NULL;

STATIC_TESTABLE_DECL STATUS_TYPE ControlInitCmdQueue (Queue* pQueue) {
    if (pQueue == NULL) {
        LOG_ERROR ("Command queue pointer is NULL");
        return eSTATUS_FAILURE;
    }

    uint16_t capacity    = 8;
    uint16_t elementSize = sizeof (EmptyCommand);

    if (capacity * elementSize > MEM_SHARED_COMMAND_QUEUE_TOTAL_LEN) {
        LOG_ERROR ("Command queue size exceeds total length");
        return eSTATUS_FAILURE;
    }

    memset ((void*)MEM_SHARED_COMMAND_QUEUE_START, 0, MEM_SHARED_COMMAND_QUEUE_TOTAL_LEN);

    void* pBuffer =
    (void*)MEM_U32_ALIGN4 (MEM_SHARED_COMMAND_QUEUE_START + sizeof (Queue));
    STATUS_TYPE status = QueueInit (pQueue, pBuffer, capacity, elementSize);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize command queue");
        return status;
    }

    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL STATUS_TYPE ControlInitFCState (FCState* pState) {
    if (pState == NULL) {
        LOG_ERROR ("Flight controller state pointer is NULL");
        return eSTATUS_FAILURE;
    }

    pState->flightMode = eFLIGHT_MODE_HOVER;
    pState->opState    = eOP_STATE_STOPPED;

    return eSTATUS_SUCCESS;
}

// STATIC_TESTABLE_DECL STATUS_TYPE ControlParseRawCmd (uint8_t* pRawCmd, uint16_t cmdSize) {
//     if (cmdSize != sizeof (EmptyCommand)) {
//         LOG_ERROR ("Invalid command size");
//         return eSTATUS_FAILURE;
//     }
//     EmptyCommand* pCmd = (EmptyCommand*)pRawCmd;
//     uint8_t* cmdData   = pCmd->data;
//     switch (pCmd->header.commandType) {
//     case eCOMMAND_TYPE_CHANGE_OP_STATE: {
//         ChangeOpStateCmd* pChangeOpStateCmd = (ChangeOpStateCmd*)pRawCmd;
//         break;
//     }
//     case eCOMMAND_TYPE_CHANGE_FLIGHT_MODE: {
//         ChangeFlightModeCmd* pChangeFlightModeCmd = (ChangeFlightModeCmd*)pRawCmd;
//         break;
//     }
//     case eCOMMAND_TYPE_CHANGE_VELOCITY: {
//         ChangeVelocityCmd* pChangeVelocityCmd = (ChangeVelocityCmd*)pRawCmd;
//         break;
//     }
//     case eCOMMAND_TYPE_CHANGE_PID: {
//         ChangePIDCmd* pChangePIDCmd = (ChangePIDCmd*)pRawCmd;
//         break;
//     }
//     default:
//         LOG_ERROR ("Unknown command type: %d", pCmd->header.commandType);
//         return eSTATUS_FAILURE;
//     }
//     return eSTATUS_SUCCESS;
// }

STATUS_TYPE ControlInit (void) {

    gpSharedCommandQueue = (Queue*)MEM_SHARED_COMMAND_QUEUE_START;
    gpFlightState        = (FCState*)MEM_SHARED_FLIGHT_STATE_START;

    if (HAL_GetCurrentCPUID () == CM4_CPUID) {
        if (ControlInitCmdQueue (gpSharedCommandQueue) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize command queue");
            return eSTATUS_FAILURE;
        }
        if (ControlInitFCState (gpFlightState) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize flight controller state");
            return eSTATUS_FAILURE;
        }
        if (ControlInit_CM4 () != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize CM4 control");
            return eSTATUS_FAILURE;
        }
    }
    return eSTATUS_SUCCESS;
}

STATUS_TYPE ControlStart (UART_HandleTypeDef* huart) {

    if (HAL_GetCurrentCPUID () == CM4_CPUID) {
        if (huart == NULL) {
            LOG_ERROR ("UART handle is NULL");
            return eSTATUS_FAILURE;
        }
        if (HAL_UART_Receive_IT (huart, gUartInterruptBuffer, sizeof (EmptyCommand)) != HAL_OK) {
            LOG_ERROR ("Failed to start UART reception");
            return eSTATUS_FAILURE;
        }
    }

    return eSTATUS_SUCCESS;
}

STATUS_TYPE ControlProcessRawCmds (void) {

    if (HAL_GetCurrentCPUID () == CM7_CPUID) {
        LOG_ERROR ("Should only be called on CM4");
        return eSTATUS_FAILURE;
    }
    /*
     * NOTE: Right now, this function assumes that the cmd sent by the GUI
     * or RF remote control is always a valid command and doesn't require parsing.
     * The buffer can simply be cast to the appriopriate command type based on the header.
     */
    if (QueueIsEmpty (&gRawCommandQueue) != TRUE) {
        EmptyCommand cmd = { 0 };
        // Get the command from the raw command queue
        QueueDequeue (&gRawCommandQueue, (void*)&cmd);
        // LOG_INFO ("Processing command of type: %d", cmd.header.commandType);
        // Process command and place it in the shared command queue
        if (QueueEnqueue (gpSharedCommandQueue, (void*)&cmd) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to process and push raw command to shared queue");
            return eSTATUS_FAILURE;
        }
    }

    return eSTATUS_SUCCESS;
}

BOOL_t ControlGetNewCmd (EmptyCommand* pOutCmd) {
    if (HAL_GetCurrentCPUID () == CM4_CPUID) {
        LOG_ERROR ("Should only be called on CM7");
        return FALSE;
    }

    if (QueueIsEmpty (gpSharedCommandQueue) == TRUE) {
        return FALSE;
    }

    // LOG_INFO ("%d", gpSharedCommandQueue->count);
    if (QueueDequeue (gpSharedCommandQueue, pOutCmd) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to dequeue command from shared queue");
        return FALSE;
    }
    return TRUE;
}

FCState ControlGetCopyFCState (void) {
    if (gpFlightState == NULL) {
        LOG_ERROR ("Flight controller state pointer is NULL");
        FCState emptyState = { 0 };
        return emptyState;
    }

    return *gpFlightState;
}

eOP_STATE_t ControlGetOpState (void) {
    if (gpFlightState == NULL) {
        LOG_ERROR ("Flight controller state pointer is NULL");
        return eOP_STATE_ERROR;
    }
    return gpFlightState->opState;
}

STATUS_TYPE ControlUpdateFCState (FCState const* pNewState) {
    if (pNewState == NULL || gpFlightState == NULL) {
        LOG_ERROR ("Invalid flight controller state pointer");
        return eSTATUS_FAILURE;
    }

    // Update the flight mode and operation state
    gpFlightState->flightMode = pNewState->flightMode;
    gpFlightState->opState    = pNewState->opState;

    return eSTATUS_SUCCESS;
}