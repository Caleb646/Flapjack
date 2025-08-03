#include "control.h"
#include "common.h"
#include "hal.h"
#include "log.h"
#include "mem/mem.h"
#include "mem/queue.h"

#define CONTROL_PRODUCER_CORE      CM4_CPUID
#define CONTROL_CONSUMER_CORE      CM7_CPUID
#define RAW_COMMAND_QUEUE_CAPACITY 8U

/* Consumer global variables */
// A local only buffer to store raw commands during the interrupt handler
static Queue gRawCommandQueue = { 0 };
static uint8_t gRawCommandQueueBuffer[RAW_COMMAND_QUEUE_CAPACITY * sizeof (EmptyCommand)] = { 0 };
static uint8_t gUartInterruptBuffer[sizeof (EmptyCommand)] = { 0 };

/* Producer global variables */
static OpStateTransitionHandler_t
gaaOpStateTransitionHandlers[eNUMBER_OF_OP_STATES][eNUMBER_OF_OP_STATES] = { 0 };
static CmdHandler_t gaCmdHandlers[eNUMBER_OF_COMMAND_TYPES] = { 0 };

/* Shared global variables */
static Queue* gpSharedCommandQueue = NULL;
static FCState* gpFlightState      = NULL;

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

#ifndef UNIT_TEST

static eSTATUS_t ControlInit_Producer (void);
static eSTATUS_t ControlInitCmdQueue (Queue* pQueue);
static eSTATUS_t ControlInit_Consumer (void);
static eSTATUS_t ControlInitFCState (FCState* pState);
static eSTATUS_t ControlProcessEmptyCmd (EmptyCommand cmd);
static eSTATUS_t ControlProcessOpStateChange (EmptyCommand cmd);
static eSTATUS_t ControlProcessFlightModeChange (EmptyCommand cmd);
static eSTATUS_t ControlProcessVelocityChange (EmptyCommand cmd);
static eSTATUS_t ControlProcessPIDChange (EmptyCommand cmd);
static BOOL_t ControlGetNewCmd (EmptyCommand* pOutCmd);

#endif /* UNIT_TEST */

STATIC_TESTABLE_DECL eSTATUS_t ControlInit_Producer (void) {

    if (QueueInit (&gRawCommandQueue, gRawCommandQueueBuffer, RAW_COMMAND_QUEUE_CAPACITY, sizeof (EmptyCommand)) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize raw command queue");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlInitCmdQueue (Queue* pQueue) {

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
    eSTATUS_t status = QueueInit (pQueue, pBuffer, capacity, elementSize);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize command queue");
        return status;
    }

    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlInit_Consumer (void) {

    if (ControlRegisterCmdHandler (eCOMMAND_TYPE_EMPTY, ControlProcessEmptyCmd) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to register empty command handler");
        return eSTATUS_FAILURE;
    }

    if (ControlRegisterCmdHandler (eCOMMAND_TYPE_CHANGE_OP_STATE, ControlProcessOpStateChange) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to register change op state command handler");
        return eSTATUS_FAILURE;
    }

    if (ControlRegisterCmdHandler (eCOMMAND_TYPE_CHANGE_FLIGHT_MODE, ControlProcessFlightModeChange) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to register change flight mode command handler");
        return eSTATUS_FAILURE;
    }

    if (ControlRegisterCmdHandler (eCOMMAND_TYPE_CHANGE_VELOCITY, ControlProcessVelocityChange) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to register change velocity command handler");
        return eSTATUS_FAILURE;
    }

    if (ControlRegisterCmdHandler (eCOMMAND_TYPE_CHANGE_PID, ControlProcessPIDChange) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to register change pid command handler");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlInitFCState (FCState* pState) {

    if (pState == NULL) {
        LOG_ERROR ("Flight controller state pointer is NULL");
        return eSTATUS_FAILURE;
    }

    pState->flightMode = eFLIGHT_MODE_HOVER;
    pState->opState    = eOP_STATE_STOPPED;

    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlProcessEmptyCmd (EmptyCommand cmd) {

    LOG_INFO ("Received empty command");
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlProcessOpStateChange (EmptyCommand cmd) {

    FCState curState           = ControlGetCopyFCState ();
    eOP_STATE_t requestedState = ((ChangeOpStateCmd*)&cmd)->requestedState;

    if (curState.opState == requestedState) {
        LOG_INFO ("Already in state: %s", ControlOpState2Char (curState.opState));
        return eSTATUS_SUCCESS;
    }

    switch (requestedState) {
    case eOP_STATE_STOPPED:
    case eOP_STATE_RUNNING:
    case eOP_STATE_ERROR: break;
    default:
        LOG_ERROR ("Invalid requested state: %s", ControlOpState2Char (requestedState));
        return eSTATUS_FAILURE;
    }

    OpStateTransitionHandler_t handler =
    gaaOpStateTransitionHandlers[curState.opState][requestedState];
    BOOL_t doTransition = TRUE;
    if (handler != NULL) {
        doTransition = handler (curState);
    }

    if (doTransition == TRUE) {
        curState.opState = requestedState;
        if (ControlUpdateFCState (&curState) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to update flight controller state");
            return eSTATUS_FAILURE;
        }
        LOG_INFO (
        "Transitioned to state: %s", ControlOpState2Char (curState.opState));
    } else {
        LOG_INFO ("Transition to state: %s was not performed", ControlOpState2Char (requestedState));
    }
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlProcessFlightModeChange (EmptyCommand cmd) {

    LOG_INFO ("Received flight mode change command");
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlProcessVelocityChange (EmptyCommand cmd) {

    LOG_INFO ("Received velocity change command");
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlProcessPIDChange (EmptyCommand cmd) {

    LOG_INFO ("Received PID change command");
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL BOOL_t ControlGetNewCmd (EmptyCommand* pOutCmd) {

    if (HAL_GetCurrentCPUID () == CONTROL_PRODUCER_CORE) {
        LOG_ERROR ("Should only be called for the core that is consuming");
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

// STATIC_TESTABLE_DECL eSTATUS_t ControlParseRawCmd (uint8_t* pRawCmd, uint16_t cmdSize) {
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

eSTATUS_t ControlInit (void) {

    gpSharedCommandQueue = (Queue*)MEM_SHARED_COMMAND_QUEUE_START;
    gpFlightState        = (FCState*)MEM_SHARED_FLIGHT_STATE_START;

    if (HAL_GetCurrentCPUID () == CONTROL_PRODUCER_CORE) {

        if (ControlInitCmdQueue (gpSharedCommandQueue) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize command queue");
            return eSTATUS_FAILURE;
        }

        if (ControlInitFCState (gpFlightState) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize flight controller state");
            return eSTATUS_FAILURE;
        }

        if (ControlInit_Producer () != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize producer control");
            return eSTATUS_FAILURE;
        }
    } else {
        // Consumer core initialization
        if (ControlInit_Consumer () != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize consumer control");
            return eSTATUS_FAILURE;
        }
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t ControlStart (UART_HandleTypeDef* huart) {

    if (HAL_GetCurrentCPUID () == CONTROL_PRODUCER_CORE) {
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

eSTATUS_t ControlProcessRawCmds (void) {

    if (HAL_GetCurrentCPUID () == CONTROL_CONSUMER_CORE) {
        LOG_ERROR ("Should only be called for the core that is producing");
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

eSTATUS_t ControlProcessCmds (void) {

    if (HAL_GetCurrentCPUID () == CONTROL_CONSUMER_CORE) {
        LOG_ERROR ("Should only be called for the core that is producing");
        return eSTATUS_FAILURE;
    }

    EmptyCommand cmd = { 0 };
    if (ControlGetNewCmd (&cmd) == FALSE) {
        return eSTATUS_SUCCESS; // No new command to process
    }

    switch (cmd.header.commandType) {
    case eCOMMAND_TYPE_EMPTY:
    case eCOMMAND_TYPE_CHANGE_OP_STATE:
    case eCOMMAND_TYPE_CHANGE_FLIGHT_MODE:
    case eCOMMAND_TYPE_CHANGE_VELOCITY:
    case eCOMMAND_TYPE_CHANGE_PID: break;
    default:
        LOG_ERROR ("Unknown command type: %u", cmd.header.commandType);
        return eSTATUS_FAILURE;
    }

    CmdHandler_t handler = gaCmdHandlers[cmd.header.commandType];
    if (handler == NULL) {
        LOG_ERROR (
        "No handler registered for command type: %s",
        ControlCmdType2Char (cmd.header.commandType));
        return eSTATUS_FAILURE;
    }

    if (handler (cmd) != eSTATUS_SUCCESS) {
        LOG_ERROR (
        "Failed to process command: %s", ControlCmdType2Char (cmd.header.commandType));
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t
ControlRegisterOPStateTransitionHandler (eOP_STATE_t fromState, eOP_STATE_t toState, OpStateTransitionHandler_t handler) {

    if (fromState >= eNUMBER_OF_OP_STATES || toState >= eNUMBER_OF_OP_STATES) {
        LOG_ERROR ("Invalid state transition: %d -> %d", fromState, toState);
        return eSTATUS_FAILURE;
    }

    gaaOpStateTransitionHandlers[fromState][toState] = handler;
    return eSTATUS_SUCCESS;
}

eSTATUS_t ControlRegisterCmdHandler (eCOMMAND_t cmdType, CmdHandler_t handler) {

    if (cmdType >= eNUMBER_OF_COMMAND_TYPES) {
        LOG_ERROR ("Invalid command type: %d", cmdType);
        return eSTATUS_FAILURE;
    }

    gaCmdHandlers[cmdType] = handler;
    return eSTATUS_SUCCESS;
}

char const* ControlOpState2Char (eOP_STATE_t opState) {

    switch (opState) {
    case eOP_STATE_STOPPED: return "[STOPPED]";
    case eOP_STATE_RUNNING: return "[RUNNING]";
    case eOP_STATE_ERROR: return "[ERROR]";
    default: return "[UNKNOWN]";
    }
}

char const* ControlCmdType2Char (eCOMMAND_t commandType) {

    switch (commandType) {
    case eCOMMAND_TYPE_EMPTY: return "[EMPTY]";
    case eCOMMAND_TYPE_CHANGE_OP_STATE: return "[CHANGE_OP_STATE]";
    case eCOMMAND_TYPE_CHANGE_FLIGHT_MODE: return "[CHANGE_FLIGHT_MODE]";
    case eCOMMAND_TYPE_CHANGE_VELOCITY: return "[CHANGE_VELOCITY]";
    case eCOMMAND_TYPE_CHANGE_PID: return "[CHANGE_PID]";
    default: return "[UNKNOWN_COMMAND_TYPE]";
    }
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

eSTATUS_t ControlUpdateFCState (FCState const* pNewState) {

    if (pNewState == NULL || gpFlightState == NULL) {
        LOG_ERROR ("Invalid flight controller state pointer");
        return eSTATUS_FAILURE;
    }

    // Update the flight mode and operation state
    gpFlightState->flightMode = pNewState->flightMode;
    gpFlightState->opState    = pNewState->opState;

    return eSTATUS_SUCCESS;
}