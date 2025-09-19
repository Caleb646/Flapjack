#include "control.h"
#include "common.h"
#include "conf.h"
#include "hal.h"
#include "log/logger.h"
#include "mem/mem.h"
#include "mem/queue.h"
#include "periphs/uart.h"


#define PRODUCER_ID            CM4_CPUID
#define CONSUMER_ID            CM7_CPUID
#define IS_PRODUCER_ME()       (HAL_GetCurrentCPUID () == PRODUCER_ID)
#define IS_CONSUMER_ME()       (HAL_GetCurrentCPUID () == CONSUMER_ID)
#define COMMAND_QUEUE_CAPACITY 16U
#define UART_RECV_BUFFER_SIZE  (COMMAND_TOTAL_SIZE) // + 2U)

/* Consumer global variables */
QUEUE_DEFINE_STATIC (RawCommand, DefaultCommand, COMMAND_QUEUE_CAPACITY, TRUE);
// A local only buffer to store raw commands during the interrupt handler
static uint8_t ga_UartInterruptBuffer[UART_RECV_BUFFER_SIZE] = { 0 };

/* Producer global variables */
static OpStateTransitionHandler_t
gaa_OpStateTransitionHandlers[eNUMBER_OF_OP_STATES][eNUMBER_OF_OP_STATES] = { 0 };
static CmdHandler_t ga_CmdHandlers[eNUMBER_OF_CMD_TYPES] = { 0 };

/* Shared global variables */
QUEUE_DEFINE_STATIC_SHARED_MEMORY (
SharedCommand,
DefaultCommand,
(Queue*)MEM_SHARED_COMMAND_QUEUE_START,
MEM_SHARED_COMMAND_QUEUE_TOTAL_LEN,
COMMAND_QUEUE_CAPACITY
);
static FCState* gp_FlightState = (FCState*)MEM_SHARED_FLIGHT_STATE_START;

#ifndef UNIT_TEST

static eSTATUS_t ControlInit_Producer (void);
static eSTATUS_t ControlInit_SharedCmdQueue (void);
static eSTATUS_t ControlInit_Consumer (void);
static eSTATUS_t ControlInit_FCState (FCState* pState);
static eSTATUS_t ControlProcessEmptyCmd (DefaultCommand cmd);
static eSTATUS_t ControlProcessOpStateChange (DefaultCommand cmd);
static eSTATUS_t ControlProcessFlightModeChange (DefaultCommand cmd);
static eSTATUS_t ControlProcessVelocityChange (DefaultCommand cmd);
static eSTATUS_t ControlProcessPIDChange (DefaultCommand cmd);
static BOOL_t ControlGetNewCmd (DefaultCommand* pOutCmd);
static BOOL_t IsCmdTypeValid (eCMD_t cmdType);

#endif /* UNIT_TEST */

/*
 * Global UART recv complete callback. The reason this can work is because
 * their is only 1 uart that is set to recv. If there are multiple receiving uarts, this will not work.
 */
void ControlRecvCallBack (eUART_BUS_ID_t busId) {
    if (RawCommandQueue_IsFull () == TRUE) {
        return;
    }
    // LOG_INFO ("%u", ga_UartInterruptBuffer[0]);
    RawCommandQueue_Push ((DefaultCommand*)ga_UartInterruptBuffer);
    UARTRead_IT (busId, ga_UartInterruptBuffer, sizeof (DefaultCommand));
}

STATIC_TESTABLE_DECL eSTATUS_t ControlInit_Producer (void) {

    if (RawCommandQueue_Init () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize raw command queue");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlInit_SharedCmdQueue (void) {

    if (SharedCommandQueue_Init () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize shared command queue");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlInit_Consumer (void) {

    if (ControlRegister_CmdHandler (eCMD_TYPE_EMPTY, ControlProcessEmptyCmd) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to register empty command handler");
        return eSTATUS_FAILURE;
    }

    if (ControlRegister_CmdHandler (eCMD_TYPE_CHANGE_OP_STATE, ControlProcessOpStateChange) !=
        eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to register change op state command handler");
        return eSTATUS_FAILURE;
    }

    if (ControlRegister_CmdHandler (eCMD_TYPE_CHANGE_FLIGHT_MODE, ControlProcessFlightModeChange) !=
        eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to register change flight mode command handler");
        return eSTATUS_FAILURE;
    }

    if (ControlRegister_CmdHandler (eCMD_TYPE_CHANGE_VELOCITY, ControlProcessVelocityChange) !=
        eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to register change velocity command handler");
        return eSTATUS_FAILURE;
    }

    if (ControlRegister_CmdHandler (eCMD_TYPE_CHANGE_PID, ControlProcessPIDChange) !=
        eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to register change pid command handler");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlInit_FCState (FCState* pState) {

    if (pState == NULL) {
        LOG_ERROR ("Flight controller state pointer is NULL");
        return eSTATUS_FAILURE;
    }

    pState->flightMode = eCMD_FLIGHT_MODE_HOVER;
    pState->opState    = eCMD_OP_STATE_STOPPED;

    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlProcessEmptyCmd (DefaultCommand cmd) {
    LOG_INFO ("Received empty command");
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlProcessOpStateChange (DefaultCommand cmd) {

    FCState curState = ControlGetCopyFCState ();
    eCMD_OP_STATE_t requestedState = ((ChangeOpStateCmd*)&cmd)->requestedState;

    if (curState.opState == requestedState) {
        LOG_INFO ("Already in state: %s", ControlOpState2Char (curState.opState));
        return eSTATUS_SUCCESS;
    }

    switch (requestedState) {
    case eCMD_OP_STATE_STOPPED:
    case eCMD_OP_STATE_RUNNING:
    case eCMD_OP_STATE_ERROR: break;
    default:
        LOG_ERROR ("Invalid requested state: %d", requestedState);
        return eSTATUS_FAILURE;
    }

    OpStateTransitionHandler_t handler =
    gaa_OpStateTransitionHandlers[curState.opState][requestedState];
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
        "Transitioned to state: %s",
        ControlOpState2Char (curState.opState)
        );
    } else {
        LOG_INFO ("Transition to state: %s was not performed", ControlOpState2Char (requestedState));
    }
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlProcessFlightModeChange (DefaultCommand cmd) {

    LOG_INFO ("Received flight mode change command");
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlProcessVelocityChange (DefaultCommand cmd) {

    LOG_INFO ("Received velocity change command");
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlProcessPIDChange (DefaultCommand cmd) {

    LOG_INFO ("Received PID change command");
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL BOOL_t ControlGetNewCmd (DefaultCommand* pOutCmd) {

    if (IS_PRODUCER_ME () == TRUE) {
        LOG_ERROR ("Should only be called for the core that is consuming");
        return FALSE;
    }

    if (SharedCommandQueue_IsEmpty () == TRUE) {
        return FALSE;
    }

    // LOG_INFO ("%d", gpSharedCommandQueue->count);
    if (SharedCommandQueue_Pop (pOutCmd) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to dequeue command from shared queue");
        return FALSE;
    }
    return TRUE;
}

STATIC_TESTABLE_DECL BOOL_t IsCmdTypeValid (eCMD_t cmdType) {
    switch (cmdType) {
    case eCMD_TYPE_EMPTY:
    case eCMD_TYPE_CHANGE_OP_STATE:
    case eCMD_TYPE_CHANGE_FLIGHT_MODE:
    case eCMD_TYPE_CHANGE_VELOCITY:
    case eCMD_TYPE_CHANGE_PID: return TRUE;
    default: LOG_ERROR ("Unknown command type: %u", cmdType); return FALSE;
    }
}

eSTATUS_t ControlInit (void) {

    if (IS_PRODUCER_ME () == TRUE) {

        if (ControlInit_SharedCmdQueue () != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize command queue");
            return eSTATUS_FAILURE;
        }

        if (ControlInit_FCState (gp_FlightState) != eSTATUS_SUCCESS) {
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

eSTATUS_t ControlStart (eUART_BUS_ID_t busId) {

    if (IS_PRODUCER_ME () == TRUE) {
        if (UARTRegisterCallback (busId, eUART_CALLBACK_ID_RX, ControlRecvCallBack) !=
            eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to register UART receive callback");
            return eSTATUS_FAILURE;
        }
        if (UARTEnableInterrupt (busId, 8) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to enable UART interrupts");
            return eSTATUS_FAILURE;
        }
        if (UARTRead_IT (busId, ga_UartInterruptBuffer, sizeof (DefaultCommand)) !=
            eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to start UART reception");
            return eSTATUS_FAILURE;
        }
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ControlProcess_RawCmds (void) {

    if (IS_CONSUMER_ME () == TRUE) {
        LOG_ERROR ("Should only be called for the core that is producing");
        return eSTATUS_FAILURE;
    }
    /*
     * NOTE: Right now, this function assumes that the cmd sent by the GUI
     * or RF remote control is always a valid command and doesn't require parsing.
     * The buffer can simply be cast to the appriopriate command type based on the header.
     */
    if (RawCommandQueue_IsEmpty () != TRUE) {
        if (SharedCommandQueue_IsFull () == TRUE) {
            LOG_ERROR ("Shared command queue is full, cannot process new raw commands");
            return eSTATUS_FAILURE;
        }
        DefaultCommand cmd = { 0 };
        // Get the command from the raw command queue
        RawCommandQueue_Pop (&cmd);
        // LOG_INFO ("Processing command of type: %d", cmd.header.commandType);
        // Process command and place it in the shared command queue
        if (SharedCommandQueue_Push (&cmd) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to process and push raw command to shared queue");
            return eSTATUS_FAILURE;
        }
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ControlProcess_Cmds (void) {

    if (IS_PRODUCER_ME () == TRUE) {
        LOG_ERROR ("Should only be called by the core that is consuming");
        return eSTATUS_FAILURE;
    }
    DefaultCommand cmd = { 0 };
    if (ControlGetNewCmd (&cmd) == FALSE) {
        return eSTATUS_SUCCESS; // No new command to process
    }
    // LOG_INFO ("2");
    if (IsCmdTypeValid (cmd.header.commandType) == FALSE) {
        LOG_ERROR ("Invalid command type: %d", cmd.header.commandType);
        return eSTATUS_FAILURE;
    }
    // LOG_INFO ("3");
    CmdHandler_t handler = ga_CmdHandlers[cmd.header.commandType];
    if (handler == NULL) {
        LOG_ERROR (
        "No handler registered for command type: %s",
        ControlCmdType2Char (cmd.header.commandType)
        );
        return eSTATUS_FAILURE;
    }
    // LOG_INFO ("4");
    if (handler (cmd) != eSTATUS_SUCCESS) {
        LOG_ERROR (
        "Failed to process command: %s",
        ControlCmdType2Char (cmd.header.commandType)
        );
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ControlRegister_OPStateTransitionHandler (
eCMD_OP_STATE_t fromState,
eCMD_OP_STATE_t toState,
OpStateTransitionHandler_t handler
) {

    if (fromState >= eNUMBER_OF_OP_STATES || toState >= eNUMBER_OF_OP_STATES) {
        LOG_ERROR ("Invalid state transition: %d -> %d", fromState, toState);
        return eSTATUS_FAILURE;
    }

    gaa_OpStateTransitionHandlers[fromState][toState] = handler;
    return eSTATUS_SUCCESS;
}

eSTATUS_t ControlRegister_CmdHandler (eCMD_t cmdType, CmdHandler_t handler) {

    if (IsCmdTypeValid (cmdType) == FALSE) {
        LOG_ERROR ("Invalid command type: %d", cmdType);
        return eSTATUS_FAILURE;
    }

    ga_CmdHandlers[cmdType] = handler;
    return eSTATUS_SUCCESS;
}

char const* ControlOpState2Char (eCMD_OP_STATE_t opState) {

    switch (opState) {
    case eCMD_OP_STATE_STOPPED: return "[STOPPED]";
    case eCMD_OP_STATE_RUNNING: return "[RUNNING]";
    case eCMD_OP_STATE_ERROR: return "[ERROR]";
    default: return "[UNKNOWN]";
    }
}

char const* ControlCmdType2Char (eCMD_t commandType) {

    switch (commandType) {
    case eCMD_TYPE_EMPTY: return "[EMPTY]";
    case eCMD_TYPE_CHANGE_OP_STATE: return "[CHANGE_OP_STATE]";
    case eCMD_TYPE_CHANGE_FLIGHT_MODE: return "[CHANGE_FLIGHT_MODE]";
    case eCMD_TYPE_CHANGE_VELOCITY: return "[CHANGE_VELOCITY]";
    case eCMD_TYPE_CHANGE_PID: return "[CHANGE_PID]";
    default: return "[UNKNOWN_COMMAND_TYPE]";
    }
}


FCState ControlGetCopyFCState (void) {

    if (gp_FlightState == NULL) {
        LOG_ERROR ("Flight controller state pointer is NULL");
        FCState emptyState = { 0 };
        return emptyState;
    }

    return *gp_FlightState;
}

eSTATUS_t ControlUpdateFCState (FCState const* pNewState) {

    if (pNewState == NULL || gp_FlightState == NULL) {
        LOG_ERROR ("Invalid flight controller state pointer");
        return eSTATUS_FAILURE;
    }

    // Update the flight mode and operation state
    gp_FlightState->flightMode = pNewState->flightMode;
    gp_FlightState->opState    = pNewState->opState;

    return eSTATUS_SUCCESS;
}

eCMD_OP_STATE_t ControlGetOpState (void) {

    if (gp_FlightState == NULL) {
        LOG_ERROR ("Flight controller state pointer is NULL");
        return eCMD_OP_STATE_ERROR;
    }
    return gp_FlightState->opState;
}
