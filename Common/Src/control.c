#include "control.h"
#include "common.h"
#include "conf/conf.h"
#include "hal.h"
#include "log/logger.h"
#include "mc/fcstate.h"
#include "mem/mem.h"
#include "mem/queue.h"
#include "mem/umap.h"
#include "peripheral/bus/uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define CMD_TYPE_VALID(CMD_TYPE) ((CMD_TYPE) != eCMD_NULL && (CMD_TYPE) < eNUMBER_OF_CMD_TYPES)
#define OP_CMD2KEY(CMD_TYPE, cSTATE, nSTATE) \
    ((((CMD_TYPE) * 31U) + (uint32_t)(cSTATE)) * 31U + (uint32_t)(nSTATE))
#define PRODUCER_ID            CM4_CPUID
#define CONSUMER_ID            CM7_CPUID
#define IS_PRODUCER_ME()       (HAL_GetCurrentCPUID () == PRODUCER_ID)
#define IS_CONSUMER_ME()       (HAL_GetCurrentCPUID () == CONSUMER_ID)
#define COMMAND_QUEUE_CAPACITY 16U
#define UART_RECV_BUFFER_SIZE  (CMD_TOTAL_SIZE_BYTES) // + 2U)

/* Consumer global variables */
QUEUE_DEFINE_STATIC (RawCommand, DefaultCommand, COMMAND_QUEUE_CAPACITY, true);
// A local only buffer to store raw commands during the interrupt handler
// static uint8_t ga_UartInterruptBuffer[UART_RECV_BUFFER_SIZE] = { 0 };

/* Shared global variables */
QUEUE_DEFINE_STATIC_SHARED (SharedCommand, DefaultCommand, COMMAND_QUEUE_CAPACITY);
// UMAP_DEFINE_STATIC_SHARED (CmdHandlers, eCMD_t, CmdHandlerFn_t, eNUMBER_OF_CMD_TYPES * 5U);

#ifndef UNIT_TEST

static eSTATUS_t ControlInit_Producer (void);
static eSTATUS_t ControlInit_Shared (void);
static eSTATUS_t ControlInit_Consumer (void);
static bool ControlGetNewCmd (DefaultCommand* pOutCmd);

#endif /* UNIT_TEST */

/*
 * Global UART recv complete callback. The reason this can work is because
 * their is only 1 uart that is set to recv. If there are multiple receiving uarts, this will not work.
 */
// void ControlRecvCallBack (eBUS_ID_t busId) {

//     if (RawCommandQueue_IsFull () == true) {
//         return;
//     }
//     // LOG_INFO ("%u", ga_UartInterruptBuffer[0]);
//     RawCommandQueue_Push ((DefaultCommand*)ga_UartInterruptBuffer);
//     UARTRead_IT (busId, ga_UartInterruptBuffer, sizeof (DefaultCommand));
// }

// STATIC_TESTABLE_DECL CmdHandlerFn_t ControlGetHandler (DefaultCommand defCmd) {

//     eCMD_t cmdType = defCmd.header.commandType;
//     if (cmdType == eCMD_CHANGE_OP_STATE) {

//         vFCState_t const* pState = FCStateGetActiveState ();
//         ChangeOpStateCmd cmd     = *(ChangeOpStateCmd*)&defCmd;
//         uint16_t cState          = pState->opState;
//         uint16_t nState          = cmd.requestedState;
//         uint32_t key             = OP_CMD2KEY (cmdType, cState, nState);
//         CmdHandlerFn_t* pHandler = CmdHandlersUMap_FindPtr (&key);
//         return pHandler ? *pHandler : NULL;
//     }

//     CmdHandlerFn_t* pHandler = CmdHandlersUMap_FindPtr (&cmdType);
//     return pHandler ? *pHandler : NULL;
// }

STATIC_TESTABLE_DECL eSTATUS_t ControlInit_Producer (void) {

    if (RawCommandQueue_Init () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize raw command queue");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlInit_Shared (void) {

    // if (CmdHandlersUMap_Init () == false) {
    //     LOG_ERROR ("Failed to initialize command handlers map");
    //     return eSTATUS_FAILURE;
    // }

    if (SharedCommandQueue_Init () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize shared command queue");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlDefaultCmdHandler (DefaultCommand cmd) {

    eCMD_t cmdType = cmd.header.commandType;
    LOG_WARN ("No handler registered for command type: %s", ControlCmdType2Char (cmdType));
    return false;
}

STATIC_TESTABLE_DECL eSTATUS_t ControlInit_Consumer (void) {

    eCMD_t cmds[]             = { eCMD_NULL,
                                  // eCMD_CHANGE_OP_STATE, // op state changes can have multiple handlers based on current and next state
                                  eCMD_CHANGE_FLIGHT_MODE,
                                  eCMD_CHANGE_VELOCITY,
                                  eCMD_CHANGE_PID };
    CmdHandlerFn_t handlers[] = { ControlDefaultCmdHandler,
                                  // ControlDefaultCmdHandler,
                                  ControlDefaultCmdHandler,
                                  ControlDefaultCmdHandler,
                                  ControlDefaultCmdHandler };

    for (size_t i = 0; i < sizeof (cmds) / sizeof (cmds[0]); i++) {

        SubCommand_t subCmd = { .raw = 0 };
        if (ControlRegisterHandler (cmds[i], subCmd, handlers[i]) != false) {
            LOG_ERROR ("Failed to register command handler for command: %d", cmds[i]);
            return eSTATUS_FAILURE;
        }
    }
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL bool ControlGetNewCmd (DefaultCommand* pOutCmd) {

    if (IS_PRODUCER_ME () == true) {
        LOG_ERROR ("Should only be called for the core that is consuming");
        return false;
    }

    if (SharedCommandQueue_IsEmpty () == true) {
        return false;
    }

    // LOG_INFO ("%d", gpSharedCommandQueue->count);
    if (SharedCommandQueue_Pop (pOutCmd) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to dequeue command from shared queue");
        return false;
    }
    return true;
}

eSTATUS_t ControlInit (void) {

    if (IS_PRODUCER_ME () == true) {

        if (ControlInit_Shared () != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize shared");
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

eSTATUS_t ControlStart (void) {

    if (IS_PRODUCER_ME () == true) {
        // if (UARTRegisterCallback (busId, eUART_CALLBACK_ID_RX, ControlRecvCallBack) !=
        //     eSTATUS_SUCCESS) {
        //     LOG_ERROR ("Failed to register UART receive callback");
        //     return eSTATUS_FAILURE;
        // }
        // if (UARTEnableInterrupt (busId, 8) != eSTATUS_SUCCESS) {
        //     LOG_ERROR ("Failed to enable UART interrupts");
        //     return eSTATUS_FAILURE;
        // }
        // if (UARTRead_IT (busId, ga_UartInterruptBuffer, sizeof (DefaultCommand)) !=
        //     eSTATUS_SUCCESS) {
        //     LOG_ERROR ("Failed to start UART reception");
        //     return eSTATUS_FAILURE;
        // }
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ControlProcess_RawCmds (void) {

    if (IS_CONSUMER_ME () == true) {
        LOG_ERROR ("Should only be called for the core that is producing");
        return eSTATUS_FAILURE;
    }
    /*
     * NOTE: Right now, this function assumes that the cmd sent by the GUI
     * or RF remote control is always a valid command and doesn't require parsing.
     * The buffer can simply be cast to the appriopriate command type based on the header.
     */
    if (RawCommandQueue_IsEmpty () != true) {
        if (SharedCommandQueue_IsFull () == true) {
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

    RETURN_IF (IS_CONSUMER_ME () == false, eSTATUS_FAILURE, "Should only be called by the core that is consuming");

    DefaultCommand cmd = { 0 };
    RETURN_IF (ControlGetNewCmd (&cmd) == false, eSTATUS_FAILURE, "Failed to get new command");

    eCMD_t cmdType = cmd.header.commandType;
    RETURN_IF (CMD_TYPE_VALID (cmdType) == false, eSTATUS_FAILURE, "Invalid command type: %d", cmdType);

    // CmdHandlerFn_t handler = ControlGetHandler (cmd);
    // RETURN_IF_NULL (handler, eSTATUS_FAILURE, "No handler registered for command type: %d", cmdType);
    // RETURN_IF (handler (cmd) != eSTATUS_SUCCESS, eSTATUS_FAILURE, "Failed to process command");

    return eSTATUS_SUCCESS;
}

bool ControlRegisterHandler (eCMD_t cmdType, SubCommand_t subCmd, CmdHandlerFn_t handler) {

    FJ_UNUSED (cmdType);
    FJ_UNUSED (subCmd);
    FJ_UNUSED (handler);

    return true;

    // TODO: re-implement. UMAP doesnt support function pointers
    // RETURN_IF (CMD_TYPE_VALID (cmdType) == false, false, "Invalid command type: %d", cmdType);
    // RETURN_IF (handler == NULL, false, "Invalid handler for command type: %d", cmdType);

    // uint32_t key = 0;
    // if (cmdType == eCMD_CHANGE_OP_STATE) {
    //     uint16_t cState = subCmd.opstate.cState;
    //     uint16_t nState = subCmd.opstate.nState;
    //     key             = OP_CMD2KEY (cmdType, cState, nState);
    // } else {
    //     key = cmdType;
    // }
    // return CmdHandlersUMap_InsertByValue (key, handler);
}

char const* ControlCmdType2Char (eCMD_t commandType) {

    switch (commandType) {
    case eCMD_NULL: return "[NULL]";
    case eCMD_CHANGE_OP_STATE: return "[CHANGE_OP_STATE]";
    case eCMD_CHANGE_FLIGHT_MODE: return "[CHANGE_FLIGHT_MODE]";
    case eCMD_CHANGE_VELOCITY: return "[CHANGE_VELOCITY]";
    case eCMD_CHANGE_PID: return "[CHANGE_PID]";
    default: return "[UNKNOWN_COMMAND_TYPE]";
    }
}