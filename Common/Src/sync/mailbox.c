#include <string.h>

#include "mem/mem.h"
#include "sync/mailbox.h"

static uint8_t volatile* SyncMailBoxGet (uint32_t mbID);

static uint8_t volatile* SyncMailBoxGet (uint32_t mbID) {
    uint8_t volatile* pMB = NULL;
    if (mbID == MAILBOX_CM7_ID) {
        pMB = (uint8_t volatile*)MEM_SHARED_MAILBOX_CM7_START;
    } else {
        pMB = (uint8_t volatile*)MEM_SHARED_MAILBOX_CM4_START;
    }
    return pMB;
}

STATUS_TYPE SyncMailBoxWrite (uint32_t mbID, uint8_t* pBuffer, uint32_t len) {
    if (len > MEM_SHARED_MAILBOX_LEN) {
        return eSTATUS_FAILURE;
    }

    uint8_t volatile* pMB = SyncMailBoxGet (mbID);
    memcpy ((void*)pMB, (void*)pBuffer, len);
    return eSTATUS_SUCCESS;
}

STATUS_TYPE SyncMailBoxWriteNotify (uint32_t mbID, uint8_t* pBuffer, uint32_t len) {
    STATUS_TYPE status = SyncMailBoxWrite (mbID, pBuffer, len);
    if (status != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }
    asm volatile ("dsb");
    asm volatile ("sev");
    return eSTATUS_SUCCESS;
}

STATUS_TYPE SyncMailBoxRead (uint32_t mbID, uint8_t* pBuffer, uint32_t len) {
    if (len > MEM_SHARED_MAILBOX_LEN) {
        return eSTATUS_FAILURE;
    }
    uint8_t volatile* pMB = SyncMailBoxGet (mbID);
    memcpy ((void*)pBuffer, (void*)pMB, len);
    return eSTATUS_SUCCESS;
}
