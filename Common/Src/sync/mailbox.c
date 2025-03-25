#include <string.h>

#include "sync/mailbox.h"
#include "mem/mem.h"

static uint8_t volatile* SyncMailBoxGet(uint32_t mbID);

static uint8_t volatile* SyncMailBoxGet(uint32_t mbID)
{
    uint8_t volatile *pMB = NULL;
    if(mbID == MAILBOX_CM7_ID) pMB = (uint8_t volatile*)MEM_SHARED_MAILBOX_CM7_START;
    else pMB = (uint8_t volatile*)MEM_SHARED_MAILBOX_CM4_START;
    return pMB;
}

int8_t SyncMailBoxWrite(uint32_t mbID, uint8_t *pBuffer, uint32_t len)
{
    if(len > MEM_SHARED_MAILBOX_LEN) return -1;
    uint8_t volatile *pMB = SyncMailBoxGet(mbID);
    memcpy((void*)pMB, (void*)pBuffer, len);
    return 0;
}

int8_t SyncMailBoxWriteNotify(uint32_t mbID, uint8_t *pBuffer, uint32_t len)
{
    int8_t status = SyncMailBoxWrite(mbID, pBuffer, len);
    if(status < 0) return status;
    asm volatile ("dsb");
    asm volatile ("sev");
    return 0;
}

int8_t SyncMailBoxRead(uint32_t mbID, uint8_t *pBuffer, uint32_t len)
{
    if(len > MEM_SHARED_MAILBOX_LEN) return -1;
    uint8_t volatile *pMB = SyncMailBoxGet(mbID);
    memcpy((void*)pBuffer, (void*)pMB, len);
    return 0;
}
