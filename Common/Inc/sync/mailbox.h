
#ifndef SYNC_MAILBOX_H
#define SYNC_MAILBOX_H

#include "common.h"
#include "stdint.h"

#define MAILBOX_CM4_ID 0
#define MAILBOX_CM7_ID 1

STATUS_TYPE SyncMailBoxWrite (uint32_t mbID, uint8_t* pBuffer, uint32_t len);
STATUS_TYPE SyncMailBoxWriteNotify (uint32_t mbID, uint8_t* pBuffer, uint32_t len);
STATUS_TYPE SyncMailBoxRead (uint32_t mbID, uint8_t* pBuffer, uint32_t len);

#endif // SYNC_MAILBOX_H
