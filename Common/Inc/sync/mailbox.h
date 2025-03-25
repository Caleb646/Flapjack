
#ifndef SYNC_MAILBOX_H
#define SYNC_MAILBOX_H

#include "stdint.h"
#include "common.h"

#define MAILBOX_CM4_ID 0
#define MAILBOX_CM7_ID 1

int8_t SyncMailBoxWrite(uint32_t mbID, uint8_t *pBuffer, uint32_t len);
int8_t SyncMailBoxWriteNotify(uint32_t mbID, uint8_t *pBuffer, uint32_t len);
int8_t SyncMailBoxRead(uint32_t mbID, uint8_t *pBuffer, uint32_t len);

#endif // SYNC_MAILBOX_H
 