#ifndef FJ_TASKS_H
#define FJ_TASKS_H

typedef struct Task_s Task_t;

typedef struct Task_s {
    eSTATUS_t (*fn) (Task_t* pSelf, uint32_t currentTimeUs);
    void* pArg;
    char const* pName;
} Task_t;


#endif // FJ_TASKS_H