#ifndef AERO_TASKS_H
#define AERO_TASKS_H

typedef struct Task_s {
    void (*fn) (void*);
    void* pArg;
    char const* pName;
} Task_t;


#endif // AERO_TASKS_H