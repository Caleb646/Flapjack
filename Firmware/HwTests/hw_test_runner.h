#ifndef HW_TEST_RUNNER_H
#define HW_TEST_RUNNER_H

#include <stdint.h>

/* Populated by hw_test_main.c; updated by HwTest_ReportResult(). */
extern int g_hw_tests_passed;
extern int g_hw_tests_total;

void HwTest_UartInit(void);
void HwTest_Printf(const char* fmt, ...);

static inline void HwTest_ReportPass(const char* name) {
    extern void HwTest_Printf(const char*, ...);
    HwTest_Printf("TEST:%s:PASS\r\n", name);
    g_hw_tests_passed++;
    g_hw_tests_total++;
}

static inline void HwTest_ReportFail(const char* name, const char* detail) {
    extern void HwTest_Printf(const char*, ...);
    HwTest_Printf("TEST:%s:FAIL:%s\r\n", name, detail);
    g_hw_tests_total++;
}

#define HW_ASSERT_EQUAL(name, expected, actual)                                  \
    do {                                                                          \
        uint32_t _exp = (uint32_t)(expected);                                    \
        uint32_t _act = (uint32_t)(actual);                                      \
        if (_exp == _act) {                                                       \
            HwTest_ReportPass(name);                                              \
        } else {                                                                  \
            char _buf[64];                                                        \
            extern int snprintf(char*, unsigned long, const char*, ...);          \
            snprintf(_buf, sizeof(_buf), "expected=%lu:actual=%lu",               \
                     (unsigned long)_exp, (unsigned long)_act);                   \
            HwTest_ReportFail(name, _buf);                                        \
        }                                                                         \
    } while (0)

#define HW_ASSERT_WITHIN(name, expected, actual, tolerance)                      \
    do {                                                                          \
        uint32_t _exp = (uint32_t)(expected);                                    \
        uint32_t _act = (uint32_t)(actual);                                      \
        uint32_t _tol = (uint32_t)(tolerance);                                   \
        uint32_t _diff = (_act > _exp) ? (_act - _exp) : (_exp - _act);          \
        if (_diff <= _tol) {                                                      \
            HwTest_Printf("TEST:%s:PASS:expected=%lu:actual=%lu\r\n",            \
                          name, (unsigned long)_exp, (unsigned long)_act);       \
            g_hw_tests_passed++;                                                  \
            g_hw_tests_total++;                                                   \
        } else {                                                                  \
            HwTest_Printf("TEST:%s:FAIL:expected=%lu:actual=%lu:tol=%lu\r\n",    \
                          name, (unsigned long)_exp, (unsigned long)_act,        \
                          (unsigned long)_tol);                                  \
            g_hw_tests_total++;                                                   \
        }                                                                         \
    } while (0)

#endif /* HW_TEST_RUNNER_H */
