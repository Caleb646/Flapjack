#ifndef MAG_MMC5983_H
#define MAG_MMC5983_H

#include <stdint.h>

// Registers definitions
#define MMC5983_X_OUT_0_REG      0x00U
#define MMC5983_X_OUT_1_REG      0X01U
#define MMC5983_Y_OUT_0_REG      0x02U
#define MMC5983_Y_OUT_1_REG      0x03U
#define MMC5983_Z_OUT_0_REG      0x04U
#define MMC5983_Z_OUT_1_REG      0x05U
#define MMC5983_XYZ_OUT_2_REG    0x06U
#define MMC5983_T_OUT_REG        0x07U
#define MMC5983_STATUS_REG       0x08U
#define MMC5983_INT_CTRL_0_REG   0x09U
#define MMC5983_INT_CTRL_1_REG   0x0AU
#define MMC5983_INT_CTRL_2_REG   0x0BU
#define MMC5983_INT_CTRL_3_REG   0x0CU
#define MMC5983_PROD_ID_REG      0x2FU
#define MMC5983_DUMMY            0x00U

// Constants definitions
#define MMC5983_I2C_ADDR         0x30U
#define MMC5983_PROD_ID          0x30U

// Bits definitions
#define MMC5983_MEAS_M_DONE      (1U << 0U)
#define MMC5983_MEAS_T_DONE      (1U << 1U)
#define MMC5983_OTP_READ_DONE    (1U << 4U)
#define MMC5983_TM_M             (1U << 0U)
#define MMC5983_TM_T             (1U << 1U)
#define MMC5983_INT_MEAS_DONE_EN (1U << 2U)
#define MMC5983_SET_OPERATION    (1U << 3U)
#define MMC5983_RESET_OPERATION  (1U << 4U)
#define MMC5983_AUTO_SR_EN       (1U << 5U)
#define MMC5983_OTP_READ         (1U << 6U)
#define MMC5983_BW0              (1U << 0U)
#define MMC5983_BW1              (1U << 1U)
#define MMC5983_X_INHIBIT        (1U << 2U)
#define MMC5983_YZ_INHIBIT       (3U << 3U)
#define MMC5983_SW_RST           (1U << 7U)
#define MMC5983_CM_FREQ_0        (1U << 0U)
#define MMC5983_CM_FREQ_1        (1U << 1U)
#define MMC5983_CM_FREQ_2        (1U << 2U)
#define MMC5983_CMM_EN           (1U << 3U)
#define MMC5983_PRD_SET_0        (1U << 4U)
#define MMC5983_PRD_SET_1        (1U << 5U)
#define MMC5983_PRD_SET_2        (1U << 6U)
#define MMC5983_EN_PRD_SET       (1U << 7U)
#define MMC5983_ST_ENP           (1U << 1U)
#define MMC5983_ST_ENM           (1U << 2U)
#define MMC5983_SPI_3W           (1U << 6U)
#define MMC5983_X2_MASK          (3U << 6U)
#define MMC5983_Y2_MASK          (3U << 4U)
#define MMC5983_Z2_MASK          (3U << 2U)
#define MMC5983_XYZ_0_SHIFT      10U
#define MMC5983_XYZ_1_SHIFT      2U


typedef int8_t eMMC5983_ERROR_t;
enum {
    eMMC5983_NONE                         = -1,
    eMMC5983_I2C_INITIALIZATION_ERROR     = -2,
    eMMC5983_SPI_INITIALIZATION_ERROR     = -3,
    eMMC5983_INVALID_DEVICE               = -4,
    eMMC5983_BUS_ERROR                    = -5,
    eMMC5983_INVALID_FILTER_BANDWIDTH     = -6,
    eMMC5983_INVALID_CONTINUOUS_FREQUENCY = -7,
    eMMC5983_INVALID_PERIODIC_SAMPLES     = -8
};


#endif // MAG_MMC5983_H