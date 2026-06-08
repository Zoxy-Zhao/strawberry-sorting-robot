#include "hal_data.h"
#include <stdbool.h>
#include <stdint.h>

#define PCA9685_MODE1          0x00
#define PCA9685_MODE2          0x01
#define PCA9685_PRESCALE       0xFE
#define PCA9685_LED0_ON_L      0x06

/* 当前只测试 0 号通道 */
#define SERVO_TEST_CHANNEL     0U

/* 标定时固定中位 */
#define SERVO_CENTER_US        1500U

/* 每次只改这个值做单点测试 */
#define SERVO_TARGET_US        1300U

/* 上电先到中位，再去目标位 */
#define SERVO_START_DELAY_MS   1500U
#define SERVO_MOVE_DELAY_MS    2000U

static volatile bool g_i2c_tx_done = false;
static volatile bool g_i2c_error   = false;

void i2c_master1_callback(i2c_master_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case I2C_MASTER_EVENT_TX_COMPLETE:
        {
            g_i2c_tx_done = true;
            g_i2c_error   = false;
            break;
        }

        case I2C_MASTER_EVENT_ABORTED:
        {
            g_i2c_tx_done = true;
            g_i2c_error   = true;
            break;
        }

        default:
        {
            break;
        }
    }
}

static fsp_err_t i2c_write_bytes(uint8_t * p_data, uint32_t length)
{
    fsp_err_t err;

    g_i2c_tx_done = false;
    g_i2c_error   = false;

    err = R_IIC_MASTER_Write(&g_i2c_master1_ctrl, p_data, length, false);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    while (!g_i2c_tx_done)
    {
        ;
    }

    if (g_i2c_error)
    {
        return FSP_ERR_ABORTED;
    }

    return FSP_SUCCESS;
}

static fsp_err_t pca9685_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2];

    buf[0] = reg;
    buf[1] = value;

    return i2c_write_bytes(buf, 2);
}

static fsp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off)
{
    uint8_t reg = (uint8_t)(PCA9685_LED0_ON_L + 4U * channel);
    uint8_t buf[5];

    buf[0] = reg;
    buf[1] = (uint8_t)(on & 0xFFU);
    buf[2] = (uint8_t)((on >> 8) & 0x0FU);
    buf[3] = (uint8_t)(off & 0xFFU);
    buf[4] = (uint8_t)((off >> 8) & 0x0FU);

    return i2c_write_bytes(buf, 5);
}

static uint16_t servo_us_to_counts(uint16_t pulse_us)
{
    /* PCA9685 频率设置为 50Hz，对应周期 20ms
     * count = pulse_us / 20000us * 4096
     */
    uint32_t counts = ((uint32_t)pulse_us * 4096U) / 20000U;
    return (uint16_t)counts;
}

static fsp_err_t servo_set_pulse_us(uint8_t channel, uint16_t pulse_us)
{
    uint16_t off_count = servo_us_to_counts(pulse_us);
    return pca9685_set_pwm(channel, 0U, off_count);
}

static fsp_err_t pca9685_init(void)
{
    fsp_err_t err;

    err = pca9685_write_reg(PCA9685_MODE1, 0x10U);   /* sleep */
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = pca9685_write_reg(PCA9685_PRESCALE, 121U); /* 50Hz */
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = pca9685_write_reg(PCA9685_MODE2, 0x04U);   /* totem pole */
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = pca9685_write_reg(PCA9685_MODE1, 0x01U);   /* wake up */
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);

    err = pca9685_write_reg(PCA9685_MODE1, 0xA1U);   /* restart + auto increment */
    return err;
}

void hal_entry(void)
{
#if BSP_TZ_SECURE_BUILD
    R_BSP_NonSecureEnter();
#else
    fsp_err_t err;

    err = R_IIC_MASTER_Open(&g_i2c_master1_ctrl, &g_i2c_master1_cfg);
    if (FSP_SUCCESS != err)
    {
        while (1)
        {
            ;
        }
    }

    err = pca9685_init();
    if (FSP_SUCCESS != err)
    {
        while (1)
        {
            ;
        }
    }

    /* 先到中位 */
    err = servo_set_pulse_us(SERVO_TEST_CHANNEL, SERVO_CENTER_US);
    if (FSP_SUCCESS != err)
    {
        while (1)
        {
            ;
        }
    }

    R_BSP_SoftwareDelay(SERVO_START_DELAY_MS, BSP_DELAY_UNITS_MILLISECONDS);

    /* 再到目标测试点 */
    err = servo_set_pulse_us(SERVO_TEST_CHANNEL, SERVO_TARGET_US);
    if (FSP_SUCCESS != err)
    {
        while (1)
        {
            ;
        }
    }

    /* 停在目标位置，便于观察是否抖动、卡死、顶限位 */
    while (1)
    {
        ;
    }
#endif
}
