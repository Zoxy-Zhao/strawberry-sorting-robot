#include "hal_data.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/*  PCA9685 寄存器地址                                                  */
/* ------------------------------------------------------------------ */
#define PCA9685_MODE1          0x00
#define PCA9685_MODE2          0x01
#define PCA9685_PRESCALE       0xFE
#define PCA9685_LED0_ON_L      0x06

/* ------------------------------------------------------------------ */
/*  舵机常量                                                            */
/* ------------------------------------------------------------------ */
#define SERVO_COUNT            6U
#define SERVO_MOVE_GAP_MS      80U

/* ------------------------------------------------------------------ */
/*  关节枚举                                                            */
/* ------------------------------------------------------------------ */
typedef enum e_joint_id
{
    JOINT_BASE = 0,          /* CH5, 30kg, 270deg */
    JOINT_SHOULDER,          /* CH1, 25kg, 180deg */
    JOINT_ELBOW,             /* CH0, 20kg, 180deg */
    JOINT_WRIST_PITCH,       /* CH2, 20kg, 180deg */
    JOINT_WRIST_ROTATE,      /* CH3, 20kg, 180deg */
    JOINT_GRIPPER,           /* CH4, 20kg, 180deg */
} joint_id_t;

/* ------------------------------------------------------------------ */
/*  舵机参数结构体                                                       */
/* ------------------------------------------------------------------ */
typedef struct
{
    uint8_t  channel;
    uint16_t angle_range_deg;
    uint16_t center_us;
    uint16_t left_limit_us;
    uint16_t right_limit_us;
    uint16_t left_safe_us;
    uint16_t right_safe_us;
    bool     reversed;
} servo_param_t;

typedef struct
{
    uint16_t angle_deg[SERVO_COUNT];
} robot_pose_t;

/* ------------------------------------------------------------------ */
/*  舵机参数表                                                           */
/* ------------------------------------------------------------------ */
static const servo_param_t g_servos[SERVO_COUNT] =
{
    [JOINT_BASE] =
    {
        .channel         = 5U,
        .angle_range_deg = 270U,
        .center_us       = 1500U,
        .left_limit_us   = 333U,
        .right_limit_us  = 2949U,
        .left_safe_us    = 500U,
        .right_safe_us   = 2750U,
        .reversed        = false,
    },
    [JOINT_SHOULDER] =
    {
        .channel         = 1U,
        .angle_range_deg = 180U,
        .center_us       = 1500U,
        .left_limit_us   = 333U,
        .right_limit_us  = 2954U,
        .left_safe_us    = 500U,
        .right_safe_us   = 2750U,
        .reversed        = false,
    },
    [JOINT_ELBOW] =
    {
        .channel         = 0U,
        .angle_range_deg = 180U,
        .center_us       = 1500U,
        .left_limit_us   = 333U,
        .right_limit_us  = 2949U,
        .left_safe_us    = 500U,
        .right_safe_us   = 2750U,
        .reversed        = false,
    },
    [JOINT_WRIST_PITCH] =
    {
        .channel         = 2U,
        .angle_range_deg = 180U,
        .center_us       = 1500U,
        .left_limit_us   = 333U,
        .right_limit_us  = 2954U,
        .left_safe_us    = 500U,
        .right_safe_us   = 2750U,
        .reversed        = false,
    },
    [JOINT_WRIST_ROTATE] =
    {
        .channel         = 3U,
        .angle_range_deg = 180U,
        .center_us       = 1500U,
        .left_limit_us   = 333U,
        .right_limit_us  = 2949U,
        .left_safe_us    = 500U,
        .right_safe_us   = 2750U,
        .reversed        = false,
    },
    [JOINT_GRIPPER] =
    {
        .channel         = 4U,
        .angle_range_deg = 180U,
        .center_us       = 1500U,
        .left_limit_us   = 333U,
        .right_limit_us  = 2958U,
        .left_safe_us    = 380U,
        .right_safe_us   = 2750U,
        .reversed        = true,
    },
};

/* ------------------------------------------------------------------ */
/*  HOME 姿态                                                           */
/* ------------------------------------------------------------------ */
static const robot_pose_t g_home_pose =
{
    .angle_deg =
    {
        135U,  /* BASE CH5 */
        70U,   /* SHOULDER CH1  增向前 */
        70U,  /* ELBOW  CH0*/
        150U,   /* WRIST_PITCH  CH2 */
        80U,   /* WRIST_ROTATE   CH3*/
        110U   /* GRIPPER  CH4 - 半开待抓 */
    }
};

/* ------------------------------------------------------------------ */
/*  I2C 全局状态                                                        */
/* ------------------------------------------------------------------ */
static volatile bool g_i2c_tx_done = false;
static volatile bool g_i2c_error   = false;

/* ------------------------------------------------------------------ */
/*  UART 全局状态                                                       */
/* ------------------------------------------------------------------ */
static volatile bool    g_uart_tx_complete = false;
static volatile uint8_t g_uart_rx_char     = 0U;
static volatile bool    g_uart_rx_ready    = false;

/* ------------------------------------------------------------------ */
/*  Teach 模式全局状态                                                  */
/* ------------------------------------------------------------------ */
static robot_pose_t g_teach_pose = { .angle_deg = {0U} };
static joint_id_t   g_teach_joint = JOINT_BASE;
static uint16_t     g_teach_step  = 5U;

static const char * const g_joint_names[SERVO_COUNT] =
{
    "BASE",
    "SHOULDER",
    "ELBOW",
    "WRIST_PITCH",
    "WRIST_ROTATE",
    "GRIPPER"
};

/* ------------------------------------------------------------------ */
/*  UART 回调                                                           */
/* ------------------------------------------------------------------ */
void uart_callback(uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
            g_uart_tx_complete = true;
            break;
        case UART_EVENT_RX_CHAR:
            g_uart_rx_char  = (uint8_t)p_args->data;
            g_uart_rx_ready = true;
            break;
        default:
            break;
    }
}

/* ------------------------------------------------------------------ */
/*  UART 阻塞发送（以 \0 结尾的字符串）                                  */
/* ------------------------------------------------------------------ */
static void uart_log(const char * str)
{
    uint32_t len = (uint32_t)strlen(str);
    if (0U == len)
    {
        return;
    }
    g_uart_tx_complete = false;
    R_SCI_UART_Write(&g_uart0_ctrl, (uint8_t *)str, len);
    while (!g_uart_tx_complete) { ; }
}

/* ------------------------------------------------------------------ */
/*  Newlib syscall 桩函数：printf 依赖这些符号                          */
/* ------------------------------------------------------------------ */
int _write(int fd, char * pBuffer, int size)
{
    (void)fd;
    if (size <= 0) { return size; }
    g_uart_tx_complete = false;
    R_SCI_UART_Write(&g_uart0_ctrl, (uint8_t *)pBuffer, (uint32_t)size);
    while (!g_uart_tx_complete) { ; }
    return size;
}

int _close(int fd)                         { (void)fd; return -1; }
int _lseek(int fd, int offset, int whence) { (void)fd; (void)offset; (void)whence; return -1; }
int _read(int fd, char *buf, int count)    { (void)fd; (void)buf; (void)count; return -1; }
int _fstat(int fd, void *buf)              { (void)fd; (void)buf; return -1; }
int _isatty(int fd)                        { (void)fd; return 1; }

/* ------------------------------------------------------------------ */
/*  I2C 回调                                                            */
/* ------------------------------------------------------------------ */
void i2c_master1_callback(i2c_master_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case I2C_MASTER_EVENT_TX_COMPLETE:
            g_i2c_tx_done = true;
            g_i2c_error   = false;
            break;
        case I2C_MASTER_EVENT_ABORTED:
            g_i2c_tx_done = true;
            g_i2c_error   = true;
            break;
        default:
            break;
    }
}

/* ------------------------------------------------------------------ */
/*  I2C 辅助函数                                                        */
/* ------------------------------------------------------------------ */
static fsp_err_t i2c_write_bytes(uint8_t * p_data, uint32_t length)
{
    fsp_err_t err;
    g_i2c_tx_done = false;
    g_i2c_error   = false;
    err = R_IIC_MASTER_Write(&g_i2c_master1_ctrl, p_data, length, false);
    if (FSP_SUCCESS != err) return err;
    while (!g_i2c_tx_done) { ; }
    return g_i2c_error ? FSP_ERR_ABORTED : FSP_SUCCESS;
}

/* ------------------------------------------------------------------ */
/*  PCA9685 驱动                                                        */
/* ------------------------------------------------------------------ */
static fsp_err_t pca9685_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = value;
    return i2c_write_bytes(buf, 2U);
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
    return i2c_write_bytes(buf, 5U);
}

/* ------------------------------------------------------------------ */
/*  舵机驱动                                                            */
/* ------------------------------------------------------------------ */
static uint16_t servo_us_to_counts(uint16_t pulse_us)
{
    uint32_t counts = ((uint32_t)pulse_us * 4096U) / 20000U;
    return (uint16_t)counts;
}

static fsp_err_t servo_set_pulse_us(uint8_t channel, uint16_t pulse_us)
{
    return pca9685_set_pwm(channel, 0U, servo_us_to_counts(pulse_us));
}

static uint16_t servo_angle_to_us(const servo_param_t * p_servo, uint16_t angle_deg)
{
    uint16_t max_angle = p_servo->angle_range_deg;
    uint32_t pulse;
    if (angle_deg > max_angle) angle_deg = max_angle;
    if (p_servo->reversed)
        pulse = p_servo->right_safe_us - ((uint32_t)(p_servo->right_safe_us - p_servo->left_safe_us) * angle_deg) / max_angle;
    else
        pulse = p_servo->left_safe_us + ((uint32_t)(p_servo->right_safe_us - p_servo->left_safe_us) * angle_deg) / max_angle;
    return (uint16_t)pulse;
}

static fsp_err_t servo_set_angle(joint_id_t joint, uint16_t angle_deg)
{
    const servo_param_t * p_servo = &g_servos[joint];
    return servo_set_pulse_us(p_servo->channel, servo_angle_to_us(p_servo, angle_deg));
}

static fsp_err_t robot_apply_pose(const robot_pose_t * p_pose)
{
    fsp_err_t err;
    uint32_t i;
    for (i = 0; i < SERVO_COUNT; i++)
    {
        err = servo_set_angle((joint_id_t)i, p_pose->angle_deg[i]);
        if (FSP_SUCCESS != err) return err;
        R_BSP_SoftwareDelay(SERVO_MOVE_GAP_MS, BSP_DELAY_UNITS_MILLISECONDS);
    }
    return FSP_SUCCESS;
}

/* ------------------------------------------------------------------ */
/*  PCA9685 初始化                                                      */
/* ------------------------------------------------------------------ */
static fsp_err_t pca9685_init(void)
{
    fsp_err_t err;
    err = pca9685_write_reg(PCA9685_MODE1, 0x10U);
    if (FSP_SUCCESS != err) return err;
    err = pca9685_write_reg(PCA9685_PRESCALE, 121U);
    if (FSP_SUCCESS != err) return err;
    err = pca9685_write_reg(PCA9685_MODE2, 0x04U);
    if (FSP_SUCCESS != err) return err;
    err = pca9685_write_reg(PCA9685_MODE1, 0x01U);
    if (FSP_SUCCESS != err) return err;
    R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
    return pca9685_write_reg(PCA9685_MODE1, 0xA1U);
}

/* ------------------------------------------------------------------ */
/*  Teach 模式：串口单字符命令控制关节                                  */
/*  0-5=选关节  w/s=增减角度  q/e/r=步进1/5/10  p=打印  h=HOME        */
/* ------------------------------------------------------------------ */
static void teach_mode(void)
{
    if (!g_uart_rx_ready)
    {
        return;
    }

    __disable_irq();
    uint8_t  rx = g_uart_rx_char;
    g_uart_rx_ready = false;
    __enable_irq();

    switch (rx)
    {
        /* ---------- 选择关节 ---------- */
        case '0': case '1': case '2':
        case '3': case '4': case '5':
            g_teach_joint = (joint_id_t)(rx - (uint8_t)'0');
            printf("[TEACH] Joint = %s (%u deg)\r\n",
                   g_joint_names[g_teach_joint],
                   (unsigned int)g_teach_pose.angle_deg[g_teach_joint]);
            break;

        /* ---------- 步进大小 ---------- */
        case 'q': case 'Q':
            g_teach_step = 1U;
            printf("[TEACH] Step = 1 deg\r\n");
            break;
        case 'e': case 'E':
            g_teach_step = 5U;
            printf("[TEACH] Step = 5 deg\r\n");
            break;
        case 'r': case 'R':
            g_teach_step = 10U;
            printf("[TEACH] Step = 10 deg\r\n");
            break;

        /* ---------- 增大角度 ---------- */
        case 'w': case 'W':
        {
            uint16_t old_a = g_teach_pose.angle_deg[g_teach_joint];
            uint16_t max_a = g_servos[g_teach_joint].angle_range_deg;
            uint16_t new_a = (uint16_t)(old_a + g_teach_step);
            if (new_a > max_a) { new_a = max_a; }
            if (new_a != old_a)
            {
                fsp_err_t e = servo_set_angle(g_teach_joint, new_a);
                if (FSP_SUCCESS == e)
                {
                    g_teach_pose.angle_deg[g_teach_joint] = new_a;
                    printf("[TEACH] %s: %u -> %u\r\n",
                           g_joint_names[g_teach_joint],
                           (unsigned int)old_a, (unsigned int)new_a);
                }
                else
                {
                    printf("[ERR] servo failed: j=%u e=%d\r\n",
                           (unsigned int)g_teach_joint, (int)e);
                }
            }
            break;
        }

        /* ---------- 减小角度 ---------- */
        case 's': case 'S':
        {
            uint16_t old_a = g_teach_pose.angle_deg[g_teach_joint];
            uint16_t new_a = (old_a >= g_teach_step)
                           ? (uint16_t)(old_a - g_teach_step) : 0U;
            if (new_a != old_a)
            {
                fsp_err_t e = servo_set_angle(g_teach_joint, new_a);
                if (FSP_SUCCESS == e)
                {
                    g_teach_pose.angle_deg[g_teach_joint] = new_a;
                    printf("[TEACH] %s: %u -> %u\r\n",
                           g_joint_names[g_teach_joint],
                           (unsigned int)old_a, (unsigned int)new_a);
                }
                else
                {
                    printf("[ERR] servo failed: j=%u e=%d\r\n",
                           (unsigned int)g_teach_joint, (int)e);
                }
            }
            break;
        }

        /* ---------- 打印当前姿态 ---------- */
        case 'p': case 'P':
            printf("===== POSE SNAPSHOT =====\r\n");
            printf("{%u, %u, %u, %u, %u, %u}\r\n",
                   (unsigned int)g_teach_pose.angle_deg[JOINT_BASE],
                   (unsigned int)g_teach_pose.angle_deg[JOINT_SHOULDER],
                   (unsigned int)g_teach_pose.angle_deg[JOINT_ELBOW],
                   (unsigned int)g_teach_pose.angle_deg[JOINT_WRIST_PITCH],
                   (unsigned int)g_teach_pose.angle_deg[JOINT_WRIST_ROTATE],
                   (unsigned int)g_teach_pose.angle_deg[JOINT_GRIPPER]);
            printf("=========================\r\n");
            break;

        /* ---------- 回 HOME ---------- */
        case 'h': case 'H':
        {
            fsp_err_t e = robot_apply_pose(&g_home_pose);
            if (FSP_SUCCESS == e)
            {
                memcpy(&g_teach_pose, &g_home_pose, sizeof(g_teach_pose));
                printf("[TEACH] HOME applied\r\n");
            }
            break;
        }

        default:
            break;
    }
}

/* ------------------------------------------------------------------ */
/*  hal_entry                                                           */
/* ------------------------------------------------------------------ */
void hal_entry(void)
{
#if BSP_TZ_SECURE_BUILD
    R_BSP_NonSecureEnter();
#else
    fsp_err_t err;

    /* 1. UART 初始化 ------------------------------------------------ */
    err = R_SCI_UART_Open(&g_uart0_ctrl, &g_uart0_cfg);
    if (FSP_SUCCESS != err)
    {
        while (1) { ; }
    }
    uart_log("[BOOT] UART OK\r\n");

    /* 2. I2C 初始化 ------------------------------------------------- */
    err = R_IIC_MASTER_Open(&g_i2c_master1_ctrl, &g_i2c_master1_cfg);
    if (FSP_SUCCESS != err)
    {
        uart_log("[ERR] I2C Open failed\r\n");
        while (1) { ; }
    }
    uart_log("[BOOT] I2C OK\r\n");

    /* 3. PCA9685 初始化 --------------------------------------------- */
    err = pca9685_init();
    if (FSP_SUCCESS != err)
    {
        uart_log("[ERR] PCA9685 init failed\r\n");
        while (1) { ; }
    }
    uart_log("[BOOT] PCA9685 OK\r\n");

    /* 4. 执行 HOME 姿态 --------------------------------------------- */
    uart_log("[POSE] Applying HOME...\r\n");
    err = robot_apply_pose(&g_home_pose);
    if (FSP_SUCCESS != err)
    {
        uart_log("[ERR] HOME pose failed\r\n");
        while (1) { ; }
    }
    memcpy(&g_teach_pose, &g_home_pose, sizeof(g_teach_pose));
    uart_log("[POSE] HOME OK\r\n");

    /* 5. Teach 模式 ------------------------------------------------- */
    uart_log("[BOOT] System ready\r\n");
    uart_log("[TEACH] === Teach Mode ===\r\n");
    uart_log("[TEACH] 0-5 : Select joint\r\n");
    uart_log("[TEACH] W/S : Increase/Decrease angle\r\n");
    uart_log("[TEACH] Q   : Step=1   E: Step=5   R: Step=10\r\n");
    uart_log("[TEACH] P   : Print pose   H: Go HOME\r\n");
    printf("[TEACH] Joint=%s Step=%u\r\n",
           g_joint_names[g_teach_joint],
           (unsigned int)g_teach_step);

    /* 6. 主循环 ----------------------------------------------------- */
    while (1)
    {
        teach_mode();
    }
#endif
}
