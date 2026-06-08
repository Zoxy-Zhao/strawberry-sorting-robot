/* ============================================================
 *  hal_entry_pickup_v2.c — pickup_v2 副本（任意位置抓取功能）
 *  ----------------------------------------------------------
 *  基于原 hal_entry.c 派生，新增能力（不改动旧路径）：
 *    - 行缓冲 + \n 终结的扩展协议解析
 *    - 命令字 M / K / J / OPEN / CLOSE / HOME / PLACE
 *    - STATE_PICKUP_IDLE / STATE_PICKUP_MOVE 状态
 *    - 兼容旧单字符协议 A/B/C/G/X
 *
 *  原文件 D:\e2studio_test\Robotic_arm\src\hal_entry.c 不变。
 *  使用：先在 e2studio 编译验证；通过后用本文件覆盖原 hal_entry.c。
 *  详见 ../docs/MCU实现说明.md
 * ============================================================ */
#include "hal_data.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "tinyml_grasp.h"

/* ------------------------------------------------------------------ */
/*  PCA9685 constants                                                  */
/* ------------------------------------------------------------------ */
#define PCA9685_I2C_ADDR       0x40U
#define PCA9685_MODE1          0x00U
#define PCA9685_MODE2          0x01U
#define PCA9685_PRESCALE       0xFEU
#define PCA9685_LED0_ON_L      0x06U

/* ------------------------------------------------------------------ */
/*  Application constants                                              */
/* ------------------------------------------------------------------ */
#define SERVO_COUNT            6U
#define SERVO_MOVE_GAP_MS      80U
#define MAIN_LOOP_SLICE_MS     10U
#define INTERP_STEP_FAST       2U      /* 常规动作：每步2° */
#define INTERP_DELAY_FAST      15U     /* 常规动作：每步15ms（快而流畅） */
#define INTERP_STEP_SLOW       1U      /* 接近草莓：每步1° */
#define INTERP_DELAY_SLOW      30U     /* 接近草莓：每步30ms（慢而稳） */

#define BLOCKING_TIMEOUT       200000U   /* busy-wait timeout count for I2C/UART */

#define BELT_STOP_DELAY_MS     500U
#define PRE_GRASP_WAIT_MS      1000U
#define GRASP_WAIT_MS          800U
#define LIFT_WAIT_MS           800U
#define PLACE_WAIT_MS          1200U
#define RETURN_WAIT_MS         1000U

/* ── 压力传感器 (FSR) ── */
#define FSR_BASELINE_SAMPLES   10U     /* 采基线次数（取最大值） */
#define FSR_R_CONTACT_DELTA    400U    /* 右指：变化多少算"碰到了"（空载波动~300，要高于噪声） */
#define FSR_R_TARGET_DELTA     600U    /* 右指：变化多少算"夹住了" */
#define FSR_MAX_AFTER_CONTACT  6U      /* 右指接触后，最多再收几度（真接触1-2°就达Target提前停） */
#define GRIPPER_CLOSE_MIN_DEG  40U     /* 夹爪最小角度（兜底） */
#define GRIPPER_CLOSE_STEP_MS  30U     /* 每步闭合的等待时间 */

#ifndef S_IFCHR
 #define S_IFCHR               0020000
#endif

/* ------------------------------------------------------------------ */
/*  Joint enum                                                         */
/* ------------------------------------------------------------------ */
typedef enum e_joint_id
{
    JOINT_BASE = 0,          /* CH5 / 270deg */
    JOINT_SHOULDER = 1,      /* CH1 / 180deg */
    JOINT_ELBOW = 2,         /* CH0 / 180deg */
    JOINT_WRIST_PITCH = 3,   /* CH2 / 180deg */
    JOINT_WRIST_ROTATE = 4,  /* CH3 / 180deg */
    JOINT_GRIPPER = 5,       /* CH4 / 180deg / reversed */
} joint_id_t;

/* ------------------------------------------------------------------ */
/*  Servo parameter structure                                          */
/* ------------------------------------------------------------------ */
typedef struct st_servo_param
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

typedef struct st_robot_pose
{
    uint16_t angle_deg[SERVO_COUNT];
} robot_pose_t;

/* ------------------------------------------------------------------ */
/*  State machine                                                      */
/* ------------------------------------------------------------------ */
typedef enum e_state
{
    STATE_IDLE,
    STATE_BELT_RUN,
    STATE_BELT_STOP,
    STATE_PRE_GRASP,
    STATE_GRASP,
    STATE_LIFT,
    STATE_PLACE,
    STATE_RETURN,
    /* pickup_v2 新增 */
    STATE_PICKUP_IDLE,    /* Pi 主导抓取流程的待机状态 */
    STATE_PICKUP_MOVE     /* 正在执行 M/K/HOME/PLACE 命令的移动 */
} state_t;

/* ── pickup_v2: 扩展协议行缓冲 ── */
#define PI_CMD_BUF_SIZE 64

/* ------------------------------------------------------------------ */
/*  Servo table                                                        */
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
/*  Calibrated poses                                                   */
/*  Order: BASE, SHOULDER, ELBOW, WRIST_PITCH, WRIST_ROTATE, GRIPPER  */
/* ------------------------------------------------------------------ */
static const robot_pose_t g_pose_home =
{
    .angle_deg = {135U, 70U, 70U, 150U, 80U, 120U}
};

static const robot_pose_t g_pose_pre_grasp =
{
    .angle_deg = {126U, 117U, 86U, 135U, 80U, 120U}
};

static const robot_pose_t g_pose_grasp =
{
    .angle_deg = {126U, 130U, 91U, 135U, 80U, 65U}
};

static const robot_pose_t g_pose_lift =
{
    .angle_deg = {126U, 112U, 98U, 135U, 80U, 65U}
};

/* TRANSIT: arm raised high for safe base rotation (shoulder/elbow same as HOME) */
static const robot_pose_t g_pose_transit =
{
    .angle_deg = {126U, 70U, 70U, 135U, 80U, 65U}
};

static const robot_pose_t g_pose_place_a =
{
    .angle_deg = {176U, 132U, 83U, 135U, 80U, 120U}
};

static const robot_pose_t g_pose_place_b =
{
    .angle_deg = {201U, 132U, 83U, 135U, 80U, 120U}
};

static const robot_pose_t g_pose_place_c =
{
    .angle_deg = {223U, 132U, 83U, 135U, 80U, 120U}
};

/* ------------------------------------------------------------------ */
/*  Globals                                                            */
/* ------------------------------------------------------------------ */
static volatile bool g_i2c_tx_done = false;
static volatile bool g_i2c_error   = false;

static volatile bool g_uart7_tx_complete = false;
static volatile bool g_uart9_tx_complete = false;

/* pickup_v2 fix: 单字节暂存改成 ring buffer
 * 旧实现每次 ISR 覆盖单字节，Pi 一次发 5 字节(HOME\n) 时主循环只能取到最后一个。
 * 现在每个 UART 分配 64 字节环形缓冲。head 仅 ISR 改，tail 仅主循环改，
 * head/tail 单字节读写在 Cortex-M 上原子，无需关中断。 */
#define UART_RXBUF_SIZE 64U

static volatile uint8_t g_uart7_rxbuf[UART_RXBUF_SIZE];
static volatile uint8_t g_uart7_rx_head = 0U;
static volatile uint8_t g_uart7_rx_tail = 0U;

static volatile uint8_t g_uart9_rxbuf[UART_RXBUF_SIZE];
static volatile uint8_t g_uart9_rx_head = 0U;
static volatile uint8_t g_uart9_rx_tail = 0U;

/* pickup_v2: Pi 串口行缓冲（扩展协议） */
static char    g_pi_cmd_buf[PI_CMD_BUF_SIZE];
static uint8_t g_pi_cmd_len = 0U;

static volatile bool g_emergency_stop = false;

static volatile state_t g_state              = STATE_IDLE;
static bool         g_auto_mode             = false;
static bool         g_teach_mode            = false;
static bool         g_button_prev_pressed   = false;
static volatile char g_target_classification = '\0';
static robot_pose_t g_current_pose          = { .angle_deg = {135U, 70U, 70U, 150U, 80U, 120U} };
static robot_pose_t g_teach_pose            = { .angle_deg = {135U, 70U, 70U, 150U, 80U, 120U} };
static joint_id_t   g_teach_joint           = JOINT_BASE;
static uint16_t     g_teach_step            = 5U;

/* ── TinyML FSR 数据采集缓冲 ── */
static uint16_t     g_fsr_log[FSR_LOG_MAX];
static uint16_t     g_fsr_log_count         = 0U;

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
/*  Forward declarations                                               */
/* ------------------------------------------------------------------ */
static void service_background(void);
static bool delay_with_service(uint32_t delay_ms, bool allow_abort);
static void teach_mode_handle_char(uint8_t rx);
static fsp_err_t pca9685_write_reg(uint8_t reg, uint8_t value);
static bool fsr_read(uint16_t * p_left, uint16_t * p_right);
static bool fsr_grasp_with_feedback(void);

/* ------------------------------------------------------------------ */
/*  UART callbacks                                                     */
/* ------------------------------------------------------------------ */
void uart_callback(uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
        {
            g_uart7_tx_complete = true;
            break;
        }

        case UART_EVENT_RX_CHAR:
        {
            uint8_t next_head = (uint8_t) ((g_uart7_rx_head + 1U) % UART_RXBUF_SIZE);
            if (next_head != g_uart7_rx_tail) /* 满则丢弃 */
            {
                g_uart7_rxbuf[g_uart7_rx_head] = (uint8_t) p_args->data;
                g_uart7_rx_head = next_head;
            }
            break;
        }

        default:
        {
            break;
        }
    }
}

void uart9_callback(uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
        {
            g_uart9_tx_complete = true;
            break;
        }

        case UART_EVENT_RX_CHAR:
        {
            uint8_t next_head = (uint8_t) ((g_uart9_rx_head + 1U) % UART_RXBUF_SIZE);
            if (next_head != g_uart9_rx_tail) /* 满则丢弃 */
            {
                g_uart9_rxbuf[g_uart9_rx_head] = (uint8_t) p_args->data;
                g_uart9_rx_head = next_head;
            }
            break;
        }

        default:
        {
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  ADC callback                                                       */
/* ------------------------------------------------------------------ */
static volatile bool g_adc_scan_done = false;

void adc_callback(adc_callback_args_t * p_args)
{
    if ((NULL != p_args) && (ADC_EVENT_SCAN_COMPLETE == p_args->event))
    {
        g_adc_scan_done = true;
    }
}

/* ------------------------------------------------------------------ */
/*  I2C callback                                                       */
/* ------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------ */
/*  UART helpers                                                       */
/* ------------------------------------------------------------------ */
static void uart7_write_blocking(uint8_t const * p_data, uint32_t length)
{
    fsp_err_t err;
    uint32_t  timeout = 0U;

    if ((NULL == p_data) || (0U == length))
    {
        return;
    }

    g_uart7_tx_complete = false;
    err = R_SCI_UART_Write(&g_uart0_ctrl, p_data, length);
    if (FSP_SUCCESS != err)
    {
        return;
    }

    while (!g_uart7_tx_complete)
    {
        if (++timeout > BLOCKING_TIMEOUT)
        {
            return;
        }
    }
}

static void uart_log(char const * p_str)
{
    if (NULL == p_str)
    {
        return;
    }

    uart7_write_blocking((uint8_t const *) p_str, (uint32_t) strlen(p_str));
}

static void uart9_send_blocking(char const * p_str)
{
    fsp_err_t err;
    uint32_t  length;
    uint32_t  timeout = 0U;

    if (NULL == p_str)
    {
        return;
    }

    length = (uint32_t) strlen(p_str);
    if (0U == length)
    {
        return;
    }

    g_uart9_tx_complete = false;
    err = R_SCI_UART_Write(&g_uart9_ctrl, (uint8_t const *) p_str, length);
    if (FSP_SUCCESS != err)
    {
        return;
    }

    while (!g_uart9_tx_complete)
    {
        if (++timeout > BLOCKING_TIMEOUT)
        {
            return;
        }
    }
}

static bool uart7_take_char(uint8_t * p_char)
{
    if (g_uart7_rx_head == g_uart7_rx_tail)
    {
        return false;
    }
    *p_char = g_uart7_rxbuf[g_uart7_rx_tail];
    g_uart7_rx_tail = (uint8_t) ((g_uart7_rx_tail + 1U) % UART_RXBUF_SIZE);
    return true;
}

static bool uart9_take_char(uint8_t * p_char)
{
    if (g_uart9_rx_head == g_uart9_rx_tail)
    {
        return false;
    }
    *p_char = g_uart9_rxbuf[g_uart9_rx_tail];
    g_uart9_rx_tail = (uint8_t) ((g_uart9_rx_tail + 1U) % UART_RXBUF_SIZE);
    return true;
}

/* ------------------------------------------------------------------ */
/*  printf redirection and newlib stubs                                */
/* ------------------------------------------------------------------ */
int _write(int fd, char * pBuffer, int size)
{
    (void) fd;

    if ((NULL == pBuffer) || (size <= 0))
    {
        return size;
    }

    uart7_write_blocking((uint8_t const *) pBuffer, (uint32_t) size);
    return size;
}

int _close(int fd)
{
    (void) fd;
    return -1;
}

int _lseek(int fd, int offset, int whence)
{
    (void) fd;
    (void) offset;
    (void) whence;
    return -1;
}

int _read(int fd, char * pBuffer, int count)
{
    (void) fd;
    (void) pBuffer;
    (void) count;
    return -1;
}

int _fstat(int fd, struct stat * p_stat)
{
    (void) fd;

    if (NULL != p_stat)
    {
        p_stat->st_mode = S_IFCHR;
    }

    return 0;
}

int _isatty(int fd)
{
    (void) fd;
    return 1;
}

/* ------------------------------------------------------------------ */
/*  Utility helpers                                                    */
/* ------------------------------------------------------------------ */
static char normalize_upper(char ch)
{
    if ((ch >= 'a') && (ch <= 'z'))
    {
        return (char) (ch - ('a' - 'A'));
    }

    return ch;
}

static char const * state_name(state_t state)
{
    switch (state)
    {
        case STATE_IDLE:       return "IDLE";
        case STATE_BELT_RUN:   return "BELT_RUN";
        case STATE_BELT_STOP:  return "BELT_STOP";
        case STATE_PRE_GRASP:  return "PRE_GRASP";
        case STATE_GRASP:      return "GRASP";
        case STATE_LIFT:       return "LIFT";
        case STATE_PLACE:      return "PLACE";
        case STATE_RETURN:     return "RETURN";
        case STATE_PICKUP_IDLE: return "PICKUP_IDLE";
        case STATE_PICKUP_MOVE: return "PICKUP_MOVE";
        default:               return "UNKNOWN";
    }
}

static char const * classification_name(char cls)
{
    switch (normalize_upper(cls))
    {
        case 'A': return "ripe";
        case 'B': return "semi_ripe";
        case 'C': return "unripe";
        default:  return "unknown";
    }
}

static robot_pose_t const * classification_to_place_pose(char cls, char const ** pp_pose_name)
{
    switch (normalize_upper(cls))
    {
        case 'A':
        {
            if (NULL != pp_pose_name)
            {
                *pp_pose_name = "PLACE_A";
            }
            return &g_pose_place_a;
        }

        case 'B':
        {
            if (NULL != pp_pose_name)
            {
                *pp_pose_name = "PLACE_B";
            }
            return &g_pose_place_b;
        }

        case 'C':
        {
            if (NULL != pp_pose_name)
            {
                *pp_pose_name = "PLACE_C";
            }
            return &g_pose_place_c;
        }

        default:
        {
            if (NULL != pp_pose_name)
            {
                *pp_pose_name = "PLACE_UNKNOWN";
            }
            return NULL;
        }
    }
}

static void fatal_fsp_error(char const * p_context, fsp_err_t err)
{
    printf("[FATAL] %s failed: %d\r\n", p_context, (int) err);

    /* Safety: stop belt relay */
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_04_PIN_15, BSP_IO_LEVEL_LOW);

    /* Safety: put PCA9685 to sleep (disable all PWM outputs) */
    pca9685_write_reg(PCA9685_MODE1, 0x10U);

    while (1)
    {
        __WFI();
    }
}

static void state_set(state_t new_state)
{
    if (g_state != new_state)
    {
        printf("[STATE] %s -> %s\r\n", state_name(g_state), state_name(new_state));
        g_state = new_state;
    }
}

static void belt_set(bool on)
{
    fsp_err_t err;

    err = R_IOPORT_PinWrite(&g_ioport_ctrl,
                            BSP_IO_PORT_04_PIN_15,
                            on ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
    if (FSP_SUCCESS != err)
    {
        fatal_fsp_error("belt relay write", err);
    }
}

static bool board_button_is_pressed(void)
{
    bsp_io_level_t level = BSP_IO_LEVEL_HIGH;

#if defined(BSP_SW1)
    if (FSP_SUCCESS == R_IOPORT_PinRead(&g_ioport_ctrl, BSP_SW1, &level))
    {
        return (BSP_IO_LEVEL_LOW == level);
    }
#elif defined(BSP_IO_PORT_00_PIN_05)
    /* DShanMCU RA6M5 docs reference KEY on P005, active low. */
    if (FSP_SUCCESS == R_IOPORT_PinRead(&g_ioport_ctrl, BSP_IO_PORT_00_PIN_05, &level))
    {
        return (BSP_IO_LEVEL_LOW == level);
    }
#endif

    return false;
}

static bool start_button_edge_detected(void)
{
    bool pressed = board_button_is_pressed();
    bool edge    = (pressed && !g_button_prev_pressed);

    g_button_prev_pressed = pressed;
    return edge;
}

static void teach_mode_enter(void)
{
    g_teach_mode = true;
    memcpy(&g_teach_pose, &g_current_pose, sizeof(g_teach_pose));

    uart_log("[TEACH] === Teach Mode ===\r\n");
    uart_log("[TEACH] 0-5 : Select joint\r\n");
    uart_log("[TEACH] W/S : Increase/Decrease angle\r\n");
    uart_log("[TEACH] Q   : Step=1   E: Step=5   R: Step=10\r\n");
    uart_log("[TEACH] P   : Print pose   H: Go HOME   T: Exit\r\n");
    printf("[TEACH] Joint=%s Step=%u\r\n",
           g_joint_names[g_teach_joint],
           (unsigned int) g_teach_step);
}

static void teach_mode_exit(void)
{
    g_teach_mode = false;
    uart_log("[TEACH] Exit\r\n");
}

/* ------------------------------------------------------------------ */
/*  I2C helpers                                                        */
/* ------------------------------------------------------------------ */
static fsp_err_t i2c_write_bytes(uint8_t * p_data, uint32_t length)
{
    fsp_err_t err;
    uint32_t  timeout = 0U;

    g_i2c_tx_done = false;
    g_i2c_error   = false;

    err = R_IIC_MASTER_Write(&g_i2c_master1_ctrl, p_data, length, false);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    while (!g_i2c_tx_done)
    {
        if (++timeout > BLOCKING_TIMEOUT)
        {
            return FSP_ERR_TIMEOUT;
        }
    }

    return g_i2c_error ? FSP_ERR_ABORTED : FSP_SUCCESS;
}

/* ------------------------------------------------------------------ */
/*  PCA9685 driver                                                     */
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
    uint8_t buf[5];
    uint8_t reg = (uint8_t) (PCA9685_LED0_ON_L + (4U * channel));

    buf[0] = reg;
    buf[1] = (uint8_t) (on & 0xFFU);
    buf[2] = (uint8_t) ((on >> 8) & 0x0FU);
    buf[3] = (uint8_t) (off & 0xFFU);
    buf[4] = (uint8_t) ((off >> 8) & 0x0FU);

    return i2c_write_bytes(buf, 5U);
}

static fsp_err_t pca9685_init(void)
{
    fsp_err_t err;

    err = pca9685_write_reg(PCA9685_MODE1, 0x10U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = pca9685_write_reg(PCA9685_PRESCALE, 121U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = pca9685_write_reg(PCA9685_MODE2, 0x04U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = pca9685_write_reg(PCA9685_MODE1, 0x01U);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
    return pca9685_write_reg(PCA9685_MODE1, 0xA1U);
}

/* ------------------------------------------------------------------ */
/*  Servo driver                                                       */
/* ------------------------------------------------------------------ */
static uint16_t servo_us_to_counts(uint16_t pulse_us)
{
    uint32_t counts = ((uint32_t) pulse_us * 4096U) / 20000U;
    return (uint16_t) counts;
}

static fsp_err_t servo_set_pulse_us(uint8_t channel, uint16_t pulse_us)
{
    return pca9685_set_pwm(channel, 0U, servo_us_to_counts(pulse_us));
}

static uint16_t servo_angle_to_us(servo_param_t const * p_servo, uint16_t angle_deg)
{
    uint16_t max_angle = p_servo->angle_range_deg;
    uint32_t pulse;
    uint32_t span;

    if (angle_deg > max_angle)
    {
        angle_deg = max_angle;
    }

    span = (uint32_t) p_servo->right_safe_us - (uint32_t) p_servo->left_safe_us;

    if (p_servo->reversed)
    {
        pulse = (uint32_t) p_servo->right_safe_us - ((span * angle_deg) / max_angle);
    }
    else
    {
        pulse = (uint32_t) p_servo->left_safe_us + ((span * angle_deg) / max_angle);
    }

    return (uint16_t) pulse;
}

static fsp_err_t servo_set_angle(joint_id_t joint, uint16_t angle_deg)
{
    servo_param_t const * p_servo = &g_servos[joint];
    return servo_set_pulse_us(p_servo->channel, servo_angle_to_us(p_servo, angle_deg));
}

static fsp_err_t robot_apply_pose_ex(robot_pose_t const * p_pose,
                                     bool allow_abort,
                                     uint32_t step_deg,
                                     uint32_t delay_ms)
{
    fsp_err_t err;
    uint32_t  i;

    /* Calculate max delta across all joints */
    uint32_t max_delta = 0U;
    for (i = 0U; i < SERVO_COUNT; i++)
    {
        uint16_t cur = g_current_pose.angle_deg[i];
        uint16_t tgt = p_pose->angle_deg[i];
        uint32_t delta = (cur > tgt) ? (uint32_t)(cur - tgt) : (uint32_t)(tgt - cur);
        if (delta > max_delta)
        {
            max_delta = delta;
        }
    }

    /* If no movement needed, return immediately */
    if (0U == max_delta)
    {
        return FSP_SUCCESS;
    }

    /* Calculate number of interpolation steps */
    uint32_t num_steps = (max_delta + step_deg - 1U) / step_deg;
    if (num_steps < 1U)
    {
        num_steps = 1U;
    }

    /* Save start angles */
    int32_t start[SERVO_COUNT];
    int32_t target[SERVO_COUNT];
    for (i = 0U; i < SERVO_COUNT; i++)
    {
        start[i]  = (int32_t) g_current_pose.angle_deg[i];
        target[i] = (int32_t) p_pose->angle_deg[i];
    }

    /* Interpolate step by step */
    for (uint32_t step = 1U; step <= num_steps; step++)
    {
        if (allow_abort && g_emergency_stop)
        {
            return FSP_ERR_ABORTED;
        }

        for (i = 0U; i < SERVO_COUNT; i++)
        {
            int32_t interp;
            if (step == num_steps)
            {
                interp = target[i];  /* final step: exact target */
            }
            else
            {
                interp = start[i] + ((target[i] - start[i]) * (int32_t) step) / (int32_t) num_steps;
            }

            err = servo_set_angle((joint_id_t) i, (uint16_t) interp);
            if (FSP_SUCCESS != err)
            {
                return err;
            }
            g_current_pose.angle_deg[i] = (uint16_t) interp;
        }

        service_background();
        R_BSP_SoftwareDelay(delay_ms, BSP_DELAY_UNITS_MILLISECONDS);
    }

    return FSP_SUCCESS;
}

/* 常规速度（快） */
static fsp_err_t robot_apply_pose(robot_pose_t const * p_pose)
{
    return robot_apply_pose_ex(p_pose, true, INTERP_STEP_FAST, INTERP_DELAY_FAST);
}

/* 强制执行，不可中断（快） */
static fsp_err_t robot_apply_pose_force(robot_pose_t const * p_pose)
{
    return robot_apply_pose_ex(p_pose, false, INTERP_STEP_FAST, INTERP_DELAY_FAST);
}

/* 慢速接近（防撞） */
static fsp_err_t robot_apply_pose_slow(robot_pose_t const * p_pose)
{
    return robot_apply_pose_ex(p_pose, true, INTERP_STEP_SLOW, INTERP_DELAY_SLOW);
}

/* ------------------------------------------------------------------ */
/*  FSR pressure sensor                                                */
/* ------------------------------------------------------------------ */
static bool fsr_read(uint16_t * p_left, uint16_t * p_right)
{
    g_adc_scan_done = false;
    fsp_err_t err = R_ADC_ScanStart(&g_adc0_ctrl);
    if (FSP_SUCCESS != err)
    {
        return false;
    }

    uint32_t timeout = 0U;
    while (!g_adc_scan_done)
    {
        if (++timeout > 100000U)
        {
            return false;
        }
    }

    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_1, p_left);
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_3, p_right);
    return true;
}

/**
 * 压力反馈夹取（右指单传感器模式）：
 *   左指传感器动态范围不足，仅依赖右指做判断。
 *   阶段1: 采右指基线（取最大值）
 *   阶段2: 自由收缩，等右指 delta > CONTACT → 开始倒数
 *   阶段3: 倒数 MAX_AFTER_CONTACT 度内，若右指 delta > TARGET → 提前停
 *   兜底:  倒数到 0 或到最小角度 → 停止
 */
static bool fsr_grasp_with_feedback(void)
{
    uint16_t angle = g_current_pose.angle_deg[JOINT_GRIPPER];

    /* ── TinyML: 重置数据采集缓冲 ── */
    g_fsr_log_count = 0U;

    /* ── 阶段1：采右指基线（取最大值，覆盖空载波动上限） ── */
    uint16_t baseline_r = 0U;
    for (uint16_t i = 0U; i < FSR_BASELINE_SAMPLES; i++)
    {
        uint16_t raw_l = 0U;
        uint16_t raw_r = 0U;
        fsr_read(&raw_l, &raw_r);
        if (raw_r > baseline_r) { baseline_r = raw_r; }
        R_BSP_SoftwareDelay(5U, BSP_DELAY_UNITS_MILLISECONDS);
    }

    printf("[FSR] Baseline R=%u, start %u°\r\n",
           (unsigned) baseline_r, (unsigned) angle);

    /* ── 阶段2+3：收缩循环 ── */
    bool contacted = false;
    uint16_t after_count = 0U;

    while (angle > GRIPPER_CLOSE_MIN_DEG)
    {
        if (g_emergency_stop)
        {
            return false;
        }

        /* 收紧 1° */
        angle--;
        fsp_err_t err = servo_set_angle(JOINT_GRIPPER, angle);
        if (FSP_SUCCESS != err)
        {
            fatal_fsp_error("FSR gripper", err);
            return false;
        }
        g_current_pose.angle_deg[JOINT_GRIPPER] = angle;

        R_BSP_SoftwareDelay(GRIPPER_CLOSE_STEP_MS, BSP_DELAY_UNITS_MILLISECONDS);
        service_background();

        /* 读右指压力 */
        uint16_t cur_l = 4095U;
        uint16_t cur_r = 4095U;
        fsr_read(&cur_l, &cur_r);

        uint16_t delta_r = (cur_r < baseline_r) ? (baseline_r - cur_r) : 0U;

        /* TinyML: 记录 delta 到采集缓冲 */
        if (g_fsr_log_count < FSR_LOG_MAX)
        {
            g_fsr_log[g_fsr_log_count++] = delta_r;
        }

        /* 检测右指接触 */
        if (!contacted && delta_r > FSR_R_CONTACT_DELTA)
        {
            contacted = true;
            printf("[FSR] Contact at %u°\r\n", (unsigned) angle);
        }

        if (contacted)
        {
            after_count++;

            /* 达到目标压力 → 立刻停 */
            if (delta_r > FSR_R_TARGET_DELTA)
            {
                printf("[FSR] Grip at %u°\r\n", (unsigned) angle);
                return true;
            }

            /* 安全上限 → 强制停 */
            if (after_count >= FSR_MAX_AFTER_CONTACT)
            {
                printf("[FSR] Grip at %u° (max)\r\n", (unsigned) angle);
                return true;
            }
        }
    }

    printf("[FSR] No contact\r\n");
    return false;
}

/* ------------------------------------------------------------------ */
/*  Command processing                                                 */
/* ------------------------------------------------------------------ */

/* pickup_v2: 旧单字符协议路径（A/B/C/G/X），保留兼容 */
static void process_pi_char_legacy(uint8_t rx)
{
    char cls = normalize_upper((char) rx);

    switch (cls)
    {
        case 'A':
        case 'B':
        case 'C':
        {
            printf("[PI] Classification: %c (%s)\r\n", cls, classification_name(cls));
            if (STATE_BELT_RUN == g_state)
            {
                g_target_classification = cls;
                state_set(STATE_BELT_STOP);
            }
            else
            {
                printf("[PI] Ignored classification in state %s\r\n", state_name(g_state));
            }
            break;
        }
        case 'G':
        {
            if (STATE_IDLE == g_state)
            {
                g_target_classification = '\0';
                printf("[PI] Belt start command received\r\n");
                state_set(STATE_BELT_RUN);
                uart9_send_blocking("BELT_ON\n");
            }
            else
            {
                printf("[PI] Belt start ignored, state=%s\r\n", state_name(g_state));
                uart9_send_blocking("BUSY\n");
            }
            break;
        }
        case 'X':
        {
            g_emergency_stop = true;
            printf("[PI] Emergency stop from Pi\r\n");
            uart9_send_blocking("STOPPED\n");
            break;
        }
        default:
            break;
    }
}

/* pickup_v2: 手写 token 提取器，避开 newlib strtok（实测在本工具链下挂死） */
static char * pv2_next_token(char ** p_save)
{
    if (NULL == p_save) return NULL;
    char * s = *p_save;
    if (NULL == s) return NULL;

    while ((' ' == *s) || ('\t' == *s)) s++;
    if ('\0' == *s) { *p_save = s; return NULL; }

    char * tok = s;
    while (('\0' != *s) && (' ' != *s) && ('\t' != *s)) s++;
    if ('\0' != *s) { *s = '\0'; s++; }
    *p_save = s;
    return tok;
}

/* pickup_v2: K 命令 — 直接发 6 个关节角（Pi 已算好 IK） */
static void cmd_K_handler(char ** p_save)
{
    uint16_t angles[SERVO_COUNT];
    for (int i = 0; i < (int) SERVO_COUNT; i++)
    {
        char * t = pv2_next_token(p_save);
        if (NULL == t) { uart9_send_blocking("NACK BADARG\n"); return; }

        char * endp = t;
        float a = strtof(t, &endp);
        if (endp == t) { uart9_send_blocking("NACK BADARG\n"); return; }

        uint16_t max_deg = g_servos[i].angle_range_deg;
        if (a < 0.0f || a > (float) max_deg)
        {
            uart9_send_blocking("NACK SAFETY\n");
            return;
        }
        angles[i] = (uint16_t) a;
    }

    if (STATE_IDLE != g_state && STATE_PICKUP_IDLE != g_state)
    {
        uart9_send_blocking("BUSY\n"); return;
    }

    state_set(STATE_PICKUP_MOVE);
    robot_pose_t target;
    for (int i = 0; i < (int) SERVO_COUNT; i++) target.angle_deg[i] = angles[i];
    fsp_err_t err = robot_apply_pose_slow(&target);
    state_set(STATE_PICKUP_IDLE);
    if (FSP_SUCCESS != err) { uart9_send_blocking("NACK SAFETY\n"); return; }
    g_current_pose = target;
    uart9_send_blocking("READY\n");
}

/* pickup_v2: M 命令 — 工作面坐标。首版 Pi 端已转 IK 后发 K，此路径直接拒绝 */
static void cmd_M_handler(char ** p_save)
{
    (void) p_save;
    uart9_send_blocking("NACK BADARG\n");
}

/* pickup_v2: J 命令 — 单关节调试 "J ch angle" */
static void cmd_J_handler(char ** p_save)
{
    char * t1 = pv2_next_token(p_save);
    char * t2 = pv2_next_token(p_save);
    if (NULL == t1 || NULL == t2) { uart9_send_blocking("NACK BADARG\n"); return; }

    char * endp = t2;
    float angle = strtof(t2, &endp);
    if (endp == t2) { uart9_send_blocking("NACK BADARG\n"); return; }

    int ch = atoi(t1);
    if (ch < 0 || ch >= (int) SERVO_COUNT) { uart9_send_blocking("NACK BADARG\n"); return; }

    uint16_t max_deg = g_servos[ch].angle_range_deg;
    if (angle < 0.0f || angle > (float) max_deg)
    {
        uart9_send_blocking("NACK SAFETY\n"); return;
    }

    fsp_err_t err = servo_set_angle((joint_id_t) ch, (uint16_t) angle);
    if (FSP_SUCCESS != err) { uart9_send_blocking("NACK SAFETY\n"); return; }
    g_current_pose.angle_deg[ch] = (uint16_t) angle;
    uart9_send_blocking("READY\n");
}

/* pickup_v2: 夹爪 / 回零 / 放置
 * 注意：失败分支（FSP_SUCCESS != err）不写回 g_current_pose，
 * 否则会与机械臂实际位置失同步。 */
static void cmd_OPEN_handler(void)
{
    fsp_err_t err = servo_set_angle(JOINT_GRIPPER, 120U);
    if (FSP_SUCCESS != err) { uart9_send_blocking("NACK SAFETY\n"); return; }
    g_current_pose.angle_deg[JOINT_GRIPPER] = 120U;
    uart9_send_blocking("READY\n");
}

static void cmd_CLOSE_handler(void)
{
    bool ok = fsr_grasp_with_feedback();
    uart9_send_blocking(ok ? "READY\n" : "NACK SAFETY\n");
}

static void cmd_HOME_handler(void)
{
    /* 仅允许 IDLE / PICKUP_IDLE，避免打断 RETURN / 自动流程 */
    if (STATE_IDLE != g_state && STATE_PICKUP_IDLE != g_state)
    {
        uart9_send_blocking("BUSY\n"); return;
    }
    state_set(STATE_PICKUP_MOVE);
    fsp_err_t err = robot_apply_pose_slow(&g_pose_home);
    state_set(STATE_PICKUP_IDLE);
    if (FSP_SUCCESS != err) { uart9_send_blocking("NACK SAFETY\n"); return; }
    g_current_pose = g_pose_home;
    uart9_send_blocking("READY\n");
}

static void cmd_PLACE_handler(char ** p_save)
{
    char * t = pv2_next_token(p_save);
    if (NULL == t) { uart9_send_blocking("NACK BADARG\n"); return; }
    char cls = normalize_upper(t[0]);
    char const * pose_name = NULL;
    robot_pose_t const * p_pose = classification_to_place_pose(cls, &pose_name);
    if (NULL == p_pose) { uart9_send_blocking("NACK BADARG\n"); return; }

    if (STATE_IDLE != g_state && STATE_PICKUP_IDLE != g_state)
    {
        uart9_send_blocking("BUSY\n"); return;
    }

    state_set(STATE_PICKUP_MOVE);
    fsp_err_t err = robot_apply_pose_slow(p_pose);
    state_set(STATE_PICKUP_IDLE);
    if (FSP_SUCCESS != err) { uart9_send_blocking("NACK SAFETY\n"); return; }
    g_current_pose = *p_pose;
    uart9_send_blocking("READY\n");
}

/* pickup_v2: 扩展命令分发器 */
static void parse_extended_cmd(char * line)
{
    char * save = line;
    char * tok = pv2_next_token(&save);
    if (NULL == tok) { uart9_send_blocking("NACK BADARG\n"); return; }

    if      (0 == strcmp(tok, "M"))     cmd_M_handler(&save);
    else if (0 == strcmp(tok, "K"))     cmd_K_handler(&save);
    else if (0 == strcmp(tok, "J"))     cmd_J_handler(&save);
    else if (0 == strcmp(tok, "OPEN"))  cmd_OPEN_handler();
    else if (0 == strcmp(tok, "CLOSE")) cmd_CLOSE_handler();
    else if (0 == strcmp(tok, "HOME"))  cmd_HOME_handler();
    else if (0 == strcmp(tok, "PLACE")) cmd_PLACE_handler(&save);
    else                                uart9_send_blocking("NACK BADARG\n");
}

/* pickup_v2: 改造后的 process_pi_char — 行缓冲 + 单字符兼容 */
static bool g_pi_drop_until_nl = false;   /* 缓冲溢出后丢弃残行直到下一个 \n */

static void process_pi_char(uint8_t rx)
{
    /* 溢出恢复：丢弃直到下一个换行，避免把行尾片段当新命令 */
    if (g_pi_drop_until_nl)
    {
        if ('\n' == rx) { g_pi_drop_until_nl = false; g_pi_cmd_len = 0U; }
        return;
    }

    /* 旧协议兼容：行缓冲为空时收到单字符命令才走 legacy。
     * 关键修复（2026-05-15）：legacy 仅在该字符当前状态下有意义时才截胡，
     * 否则会跟扩展命令字头冲突（最典型：'C' 吞掉 "CLOSE" 开头）。
     *   - X 任意状态都急停 → 永远 legacy
     *   - G 仅 IDLE 启动传送带 → 仅 IDLE 走 legacy
     *   - A/B/C 仅 BELT_RUN 设分类 → 仅 BELT_RUN 走 legacy
     * 其他情况让字符进入行缓冲，由 \n 触发 parse_extended_cmd 解析。 */
    if (0U == g_pi_cmd_len)
    {
        char c = normalize_upper((char) rx);
        bool is_legacy = false;
        if ('X' == c) {
            is_legacy = true;
        } else if (('G' == c) && (STATE_IDLE == g_state)) {
            is_legacy = true;
        } else if ((('A' == c) || ('B' == c) || ('C' == c)) && (STATE_BELT_RUN == g_state)) {
            is_legacy = true;
        }
        if (is_legacy)
        {
            process_pi_char_legacy(rx);
            return;
        }
    }

    if ('\r' == rx) return;

    if ('\n' == rx)
    {
        if (g_pi_cmd_len > 0U)
        {
            g_pi_cmd_buf[g_pi_cmd_len] = '\0';
            parse_extended_cmd(g_pi_cmd_buf);
            g_pi_cmd_len = 0U;
        }
        return;
    }

    if (g_pi_cmd_len < (uint8_t) (PI_CMD_BUF_SIZE - 1U))
    {
        g_pi_cmd_buf[g_pi_cmd_len++] = (char) rx;
    }
    else
    {
        /* 溢出：标记丢弃残行，等下一个 \n 重置 */
        g_pi_cmd_len = 0U;
        g_pi_drop_until_nl = true;
        uart9_send_blocking("NACK BADARG\n");
    }
}

static void process_debug_char(uint8_t rx)
{
    switch (rx)
    {
        case 'x':
        case 'X':
        {
            g_emergency_stop = true;
            uart_log("[CTRL] Emergency stop requested\r\n");
            break;
        }

        case 'a':
        case 'A':
        {
            g_auto_mode = !g_auto_mode;
            printf("[CTRL] Auto mode: %s\r\n", g_auto_mode ? "ON" : "OFF");
            break;
        }

        case 'g':
        case 'G':
        {
            if (g_teach_mode)
            {
                teach_mode_exit();
            }

            if (STATE_IDLE == g_state)
            {
                g_target_classification = '\0';
                uart_log("[CTRL] Start command received\r\n");
                state_set(STATE_BELT_RUN);
            }
            else
            {
                printf("[CTRL] Start ignored, state=%s\r\n", state_name(g_state));
            }
            break;
        }

        case 't':
        case 'T':
        {
            if (g_teach_mode)
            {
                teach_mode_exit();
            }
            else if (STATE_IDLE == g_state)
            {
                teach_mode_enter();
            }
            else
            {
                printf("[TEACH] Busy, state=%s\r\n", state_name(g_state));
            }
            break;
        }

        default:
        {
            if (g_teach_mode)
            {
                teach_mode_handle_char(rx);
            }
            break;
        }
    }
}

static void service_debug_uart(void)
{
    uint8_t rx;

    while (uart7_take_char(&rx))
    {
        process_debug_char(rx);
    }
}

static void service_pi_uart(void)
{
    uint8_t rx;

    /* pickup_v2 fix: 一次消化所有积压字节，否则 10ms 周期下 HOME\n 需 50ms */
    while (uart9_take_char(&rx))
    {
        process_pi_char(rx);
    }
}

static void service_background(void)
{
    service_debug_uart();
    service_pi_uart();
}

/* ------------------------------------------------------------------ */
/*  Delay / motion helpers                                             */
/* ------------------------------------------------------------------ */
static bool delay_with_service(uint32_t delay_ms, bool allow_abort)
{
    uint32_t remaining = delay_ms;

    while (remaining > 0U)
    {
        uint32_t slice = (remaining > MAIN_LOOP_SLICE_MS) ? MAIN_LOOP_SLICE_MS : remaining;

        service_background();

        if (allow_abort && g_emergency_stop)
        {
            return false;
        }

        R_BSP_SoftwareDelay(slice, BSP_DELAY_UNITS_MILLISECONDS);
        remaining -= slice;
    }

    service_background();

    if (allow_abort && g_emergency_stop)
    {
        return false;
    }

    return true;
}

static bool apply_pose_and_wait(char const * p_pose_name, robot_pose_t const * p_pose, uint32_t wait_ms)
{
    fsp_err_t err;

    printf("[POSE] Applying %s\r\n", p_pose_name);
    err = robot_apply_pose(p_pose);

    if (FSP_ERR_ABORTED == err)
    {
        return false;
    }

    if (FSP_SUCCESS != err)
    {
        fatal_fsp_error(p_pose_name, err);
    }

    return delay_with_service(wait_ms, true);
}

static void emergency_stop_execute(void)
{
    fsp_err_t err;

    g_emergency_stop        = false;
    g_teach_mode            = false;
    g_target_classification = '\0';

    belt_set(false);
    uart_log("[CTRL] Emergency stop executing: HOME + IDLE\r\n");

    err = robot_apply_pose_force(&g_pose_home);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ABORTED != err))
    {
        fatal_fsp_error("emergency HOME", err);
    }

    memcpy(&g_current_pose, &g_pose_home, sizeof(g_current_pose));
    state_set(STATE_IDLE);
}

/* ------------------------------------------------------------------ */
/*  Teach mode                                                         */
/* ------------------------------------------------------------------ */
static void teach_mode_handle_char(uint8_t rx)
{
    switch (rx)
    {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        {
            g_teach_joint = (joint_id_t) (rx - (uint8_t) '0');
            printf("[TEACH] Joint = %s (%u deg)\r\n",
                   g_joint_names[g_teach_joint],
                   (unsigned int) g_teach_pose.angle_deg[g_teach_joint]);
            break;
        }

        case 'q':
        case 'Q':
        {
            g_teach_step = 1U;
            printf("[TEACH] Step = 1 deg\r\n");
            break;
        }

        case 'e':
        case 'E':
        {
            g_teach_step = 5U;
            printf("[TEACH] Step = 5 deg\r\n");
            break;
        }

        case 'r':
        case 'R':
        {
            g_teach_step = 10U;
            printf("[TEACH] Step = 10 deg\r\n");
            break;
        }

        case 'w':
        case 'W':
        {
            uint16_t old_angle = g_teach_pose.angle_deg[g_teach_joint];
            uint16_t max_angle = g_servos[g_teach_joint].angle_range_deg;
            uint16_t new_angle = (uint16_t) (old_angle + g_teach_step);
            fsp_err_t err;

            if (new_angle > max_angle)
            {
                new_angle = max_angle;
            }

            if (new_angle == old_angle)
            {
                break;
            }

            err = servo_set_angle(g_teach_joint, new_angle);
            if (FSP_SUCCESS != err)
            {
                fatal_fsp_error("teach increment", err);
            }

            g_teach_pose.angle_deg[g_teach_joint] = new_angle;
            g_current_pose.angle_deg[g_teach_joint] = new_angle;

            printf("[TEACH] %s: %u -> %u\r\n",
                   g_joint_names[g_teach_joint],
                   (unsigned int) old_angle,
                   (unsigned int) new_angle);
            break;
        }

        case 's':
        case 'S':
        {
            uint16_t old_angle = g_teach_pose.angle_deg[g_teach_joint];
            uint16_t new_angle = (old_angle >= g_teach_step) ? (uint16_t) (old_angle - g_teach_step) : 0U;
            fsp_err_t err;

            if (new_angle == old_angle)
            {
                break;
            }

            err = servo_set_angle(g_teach_joint, new_angle);
            if (FSP_SUCCESS != err)
            {
                fatal_fsp_error("teach decrement", err);
            }

            g_teach_pose.angle_deg[g_teach_joint] = new_angle;
            g_current_pose.angle_deg[g_teach_joint] = new_angle;

            printf("[TEACH] %s: %u -> %u\r\n",
                   g_joint_names[g_teach_joint],
                   (unsigned int) old_angle,
                   (unsigned int) new_angle);
            break;
        }

        case 'p':
        case 'P':
        {
            printf("===== POSE SNAPSHOT =====\r\n");
            printf("{%u, %u, %u, %u, %u, %u}\r\n",
                   (unsigned int) g_teach_pose.angle_deg[JOINT_BASE],
                   (unsigned int) g_teach_pose.angle_deg[JOINT_SHOULDER],
                   (unsigned int) g_teach_pose.angle_deg[JOINT_ELBOW],
                   (unsigned int) g_teach_pose.angle_deg[JOINT_WRIST_PITCH],
                   (unsigned int) g_teach_pose.angle_deg[JOINT_WRIST_ROTATE],
                   (unsigned int) g_teach_pose.angle_deg[JOINT_GRIPPER]);
            printf("=========================\r\n");
            break;
        }

        case 'h':
        case 'H':
        {
            fsp_err_t err = robot_apply_pose_force(&g_pose_home);
            if ((FSP_SUCCESS != err) && (FSP_ERR_ABORTED != err))
            {
                fatal_fsp_error("teach HOME", err);
            }

            memcpy(&g_current_pose, &g_pose_home, sizeof(g_current_pose));
            memcpy(&g_teach_pose, &g_pose_home, sizeof(g_teach_pose));
            uart_log("[TEACH] HOME applied\r\n");
            break;
        }

        default:
        {
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  hal_entry                                                          */
/* ------------------------------------------------------------------ */
void hal_entry(void)
{
#if BSP_TZ_SECURE_BUILD
    R_BSP_NonSecureEnter();
#else
    fsp_err_t err;

    err = R_SCI_UART_Open(&g_uart0_ctrl, &g_uart0_cfg);
    if (FSP_SUCCESS != err)
    {
        while (1)
        {
            ;
        }
    }

    err = R_SCI_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("[ERR] SCI9 open failed: %d\r\n", (int) err);
        while (1)
        {
            ;
        }
    }

    err = R_IIC_MASTER_Open(&g_i2c_master1_ctrl, &g_i2c_master1_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("[ERR] IIC1 open failed: %d\r\n", (int) err);
        while (1)
        {
            ;
        }
    }

    err = R_IIC_MASTER_SlaveAddressSet(&g_i2c_master1_ctrl, PCA9685_I2C_ADDR, I2C_MASTER_ADDR_MODE_7BIT);
    if (FSP_SUCCESS != err)
    {
        printf("[ERR] IIC1 slave address set failed: %d\r\n", (int) err);
        while (1)
        {
            ;
        }
    }

    err = pca9685_init();
    if (FSP_SUCCESS != err)
    {
        printf("[ERR] PCA9685 init failed: %d\r\n", (int) err);
        while (1)
        {
            ;
        }
    }

    belt_set(false);

    /* ── ADC0 初始化 (压力传感器) ── */
    err = R_ADC_Open(&g_adc0_ctrl, &g_adc0_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("[ERR] ADC0 open failed: %d\r\n", (int) err);
    }
    else
    {
        err = R_ADC_ScanCfg(&g_adc0_ctrl, &g_adc0_channel_cfg);
        if (FSP_SUCCESS != err)
        {
            printf("[ERR] ADC0 ScanCfg failed: %d\r\n", (int) err);
        }
    }

    /* 开机强制给每个舵机发一次 PWM，确保舵机初始化到 HOME 位置 */
    for (uint32_t j = 0U; j < SERVO_COUNT; j++)
    {
        err = servo_set_angle((joint_id_t) j, g_pose_home.angle_deg[j]);
        if (FSP_SUCCESS != err)
        {
            printf("[ERR] HOME servo %u failed: %d\r\n", (unsigned) j, (int) err);
        }
    }
    R_BSP_SoftwareDelay(500U, BSP_DELAY_UNITS_MILLISECONDS);

    memcpy(&g_current_pose, &g_pose_home, sizeof(g_current_pose));
    memcpy(&g_teach_pose, &g_pose_home, sizeof(g_teach_pose));
    g_button_prev_pressed = board_button_is_pressed();

    uart_log("[BOOT] SCI7 debug OK\r\n");
    uart_log("[BOOT] SCI9 Pi UART OK\r\n");
    uart_log("[BOOT] IIC1 + PCA9685 OK\r\n");
    uart_log("[BOOT] *** PICKUP_V2 STAGE-D BUILD 2026-05-15 ***\r\n");
    uart_log("[BOOT] ADC0 FSR OK\r\n");
    uart_log("[BOOT] HOME applied\r\n");
    uart_log("[BOOT] Commands: g=start, t=teach, a=auto, x=stop\r\n");
    uart_log("[BOOT] Waiting in IDLE\r\n");

    while (1)
    {
        if (g_emergency_stop)
        {
            emergency_stop_execute();
            continue;
        }

        service_debug_uart();

        if (g_emergency_stop)
        {
            emergency_stop_execute();
            continue;
        }

        if (g_teach_mode)
        {
            R_BSP_SoftwareDelay(MAIN_LOOP_SLICE_MS, BSP_DELAY_UNITS_MILLISECONDS);
            continue;
        }

        switch (g_state)
        {
            case STATE_IDLE:
            {
                service_pi_uart();

                if (start_button_edge_detected())
                {
                    uart_log("[CTRL] Start button pressed\r\n");
                    g_target_classification = '\0';
                    state_set(STATE_BELT_RUN);
                }
                else
                {
                    R_BSP_SoftwareDelay(MAIN_LOOP_SLICE_MS, BSP_DELAY_UNITS_MILLISECONDS);
                }
                break;
            }

            case STATE_BELT_RUN:
            {
                belt_set(true);

                while ((STATE_BELT_RUN == g_state) && !g_emergency_stop)
                {
                    service_background();
                    if (STATE_BELT_RUN == g_state)
                    {
                        R_BSP_SoftwareDelay(MAIN_LOOP_SLICE_MS, BSP_DELAY_UNITS_MILLISECONDS);
                    }
                }
                break;
            }

            case STATE_BELT_STOP:
            {
                belt_set(false);

                if (delay_with_service(BELT_STOP_DELAY_MS, true))
                {
                    state_set(STATE_PRE_GRASP);
                }
                break;
            }

            case STATE_PRE_GRASP:
            {
                if (apply_pose_and_wait("PRE_GRASP", &g_pose_pre_grasp, PRE_GRASP_WAIT_MS))
                {
                    state_set(STATE_GRASP);
                }
                break;
            }

            case STATE_GRASP:
            {
                /* Step 1: Move arm to GRASP position slowly (prevent collision) */
                {
                    robot_pose_t grasp_open = g_pose_grasp;
                    grasp_open.angle_deg[JOINT_GRIPPER] = g_current_pose.angle_deg[JOINT_GRIPPER];
                    fsp_err_t err_move = robot_apply_pose_slow(&grasp_open);
                    if (FSP_SUCCESS != err_move)
                    {
                        if (!g_emergency_stop)
                        {
                            fatal_fsp_error("GRASP move", err_move);
                        }
                        break;
                    }
                }

                /* Step 2: Close gripper with pressure feedback */
                uart_log("[MOVE] Closing gripper (pressure feedback)\r\n");
                fsr_grasp_with_feedback();

                /* Step 3: TinyML 端侧推理 — 分析夹取力曲线 */
                if (g_fsr_log_count > 0U)
                {
                    float ml_input[TINYML_INPUT_LEN];
                    tinyml_prepare_input(g_fsr_log, g_fsr_log_count, ml_input);

                    float conf = 0.0f;
                    grasp_class_t ml_result = tinyml_classify(ml_input, &conf);

                    int conf_int = (int)(conf * 100.0f);
                    printf("[TinyML] Grasp: %s (logit=%d.%02d, samples=%u)\r\n",
                           g_grasp_class_names[ml_result],
                           conf_int / 100,
                           (conf_int < 0 ? -conf_int : conf_int) % 100,
                           (unsigned) g_fsr_log_count);
                }

                /* 无论是否检测到压力，都等一下让夹持稳定 */
                if (delay_with_service(GRASP_WAIT_MS, true))
                {
                    state_set(STATE_LIFT);
                }
                break;
            }

            case STATE_LIFT:
            {
                /* 保持夹爪角度不变，只移动手臂 */
                robot_pose_t lift_keep = g_pose_lift;
                lift_keep.angle_deg[JOINT_GRIPPER] = g_current_pose.angle_deg[JOINT_GRIPPER];
                if (apply_pose_and_wait("LIFT", &lift_keep, LIFT_WAIT_MS))
                {
                    state_set(STATE_PLACE);
                }
                break;
            }

            case STATE_PLACE:
            {
                char const *       p_pose_name  = NULL;
                robot_pose_t const * p_place_pose = classification_to_place_pose(g_target_classification, &p_pose_name);

                if (NULL == p_place_pose)
                {
                    printf("[ERR] No valid classification latched\r\n");
                    state_set(STATE_RETURN);
                    break;
                }

                /* Step 1: Raise arm to transit height, keep gripper unchanged */
                uart_log("[MOVE] Raising to TRANSIT height\r\n");
                {
                    robot_pose_t transit_keep = g_pose_transit;
                    transit_keep.angle_deg[JOINT_GRIPPER] = g_current_pose.angle_deg[JOINT_GRIPPER];
                    fsp_err_t err_move = robot_apply_pose(&transit_keep);
                    if (FSP_SUCCESS != err_move)
                    {
                        if (!g_emergency_stop)
                        {
                            fatal_fsp_error("PLACE transit", err_move);
                        }
                        break;
                    }
                }

                /* Step 2: Rotate base to target position while arm is high */
                {
                    robot_pose_t transit_rotated = g_pose_transit;
                    transit_rotated.angle_deg[JOINT_GRIPPER] = g_current_pose.angle_deg[JOINT_GRIPPER];
                    transit_rotated.angle_deg[JOINT_BASE] = p_place_pose->angle_deg[JOINT_BASE];
                    uart_log("[MOVE] Rotating base to target\r\n");
                    {
                        fsp_err_t err_move = robot_apply_pose(&transit_rotated);
                        if (FSP_SUCCESS != err_move)
                        {
                            if (!g_emergency_stop)
                            {
                                fatal_fsp_error("PLACE rotate", err_move);
                            }
                            break;
                        }
                    }
                }

                /* Step 3: Lower arm to place position slowly, keep gripper closed */
                {
                    robot_pose_t place_closed = *p_place_pose;
                    place_closed.angle_deg[JOINT_GRIPPER] = g_current_pose.angle_deg[JOINT_GRIPPER];
                    printf("[POSE] Lowering to %s\r\n", p_pose_name);
                    fsp_err_t err_move = robot_apply_pose_slow(&place_closed);
                    if (FSP_SUCCESS != err_move)
                    {
                        if (!g_emergency_stop)
                        {
                            fatal_fsp_error("PLACE lower", err_move);
                        }
                        break;
                    }
                }

                /* Step 4: Wait for arm to stabilize, then open gripper */
                R_BSP_SoftwareDelay(300U, BSP_DELAY_UNITS_MILLISECONDS);
                uart_log("[MOVE] Opening gripper\r\n");
                {
                    fsp_err_t err_move = servo_set_angle(JOINT_GRIPPER, p_place_pose->angle_deg[JOINT_GRIPPER]);
                    if (FSP_SUCCESS != err_move)
                    {
                        fatal_fsp_error("PLACE gripper", err_move);
                    }
                    g_current_pose.angle_deg[JOINT_GRIPPER] = p_place_pose->angle_deg[JOINT_GRIPPER];
                }

                if (delay_with_service(PLACE_WAIT_MS, true))
                {
                    state_set(STATE_RETURN);
                }
                break;
            }

            case STATE_RETURN:
            {
                /* Step 1: Raise arm to transit height first */
                uart_log("[MOVE] Raising to TRANSIT height\r\n");
                {
                    robot_pose_t transit_rotated = g_pose_transit;
                    transit_rotated.angle_deg[JOINT_BASE] = g_current_pose.angle_deg[JOINT_BASE];
                    fsp_err_t err_move = robot_apply_pose(&transit_rotated);
                    if (FSP_SUCCESS != err_move)
                    {
                        if (!g_emergency_stop)
                        {
                            fatal_fsp_error("RETURN transit", err_move);
                        }
                        break;
                    }
                }

                /* Step 2: Move to HOME (base rotates while arm is high, then lowers) */
                if (apply_pose_and_wait("HOME", &g_pose_home, RETURN_WAIT_MS))
                {
                    uart9_send_blocking("READY\n");
                    uart_log("[PI] READY sent\r\n");
                    g_target_classification = '\0';
                    state_set(g_auto_mode ? STATE_BELT_RUN : STATE_IDLE);
                }
                break;
            }

            case STATE_PICKUP_IDLE:
            case STATE_PICKUP_MOVE:
            {
                /* pickup_v2 fix: 注释承诺要 service 串口，但原来 case body
                 * 实际为空 → Pi 端第一条 HOME 后状态切到 PICKUP_IDLE，
                 * 主循环就不再 service 串口，后续所有命令的字节进 ring
                 * buffer 但永远不被处理。这里补上。 */
                service_pi_uart();
                R_BSP_SoftwareDelay(MAIN_LOOP_SLICE_MS, BSP_DELAY_UNITS_MILLISECONDS);
                break;
            }

            default:
            {
                state_set(STATE_IDLE);
                break;
            }
        }
    }
#endif
}
