#include "hal_data.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

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
#define INTERP_STEP_DEG        2U
#define INTERP_DELAY_MS        20U

#define BLOCKING_TIMEOUT       200000U   /* busy-wait timeout count for I2C/UART */

#define BELT_STOP_DELAY_MS     500U
#define PRE_GRASP_WAIT_MS      1000U
#define GRASP_WAIT_MS          800U
#define LIFT_WAIT_MS           800U
#define PLACE_WAIT_MS          1200U
#define RETURN_WAIT_MS         1000U

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
    STATE_RETURN
} state_t;

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

static volatile uint8_t g_uart7_rx_char  = 0U;
static volatile bool    g_uart7_rx_ready = false;

static volatile uint8_t g_uart9_rx_char  = 0U;
static volatile bool    g_uart9_rx_ready = false;

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
            g_uart7_rx_char  = (uint8_t) p_args->data;
            g_uart7_rx_ready = true;
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
            g_uart9_rx_char  = (uint8_t) p_args->data;
            g_uart9_rx_ready = true;
            break;
        }

        default:
        {
            break;
        }
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
    bool ready;

    __disable_irq();
    ready = g_uart7_rx_ready;
    if (ready)
    {
        *p_char = g_uart7_rx_char;
        g_uart7_rx_ready = false;
    }
    __enable_irq();

    return ready;
}

static bool uart9_take_char(uint8_t * p_char)
{
    bool ready;

    __disable_irq();
    ready = g_uart9_rx_ready;
    if (ready)
    {
        *p_char = g_uart9_rx_char;
        g_uart9_rx_ready = false;
    }
    __enable_irq();

    return ready;
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

static fsp_err_t robot_apply_pose_internal(robot_pose_t const * p_pose, bool allow_abort)
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
    uint32_t num_steps = (max_delta + INTERP_STEP_DEG - 1U) / INTERP_STEP_DEG;
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
        R_BSP_SoftwareDelay(INTERP_DELAY_MS, BSP_DELAY_UNITS_MILLISECONDS);
    }

    return FSP_SUCCESS;
}

static fsp_err_t robot_apply_pose(robot_pose_t const * p_pose)
{
    return robot_apply_pose_internal(p_pose, true);
}

static fsp_err_t robot_apply_pose_force(robot_pose_t const * p_pose)
{
    return robot_apply_pose_internal(p_pose, false);
}

/* ------------------------------------------------------------------ */
/*  Command processing                                                 */
/* ------------------------------------------------------------------ */
static void process_pi_char(uint8_t rx)
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

        case '\r':
        case '\n':
        {
            break;
        }

        default:
        {
            printf("[PI] Unknown command: %c\r\n", (char) rx);
            break;
        }
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

    if (uart7_take_char(&rx))
    {
        process_debug_char(rx);
    }
}

static void service_pi_uart(void)
{
    uint8_t rx;

    if (uart9_take_char(&rx))
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

    err = robot_apply_pose_force(&g_pose_home);
    if (FSP_SUCCESS != err)
    {
        printf("[ERR] HOME pose failed: %d\r\n", (int) err);
        while (1)
        {
            ;
        }
    }

    memcpy(&g_current_pose, &g_pose_home, sizeof(g_current_pose));
    memcpy(&g_teach_pose, &g_pose_home, sizeof(g_teach_pose));
    g_button_prev_pressed = board_button_is_pressed();

    uart_log("[BOOT] SCI7 debug OK\r\n");
    uart_log("[BOOT] SCI9 Pi UART OK\r\n");
    uart_log("[BOOT] IIC1 + PCA9685 OK\r\n");
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
                /* Step 1: Move arm to GRASP position but keep gripper open */
                {
                    robot_pose_t grasp_open = g_pose_grasp;
                    grasp_open.angle_deg[JOINT_GRIPPER] = g_current_pose.angle_deg[JOINT_GRIPPER];
                    fsp_err_t err_move = robot_apply_pose(&grasp_open);
                    if (FSP_SUCCESS != err_move)
                    {
                        if (!g_emergency_stop)
                        {
                            fatal_fsp_error("GRASP move", err_move);
                        }
                        break;
                    }
                }

                /* Step 2: Close gripper */
                uart_log("[MOVE] Closing gripper\r\n");
                {
                    fsp_err_t err_move = servo_set_angle(JOINT_GRIPPER, g_pose_grasp.angle_deg[JOINT_GRIPPER]);
                    if (FSP_SUCCESS != err_move)
                    {
                        fatal_fsp_error("GRASP gripper", err_move);
                    }
                    g_current_pose.angle_deg[JOINT_GRIPPER] = g_pose_grasp.angle_deg[JOINT_GRIPPER];
                }

                if (delay_with_service(GRASP_WAIT_MS, true))
                {
                    state_set(STATE_LIFT);
                }
                break;
            }

            case STATE_LIFT:
            {
                if (apply_pose_and_wait("LIFT", &g_pose_lift, LIFT_WAIT_MS))
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

                /* Step 1: Raise arm to transit height (safe for rotation) */
                uart_log("[MOVE] Raising to TRANSIT height\r\n");
                {
                    fsp_err_t err_move = robot_apply_pose(&g_pose_transit);
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

                /* Step 3: Lower arm to place position, keep gripper closed */
                {
                    robot_pose_t place_closed = *p_place_pose;
                    place_closed.angle_deg[JOINT_GRIPPER] = g_current_pose.angle_deg[JOINT_GRIPPER];
                    printf("[POSE] Lowering to %s\r\n", p_pose_name);
                    fsp_err_t err_move = robot_apply_pose(&place_closed);
                    if (FSP_SUCCESS != err_move)
                    {
                        if (!g_emergency_stop)
                        {
                            fatal_fsp_error("PLACE lower", err_move);
                        }
                        break;
                    }
                }

                /* Step 4: Open gripper to release */
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

            default:
            {
                state_set(STATE_IDLE);
                break;
            }
        }
    }
#endif
}
