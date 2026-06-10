/* ================================================================
 *  tinyml_grasp.h — 端侧 TinyML 夹取状态分类器
 *
 *  纯 C 手写 MLP 前向传播，不依赖任何外部库。
 *  运行于 RA6M5 Cortex-M33 FPU，推理耗时 < 1ms。
 *
 *  输入: 16 步 FSR delta 归一化序列 (delta / 1000.0)
 *  输出: GRASP_STABLE / GRASP_SLIP_RISK / GRASP_OVERFORCE
 *
 *  配合 train_tinyml.py 生成的 tinyml_weights.h 使用。
 * ================================================================ */

#ifndef TINYML_GRASP_H
#define TINYML_GRASP_H

#include "tinyml_weights.h"
#include <string.h>

/* ── 分类结果枚举 ── */
typedef enum e_grasp_class
{
    GRASP_STABLE    = 0,   /* 夹持稳定 */
    GRASP_SLIP_RISK = 1,   /* 滑脱风险 */
    GRASP_OVERFORCE = 2,   /* 力量过大 */
} grasp_class_t;

static const char * const g_grasp_class_names[TINYML_NUM_CLASSES] =
{
    "STABLE",
    "SLIP_RISK",
    "OVERFORCE"
};

/* ── FSR 数据采集缓冲 ── */
#define FSR_LOG_MAX  64U

/* ── 内联 ReLU ── */
static inline float tinyml_relu(float x)
{
    return (x > 0.0f) ? x : 0.0f;
}

/* ================================================================
 *  tinyml_classify — MLP 前向传播
 *
 *  参数: input — 长度为 TINYML_INPUT_LEN 的 float 数组
 *        confidence — 输出最大 logit 值（可传 NULL）
 *  返回: grasp_class_t 分类结果
 * ================================================================ */
static grasp_class_t tinyml_classify(const float * input, float * confidence)
{
    /* Layer 1: input(16) -> hidden1(8), ReLU */
    float h1[TINYML_H1_SIZE];
    for (uint32_t i = 0U; i < TINYML_H1_SIZE; i++)
    {
        float sum = tinyml_b1[i];
        for (uint32_t j = 0U; j < TINYML_INPUT_LEN; j++)
        {
            sum += input[j] * tinyml_w1[j][i];
        }
        h1[i] = tinyml_relu(sum);
    }

    /* Layer 2: hidden1(8) -> hidden2(4), ReLU */
    float h2[TINYML_H2_SIZE];
    for (uint32_t i = 0U; i < TINYML_H2_SIZE; i++)
    {
        float sum = tinyml_b2[i];
        for (uint32_t j = 0U; j < TINYML_H1_SIZE; j++)
        {
            sum += h1[j] * tinyml_w2[j][i];
        }
        h2[i] = tinyml_relu(sum);
    }

    /* Output: hidden2(4) -> classes(3) */
    float out[TINYML_NUM_CLASSES];
    for (uint32_t i = 0U; i < TINYML_NUM_CLASSES; i++)
    {
        float sum = tinyml_b3[i];
        for (uint32_t j = 0U; j < TINYML_H2_SIZE; j++)
        {
            sum += h2[j] * tinyml_w3[j][i];
        }
        out[i] = sum;
    }

    /* Argmax */
    uint32_t max_idx = 0U;
    float max_val = out[0];
    for (uint32_t i = 1U; i < TINYML_NUM_CLASSES; i++)
    {
        if (out[i] > max_val)
        {
            max_val = out[i];
            max_idx = i;
        }
    }

    if (NULL != confidence)
    {
        *confidence = max_val;
    }

    return (grasp_class_t) max_idx;
}

/* ================================================================
 *  tinyml_prepare_input — 从 FSR delta 日志提取模型输入
 *
 *  从 fsr_log[] 中取最后 TINYML_INPUT_LEN 个值，归一化后填入 input[]。
 *  不足 TINYML_INPUT_LEN 个时前面补零。
 * ================================================================ */
static void tinyml_prepare_input(const uint16_t * fsr_log,
                                 uint16_t         log_count,
                                 float *          input)
{
    memset(input, 0, sizeof(float) * TINYML_INPUT_LEN);

    int total  = (int) log_count;
    int window = (total < (int) TINYML_INPUT_LEN) ? total : (int) TINYML_INPUT_LEN;
    int offset = (int) TINYML_INPUT_LEN - window;

    for (int i = 0; i < window; i++)
    {
        input[offset + i] = (float) fsr_log[total - window + i] / 1000.0f;
    }
}

#endif /* TINYML_GRASP_H */
