#ifndef REF_HPP
#define REF_HPP

#include <cstdint>

/**
 * @brief 参考曲线形状（整定用的“给定信号发生器”）。
 *
 * 建议的实验顺序：Constant（恒值确认方向）→ Step（阶跃看响应）→ Triangle（三角波看跟随误差）。
 * 语义与参考工程 `RM27-tutorial-PID/User/Lib/Ref/ref.hpp` 保持一致。
 */
enum class RefShape
{
    Constant = 0,           ///< 恒值：`Value()` 恒返回 high（本工程的恒值另走各模式的手动 set 变量）
    Step = 1,               ///< 单次阶跃：delayMs 之前为 low，之后为 high
    Triangle = 2,           ///< 三角波：low → high → low，周期 periodMs
    PeriodicSmoothStep = 3, ///< 周期平滑阶跃：保持 low 约 1/5 周期，再用半余弦升到 high
};

/**
 * @struct RefCurve
 * @brief 参考曲线：把“实验开始后的时间”映射成目标值（单位与 low/high 一致）。
 *
 * 纯算法、无状态依赖 → 归属 libs（被 bsp/modules/threads 三层共享），不碰 HAL。
 * 单位随使用场合：速度目标 rad/s（输出轴）、位置目标 rad（输出轴）。
 */
struct RefCurve
{
    RefShape shape = RefShape::Constant;
    float low = 0.0f;             ///< 曲线低值
    float high = 0.0f;            ///< 曲线高值
    uint32_t delayMs = 1000U;     ///< Step 的阶跃延时
    uint32_t periodMs = 4000U;    ///< Triangle / PeriodicSmoothStep 的周期

    /**
     * @brief 取 tMs 时刻的参考值。
     * @param tMs 本次实验开始后的毫秒数（本工程用 ThreadX tick：1 tick = 1 ms）。
     * @note 无内部状态、可安全地在 1 kHz 循环里每拍调用。
     */
    float Value(uint32_t tMs) const;
};

#endif // REF_HPP
