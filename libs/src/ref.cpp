#include "ref.hpp"

#include <cmath>

namespace
{
    constexpr float PI_F = 3.14159265358979323846f;

    /** @brief 周期平滑阶跃：每个周期先保持低值的时间占比（与参考工程一致） */
    constexpr float SMOOTH_STEP_HOLD_RATIO = 0.2f;
}

float RefCurve::Value(uint32_t tMs) const
{
    switch (shape)
    {
    case RefShape::Constant:
        return high;

    case RefShape::Step:
        return (tMs < delayMs) ? low : high;

    case RefShape::PeriodicSmoothStep:
    {
        if (periodMs < 2U)
        {
            return low;
        }
        const float phase = static_cast<float>(tMs % periodMs) / static_cast<float>(periodMs);
        if (phase < SMOOTH_STEP_HOLD_RATIO)
        {
            return low;
        }
        const float progress = (phase - SMOOTH_STEP_HOLD_RATIO) / (1.0f - SMOOTH_STEP_HOLD_RATIO);
        const float smooth = 0.5f * (1.0f - std::cos(PI_F * progress));
        return low + (high - low) * smooth;
    }

    case RefShape::Triangle:
    default:
    {
        /* 未知取值一律按三角波处理：算不出定值也不会失控，只是给定在 low/high 之间扫。 */
        if (periodMs < 2U)
        {
            return low;
        }
        const float phase = static_cast<float>(tMs % periodMs) / static_cast<float>(periodMs);
        const float ramp = (phase < 0.5f) ? (2.0f * phase) : (2.0f * (1.0f - phase));
        return low + (high - low) * ramp;
    }
    }
}
