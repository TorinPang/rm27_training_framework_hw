#include "pid.hpp"

#include <cmath>

PID::PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode)
    : mode(mode), kp(kp), ki(ki), kd(kd), maxOut(maxOut), maxIOut(maxIOut)
{
    Clear();
}

void PID::Tuning(float tuning_kp, float tuning_ki, float tuning_kd)
{
    kp = tuning_kp;
    ki = tuning_ki;
    kd = tuning_kd;
}

void PID::UpdateResult()
{
    /* 误差序列（新→旧）：err[0] 本次、err[1] 上次、err[2] 上上次 */
    err[2] = err[1];
    err[1] = err[0];
    err[0] = ref - fdb;

    if (mode == PID_DELTA)
    {
        /* 增量式：u(k) = u(k-1) + Δu，Δu = kp·(e0-e1) + ki·e0 + kd·(e0-2e1+e2)
         * 三个中间量都表示“本次增量”，result 自身累加即为总输出。
         * 增量式没有独立的积分累加器——积分作用体现在 result 的累加里，
         * 因此不用 maxIOut，靠 maxOut 限住累加结果即可（这本身就是抗积分饱和）。 */
        pResult = kp * (err[0] - err[1]);
        iResult = ki * err[0];
        dResult = kd * (err[0] - 2.0f * err[1] + err[2]);

        result += pResult + iResult + dResult;
    }
    else
    {
        /* 位置式：u(k) = kp·e0 + Σ(ki·e0) + kd·(e0-e1) */
        pResult = kp * err[0];
        dResult = kd * (err[0] - err[1]);

        /* 积分限幅：先限住累加的积分项，再参与求和，防止积分饱和 */
        iResult = Numeric::LimitABS(iResult + ki * err[0], maxIOut);
        result  = pResult + iResult + dResult;
    }

    if (!std::isfinite(result))
    {
        /* 反馈/参数异常导致 NaN、Inf 时，清零回到安全状态（本拍输出为 0） */
        Clear();
        return;
    }

    /* 输出限幅 */
    result = Numeric::LimitABS(result, maxOut);
}

void PID::Clear()
{
    ref = fdb = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
}
