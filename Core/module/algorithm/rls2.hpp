#pragma once

/// 2 维递推最小二乘 (RLS) 在线参数估计器
///
/// 手动展开 2×2 矩阵运算，无矩阵库依赖。
/// 用于功率模型辨识: y = x^T * θ, 其中 θ = [k1, k2]
class Rls2 {
public:
    /// 初始化协方差和参数
    /// @param lambda  遗忘因子 (0 < λ ≤ 1, 越接近 1 记忆越长)
    /// @param delta   初始协方差 P = δ·I
    /// @param k1_init 参数初始值 θ[0]
    /// @param k2_init 参数初始值 θ[1]
    void Init(float lambda, float delta, float k1_init, float k2_init) {
        lambda_  = lambda;
        inv_lam_ = 1.0f / lambda;
        theta_[0] = k1_init;
        theta_[1] = k2_init;
        // P = delta * I
        P_[0][0] = delta;  P_[0][1] = 0;
        P_[1][0] = 0;      P_[1][1] = delta;
    }

    /// RLS 更新
    /// @param x  输入向量 [x0, x1]
    /// @param y  观测输出
    void Update(const float x[2], float y) {
        // Px = P * x
        const float Px0 = P_[0][0] * x[0] + P_[0][1] * x[1];
        const float Px1 = P_[1][0] * x[0] + P_[1][1] * x[1];

        // denom = λ + x^T * P * x
        const float denom = lambda_ + x[0] * Px0 + x[1] * Px1;
        if (denom < 1e-10f)
            return;  // 数值保护
        const float inv_denom = 1.0f / denom;

        // K = Px / denom
        const float K0 = Px0 * inv_denom;
        const float K1 = Px1 * inv_denom;

        // θ += K * (y - x^T * θ)
        const float err = y - (x[0] * theta_[0] + x[1] * theta_[1]);
        theta_[0] += K0 * err;
        theta_[1] += K1 * err;

        // P = (P - K * x^T * P) / λ
        // 先算 KxT_P: 外积 K⊗x 然后乘 P
        // KxT[i][j] = K[i] * x[j], 然后 (KxT * P)[i][j] = Σ_k KxT[i][k] * P[k][j]
        // 但更高效: (K * xT) * P = K ⊗ (xT * P) = K ⊗ Px^T
        // 即 new_P[i][j] = (P[i][j] - K[i] * Px_j) / λ
        // 其中 Px_j = x^T * P 的第 j 列 = (P^T * x)[j]
        // 由于 P 对称: xTP[j] = Px[j] 对第一列, 需要重新算
        const float xTP0 = x[0] * P_[0][0] + x[1] * P_[1][0];  // = Px0 (P对称)
        const float xTP1 = x[0] * P_[0][1] + x[1] * P_[1][1];  // = Px1 (P对称)

        P_[0][0] = (P_[0][0] - K0 * xTP0) * inv_lam_;
        P_[0][1] = (P_[0][1] - K0 * xTP1) * inv_lam_;
        P_[1][0] = (P_[1][0] - K1 * xTP0) * inv_lam_;
        P_[1][1] = (P_[1][1] - K1 * xTP1) * inv_lam_;
    }

    void Reset(float delta, float k1_init, float k2_init) {
        theta_[0] = k1_init;
        theta_[1] = k2_init;
        P_[0][0] = delta;  P_[0][1] = 0;
        P_[1][0] = 0;      P_[1][1] = delta;
    }

    float K1() const { return theta_[0]; }
    float K2() const { return theta_[1]; }
    const float* Params() const { return theta_; }

private:
    float lambda_  = 0.9999f;
    float inv_lam_ = 1.0f / 0.9999f;
    float theta_[2] = {};
    float P_[2][2]  = {};
};
