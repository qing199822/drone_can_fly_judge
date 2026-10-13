#include "tracking/kalman_filter.h"
#include <cmath>

// For multivariate normal PDF calculation
constexpr double LOG_2_PI = 1.83787706640934548356;
constexpr int IMMFilter::NUM_MODELS;

IMMFilter::IMMFilter(double dt, double u_init, double v_init, double s_init) {
    // Initialize fused state and probabilities
    x = Eigen::VectorXd::Zero(dim_x_);
    x << u_init, v_init, 0, 0, 0, 0, s_init;

    P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 100.0;
    
    mu = Eigen::VectorXd(NUM_MODELS);
    mu << 0.8, 0.1, 0.1; // Initial belief: CV is most likely

    // Define Markov transition matrix M_
    M_ = Eigen::MatrixXd(NUM_MODELS, NUM_MODELS);
    M_ << 0.4,   0.025, 0.025,
          0.3,   0.90,  0.05,
          0.3,   0.05,  0.90;

    omega_ = Eigen::MatrixXd(NUM_MODELS, NUM_MODELS);
    likelihoods_.resize(NUM_MODELS);

    // Setup individual Kalman Filters
    setupFilters(dt, u_init, v_init, s_init);

    // Initialize Low-Pass Filters
    Eigen::VectorXd initial_5d_state(5);
    initial_5d_state << u_init, v_init, 0.0, 0.0, s_init;
    lpf1_state_ = initial_5d_state;
    lpf2_state_ = initial_5d_state;
    lpf3_state_ = initial_5d_state;
    lpf4_state_ = initial_5d_state;
}

void IMMFilter::setupFilters(double dt, double u_init, double v_init, double s_init) {
    filters_.clear();
    
    Eigen::VectorXd x_init(dim_x_);
    x_init << u_init, v_init, 0, 0, 0, 0, s_init;
    Eigen::MatrixXd P_init = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 100.0;

    // Common Measurement Matrix H and Noise R
    Eigen::MatrixXd H(dim_z_, dim_x_);
    H << 1, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1;

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(dim_z_, dim_z_);
    R(0,0) = 100.0;
    R(1,1) = 100.0;
    R(2,2) = 1500.0;

    // --- Model 1: Constant Velocity (CV) ---
    KalmanFilterBase cv_filter(dim_x_, dim_z_);
    cv_filter.F << 1, 0, dt, 0, 0, 0, 0,
                   0, 1, 0, dt, 0, 0, 0,
                   0, 0, 1, 0, 0, 0, 0,
                   0, 0, 0, 1, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, // Accel terms are zero
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 1;
    Eigen::VectorXd cv_q_diag(dim_x_); // 1. 创建一个7维的空向量
    cv_q_diag << 0.1, 0.1, 50.0, 50.0, 1, 1, 150.0; // 2. 使用 << 填充数据
    cv_filter.Q = cv_q_diag.asDiagonal(); // 3. 将其转换为对角矩阵
    cv_filter.H = H;
    cv_filter.R = R;
    cv_filter.init(x_init, P_init);
    filters_.push_back(cv_filter);

    // --- Model 2: Constant Acceleration (CA) ---
    KalmanFilterBase ca_filter(dim_x_, dim_z_);
    double dt2 = 0.5 * dt * dt;
    ca_filter.F << 1, 0, dt, 0, dt2, 0, 0,
                   0, 1, 0, dt, 0, dt2, 0,
                   0, 0, 1, 0, dt, 0, 0,
                   0, 0, 0, 1, 0, dt, 0,
                   0, 0, 0, 0, 1, 0, 0,
                   0, 0, 0, 0, 0, 1, 0,
                   0, 0, 0, 0, 0, 0, 1;
    Eigen::VectorXd ca_q_diag(dim_x_);
    ca_q_diag << 0.1, 0.1, 25, 25, 50.0, 50.0, 300.0;
    ca_filter.Q = ca_q_diag.asDiagonal();
    ca_filter.H = H;
    ca_filter.R = R;
    ca_filter.init(x_init, P_init);
    filters_.push_back(ca_filter);

    // --- Model 3: High Maneuver (HM) ---
    KalmanFilterBase hm_filter(dim_x_, dim_z_);
    hm_filter.F = cv_filter.F; // Same transition as CV
    Eigen::VectorXd hm_q_diag(dim_x_);
    hm_q_diag << 0.5, 0.5, 150.0, 150.0, 50.0, 50.0, 1500.0;
    hm_filter.Q = hm_q_diag.asDiagonal();
    hm_filter.H = H;
    hm_filter.R = R;
    hm_filter.init(x_init, P_init);
    filters_.push_back(hm_filter);
}

void IMMFilter::predict() {
    for (auto& f : filters_) {
        f.predict();
    }
}

void IMMFilter::update(const Eigen::VectorXd& z) {
    // --- Step 1: Mixing and Interaction ---
    computeMixingProbabilities();
    mixStates();

    // --- Step 2: Individual KF Updates and Likelihood Calculation ---
    for (int i = 0; i < NUM_MODELS; ++i) {
        filters_[i].update(z);
        
        // Use log-likelihood to avoid numerical underflow
        double S_det = filters_[i].S.determinant();
        if (S_det <= 0) S_det = 1e-9; // Avoid log of non-positive
        double log_likelihood = -0.5 * (filters_[i].y.transpose() * filters_[i].S.inverse() * filters_[i].y + 
                                  log(S_det) + dim_z_ * LOG_2_PI);
        likelihoods_[i] = exp(log_likelihood);
    }

    // --- Step 3: Update Model Probabilities ---
    updateModelProbabilities();

    // --- Step 4: Fuse State and Covariance for Final Estimate ---
    fuseStateAndCovariance();

    // --- Apply LPF to the final result ---
    //applyLowPassFilter();
    
    // --- Log mode switches ---
    int new_mode_idx = 0;
    mu.maxCoeff(&new_mode_idx);
    if (new_mode_idx != current_mode_idx_) {
        ROS_INFO("[IMM] Mode switched: %s -> %s (Probs: [%.2f; %.2f; %.2f])",
                 model_names_[current_mode_idx_].c_str(),
                 model_names_[new_mode_idx].c_str(),
                 mu(0), mu(1), mu(2));
        current_mode_idx_ = new_mode_idx;
    }
}

void IMMFilter::computeMixingProbabilities() {
    // c_bar_ 现在是 RowVectorXd 类型，赋值操作类型匹配，不再报错
    c_bar_ = mu.transpose() * M_;
    for (int j = 0; j < NUM_MODELS; ++j) {
        for (int i = 0; i < NUM_MODELS; ++i) {
            // 【修正】使用 (j) 来访问 c_bar_ 的第 j 个元素
            // 同时增加一个除零保护，使代码更健壮
            if (c_bar_(j) > 1e-9) {
                omega_(i, j) = (M_(i, j) * mu(i)) / c_bar_(j);
            } else {
                omega_(i, j) = 0.0;
            }
        }
    }
}

void IMMFilter::mixStates() {
    for (int j = 0; j < NUM_MODELS; ++j) {
        Eigen::VectorXd x_mixed = Eigen::VectorXd::Zero(dim_x_);
        Eigen::MatrixXd P_mixed = Eigen::MatrixXd::Zero(dim_x_, dim_x_);
        
        for (int i = 0; i < NUM_MODELS; ++i) {
            x_mixed += filters_[i].x * omega_(i, j);
        }

        for (int i = 0; i < NUM_MODELS; ++i) {
            Eigen::VectorXd diff = filters_[i].x - x_mixed;
            P_mixed += omega_(i, j) * (filters_[i].P + diff * diff.transpose());
        }
        
        filters_[j].x = x_mixed;
        filters_[j].P = P_mixed;
        
        // After mixing, perform the prediction for this filter
        filters_[j].predict();
    }
}

void IMMFilter::updateModelProbabilities() {
    double mu_sum = 0.0;
    for(int i = 0; i < NUM_MODELS; ++i) {
        // 【修正】使用 (i) 来访问 c_bar_ 的第 i 个元素
        mu(i) = likelihoods_[i] * c_bar_(i);
        mu_sum += mu(i);
    }
    // Normalize probabilities
    if (mu_sum > 1e-9) {
        mu /= mu_sum;
    } else {
        // Fallback in case of numerical issues
        mu.fill(1.0 / NUM_MODELS);
    }
}

void IMMFilter::fuseStateAndCovariance() {
    x.setZero();
    P.setZero();

    for (int i = 0; i < NUM_MODELS; ++i) {
        x += mu(i) * filters_[i].x;
    }
    
    for (int i = 0; i < NUM_MODELS; ++i) {
        Eigen::VectorXd diff = filters_[i].x - x;
        P += mu(i) * (filters_[i].P + diff * diff.transpose());
    }
}

void IMMFilter::applyLowPassFilter() {
    // 1. Convert the raw 7D IMM state to the 5D state needed by the LPFs
    Eigen::VectorXd x_5d_raw(5);
    x_5d_raw << x(0), x(1), x(2), x(3), x(6); // u, v, du, dv, s

    // 2. Apply the four stages of the low-pass filter
    lpf1_state_ = lpf_alpha_ * x_5d_raw + (1.0 - lpf_alpha_) * lpf1_state_;
    lpf2_state_ = lpf_alpha_ * lpf1_state_ + (1.0 - lpf_alpha_) * lpf2_state_;
    lpf3_state_ = lpf_alpha_ * lpf2_state_ + (1.0 - lpf_alpha_) * lpf3_state_;
    lpf4_state_ = lpf_alpha_ * lpf3_state_ + (1.0 - lpf_alpha_) * lpf4_state_;
}


Eigen::VectorXd IMMFilter::getState() const {
    //return lpf4_state_;
    Eigen::VectorXd x_5d_raw(5);
    x_5d_raw << x(0), x(1), x(2), x(3), x(6); // u, v, du, dv, s
    return x_5d_raw; // <--- 3. 返回这个原始的、最快的状态估计
}

void IMMFilter::reset(double u_init, double v_init, double s_init) {
    Eigen::VectorXd x_init(dim_x_);
    x_init << u_init, v_init, 0, 0, 0, 0, s_init;

    Eigen::MatrixXd P_init = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 100.0;

    for (auto& f : filters_) {
        f.init(x_init, P_init);
    }

    x = x_init;
    P = P_init;
    mu << 0.8, 0.1, 0.1; // Reset to initial probabilities
    current_mode_idx_ = 0;
    
    // Reset LPFs
    Eigen::VectorXd initial_5d_state(5);
    initial_5d_state << u_init, v_init, 0.0, 0.0, s_init;
    lpf1_state_ = initial_5d_state;
    lpf2_state_ = initial_5d_state;
    lpf3_state_ = initial_5d_state;
    lpf4_state_ = initial_5d_state;

    ROS_INFO("[IMM] All filters, including the 4-stage LPF, have been reset.");
}