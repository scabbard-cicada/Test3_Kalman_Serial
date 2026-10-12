// Copyright 2026 Target Tracker Exam
#ifndef TARGET_TRACKER__KALMAN_FILTER_HPP_
#define TARGET_TRACKER__KALMAN_FILTER_HPP_

#include <Eigen/Dense>

namespace target_tracker
{

/**
 * @brief 二维匀加速 (CA) 运动模型 + 极坐标观测的扩展卡尔曼滤波器 (EKF)
 *
 * 状态向量 (6x1):  x = [px, py, vx, vy, ax, ay]^T        世界系
 * 观测向量 (2x1):  z = [r, θ]^T                           从站点 S = (sx, sy) 测得
 *
 * ---------------------------------------------------------------------------
 * 运动模型（线性，与 Test2 完全相同）
 *
 *      p' = p + v*dt + 0.5*a*dt^2
 *      v' = v + a*dt
 *      a' = a
 *
 * 过程噪声：把 jerk 视为连续时间白噪声（标准差 sigma_jerk, m/s^3），对
 * exp(A τ) G Gᵀ exp(A τ)ᵀ 在 [0, dt] 上积分。单轴 [p, v, a] 的结果：
 *
 *               ⎡ dt^5/20  dt^4/8  dt^3/6 ⎤
 *      Q_axis = ⎢ dt^4/8   dt^3/3  dt^2/2 ⎥ · sigma_jerk^2
 *               ⎣ dt^3/6   dt^2/2  dt     ⎦
 *
 * ---------------------------------------------------------------------------
 * 观测模型（非线性 —— 这是要用 EKF 的原因）
 *
 *      dx = px - sx,  dy = py - sy,  r = sqrt(dx^2 + dy^2)
 *
 *      h(x) = [ r, atan2(dy, dx) ]^T
 *
 *      H = ∂h/∂x = ⎡  dx/r    dy/r    0 0 0 0 ⎤      2x6，每次更新时在先验状态处现算
 *                  ⎣ -dy/r^2  dx/r^2  0 0 0 0 ⎦
 *
 * 新息的方位分量必须 wrap 到 [-π, π]：观测和预测分处 ±π 两侧时，直接相减会得到约 2π
 * 的假新息。
 *
 * r 很小时 1/r^2 发散：r < kMinRange 时跳过更新（本题推荐站点下最近 5 m，不会触发）。
 * ---------------------------------------------------------------------------
 */
class KalmanFilterCA
{
public:
  using Vec6 = Eigen::Matrix<double, 6, 1>;
  using Mat6 = Eigen::Matrix<double, 6, 6>;
  using Vec2 = Eigen::Matrix<double, 2, 1>;
  using Mat2 = Eigen::Matrix<double, 2, 2>;
  using Mat26 = Eigen::Matrix<double, 2, 6>;
  using Mat62 = Eigen::Matrix<double, 6, 2>;

  /// 小于这个距离（m）时雅可比数值不可靠，跳过更新
  static constexpr double kMinRange = 0.1;

  KalmanFilterCA();

  /// 设置站点位置（世界系）
  void setStation(const Vec2 & station) {station_ = station;}
  const Vec2 & station() const {return station_;}

  /**
   * @brief 用第一帧极坐标观测初始化
   *
   * 位置由 (r, θ) 反解；位置的初始协方差 = G R Gᵀ，G = ∂(px,py)/∂(r,θ)，
   * 即把极坐标下的观测噪声换算到笛卡尔系（沿视线方向长、横向窄的椭圆）。
   * 速度、加速度未知，置零并给较大方差。
   */
  void init(const Vec2 & z_polar, double p0_vel, double p0_acc);

  bool initialized() const {return initialized_;}
  void reset() {initialized_ = false;}

  /// 过程噪声强度（连续时间 jerk 标准差，m/s^3）
  void setProcessNoise(double sigma_jerk) {sigma_jerk_ = sigma_jerk;}

  /// 观测噪声：距离标准差（m）、方位标准差（rad）
  void setMeasurementNoise(double sigma_range, double sigma_bearing);
  const Mat2 & measurementNoise() const {return R_;}

  /// 预测步：把状态推进 dt 秒（dt <= 0 时什么都不做）
  void predict(double dt);

  /**
   * @brief 更新步（带卡方门控的离群点抑制）
   *
   * NIS = νᵀ S⁻¹ ν 服从自由度 2 的卡方分布；超过 gate_threshold 判为离群点：
   *   - reject：丢弃这一帧（只保留预测）
   *   - inflate：R 放大 inflate_factor 倍做一次弱更新
   *
   * @return true 表示观测被正常融合；false 表示被判为离群点（或因 r 过小被跳过）
   */
  bool update(const Vec2 & z_polar);

  /// 以当前状态为起点外推 dt 秒的完整状态（不改内部状态），供计算云台角速度前馈
  Vec6 predictStateAhead(double dt) const;

  const Vec6 & state() const {return x_;}
  const Mat6 & covariance() const {return P_;}
  Vec2 position() const {return x_.head<2>();}

  /// 最近一次 update 的 NIS
  double lastNis() const {return last_nis_;}

  /// 门控阈值（卡方，自由度 2）。<= 0 关闭门控
  void setGateThreshold(double thr) {gate_threshold_ = thr;}

  enum class OutlierPolicy { kReject, kInflate };
  void setOutlierPolicy(OutlierPolicy p) {outlier_policy_ = p;}
  void setInflateFactor(double f) {inflate_factor_ = f;}

  /// 是否对新息的方位分量做 [-π, π] wrap。只用于复现"不做 wrap"的错误，正常必须为 true
  void setWrapInnovation(bool on) {wrap_innovation_ = on;}

  // ---- 观测模型（static，便于单元测试单独验证） ----
  static Vec2 h(const Vec6 & x, const Vec2 & station);
  static Mat26 jacobianH(const Vec6 & x, const Vec2 & station);
  /// 把角度 wrap 到 [-π, π]
  static double wrapAngle(double a);

  // ---- 运动模型 ----
  static Mat6 makeF(double dt);
  static Mat6 makeQ(double dt, double sigma_jerk);

private:
  Vec6 x_;
  Mat6 P_;
  Mat2 R_;
  Vec2 station_{Vec2::Zero()};

  double sigma_jerk_{6.0};
  double gate_threshold_{13.8};   // chi2(0.999, dof=2) ≈ 13.82
  double inflate_factor_{50.0};
  OutlierPolicy outlier_policy_{OutlierPolicy::kReject};
  bool wrap_innovation_{true};

  double last_nis_{0.0};
  bool initialized_{false};
};

}  // namespace target_tracker

#endif  // TARGET_TRACKER__KALMAN_FILTER_HPP_
