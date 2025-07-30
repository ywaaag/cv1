#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/fire_command.hpp"
#include "ballistic_solver/SolveTrajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cmath>

using auto_aim_interfaces::msg::Armors;
using auto_aim_interfaces::msg::FireCommand;

class BallisticSolverNode : public rclcpp::Node {
public:
  BallisticSolverNode() : Node("ballistic_solver") {

    // 声明参数
    declareParameters();
    loadParameters();

    // 订阅与发布
    subscription_ = this->create_subscription<Armors>(
        "/detector/armors", rclcpp::SensorDataQoS().keep_last(10),
        std::bind(&BallisticSolverNode::armorCallback, this,
                  std::placeholders::_1));

    publisher_ =
        this->create_publisher<FireCommand>("/solver/target_angle", 10);

    // 计算最大有效射程
    float k1_value = (drag_coefficient_ > 0)
                         ? static_cast<float>(drag_coefficient_)
                         : calculateDragCoefficient();
    max_range_ =
        calculateMaxRange(static_cast<float>(initial_speed_), k1_value);

    RCLCPP_INFO(this->get_logger(),
                "BallisticSolverNode started with initial_speed=%.1f m/s, "
                "max_range=%.1f m",
                initial_speed_, max_range_);
  }

private:
  void declareParameters() {
    // 基本参数
    this->declare_parameter<double>("initial_speed", 25.0);
    this->declare_parameter<double>("drag_coefficient", 0.0); // 0表示自动计算

    // 目标预测参数
    this->declare_parameter<int>("bias_time_ms", 100);
    this->declare_parameter<bool>("enable_motion_prediction", false);

    // 补偿参数
    this->declare_parameter<double>("s_bias", 0.19133);
    this->declare_parameter<double>("z_bias", 0.21265);

    // 安全参数
    this->declare_parameter<double>("max_target_distance", 8.0);
    this->declare_parameter<double>("min_target_distance", 0.05);
    this->declare_parameter<double>("max_pitch_angle_deg", 45.0);

    // 调试参数
    this->declare_parameter<bool>("enable_debug_output", true);
    this->declare_parameter<int>("armor_selection_strategy",
                                 0); // 0=最近, 1=最大
  }

  void loadParameters() {
    initial_speed_ = this->get_parameter("initial_speed").as_double();
    drag_coefficient_ = this->get_parameter("drag_coefficient").as_double();
    bias_time_ms_ = this->get_parameter("bias_time_ms").as_int();
    enable_motion_prediction_ =
        this->get_parameter("enable_motion_prediction").as_bool();
    s_bias_ = this->get_parameter("s_bias").as_double();
    z_bias_ = this->get_parameter("z_bias").as_double();
    max_target_distance_ =
        this->get_parameter("max_target_distance").as_double();
    min_target_distance_ =
        this->get_parameter("min_target_distance").as_double();
    max_pitch_angle_deg_ =
        this->get_parameter("max_pitch_angle_deg").as_double();
    enable_debug_output_ = this->get_parameter("enable_debug_output").as_bool();
    armor_selection_strategy_ =
        this->get_parameter("armor_selection_strategy").as_int();
  }

  // 装甲板选择逻辑
  const auto &
  selectBestArmor(const std::vector<auto_aim_interfaces::msg::Armor> &armors) {
    if (armors.size() == 1) {
      return armors[0];
    }

    auto distanceSquared = [](const auto &armor) {
      return armor.pose.position.x * armor.pose.position.x +
             armor.pose.position.y * armor.pose.position.y +
             armor.pose.position.z * armor.pose.position.z;
    };

    if (armor_selection_strategy_ == 1) {
      // 选择最远的装甲板
      return *std::max_element(armors.begin(), armors.end(),
                               [&](const auto &a, const auto &b) {
                                 return distanceSquared(a) < distanceSquared(b);
                               });
    } else {
      // 默认选择最近的装甲板
      return *std::min_element(armors.begin(), armors.end(),
                               [&](const auto &a, const auto &b) {
                                 return distanceSquared(a) < distanceSquared(b);
                               });
    }
  }

  // 目标有效性验证
  bool isTargetValid(float x, float y, float z, float pitch_deg) {
    return true;
    float distance = std::sqrt(x * x + y * y + z * z);

    // 距离检查
    if (distance < min_target_distance_ || distance > max_target_distance_) {
      if (enable_debug_output_) {
        RCLCPP_WARN(this->get_logger(),
                    "Target distance %.2fm out of range [%.2f, %.2f]", distance,
                    min_target_distance_, max_target_distance_);
      }
      return false;
    }

    // 俯仰角检查
    if (std::abs(pitch_deg) > max_pitch_angle_deg_) {
      if (enable_debug_output_) {
        RCLCPP_WARN(this->get_logger(),
                    "Target pitch angle %.2f° exceeds limit %.2f°", pitch_deg,
                    max_pitch_angle_deg_);
      }
      return false;
    }

    // 射程检查
    float horizontal_distance = std::sqrt(x * x + y * y);
    if (horizontal_distance > max_range_) {
      if (enable_debug_output_) {
        RCLCPP_WARN(this->get_logger(),
                    "Target beyond max range: %.2fm > %.2fm",
                    horizontal_distance, max_range_);
      }
      return false;
    }

    return true;
  }

  void armorCallback(const Armors::SharedPtr msg) {
    if (msg->armors.empty()) {
      if (enable_debug_output_) {
        RCLCPP_DEBUG(this->get_logger(), "No armors detected");
      }
      return;
    }

    // 选择最佳装甲板
    const auto &armor = selectBestArmor(msg->armors);

    // 坐标系变换 (相机坐标系 -> 机器人坐标系)
    float x_robot = armor.pose.position.z;
    float y_robot = -armor.pose.position.x;
    float z_robot = -armor.pose.position.y;

    // 详细的坐标调试输出
    if (enable_debug_output_) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Coordinate transform: Camera(%.3f,%.3f,%.3f) -> "
                   "Robot(%.3f,%.3f,%.3f)",
                   armor.pose.position.x, armor.pose.position.y,
                   armor.pose.position.z, x_robot, y_robot, z_robot);
    }

    // 填充参数结构体
    SolveTrajectoryParams params;

    // 阻力系数：使用参数值或自动计算
    params.k = (drag_coefficient_ > 0) ? static_cast<float>(drag_coefficient_)
                                       : calculateDragCoefficient();
    params.current_v = static_cast<float>(initial_speed_);

    // 目标位置
    params.xw = x_robot;
    params.yw = y_robot;
    params.zw = z_robot;

    // 目标速度（暂时假设静止）
    params.vxw = 0.0f;
    params.vyw = 0.0f;
    params.vzw = 0.0f;

    // 补偿参数
    params.bias_time = bias_time_ms_;
    params.s_bias = static_cast<float>(s_bias_);
    params.z_bias = static_cast<float>(z_bias_);

    // 调用弹道解算
    float pitch_rad = 0.0f, yaw_rad = 0.0f, time_of_flight = 0.0f;
    solveForStaticTarget(params, &pitch_rad, &yaw_rad, &time_of_flight);

    // 转换为角度
    static constexpr float RAD2DEG = 180.0f / PI;
    float target_yaw_deg = yaw_rad * RAD2DEG;
    float target_pitch_deg = pitch_rad * RAD2DEG;

    // 验证解算结果
    if (!isBallisticSolutionValid(pitch_rad, yaw_rad, time_of_flight) ||
        !isTargetValid(x_robot, y_robot, z_robot, target_pitch_deg)) {
      if (enable_debug_output_) {
        RCLCPP_WARN(this->get_logger(),
                    "Invalid ballistic solution, skipping...");
      }
      return;
    }

    // 发布结果
    auto cmd = FireCommand();
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "gimbal_base_link";
    cmd.yaw = target_yaw_deg;
    cmd.pitch = target_pitch_deg;

    publisher_->publish(cmd);

    // 调试输出
    if (enable_debug_output_) {
      float distance =
          std::sqrt(x_robot * x_robot + y_robot * y_robot + z_robot * z_robot);
      float horizontal_dist = std::sqrt(x_robot * x_robot + y_robot * y_robot);
      float compensated_z = z_robot + params.z_bias;

      RCLCPP_INFO(this->get_logger(),
                  "Target: Pos(%.2f,%.2f,%.2f) HDist=%.2fm Z_comp=%.3fm -> "
                  "k=%.6f Angle(Y=%.2f°,P=%.2f°) ToF=%.3fs",
                  x_robot, y_robot, z_robot, horizontal_dist, compensated_z,
                  params.k, target_yaw_deg, target_pitch_deg, time_of_flight);

      // 理论检查：无阻力情况下的抛物线角度
      float simple_angle = std::atan2(z_robot, horizontal_dist) * 180.0f / PI;
      RCLCPP_DEBUG(this->get_logger(),
                   "Simple geometric angle: %.2f° (without ballistics)",
                   simple_angle);
    }
  }

  // 成员变量
  rclcpp::Subscription<Armors>::SharedPtr subscription_;
  rclcpp::Publisher<FireCommand>::SharedPtr publisher_;

  // 参数
  double initial_speed_;
  double drag_coefficient_;
  int bias_time_ms_;
  bool enable_motion_prediction_;
  double s_bias_;
  double z_bias_;
  double max_target_distance_;
  double min_target_distance_;
  double max_pitch_angle_deg_;
  bool enable_debug_output_;
  int armor_selection_strategy_;

  // 计算值
  float max_range_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallisticSolverNode>());
  rclcpp::shutdown();
  return 0;
}