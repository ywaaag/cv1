#include "rclcpp/rclcpp.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/fire_command.hpp"
#include "ballistic_solver/SolveTrajectory.hpp"

using std::placeholders::_1;

class BallisticSolverNode : public rclcpp::Node {
public:
  BallisticSolverNode()
  : Node("ballistic_solver") {
    // 参数声明与读取
    this->declare_parameter<double>("initial_speed", 25.0);
    st.current_v = this->get_parameter("initial_speed").as_double();

    // 初始化模型参数
    st.k = 0.092f;
    st.bullet_type = BULLET_17;
    st.current_yaw = 0.0f;
    st.current_pitch = 0.0f;
    st.vxw = st.vyw = st.vzw = st.v_yaw = 0.0f;
    st.tar_yaw = 0.0f;
    st.r1 = st.r2 = 0.5f; st.dz = 0.1f;
    st.bias_time = 100;
    st.s_bias = 0.19133f;
    st.z_bias = 0.21265f;
    st.armor_id = ARMOR_INFANTRY3;
    st.armor_num = ARMOR_NUM_NORMAL;

    // 订阅装甲板位姿
    subscription_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
      "/detector/armors", rclcpp::SensorDataQoS().keep_last(10),
      std::bind(&BallisticSolverNode::armorCallback, this, _1)
    );

    // 发布增量指令
    publisher_ = this->create_publisher<auto_aim_interfaces::msg::FireCommand>(
      "/solver/fire_command", 10
    );

    RCLCPP_INFO(this->get_logger(), "BallisticSolverNode started");
  }

private:
  void armorCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
    if (msg->armors.empty()) {
      RCLCPP_WARN(this->get_logger(), "No armors detected.");
      return;
    }

    // 提取目标位姿
    const auto &armor = msg->armors[0];
    st.xw = armor.pose.position.x;
    st.yw = armor.pose.position.y;
    st.zw = armor.pose.position.z;

    // 同步模型参数
    setSolveTrajectoryParams(st);

    // 计算绝对角度（弧度）
    float yaw_abs = 0.0f, pitch_abs = 0.0f, aim_x, aim_y, aim_z;
    autoSolveTrajectory(&pitch_abs, &yaw_abs, &aim_x, &aim_y, &aim_z);

    // 计算相对于当前的增量（弧度）
    float delta_yaw_rad   = yaw_abs   - current_yaw_;
    float delta_pitch_rad = pitch_abs - current_pitch_;

    // 更新当前姿态绝对值（弧度）
    current_yaw_   = yaw_abs;
    current_pitch_ = pitch_abs;

    // 转换为角度
    constexpr float RAD2DEG = 180.0f / PI;
    float delta_yaw_deg   = delta_yaw_rad   * RAD2DEG;
    float delta_pitch_deg = delta_pitch_rad * RAD2DEG;

    // 发布增量角度（度）
    auto cmd = auto_aim_interfaces::msg::FireCommand();
    cmd.header.stamp = this->now();
    cmd.yaw   = delta_yaw_deg;
    cmd.pitch = delta_pitch_deg;
    publisher_->publish(cmd);

    RCLCPP_INFO(this->get_logger(), "Published delta_yaw=%.2f°, delta_pitch=%.2f°", delta_yaw_deg, delta_pitch_deg);
  }

  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr subscription_;
  rclcpp::Publisher<auto_aim_interfaces::msg::FireCommand>::SharedPtr publisher_;

  SolveTrajectoryParams st;
  float current_yaw_{0.0f};   // 弧度
  float current_pitch_{0.0f}; // 弧度
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallisticSolverNode>());
  rclcpp::shutdown();
  return 0;
}