#ifndef ORIGINBOT_BASE_H
#define ORIGINBOT_BASE_H

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <originbot_msgs/msg/originbot_status.hpp>
#include <originbot_msgs/srv/originbot_buzzer.hpp>
#include <originbot_msgs/srv/originbot_led.hpp>
#include <originbot_msgs/srv/originbot_pid.hpp>
#include "originbot_msgs/srv/set_int32.hpp"
#include <serial/serial.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <thread>
#include <signal.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <mutex>

//机器人通信帧头定义
#define  ID_CPR2ROS_DATA    0x10    //下位计向ROS发送的综合数据
#define  ID_ROS2CTR_VEL     0x50    //ROS向下位机发送的速度数据
#define  ID_ROS2CTR_IMU     0x51    //ROS向下位机发送的IMU陀螺仪零偏校准指令
#define  ID_ROS2CTR_RTY     0x5a    //ROS向下位机发送的车型参数
#define  ID_ROS2CTR_LGT     0x52    //ROS向下位机发送的灯光调试数据
#define  ID_ROS2CTR_LST     0x53    //ROS向下位机发送的灯光保存数据
#define  ID_ROS2CTR_BEEP    0x54    //ROS向下位机发送的蜂鸣器数据

//#define PI 3.1415926

//IMU加速度计量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACC_RATIO 	  (2*9.8/32768)

//IMU陀螺仪量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位
#define GYRO_RATIO   ((500*3.1415926/180)/32768)

//机器人数据处理周期,单位S
#define DATA_PERIOD   0.02f


// Tarkbot 协议常量
#define FRAME_HEAD1 0xAA
#define FRAME_HEAD2 0x55
#define FRAME_TAIL  0x0D



enum FrameID {
    FRAME_ID_VELOCITY    = 0x01,
    FRAME_ID_IMU         = 0x02,
    FRAME_ID_BATTERY     = 0x03,
    FRAME_ID_PID_LEFT    = 0x04,
    FRAME_ID_PID_RIGHT   = 0x05,
    FRAME_ID_BUZZER_LED  = 0x06
};

// 数据帧结构
#pragma pack(push,1)
struct TarkbotFrame {
    uint8_t header1;
    uint8_t header2;
    uint8_t length;   // data长度
    uint8_t id;
    uint8_t data[32]; // 数据段（可变长）
    uint8_t checksum; // 校验和（length+id+data[]）
};
#pragma pack(pop)

// IMU 数据结构
struct imu_data {
    float acc_x;
    float acc_y;
    float acc_z;    
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

//IMU四元数结构体
struct imu_orientation_data
{
    float w;
    float x;
    float y;
    float z;    
};

//机器人速度数据结构体
struct velocity_data
{
    float linear_x;
    float linear_y;
    float angular_z;    
};

//机器人位置数据结构体
struct pose_data
{
    float pos_x;
    float pos_y;
    float angular_z;    
};

// 机器人状态
struct RobotStatus {
    float battery_voltage;
    bool buzzer_on;
    bool led_on;
};

class OriginbotBase : public rclcpp::Node {
public:
    OriginbotBase(std::string nodeName);
    ~OriginbotBase();

private:
    // 串口通信
    geometry_msgs::msg::TransformStamped t;
    nav_msgs::msg::Odometry odom;

    serial::Serial serial_;
    std::shared_ptr<std::thread> read_thread_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr imu_calib_client_;
    bool imu_calibrated_ = false;


    // ROS 订阅发布
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<originbot_msgs::msg::OriginbotStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr imu_raw_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    // ROS 服务
    rclcpp::Service<originbot_msgs::srv::OriginbotBuzzer>::SharedPtr buzzer_srv_;
    rclcpp::Service<originbot_msgs::srv::OriginbotLed>::SharedPtr led_srv_;
    rclcpp::Service<originbot_msgs::srv::OriginbotPID>::SharedPtr pid_left_srv_;
    rclcpp::Service<originbot_msgs::srv::OriginbotPID>::SharedPtr pid_right_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr imu_calib_srv_;
    rclcpp::Service<originbot_msgs::srv::SetInt32>::SharedPtr led_debug_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr led_save_srv_;
    // 自动标定服务（原地旋转 N 圈，校准 vth）
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr vth_calib_srv_;


    // TF 广播
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_100ms_;

    // 状态变量
    double odom_x_{0.0}, odom_y_{0.0}, odom_th_{0.0};
    double correct_factor_vx_{1.0}, correct_factor_vth_{1.0};
    bool auto_stop_on_{true}, use_imu_{false}, pub_odom_{false};
    int auto_stop_count_{255};
    bool buzzer_mute_{false};

    rclcpp::Time current_time_;
    imu_data imu_data_;
    RobotStatus robot_status_;
    std::mutex status_mutex_;
 

    // 串口数据处理
    void readLoop();
    void processFrame(const TarkbotFrame &frame);
    bool checkFrame(const TarkbotFrame &frame);

    // 协议帧生成
    void sendVelocity(float vx, float vy, float vw);
    //串口发送数据，塔克X-Protocol协议
    void sendSerialPacket(uint8_t *pbuf, uint8_t len, uint8_t num);

    void sendBuzzer(bool on);
    void sendLed(bool on);
    void sendPID(uint8_t id, float p, float i, float d);

    void sendImuCalibrate();              // IMU 陀螺仪零偏校准
    void sendLedDebug(uint8_t mode, uint8_t value); // 灯光调试
    void sendLedSave();                   // 灯光保存

    // ROS 回调
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void buzzerCallback(const std::shared_ptr<originbot_msgs::srv::OriginbotBuzzer::Request> req,
                        std::shared_ptr<originbot_msgs::srv::OriginbotBuzzer::Response> res);
    void ledCallback(const std::shared_ptr<originbot_msgs::srv::OriginbotLed::Request> req,
                     std::shared_ptr<originbot_msgs::srv::OriginbotLed::Response> res);
    void pidLeftCallback(const std::shared_ptr<originbot_msgs::srv::OriginbotPID::Request> req,
                         std::shared_ptr<originbot_msgs::srv::OriginbotPID::Response> res);
    void pidRightCallback(const std::shared_ptr<originbot_msgs::srv::OriginbotPID::Request> req,
                          std::shared_ptr<originbot_msgs::srv::OriginbotPID::Response> res);

    // 新增的 ROS Service 回调
    void imuCalibCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void ledDebugCallback(const std::shared_ptr<originbot_msgs::srv::SetInt32::Request> req,
                      std::shared_ptr<originbot_msgs::srv::SetInt32::Response> res);
    void ledSaveCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    // 定时器回调
    void timer100ms();

    // 自动标定回调
    void vthCalibCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // 工具
    double deg2rad(double deg) { return deg / 180.0 * M_PI; }

    //计算IMU 四元数
    void calculateImuQuaternion(struct imu_data imu_cel);
    //发布话题消息
    
    void publishOdom();     //发布里程计话题
    void publishImu();      //发布IMU传感器话题

    struct imu_orientation_data orient_data_;  //IMU四元数姿态数据
    struct velocity_data vel_data_;    //机器人的速度
    struct pose_data pos_data_;    //机器人的位置
    double yaw_unwrapped_{0.0}; // 累计的、不环绕的 yaw（弧度），用于多圈测量
};

#endif
