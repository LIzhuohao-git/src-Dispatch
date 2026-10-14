#include "originbot_base/originbot_base.h"
#include <unistd.h>
#include <algorithm>
#include <cstring>
#include <iomanip>
#include <limits>
#include <rclcpp/logging.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using namespace std::placeholders;
using namespace std::chrono_literals;

namespace {

constexpr double kMaxLinearX = 1.5;
constexpr double kMaxLinearY = 1.5;
constexpr double kMaxAngularZ = 5.0;

double clampValue(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

int16_t velocityToInt16(double value, double limit) {
  if (!std::isfinite(value)) {
    return 0;
  }
  return static_cast<int16_t>(clampValue(value, -limit, limit) * 1000.0);
}

}  // namespace

OriginbotBase::OriginbotBase(std::string nodeName) : Node(nodeName) {
    // ---------------- ROS 参数 ----------------
    std::string port_name = "ttyS3";
    port_name = this->declare_parameter<std::string>("port_name", "ttyS3");


    correct_factor_vx_ = this->declare_parameter<double>("correct_factor_vx", 1.0);
    correct_factor_vth_ = this->declare_parameter<double>("correct_factor_vth", 1.0);


    RCLCPP_INFO(this->get_logger(), "Loaded parameters: correct_factor_vx = %.3f, correct_factor_vth = %.3f", 
            correct_factor_vx_, correct_factor_vth_);

    this->declare_parameter("auto_stop_on", true);        // 默认 true 或你想要的默认值
    this->get_parameter("auto_stop_on", auto_stop_on_);

    this->declare_parameter("use_imu", true);
    this->get_parameter("use_imu", use_imu_);

    this->declare_parameter("pub_odom", true);
    this->get_parameter("pub_odom", pub_odom_);


    // ---------------- ROS 话题/服务 ----------------
    odom_pub_   = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    rclcpp::QoS status_qos(rclcpp::KeepLast(10));
    status_qos.reliable();
    status_qos.durability_volatile();
    status_pub_ = this->create_publisher<originbot_msgs::msg::OriginbotStatus>(
        "originbot_status", status_qos);
    if (use_imu_)
    {
        imu_calib_client_ = this->create_client<std_srvs::srv::Trigger>("/imu_calibrate");
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    }

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {this->cmdVelCallback(msg);});


    buzzer_srv_ = this->create_service<originbot_msgs::srv::OriginbotBuzzer>(
        "originbot_buzzer", std::bind(&OriginbotBase::buzzerCallback, this, std::placeholders::_1, std::placeholders::_2));
    led_srv_ = this->create_service<originbot_msgs::srv::OriginbotLed>(
        "originbot_led", std::bind(&OriginbotBase::ledCallback, this, std::placeholders::_1, std::placeholders::_2));
    pid_left_srv_ = this->create_service<originbot_msgs::srv::OriginbotPID>(
        "originbot_left_pid", std::bind(&OriginbotBase::pidLeftCallback, this, std::placeholders::_1, std::placeholders::_2));
    pid_right_srv_ = this->create_service<originbot_msgs::srv::OriginbotPID>(
        "originbot_right_pid", std::bind(&OriginbotBase::pidRightCallback, this, std::placeholders::_1, std::placeholders::_2));

    imu_calib_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "imu_calibrate", std::bind(&OriginbotBase::imuCalibCallback, this, _1, _2));

    led_debug_srv_ = this->create_service<originbot_msgs::srv::SetInt32>(
    "led_debug", std::bind(&OriginbotBase::ledDebugCallback, this, _1, _2));

    led_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "led_save", std::bind(&OriginbotBase::ledSaveCallback, this, _1, _2));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ---------------- 打开串口 ----------------
    try {
        serial_.setPort("/dev/" + port_name);
        serial_.setBaudrate(230400);
        auto to = serial::Timeout::simpleTimeout(2000);
        serial_.setTimeout(to);
        serial_.open();
    } catch (serial::IOException &e) {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口，请检查连接");
    }

    if (serial_.isOpen()) {
        RCLCPP_INFO(this->get_logger(), "串口已打开");
        read_thread_ = std::make_shared<std::thread>(&OriginbotBase::readLoop, this);
    }

    // ---------------- 定时器 ----------------
    timer_100ms_ = this->create_wall_timer(
    100ms, [this]() { this->timer100ms(); });

}

OriginbotBase::~OriginbotBase() {
    if (serial_.isOpen())
        serial_.close();
}

// ================= 串口读取线程 =================
void OriginbotBase::readLoop() {
    uint8_t byte;
    TarkbotFrame frame;
    size_t idx = 0;

    while (rclcpp::ok()) {
        if (serial_.read(&byte, 1) < 1) continue;

        if (idx == 0 && byte != FRAME_HEAD1) continue;
        if (idx == 1 && byte != FRAME_HEAD2) { idx = 0; continue; }

        if (idx >= sizeof(TarkbotFrame)) {
            idx = 0;
            if (byte == FRAME_HEAD1) {
                ((uint8_t*)&frame)[idx++] = byte;
            }
            continue;
        }

        ((uint8_t*)&frame)[idx++] = byte;

        if (idx >= 4) {
            size_t total_len = frame.length;
            if (total_len < 5 || total_len > sizeof(TarkbotFrame)) {
                RCLCPP_WARN(this->get_logger(), "Invalid frame length %zu, realign...", total_len);
                idx = 0;
                if (byte == FRAME_HEAD1) {
                    ((uint8_t*)&frame)[idx++] = byte;
                }
                continue;
            }
            if (idx == total_len) {
                // 日志输出...
                if (checkFrame(frame)) {
                    processFrame(frame);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Checksum error!");
                }
                idx = 0;
            } else if (idx > total_len) {
                RCLCPP_WARN(this->get_logger(), "Frame length mismatch, realign...");
                idx = 0;
            }
        }
    }
}


bool OriginbotBase::checkFrame(const TarkbotFrame &frame) {
    if (frame.length < 5 || frame.length > sizeof(TarkbotFrame)) {
        return false;
    }

    uint8_t sum = 0;
    int total_len = frame.length;  // 总帧长
    const uint8_t *p = reinterpret_cast<const uint8_t*>(&frame);
    for (int i = 0; i < total_len - 1; i++) {  // 修正：从0开始，到倒数第二个字节
        sum += p[i];
    }
    return (sum == p[total_len - 1]);  // MCU协议：uint8_t sum == checksum
}


/***************四元数计算**************************************************/
volatile float twoKp = 1.0f;     // 2 * proportional gain (Kp)
volatile float twoKi = 0.0f;     // 2 * integral gain (Ki)
 // quaternion of sensor frame relative to auxiliary frame
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;         
 // integral error terms scaled by Ki
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
volatile const float sampling_period  = DATA_PERIOD;

float invSqrt(float number);
float invSqrt(float x)
{
    volatile long i;
    volatile float halfx, y;
    volatile const float f = 1.5F;

    halfx = x * 0.5F;
    y = x;
    i = * (( long * ) &y);
    
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( halfx * y * y ) );

    return y;
}
void OriginbotBase::calculateImuQuaternion(struct imu_data imu_cel)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    //首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
    recipNorm = invSqrt(imu_cel.acc_x * imu_cel.acc_x + imu_cel.acc_y * imu_cel.acc_y + imu_cel.acc_z * imu_cel.acc_z);

    imu_cel.acc_x *= recipNorm;
    imu_cel.acc_y *= recipNorm;
    imu_cel.acc_z *= recipNorm;      

    // 把四元数换算成方向余弦中的第三行的三个元素
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    //误差是估计的重力方向和测量的重力方向的交叉乘积之和
    halfex = (imu_cel.acc_y * halfvz - imu_cel.acc_z * halfvy);
    halfey = (imu_cel.acc_z * halfvx - imu_cel.acc_x * halfvz);
    halfez = (imu_cel.acc_x * halfvy - imu_cel.acc_y * halfvx);

    // 计算并应用积分反馈（如果启用）
    if(twoKi > 0.0f) 
    {
        integralFBx += twoKi * halfex * sampling_period;  // integral error scaled by Ki
        integralFBy += twoKi * halfey * sampling_period;
        integralFBz += twoKi * halfez * sampling_period;
        imu_cel.gyro_x += integralFBx;        // apply integral feedback
        imu_cel.gyro_y += integralFBy;
        imu_cel.gyro_z += integralFBz;
    }
    else 
    {
        integralFBx = 0.0f;       // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }
    // Apply proportional feedback
    imu_cel.gyro_x += twoKp * halfex;
    imu_cel.gyro_y += twoKp * halfey;
    imu_cel.gyro_z += twoKp * halfez;        

    // Integrate rate of change of quaternion
    imu_cel.gyro_x *= (0.5f * sampling_period);   // pre-multiply common factors
    imu_cel.gyro_y *= (0.5f * sampling_period);
    imu_cel.gyro_z *= (0.5f * sampling_period);

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * imu_cel.gyro_x - qc * imu_cel.gyro_y - q3 * imu_cel.gyro_z);
    q1 += (qa * imu_cel.gyro_x + qc * imu_cel.gyro_z - q3 * imu_cel.gyro_y);
    q2 += (qa * imu_cel.gyro_y - qb * imu_cel.gyro_z + q3 * imu_cel.gyro_x);
    q3 += (qa * imu_cel.gyro_z + qb * imu_cel.gyro_y - qc * imu_cel.gyro_x); 

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    //计算结果赋值到
    orient_data_.w = q0;
    orient_data_.x = q1;
    orient_data_.y = q2;
    orient_data_.z = q3;

    //计算欧拉角
    // roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	// pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	// yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);


}
/*
 * @功  能  计算IMU四元数
 */

// ================= 协议帧处理 =================
#if 1
void OriginbotBase::processFrame(const TarkbotFrame &frame) {
    if (frame.id != ID_CPR2ROS_DATA) {
        RCLCPP_WARN(this->get_logger(), "Unsupported frame id=0x%02X len=%d", frame.id, frame.length);
        return;
    }
    // 综合数据帧总长度应为 25 字节（含校验），数据区 22 字节
    if (frame.length != 25) {
        RCLCPP_WARN(this->get_logger(), "Invalid data length: %d (expect 25)", frame.length);
        return;
    }

    auto be_int16 = [](const uint8_t *d) { return (int16_t)((d[0] << 8) | d[1]); };
    auto be_uint16 = [](const uint8_t *d) { return (uint16_t)((d[0] << 8) | d[1]); };

    const uint8_t *p = frame.data; // 数据起始指针

    // ---------------- 1. IMU 原始数据 ----------------
    //解析IMU加速度数据
    int16_t acc_x   = be_int16(p + 0);
    int16_t acc_y   = be_int16(p + 2);
    int16_t acc_z   = be_int16(p + 4);
    //解析IMU陀螺仪数据
    int16_t gyro_x  = be_int16(p + 6);
    int16_t gyro_y  = be_int16(p + 8);
    int16_t gyro_z  = be_int16(p + 10);
    // 解析机器人速度
    int16_t vel_x_raw = be_int16(p + 12);  // m/s * 1000
    int16_t vel_y_raw = be_int16(p + 14);
    int16_t vel_w_raw = be_int16(p + 16);  // rad/s * 1000

    imu_data_.acc_x = acc_x * ACC_RATIO;
    imu_data_.acc_y = acc_y * ACC_RATIO;
    imu_data_.acc_z = acc_z * ACC_RATIO;
    imu_data_.gyro_x = gyro_x * GYRO_RATIO;
    imu_data_.gyro_y = gyro_y * GYRO_RATIO;
    imu_data_.gyro_z = gyro_z * GYRO_RATIO;

    float vx  = correct_factor_vx_  * (vel_x_raw / 1000.0f);
    float vy  = vel_y_raw / 1000.0f;
    float vth = correct_factor_vth_ * (vel_w_raw / 1000.0f);

    if (use_imu_) {
            // 第一次启动时自动标定
    if (!imu_calibrated_) {
        if (imu_calib_client_->wait_for_service(std::chrono::seconds(2))) {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            imu_calib_client_->async_send_request(request);
            // 不再等待返回，避免把 node 添加到其它 executor
            RCLCPP_INFO(this->get_logger(), "IMU calibration request sent (fire-and-forget).");
        } else {
            RCLCPP_WARN(this->get_logger(), "IMU calibration service not available!");
        }

        imu_calibrated_ = true;  // 只调用一次
    }
         //计算IMU四元数数据
        calculateImuQuaternion(imu_data_);
        //发布IMU传感器话题
        publishImu();
    
        // 设置速度数据
    vel_data_.linear_x = vx;
    vel_data_.linear_y = vy;
    vel_data_.angular_z = vth;
// ---------------- 3. 里程积分 ----------------
        static rclcpp::Time last_time = this->now();
        current_time_ = this->now();
        float dt = (current_time_ - last_time).seconds();
        last_time = current_time_;

        // 使用轮式里程计（来自速度和角速度）进行里程积分，不使用 IMU 的 yaw/角速度来覆盖位姿
        pos_data_.angular_z += vel_data_.angular_z * dt;

        // 积分位置（基于轮式线速度和当前轮式朝向）
        pos_data_.pos_x += vel_data_.linear_x * cos(pos_data_.angular_z) * dt;
        pos_data_.pos_y += vel_data_.linear_x * sin(pos_data_.angular_z) * dt;

        // 处理角度环绕
        if (pos_data_.angular_z > M_PI) pos_data_.angular_z -= 2 * M_PI;
        else if (pos_data_.angular_z < -M_PI) pos_data_.angular_z += 2 * M_PI;
        } else {
            // ---------------- 3. 里程积分 ----------------
            static rclcpp::Time last_time = this->now();
            current_time_ = this->now();
            float dt = (current_time_ - last_time).seconds();
            last_time = current_time_;

            // 如果不使用 IMU，用轮式 vth 积分
            pos_data_.angular_z += vel_data_.angular_z * dt;

            // 积分位置
            pos_data_.pos_x += vel_data_.linear_x * cos(pos_data_.angular_z) * dt;
            pos_data_.pos_y += vel_data_.linear_x * sin(pos_data_.angular_z) * dt;

            // 处理角度环绕
            if (pos_data_.angular_z > M_PI) pos_data_.angular_z -= 2 * M_PI;
            else if (pos_data_.angular_z < -M_PI) pos_data_.angular_z += 2 * M_PI;
        }

    publishOdom();

    // ---------------- 4. 电池电压 ----------------
    uint16_t batt_raw = be_uint16(p + 18);
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        robot_status_.battery_voltage = batt_raw / 100.0;
    }

    //RCLCPP_INFO(this->get_logger(), "Raw vel_w: %d, vel_x: %d", vel_w_raw, vel_x_raw);
}
#endif

// ================= 协议帧发送 =================
#if 1
void OriginbotBase::sendVelocity(float vx, float vy, float vw) {
    TarkbotFrame f{};
    f.header1 = FRAME_HEAD1;  // AA
    f.header2 = FRAME_HEAD2;  // 55
    f.id = 0x50;  // ID_URX_VEL

    int16_t ix = static_cast<int16_t>(vx * 1000);
    int16_t iy = static_cast<int16_t>(vy * 1000);
    int16_t iw = static_cast<int16_t>(vw * 1000);

    f.data[0] = (ix >> 8) & 0xFF;  // 大端，高位先
    f.data[1] = ix & 0xFF;
    f.data[2] = (iy >> 8) & 0xFF;
    f.data[3] = iy & 0xFF;
    f.data[4] = (iw >> 8) & 0xFF;
    f.data[5] = iw & 0xFF;

    uint8_t data_len = 6;
    f.length = data_len + 5;  // 总帧长=头(2)+长(1)+ID(1)+数据(6)+校验(1)=11

    uint8_t sum = 0;
    uint8_t *p = (uint8_t*)&f;
    for (int i = 0; i < f.length - 1; i++) {
        sum += p[i];
    }
    f.checksum = sum;

    try {
        size_t bytes_written = serial_.write((uint8_t*)&f, f.length);
        RCLCPP_INFO(this->get_logger(), "Bytes written to serial: %zu", bytes_written);
        RCLCPP_INFO(this->get_logger(), "========================================");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", e.what());
    }
}

#endif

void OriginbotBase::sendSerialPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
{
	uint8_t i,cnt;	
    uint8_t tx_checksum = 0;//发送校验和
    uint8_t tx_buf[64];
	
    //判断是否超出长度
	if(len <= 64)
	{
		//获取数据
		tx_buf[0] = 0xAA;    //帧头
		tx_buf[1] = 0x55;    //
		tx_buf[2] = len+5;  //根据输出长度计算帧长度
		tx_buf[3] = num;    //帧编码
		
		for(i=0; i<len; i++)
		{
			tx_buf[4+i] = *(pbuf+i);
		}
		
		//计算校验和	
		cnt = 4+len;
		for(i=0; i<cnt; i++)
		{
			tx_checksum = tx_checksum + tx_buf[i];
		}
		tx_buf[i] = tx_checksum;

        //计算帧长度
        cnt = len+5;
		
        //发送数据
       // boost::asio::write(*serial_ptr_.get(),boost::asio::buffer(tx_buf,cnt),err_code_);

        size_t bytes_written = serial_.write((uint8_t*)tx_buf, cnt);
	}
}


void OriginbotBase::sendBuzzer(bool on) {
    uint8_t buzzer_data[1];
    buzzer_data[0] = on ? 1 : 0;
    sendSerialPacket(buzzer_data, 1, ID_ROS2CTR_BEEP);
}

void OriginbotBase::sendLed(bool on) {
   uint8_t led_data[1];
    led_data[0] = on ? 2 : 0;
    sendSerialPacket(led_data, 1, FRAME_ID_BUZZER_LED);
}

void OriginbotBase::sendPID(uint8_t id, float p, float i, float d) {
    uint8_t pid_data[6];
    int16_t P = p * 1000;
    int16_t I = i * 1000;
    int16_t D = d * 1000;
    
    pid_data[0] = (P >> 8) & 0xFF;
    pid_data[1] = P & 0xFF;
    pid_data[2] = (I >> 8) & 0xFF;
    pid_data[3] = I & 0xFF;
    pid_data[4] = (D >> 8) & 0xFF;
    pid_data[5] = D & 0xFF;
    sendSerialPacket(pid_data, 6, id);
}

// ========== 发送函数 ==========
void OriginbotBase::sendImuCalibrate() {
    uint8_t buf[1] = {0x01};  // MCU 协议里只要发一个字节触发校准
    sendSerialPacket(buf, 1, ID_ROS2CTR_IMU);
}

void OriginbotBase::sendLedDebug(uint8_t mode, uint8_t value) {
    uint8_t buf[2];
    buf[0] = mode;
    buf[1] = value;
    sendSerialPacket(buf, 2, ID_ROS2CTR_LGT);
}

void OriginbotBase::sendLedSave() {
    uint8_t buf[1] = {0x55};  // 固定值触发保存
    sendSerialPacket(buf, 1, ID_ROS2CTR_LST);
}

// ================= ROS 回调 =================
void OriginbotBase::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    const double linear_x = std::isfinite(msg->linear.x) ? msg->linear.x : 0.0;
    const double linear_y = std::isfinite(msg->linear.y) ? msg->linear.y : 0.0;
    const double angular_z = std::isfinite(msg->angular.z) ? msg->angular.z : 0.0;

    int16_t vx = velocityToInt16(linear_x, kMaxLinearX);
    int16_t vy = velocityToInt16(linear_y, kMaxLinearY);
    int16_t vth = velocityToInt16(angular_z, kMaxAngularZ);

    uint8_t vel_data[6];

    //数据转换
    vel_data[0] = (vx >> 8) & 0xFF;
    vel_data[1] = vx & 0xFF;
    vel_data[2] = (vy >> 8) & 0xFF;
    vel_data[3] = vy & 0xFF;
    vel_data[4] = (vth >> 8) & 0xFF;
    vel_data[5] = vth & 0xFF;

    //发送串口数据
    sendSerialPacket(vel_data, 6, ID_ROS2CTR_VEL);


    if (fabs(linear_x) > 0.0001 || fabs(angular_z) > 0.0001)
        auto_stop_count_ = 0;
}

void OriginbotBase::buzzerCallback(const std::shared_ptr<originbot_msgs::srv::OriginbotBuzzer::Request> req,
                                   std::shared_ptr<originbot_msgs::srv::OriginbotBuzzer::Response> res) {
    sendBuzzer(req->on);
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        robot_status_.buzzer_on = req->on;
    }
    res->result = true;
}

void OriginbotBase::ledCallback(const std::shared_ptr<originbot_msgs::srv::OriginbotLed::Request> req,
                                std::shared_ptr<originbot_msgs::srv::OriginbotLed::Response> res) {
    sendLed(req->on);
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        robot_status_.led_on = req->on;
    }
    res->result = true;
}

void OriginbotBase::pidLeftCallback(const std::shared_ptr<originbot_msgs::srv::OriginbotPID::Request> req,
                                    std::shared_ptr<originbot_msgs::srv::OriginbotPID::Response> res) {
    sendPID(FRAME_ID_PID_LEFT, req->p, req->i, req->d);
    res->result = true;
}

void OriginbotBase::pidRightCallback(const std::shared_ptr<originbot_msgs::srv::OriginbotPID::Request> req,
                                     std::shared_ptr<originbot_msgs::srv::OriginbotPID::Response> res) {
    sendPID(FRAME_ID_PID_RIGHT, req->p, req->i, req->d);
    res->result = true;
}
// ========== ROS Service 回调 ==========
void OriginbotBase::imuCalibCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

    (void)req;  // 不用请求内容
    sendImuCalibrate();
    res->success = true;
    res->message = "IMU calibration command sent";
}

void OriginbotBase::ledDebugCallback(
    const std::shared_ptr<originbot_msgs::srv::SetInt32::Request> req,
    std::shared_ptr<originbot_msgs::srv::SetInt32::Response> res) {

    // 简单把 req->data 拆成 mode 和 value，例如高字节/低字节
    uint8_t mode = (req->data >> 8) & 0xFF;
    uint8_t value = req->data & 0xFF;
    sendLedDebug(mode, value);
    res->success = true;
}

void OriginbotBase::ledSaveCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

    (void)req;
    sendLedSave();
    res->success = true;
    res->message = "LED parameters saved";
}

void OriginbotBase::publishOdom() {
    // 计算里程计TF四元数
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, pos_data_.angular_z);

    this->odom.header.stamp = current_time_;
    this->odom.header.frame_id = "odom";
    this->odom.child_frame_id = "base_footprint";

    this->odom.pose.pose.position.x = pos_data_.pos_x;
    this->odom.pose.pose.position.y = pos_data_.pos_y;
    this->odom.pose.pose.position.z = 0;  //高度为0
    this->odom.pose.pose.orientation.x = odom_quat.getX();
    this->odom.pose.pose.orientation.y = odom_quat.getY();
    this->odom.pose.pose.orientation.z = odom_quat.getZ();
    this->odom.pose.pose.orientation.w = odom_quat.getW();
    this->odom.twist.twist.linear.x = vel_data_.linear_x;
    this->odom.twist.twist.linear.y = vel_data_.linear_y;
    this->odom.twist.twist.angular.z = vel_data_.angular_z;

    // 始终使用轮式里程计计算得到的 yaw 作为 odom 的 orientation（不再用 IMU 覆盖）

    // 里程计协防差矩阵，用于robt_pose_ekf功能包，静止和运动使用不同的参数
    if(vel_data_.linear_x==0 && vel_data_.linear_y==0 && vel_data_.angular_z==0) {
        // 静止时保守置信，线速度很可信，角度置信适中（为轮式里程计场景调优）
        this->odom.pose.covariance = {   1e-6,    0,    0,    0,    0,    0,
                                            0, 1e-6,    0,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0,  0.1 };

        this->odom.twist.covariance = {  1e-6,    0,    0,    0,    0,    0,
                                            0, 1e-6,    0,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0,  0.1 };
    } else {
        // 运动时：线速度协方差设为中等（代表轮速/编码器噪声和滑移），角度不确定性增大
        this->odom.pose.covariance = {   0.02,    0,    0,    0,    0,    0,
                                            0, 0.02,    0,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0,  0.5 };

        this->odom.twist.covariance = {  0.02,    0,    0,    0,    0,    0,
                                            0, 0.02,    0,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0,  0.5 };
    }

    odom_pub_->publish(this->odom);

    //RCLCPP_INFO(this->get_logger(), "Odom: x=%.3f, y=%.3f, yaw=%.3f", 
      //       this->odom.pose.pose.position.x, this->odom.pose.pose.position.y, 
        //    tf2::getYaw(this->odom.pose.pose.orientation));

    // 计算里程计TF四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, pos_data_.angular_z);

    this->t.header.stamp = current_time_;
    this->t.header.frame_id = "odom";
    this->t.child_frame_id = "base_footprint";
    this->t.transform.translation.x = pos_data_.pos_x;
    this->t.transform.translation.y = pos_data_.pos_y;
    this->t.transform.translation.z = 0.0;
    this->t.transform.rotation = this->odom.pose.pose.orientation;

    // odom->base_footprint 的 TF 只由底盘发布；pub_odom 为 false 时交给外部 EKF
    if (pub_odom_) {
        tf_broadcaster_->sendTransform(this->t);
    }
}

void OriginbotBase::publishImu()
{
        sensor_msgs::msg::Imu imu_msgs_;
        imu_msgs_.header.stamp = this->now();
        imu_msgs_.header.frame_id = "imu_link";

        imu_msgs_.angular_velocity.x = imu_data_.gyro_x;
        imu_msgs_.angular_velocity.y = imu_data_.gyro_y;
        imu_msgs_.angular_velocity.z = imu_data_.gyro_z;
        imu_msgs_.linear_acceleration.x = imu_data_.acc_x;
        imu_msgs_.linear_acceleration.y = imu_data_.acc_y;
        imu_msgs_.linear_acceleration.z = imu_data_.acc_z;
        imu_msgs_.orientation.x = orient_data_.x; 
        imu_msgs_.orientation.y = orient_data_.y; 
        imu_msgs_.orientation.z = orient_data_.z;
        imu_msgs_.orientation.w = orient_data_.w;

        //不使用姿态角度
        // imu_msgs_.orientation.x = 0; 
        // imu_msgs_.orientation.y = 0; 

            //协防差矩阵
        imu_msgs_.orientation_covariance = {  1e6,    0,    0,
                                                0,  1e6,    0,
                                                0,    0, 1e-6};
        imu_msgs_.angular_velocity_covariance = { 1e6,    0,    0,
                                                    0,  1e6,    0,
                                                    0,    0, 1e-6};
        imu_msgs_.linear_acceleration_covariance = {    0,    0,    0,
                                                        0,    0,    0,
                                                        0,    0,    0};

        imu_pub_->publish(imu_msgs_);
}

// ================= 定时器 =================
void OriginbotBase::timer100ms() {
    if (auto_stop_on_ && auto_stop_count_ < 20) {
        auto_stop_count_++;
        if (auto_stop_count_ > 5) {
            auto_stop_count_ = 255;
            sendVelocity(0, 0 , 0);
        }
    }
    originbot_msgs::msg::OriginbotStatus s;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        s.battery_voltage = robot_status_.battery_voltage;
        s.buzzer_on = robot_status_.buzzer_on;
        s.led_on = robot_status_.led_on;
    }
    status_pub_->publish(s);
}

// ================= main =================
static void sigHandler(int) {
    printf("OriginBot shutdown...\n");
    rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    signal(SIGINT, sigHandler);
    rclcpp::spin(std::make_shared<OriginbotBase>("originbot_base"));
    rclcpp::shutdown();
    return 0;
}
