#include "car_base/car_base.h"
#include "rclcpp/rclcpp.hpp"
#include "car_base/Quaternion_Solution.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" 
#include "car_msg/msg/data.hpp"
#include "car_msg/msg/beep.hpp"

using std::placeholders::_1;
using namespace std;
sensor_msgs::msg::Imu Mpu6050;
rclcpp::Node::SharedPtr node_handle = nullptr;


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    car_base Robot_Control;
    Robot_Control.Control();
    rclcpp::shutdown();
    return 0;
}

short car_base::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    short transition_16;
    transition_16 = 0;
    transition_16 |=  Data_High<<8;
    transition_16 |=  Data_Low;
    return transition_16;
}

float car_base::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    float data_return;
    short transition_16;
    transition_16 = 0;
    transition_16 |=  Data_High<<8;
    transition_16 |=  Data_Low;
    data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001;
    return data_return;
  }
float car_base::Joint_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    short transition_16;
    transition_16 = 0;
    transition_16 |=  Data_High<<8;
    transition_16 |=  Data_Low;
    return (transition_16/1000+(transition_16 % 1000)*0.001)*2.356;
  }

void car_base::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
    short  transition;
    Send_vel_Data.tx[0]=FRAME_HEADER; //frame head 0xAA //帧头0xAA
    Send_vel_Data.tx[1] = 0x55; //set aside //预留位
    Send_vel_Data.tx[2] = 0x0B; //set aside //数据长度
    Send_vel_Data.tx[3] = 0x50; //set aside //通信类型
    //The target velocity of the X-axis of the robot
    //机器人x轴的目标线速度
    transition=0;
    transition = twist_aux->linear.x*1000; //将浮点数放大一千倍，简化传输
    Send_vel_Data.tx[5] = transition;     //取数据的低8位
    Send_vel_Data.tx[4] = transition>>8;  //取数据的高8位

    //The target velocity of the Y-axis of the robot
    //机器人y轴的目标线速度
    transition=0;
    transition = twist_aux->linear.y*1000;
    Send_vel_Data.tx[7] = transition;
    Send_vel_Data.tx[6] = transition>>8;

    //The target angular velocity of the robot's Z axis
    //机器人z轴的目标角速度
    transition=0;
    transition = twist_aux->angular.z*1000;
    Send_vel_Data.tx[9] = transition;
    Send_vel_Data.tx[8] = transition>>8;

    Send_vel_Data.tx[10]=Check_vel_Sum(10,Send_vel_Data.tx); 

    try {
        Stm32_Serial.write(Send_vel_Data.tx,sizeof (Send_vel_Data.tx));
        // std::stringstream ss;
        // ss << "Sent data: ";
        // for (size_t i = 0; i < sizeof(Send_vel_Data.tx); ++i) {
        //     ss << std::hex << std::setw(2) << std::setfill('0') << (int)Send_vel_Data.tx[i] << " ";
        // }
        // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port"));
    }
}

void car_base::beep_Callback(const car_msg::msg::Beep::SharedPtr beep)
{
  //ROS_INFO_STREAM("ok");//ready显示状态
  short  transition;  //中间变量
  Send_beep_Data.tx[0]=FRAME_HEADER;//帧头 固定值
  Send_beep_Data.tx[1]=0x55;//帧头 固定值
  Send_beep_Data.tx[2]=0x0B;//数据长度
  Send_beep_Data.tx[3]=0x66;//通信类型
  transition=0;
  transition = beep->times*1000; //将浮点数放大一千倍，简化传输
  Send_beep_Data.tx[5] = transition;     //取数据的低8位
  Send_beep_Data.tx[4] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = beep->on_time*1000; //将浮点数放大一千倍，简化传输
  Send_beep_Data.tx[7] = transition;     //取数据的低8位
  Send_beep_Data.tx[6] = transition>>8;  //取数据的高8位

  transition=0;
  transition = beep->off_time*1000; //将浮点数放大一千倍，简化传输
  Send_beep_Data.tx[9] = transition;     //取数据的低8位
  Send_beep_Data.tx[8] = transition>>8;  //取数据的高8位

  Send_beep_Data.tx[10]=Check_beep_Sum(10,Send_beep_Data.tx);//帧尾校验位

  try
  {
  Stm32_Serial.write(Send_beep_Data.tx,sizeof (Send_beep_Data.tx)); //向串口发数据
  // RCLCPP_INFO(this->get_logger(),"send success");  
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port"));
  }
}

void car_base::arm_states_Callback(const sensor_msgs::msg::JointState::SharedPtr arm_joint)
{
  //ROS_INFO_STREAM("ok");//ready显示状态
  short  transition;  //中间变量
  Send_arm_Data.tx[0]=FRAME_HEADER;//帧头 固定值
  Send_arm_Data.tx[1]=0x55;//帧头 固定值
  Send_arm_Data.tx[2]=0x11;//数据长度
  Send_arm_Data.tx[3]=0x80;//通信类型
  transition=0;
  transition = arm_joint->position[0]*1000; //将浮点数放大一千倍，简化传输
  //ROS_INFO("%x",arm_joint.position[0]); 
  Send_arm_Data.tx[5] = transition;     //取数据的低8位
  Send_arm_Data.tx[4] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = arm_joint->position[1]*1000; //将浮点数放大一千倍，简化传输
  //ROS_INFO("%x",arm_joint.position[0]); 
  Send_arm_Data.tx[7] = transition;     //取数据的低8位
  Send_arm_Data.tx[6] = transition>>8;  //取数据的高8位

  transition=0;
  transition = arm_joint->position[2]*1000; //将浮点数放大一千倍，简化传输
  Send_arm_Data.tx[9] = transition;     //取数据的低8位
  Send_arm_Data.tx[8] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = arm_joint->position[3]*1000; //将浮点数放大一千倍，简化传输
  Send_arm_Data.tx[11] = transition;     //取数据的低8位
  Send_arm_Data.tx[10] = transition>>8;  //取数据的高8位

  transition=0;
  transition = arm_joint->position[4]*1000; //将浮点数放大一千倍，简化传输
  Send_arm_Data.tx[13] = transition;     //取数据的低8位
  Send_arm_Data.tx[12] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = arm_joint->position[5]*1000; //将浮点数放大一千倍，简化传输
  Send_arm_Data.tx[15] = transition;     //取数据的低8位
  Send_arm_Data.tx[14] = transition>>8;  //取数据的高8位


  Send_arm_Data.tx[16]=Check_arm_Sum(16,Send_arm_Data.tx);//帧尾校验位
  
  try
  {
  Stm32_Serial.write(Send_arm_Data.tx,sizeof (Send_arm_Data.tx)); //向串口发数据
  //RCLCPP_INFO(this->get_logger(),"send success");
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port"));
  }
}
/************************************************************************************************
功能：订阅回调函数，订阅逆运动学解的关节角度和动作时间，通过串口发送控制机械臂的动作
************************************************************************************************/
void car_base::ik_states_Callback(const sensor_msgs::msg::JointState::SharedPtr ik_joint)
{
  //ROS_INFO_STREAM("ok");//ready显示状态
  short  transition;  //中间变量
  Send_ik_Data.tx[0]=FRAME_HEADER;//帧头 固定值
  Send_ik_Data.tx[1]=0x55;//帧头 固定值
  Send_ik_Data.tx[2]=0x13;//数据长度
  Send_ik_Data.tx[3]=0x90;//通信类型
  transition=0;
  transition = ik_joint->position[0]*1000; //将浮点数放大一千倍，简化传输
  //ROS_INFO("%x",ik_joint.position[0]); 
  Send_ik_Data.tx[5] = transition;     //取数据的低8位
  Send_ik_Data.tx[4] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = ik_joint->position[1]*1000; //将浮点数放大一千倍，简化传输
  Send_ik_Data.tx[7] = transition;     //取数据的低8位
  Send_ik_Data.tx[6] = transition>>8;  //取数据的高8位

  transition=0;
  transition = ik_joint->position[2]*1000; //将浮点数放大一千倍，简化传输
  Send_ik_Data.tx[9] = transition;     //取数据的低8位
  Send_ik_Data.tx[8] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = ik_joint->position[3]*1000; //将浮点数放大一千倍，简化传输
  Send_ik_Data.tx[11] = transition;     //取数据的低8位
  Send_ik_Data.tx[10] = transition>>8;  //取数据的高8位

  transition=0;
  transition = ik_joint->position[4]*1000; //将浮点数放大一千倍，简化传输
  Send_ik_Data.tx[13] = transition;     //取数据的低8位
  Send_ik_Data.tx[12] = transition>>8;  //取数据的高8位
 
  transition=0;
  transition = ik_joint->position[5]*1000; //将浮点数放大一千倍，简化传输
  Send_ik_Data.tx[15] = transition;     //取数据的低8位
  Send_ik_Data.tx[14] = transition>>8;  //取数据的高8位

  transition=0;
  transition = ik_joint->position[6]; //控制时间
  Send_ik_Data.tx[17] = transition;     //取数据的低8位
  Send_ik_Data.tx[16] = transition>>8;  //取数据的高8位

  Send_ik_Data.tx[18]=Check_ik_Sum(18,Send_ik_Data.tx);//帧尾校验位

  try
  {
  Stm32_Serial.write(Send_ik_Data.tx,sizeof (Send_ik_Data.tx)); //向串口发数据
  // RCLCPP_INFO(this->get_logger(),"send success");  
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port"));
  }
}

void car_base::Publish_ImuSensor()
{
    sensor_msgs::msg::Imu Imu_Data_Pub;
    Imu_Data_Pub.header.stamp = rclcpp::Node::now();
    Imu_Data_Pub.header.frame_id = gyro_frame_id; 
                                                  
    Imu_Data_Pub.orientation.x = Mpu6050.orientation.x;
    Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;
    Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
    Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
    Imu_Data_Pub.orientation_covariance[0] = 1e6; 
    Imu_Data_Pub.orientation_covariance[4] = 1e6;
    Imu_Data_Pub.orientation_covariance[8] = 1e-6;
    Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x;
    Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
    Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
    Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
    Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x;
    Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
    Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;

    imu_publisher->publish(Imu_Data_Pub);

}

void car_base::Publish_Odom()
{
    tf2::Quaternion q;
    q.setRPY(0,0,Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat=tf2::toMsg(q);
    
    // car_msg::msg::Data robotpose;
    // car_msg::msg::Data robotvel;
    nav_msgs::msg::Odometry odom;
    
    odom.header.stamp = rclcpp::Node::now();
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = robot_frame_id;

    odom.pose.pose.position.x = Robot_Pos.X;
    odom.pose.pose.position.y = Robot_Pos.Y;

    odom.pose.pose.position.z = Robot_Pos.Z;
    odom.pose.pose.orientation = odom_quat;


    odom.twist.twist.linear.x =  Robot_Vel.X;
    odom.twist.twist.linear.y =  Robot_Vel.Y;
    odom.twist.twist.angular.z = Robot_Vel.Z; 

    // robotpose.x = Robot_Pos.X;
    // robotpose.y = Robot_Pos.Y;
    // robotpose.z = Robot_Pos.Z;

    // robotvel.x = Robot_Vel.X;
    // robotvel.y = Robot_Vel.Y;
    // robotvel.z = Robot_Vel.Z;
    memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
    memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    odom_publisher->publish(odom);
    // robotpose_publisher->publish(robotpose);
    // robotvel_publisher->publish(robotvel); 
}

void car_base::Publish_Joint_states()
{
  
  sensor_msgs::msg::JointState joint_states;

  joint_states.name.resize(15);
  joint_states.position.resize(15);
  joint_states.header.stamp = rclcpp::Node::now();

  // 为关节命名并赋值
  joint_states.name[0] = "wheel_lf_joint";
  joint_states.name[1] = "wheel_rf_joint";
  joint_states.name[2] = "wheel_lb_joint";
  joint_states.name[3] = "wheel_rb_joint";
  joint_states.name[4] = "arm_0_joint";
  joint_states.name[5] = "arm_1_joint";
  joint_states.name[6] = "arm_2_joint";
  joint_states.name[7] = "arm_3_joint";
  joint_states.name[8] = "arm_4_joint";
  joint_states.name[9] = "arm_5_1_joint";
  joint_states.name[10] = "arm_5_2_joint";
  joint_states.name[11] = "arm_5_3_joint";
  joint_states.name[12] = "arm_5_4_joint";
  joint_states.name[13] = "arm_5_5_joint";
  joint_states.name[14] = "arm_5_6_joint";

  // 示例角度值
  joint_states.position[0] = 0.0;
  joint_states.position[1] = 0.0;
  joint_states.position[2] = 0.0;
  joint_states.position[3] = 0.0;
  joint_states.position[4] = Joint_Data.joint_0; // 角度值1
  joint_states.position[5] = Joint_Data.joint_1; // 角度值2
  joint_states.position[6] = Joint_Data.joint_2; // 角度值3
  joint_states.position[7] = Joint_Data.joint_3; // 角度值4
  joint_states.position[8] = Joint_Data.joint_4; // 角度值5
  joint_states.position[9] = Joint_Data.joint_5; // 角度值1
  joint_states.position[10] = Joint_Data.joint_5; // 角度值2
  joint_states.position[11] = -Joint_Data.joint_5; // 角度值3
  joint_states.position[12] = -Joint_Data.joint_5; // 角度值4
  joint_states.position[13] = -Joint_Data.joint_5; // 角度值5
  joint_states.position[14] = Joint_Data.joint_5; // 角度值5

  joint_states_publisher->publish(joint_states);
}

void car_base::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs;
    static float Count_Voltage_Pub = 0;

    if (Count_Voltage_Pub++ > 10) {
        Count_Voltage_Pub = 0;
        voltage_msgs.data = Power_voltage;
        voltage_publisher->publish(voltage_msgs);
    }
}

unsigned char car_base::Check_vel_Sum(unsigned char Count_Number,uint8_t Data[])
{
    unsigned char check_sum = 0, k;

    for(k=0; k < Count_Number; k++) {
      check_sum = check_sum+Data[k];
    }

    return check_sum;
}

unsigned char car_base::Check_beep_Sum(unsigned char Count_Number,uint8_t Data[])
{
    unsigned char check_sum = 0, k;

    for(k=0; k < Count_Number; k++) {
      check_sum = check_sum+Data[k];
    }

    return check_sum;
}

unsigned char car_base::Check_arm_Sum(unsigned char Count_Number,uint8_t Data[])
{
    unsigned char check_sum = 0, k;

    for(k=0; k < Count_Number; k++) {
      check_sum = check_sum+Data[k];
    }

    return check_sum;
}

unsigned char car_base::Check_ik_Sum(unsigned char Count_Number,uint8_t Data[])
{
    unsigned char check_sum = 0, k;

    for(k=0; k < Count_Number; k++) {
      check_sum = check_sum+Data[k];
    }

    return check_sum;
}

unsigned char car_base::Check_rx_Sum(unsigned char Count_Number,uint8_t Data[])
{
    unsigned char check_sum = 0, k;

    for(k=0; k < Count_Number; k++) {
      check_sum = check_sum+Data[k];
    }

    return check_sum;
}

bool car_base::Get_Sensor_Data()
{
 short transition_16=0; //Intermediate variable //中间变量
  uint8_t i=0,check=0, error=1,Receive_Data_Pr[1]; //Temporary variable to save the data of the lower machine //临时变量，保存下位机数据
  static int count; //Static variable for counting //静态变量，用于计数
  try {
    if (!Stm32_Serial.isOpen()) {
      return false;
    }
    if (Stm32_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr)) != sizeof(Receive_Data_Pr)) {
      return false;
    }
  } catch (const serial::SerialException& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Serial read failed on %s: %s", usart_port_name.c_str(), e.what());
    return false;
  } catch (const serial::IOException& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Serial IO error on %s: %s", usart_port_name.c_str(), e.what());
    return false;
  }
  
  Receive_Data.rx[count] = Receive_Data_Pr[0]; //Fill the array with serial data //串口数据填入数组

  Receive_Data.Frame_Header = Receive_Data.rx[0]; //The first part of the data is the frame header 0XAA //数据的第一位是帧头0XAA
  Receive_Data.Frame_Tail = Receive_Data.rx[35];  //The last bit of data is frame tail 0X7D //数据的最后一位是帧尾0X7D

  if(Receive_Data_Pr[0] == FRAME_HEADER || count>0) //Ensure that the first data in the array is FRAME_HEADER //确保数组第一个数据为FRAME_HEADER
    count++;
  else 
  	count=0;
  if(count == 36) //Verify the length of the packet //验证数据包的长度
  {
    count=0;  //Prepare for the serial port data to be refill into the array //为串口数据重新填入数组做准备
    if(Receive_Data.Frame_Tail == FRAME_TAIL) //Verify the frame tail of the packet //验证数据包的帧尾
    {
      check=Check_rx_Sum(34,Receive_Data.rx);  //BCC check passes or two packets are interlaced //BCC校验通过或者两组数据包交错

      if(check == Receive_Data.rx[34])  
      {
        error=0;  //XOR bit check successful //异或位校验成功
      }
      if(error == 0)
      {
        
        Receive_Data.Flag_Stop=Receive_Data.rx[1]; //set aside //预留位

        //MPU6050 stands for IMU only and does not refer to a specific model. It can be either MPU6050 or MPU9250
        //Mpu6050仅代表IMU，不指代特定型号，既可以是MPU6050也可以是MPU9250
        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[2],Receive_Data.rx[3]);   //Get the X-axis acceleration of the IMU     //获取IMU的X轴加速度
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[4],Receive_Data.rx[5]);   //Get the Y-axis acceleration of the IMU     //获取IMU的Y轴加速度
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[6],Receive_Data.rx[7]);   //Get the Z-axis acceleration of the IMU     //获取IMU的Z轴加速度
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);    //Get the X-axis angular velocity of the IMU //获取IMU的X轴角速度
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]);  //Get the Y-axis angular velocity of the IMU //获取IMU的Y轴角速度
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]);  //Get the Z-axis angular velocity of the IMU //获取IMU的Z轴角速度
        //Linear acceleration unit conversion is related to the range of IMU initialization of STM32, where the range is ±2g=19.6m/s^2
        //线性加速度单位转化，和STM32的IMU初始化的时候的量程有关,这里量程±2g=19.6m/s^2
        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
        //The gyroscope unit conversion is related to the range of STM32's IMU when initialized. Here, the range of IMU's gyroscope is ±500°/s
        //Because the robot generally has a slow Z-axis speed, reducing the range can improve the accuracy
        //陀螺仪单位转化，和STM32的IMU初始化的时候的量程有关，这里IMU的陀螺仪的量程是±500°/s
        //因为机器人一般Z轴速度不快，降低量程可以提高精度
        Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.z =  Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

        // 启动陀螺零偏标定:前 GYRO_CAL_SAMPLES 帧(车须静止)累加求均值作零偏,之后每帧减掉。
        // 2D 建图下 IMU 对 yaw 无绝对参照,残余零偏(实测~0.09°/s)会无校正地积分进航向,
        // 几分钟累积十几度,致各 submap 几何扭曲。零偏每次上电不同故每次启动实测。
        if (!gyro_calibrated) {
          // 运动剔除:任一轴原始角速度过大说明车在动,重置累加,等静止窗口再标。
          if (fabs(Mpu6050.angular_velocity.x) > 0.05 ||
              fabs(Mpu6050.angular_velocity.y) > 0.05 ||
              fabs(Mpu6050.angular_velocity.z) > 0.05) {
            gyro_cal_count = 0;
            gyro_cal_sum_x = gyro_cal_sum_y = gyro_cal_sum_z = 0.0;
          } else {
            gyro_cal_sum_x += Mpu6050.angular_velocity.x;
            gyro_cal_sum_y += Mpu6050.angular_velocity.y;
            gyro_cal_sum_z += Mpu6050.angular_velocity.z;
            gyro_cal_count++;
            if (gyro_cal_count >= GYRO_CAL_SAMPLES) {
              gyro_bias_x = gyro_cal_sum_x / gyro_cal_count;
              gyro_bias_y = gyro_cal_sum_y / gyro_cal_count;
              gyro_bias_z = gyro_cal_sum_z / gyro_cal_count;
              gyro_calibrated = true;
              RCLCPP_INFO(this->get_logger(),
                "Gyro bias calibrated: x=%.5f y=%.5f z=%.5f rad/s",
                gyro_bias_x, gyro_bias_y, gyro_bias_z);
            }
          }
          // 标定期间车静止,输出 0 角速度,避免未标定的偏置喂给下游积分。
          Mpu6050.angular_velocity.x = 0.0;
          Mpu6050.angular_velocity.y = 0.0;
          Mpu6050.angular_velocity.z = 0.0;
        } else {
          Mpu6050.angular_velocity.x -= gyro_bias_x;
          Mpu6050.angular_velocity.y -= gyro_bias_y;
          Mpu6050.angular_velocity.z -= gyro_bias_z;
        }

        Robot_Vel.X = Odom_Trans(Receive_Data.rx[14],Receive_Data.rx[15]); //Get the speed of the moving chassis in the X direction //获取运动底盘X方向速度
        
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[16],Receive_Data.rx[17]); //Get the speed of the moving chassis in the Y direction, The Y speed is only valid in the omnidirectional mobile robot chassis
                                                                          //获取运动底盘Y方向速度，Y速度仅在全向移动机器人底盘有效
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[18],Receive_Data.rx[19]); //Get the speed of the moving chassis in the Z direction //获取运动底盘Z方向速度   
        
        Joint_Data.joint_0 = Joint_Trans(Receive_Data.rx[20],Receive_Data.rx[21]);
        Joint_Data.joint_1 = Joint_Trans(Receive_Data.rx[22],Receive_Data.rx[23]);
        Joint_Data.joint_2 = Joint_Trans(Receive_Data.rx[24],Receive_Data.rx[25]);
        Joint_Data.joint_3 = Joint_Trans(Receive_Data.rx[26],Receive_Data.rx[27]);
        Joint_Data.joint_4 = Joint_Trans(Receive_Data.rx[28],Receive_Data.rx[29]);
        Joint_Data.joint_5 = Joint_Trans(Receive_Data.rx[30],Receive_Data.rx[31]);

        //Get the battery voltage
        //获取电池电压
        Power_voltage = Receive_Data.rx[32];
        
        return true;

      }
      
    }

  }
  return false;
}

void car_base::Control()
{
  rclcpp::Time current_time, last_time;
  current_time = rclcpp::Node::now();
  last_time = rclcpp::Node::now();
  while(rclcpp::ok()) {
    if (true == Get_Sensor_Data()) {
      current_time = rclcpp::Node::now();
      Sampling_Time = (current_time - last_time).seconds();
      Robot_Pos.X+=1.03*(Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
      Robot_Pos.Y+=1.125*(Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
      Robot_Pos.Z+=Mpu6050.angular_velocity.z  * Sampling_Time;

      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);
      Publish_Voltage();
      Publish_Odom();
      Publish_ImuSensor();
      Publish_Joint_states();
      last_time = current_time;
    }
    rclcpp::spin_some(this->get_node_base_interface());
  }
}

car_base::car_base()
: rclcpp::Node ("car_base")
{
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_vel_Data, 0, sizeof(Send_vel_Data));
  memset(&Send_beep_Data, 0, sizeof(Send_beep_Data));
  memset(&Send_arm_Data, 0, sizeof(Send_arm_Data));
  memset(&Send_ik_Data, 0, sizeof(Send_ik_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  this->declare_parameter<int>("serial_baud_rate", 115200);
  this->declare_parameter<std::string>("usart_port_name", "/dev/ttyS9");
  this->declare_parameter<std::string>("cmd_vel", "cmd_vel");
  this->declare_parameter<std::string>("beep_states", "beep_states");
  this->declare_parameter<std::string>("ik_states", "ik_states");
  this->declare_parameter<std::string>("arm_states", "arm_states");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_link");
  this->declare_parameter<std::string>("gyro_frame_id", "imu_link");

  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("cmd_vel", cmd_vel);
  this->get_parameter("beep_states", beep_states);
  this->get_parameter("ik_states", ik_states);
  this->get_parameter("arm_states", arm_states);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);

  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

  joint_states_publisher = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 10);

  // robotpose_publisher = create_publisher<car_msg::msg::Data>("robotpose", 10);

  // robotvel_publisher = create_publisher<car_msg::msg::Data>("robotvel", 10);

  tf_bro = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(cmd_vel, 1, std::bind(&car_base::Cmd_Vel_Callback, this, _1));
  arm_teleop_Sub = create_subscription<sensor_msgs::msg::JointState>(arm_states, 1, std::bind(&car_base::arm_states_Callback, this, _1));
  ik_teleop_Sub = create_subscription<sensor_msgs::msg::JointState>(ik_states, 1, std::bind(&car_base::ik_states_Callback, this, _1));
  beep_Sub = create_subscription<car_msg::msg::Beep>(beep_states, 1, std::bind(&car_base::beep_Callback, this, _1));
  try  {
    Stm32_Serial.setPort(usart_port_name);
    Stm32_Serial.setBaudrate(serial_baud_rate);
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open();
  } catch (serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(),"car_base can not open serial port,Please check the serial port cable! ");
  }
  if(Stm32_Serial.isOpen()) {
    RCLCPP_INFO(this->get_logger(),"car_base serial port opened");
  }
}

car_base::~car_base()
{
  //Sends the stop motion command to the lower machine before the turn_on_robot object ends
  //对象turn_on_robot结束前向下位机发送停止运动命令
  Send_vel_Data.tx[0]=FRAME_HEADER;
  Send_vel_Data.tx[1] = 0x55;
  Send_vel_Data.tx[2] = 0x0B;
  Send_vel_Data.tx[3] = 0x50;
  //The target velocity of the X-axis of the robot //机器人X轴的目标线速度 
  Send_vel_Data.tx[5] = 0;
  Send_vel_Data.tx[4] = 0;

  //The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度 
  Send_vel_Data.tx[7] = 0;
  Send_vel_Data.tx[6] = 0;
  
  //The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度 
  Send_vel_Data.tx[9] = 0;
  Send_vel_Data.tx[8] = 0;
  Send_vel_Data.tx[10]=Check_vel_Sum(10,Send_vel_Data.tx); //Check the bits //校验位
  try
  {
    Stm32_Serial.write(Send_vel_Data.tx,sizeof (Send_vel_Data.tx)); //Send data to the serial port //向串口发数据  
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port"));//If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
  }
  Stm32_Serial.close(); //Close the serial port //关闭串口
  RCLCPP_INFO(this->get_logger(),"Shutting down");
}


