/*
*Team Unware Basketball Robot NWPU
*
*接收其他节点的移动指令，转化为下位机协议，并转发给串口节点，发给下位机控制移动
*get data_type : geometry_msgs/Twist
*out data_type : basketball_msgs::robot_message
*
*Author = liao-zhihan
*
*first_debug_date:2016-01-20
*测试通过
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <basketball_msgs/robot_message.h>

class RobotDriver
{
public:
    RobotDriver(ros::NodeHandle node) ;
    ~RobotDriver() ;
protected:
private:
    ros::NodeHandle p_nh_ ;
    ros::NodeHandle nh_ ;
    double publish_rate_ ;
    double wheel_center_ ;
    double wheel_radius_ ;
    uint8_t base_cmd_id_ ;
    ros::Subscriber cmd_vel_sub_ ;
    ros::Publisher robot_message_pub_ ;
private:
    //消息回调函数
    void cmdMoveCallBack(const geometry_msgs::TwistConstPtr &ptr) ;
    //速度发布接口
    void pubBaseCmd(const uint8_t func, const double move_x, const double move_y, const double speed_w) ;
    //急停接口
    void brake() ;
} ;

RobotDriver::RobotDriver(ros::NodeHandle node)
    :nh_(node),
      p_nh_("~"),
     base_cmd_id_(0x01)
{
//    p_nh_.param("wheel_center",wheel_center_,0.0) ;
//    p_nh_.param("wheel_radius",wheel_radius_,0.0) ;
    robot_message_pub_ = nh_.advertise<basketball_msgs::robot_message>("robot_cmd",10) ;
    cmd_vel_sub_ = nh_.subscribe("cmd_move",1,&RobotDriver::cmdMoveCallBack,this) ;
}


RobotDriver::~RobotDriver()
{
    brake() ;
    nh_.shutdown();
}

void RobotDriver::brake()
{
	pubBaseCmd(0x01 , 0 , 0 , 0) ; 
}

void RobotDriver::pubBaseCmd(const uint8_t func, const double move_x, const double move_y, const double speed_w)
{
  //进行上下位机协议转换
	basketball_msgs::robot_message robot_cmd_msg ;
	robot_cmd_msg.data.resize(18 , 0) ;
	uint8_t *data_ptr = robot_cmd_msg.data.data() ;
	int data_len = 13 ;
	data_ptr[0] = data_ptr[1] = 0xff ;
	data_ptr[2] = base_cmd_id_ ;
	data_ptr[3] = (u_int8_t)(data_len>>8) ;
  data_ptr[4] = (u_int8_t)(data_len & 0xff) ;
  data_ptr[5] = func ;
	*(float *)(data_ptr+6)  = move_x;
  *(float *)(data_ptr+10)  = move_y;
  *(float *)(data_ptr+14)  = speed_w;
  //协议转换完成，向下位机发送指令
	robot_message_pub_.publish(robot_cmd_msg) ;
}

void RobotDriver::cmdMoveCallBack(const geometry_msgs::TwistConstPtr &ptr)
{
    ROS_INFO("send cmd info\n") ;
    pubBaseCmd(0x01,ptr->linear.x , ptr->linear.y,ptr->angular.z);
}



int main(int argc , char **argv)
{
    ros::init(argc , argv , "robot_driver") ;
    ros::NodeHandle node ;
    RobotDriver robot_driver(node) ;
    ros::spin() ;
}
