//author : rescuer liao
//https://github.com/rescuer-liao
//date : 2016 - 1 - 21
//Team Explorer(rescue robot)
//Team Unware (NWPU Basketball robot)
//this package get cmd_vel order and send it to handware
//add func : move in robotic  coordinate system
// add the topic "cmd_move_robot"
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
    ros::Subscriber cmd_vel_world_sub_ ;
    ros::Subscriber cmd_vel_robot_sub_ ;
    ros::Publisher robot_message_pub_ ;
private:
    void cmdWorldMoveCallBack(const geometry_msgs::TwistConstPtr &ptr) ;
    void cmdRobotMoveCallBack(const geometry_msgs::TwistConstPtr &ptr) ;
    void pubBaseCmd(const uint8_t func, const double move_x, const double move_y, const double speed_w) ;

    void brake() ; 
} ;

RobotDriver::RobotDriver(ros::NodeHandle node)
    :nh_(node),
      p_nh_("~"),
     base_cmd_id_(0x01)
{
//    p_nh_.param("wheel_center",wheel_center_,0.0) ;
//    p_nh_.param("wheel_radius",wheel_radius_,0.0) ;
    robot_message_pub_ = nh_.advertise<basketball_msgs::robot_message>("robot_cmd",1000) ;
    cmd_vel_world_sub_ = nh_.subscribe("cmd_move",1,&RobotDriver::cmdWorldMoveCallBack,this) ;
    cmd_vel_robot_sub_ = nh_.subscribe("cmd_move_robot",1,&RobotDriver::cmdRobotMoveCallBack,this) ;
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
	robot_message_pub_.publish(robot_cmd_msg) ; 
}

void RobotDriver::cmdWorldMoveCallBack(const geometry_msgs::TwistConstPtr &ptr)
{
    ROS_INFO("send cmd info\n") ; 
    pubBaseCmd(0x01,ptr->linear.x , ptr->linear.y,ptr->angular.z);
}
void RobotDriver::cmdRobotMoveCallBack(const geometry_msgs::TwistConstPtr &ptr)
{
    ROS_INFO("send cmd  info\n") ; 
    pubBaseCmd(0x05,ptr->linear.x , ptr->linear.y,ptr->angular.z);
}


int main(int argc , char **argv)
{
    ros::init(argc , argv , "robot_driver") ;
    ros::NodeHandle node ;
    RobotDriver robot_driver(node) ;
    ros::spin() ;
}
