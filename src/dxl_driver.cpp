#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>                                  // Uses Dynamixel SDK library
#include <string>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

class DxlServoDriver
{
 public:
 	DxlServoDriver(ros::NodeHandle nh, std::string portName, int baudrate, int num, char** actuators_name)
      : nh_(nh), rate_(20), port_(portName, baudrate), loop_(0)
   {
   	for (int i = 0; i < num; i++) {
      boost::shared_ptr<DxlServo> actuator(new DxlServo(std::string(actuators_name[i])));
      actuator_vector_.push_back(actuator);
      joint_states_.name.push_back(std::string(actuators_name[i]));
    }

    joint_states_.position.resize(num);
    joint_states_.velocity.resize(num);
    joint_states_.effort.resize(num);

    /*for (int i = 0; i < num; i++) {
      actuator_vector_[i]->setFreePosMode(&port_);
      usleep(10000);
      actuator_vector_[i]->setNormalPosMode(&port_);
      usleep(10000);
      actuator_vector_[i]->setTrajectoryMode(&port_, TRAJECTORY_EVEN_MODE);
      usleep(10000);
      actuator_vector_[i]->setGainParam(&port_, 0x00);
      usleep(10000);
    }*/

    angles_.resize(actuator_vector_.size());
    multi_ctrl_ = new DxlServoMultiCtrl(actuator_vector_);

    ros::NodeHandle n("~");
    joint_cmd_sub_ = nh_.subscribe<sensor_msgs::JointState>(n.param<std::string>("joint_cmd_topic_name", "/joint_pos_cmd"), 1, boost::bind(&DxlServoDriver::jointPositionCallback, this, _1));
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(n.param<std::string>("joint_state_topic_name", "/joint_states"), 10);

   }

 private:
  std::vector<boost::shared_ptr<DxlServo> > actuator_vector_;
  DxlServoMultiCtrl *multi_ctrl_;
  SerialPort port_;
  ros::NodeHandle nh_;
  ros::Rate rate_;
  ros::Subscriber joint_cmd_sub_;
  int loop_;
  std::vector<short> angles_;

  ros::Publisher joint_state_pub_;
  sensor_msgs::JointState joint_states_;

};

void jointPositionCallback(const sensor_msgs::JointStateConstPtr& joint_pos_cmd)
  {
    short target_time = 0;
    double angle_deg = 0;
    for(int i = 0; i < actuator_vector_.size(); ++i)
    {
      angle_deg = (joint_pos_cmd->position[i]*180.0)/M_PI;
      angles_[i] = (short)(angle_deg * 1000);
    }
    multi_ctrl_->setPositionMulti(&port_, angles_, target_time);
    usleep(10000);
  }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_driver");
  ros::NodeHandle nh;

  DxlServoDriver driver(nh, "/dev/ttyUSB0", B55700, argc-1, &argv[1]);
  driver.run();

  return 0;
}
