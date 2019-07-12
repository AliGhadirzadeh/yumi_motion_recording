#include <ros/ros.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <sstream>
#include <vector>
#include <string>

#include <iostream>
#include <fstream>
#include <math.h>

using namespace std;

KDL::JntArray right_arm_joint_positions;
KDL::JntArray left_arm_joint_positions;
vector<double> right_arm_joint_velocity;
vector<double> left_arm_joint_velocity;

// function declarations
void print_joint_values();

void joint_state_callback(const sensor_msgs::JointState & msg)
{
    int right_arm_indecis[7] = {1,3,13,5,7,9,11};
    int left_arm_indecis[7] = {0,2,12,4,6,8,10};
    for(int i = 0;i < 7; i++)
    {
        right_arm_joint_positions(i) = msg.position[right_arm_indecis[i]];
        left_arm_joint_positions(i) = msg.position[left_arm_indecis[i]];
        right_arm_joint_velocity[i] = msg.velocity[right_arm_indecis[i]];
        left_arm_joint_velocity[i] = msg.velocity[left_arm_indecis[i]];
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "record_motion_node"); // init the ROS node

  right_arm_joint_positions.resize(7);
  right_arm_joint_velocity.resize(7);
  left_arm_joint_positions.resize(7);
  left_arm_joint_velocity.resize(7);

  ros::NodeHandle joint_node;
  ros::Subscriber joint_subscriber = joint_node.subscribe("/yumi/joint_states", 1, joint_state_callback);

  srand( time(NULL) );
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (argc < 4)
  {
    cout << "Error! following arguments are missing:" << endl;
    cout << "\t filename (string): the filename to save the data" << endl;
    cout << "\t sampling rate (double): the sample rate to collect the data" << endl;
    cout << "\t duration (double): the run duration of the node" << endl;
    exit(1);
  }

  string filename = argv[1];
  double sampling_period = 1.0 / atof(argv[2]);
  double max_run_duration = atof(argv[3]);

  KDLWrapper right_arm_kdl_wrapper;
  KDLWrapper left_arm_kdl_wrapper;

  if(!right_arm_kdl_wrapper.init("yumi_body", "yumi_link_7_r"))
      ROS_ERROR("Error initiliazing right_arm_kdl_wrapper");
  if(!left_arm_kdl_wrapper.init("yumi_body", "yumi_link_7_l"))
      ROS_ERROR("Error initiliazing left_arm_kdl_wrapper");

  KDL::Frame right_tool_tip_frame, left_tool_tip_frame;
  std::ofstream fwriter;
  fwriter.open (filename.c_str());
  fwriter << "TIME,";
  for (int i = 0; i <7; i++){
    fwriter << "R-JOINT-P" << (i+1) << ",";
    fwriter << "L-JOINT-P" << (i+1) << ",";
  }
  for (int i = 0; i <7; i++){
    fwriter << "R-JOINT-V" << (i+1) << ",";
    fwriter << "L-JOINT-V" << (i+1) << ",";
  }
  for (int i = 0; i <3; i++)
  {
    fwriter << "R-CART"<< (i+1) << ",";
    fwriter << "L-CART"<< (i+1) << ",";
  }
  fwriter << endl;

  sleep(1);

  double current_time, start_time, time_to_log, time_to_print;
  double time_to_calc = 0;
  double time_to_save = 5.0;
  start_time = ros::Time::now().toSec();
  time_to_log = 0;
  time_to_print = 0;

  while (ros::ok()) {
    current_time = ros::Time::now().toSec() - start_time;

    if (current_time > max_run_duration)
      break;

    if (current_time > time_to_calc)
    {
      right_arm_kdl_wrapper.fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
      left_arm_kdl_wrapper.fk_solver_pos->JntToCart(left_arm_joint_positions, left_tool_tip_frame, -1);
      time_to_calc = time_to_log + sampling_period - 0.001; // 1msec before the next log
    }

    if(current_time > time_to_log)
    {
      fwriter << current_time << ',';
      for (int i = 0; i <7; i++){
        fwriter << right_arm_joint_positions(i) << ",";
        fwriter << left_arm_joint_positions(i) << ",";
      }
      for (int i = 0; i <7; i++){
        fwriter << right_arm_joint_velocity[i] << ",";
        fwriter << left_arm_joint_velocity[i] << ",";
      }
      for (int i = 0; i <3; i++)
      {
        fwriter <<right_tool_tip_frame.p(i) << ",";
        fwriter <<left_tool_tip_frame.p(i) << ",";
      }
      fwriter << endl;

      time_to_log = current_time + sampling_period;
    }

    if(current_time > time_to_print)
    {
      print_joint_values();
      time_to_print = current_time + 1.0;
    }

    if(current_time > time_to_save)
    {
      fwriter.close();
      fwriter.open(filename.c_str(), ios::app);
      time_to_save = current_time + 5.0;
    }

  }

  fwriter.close();
  std::cout << "Program successfully done!" << std::endl;
  return 0;
}


void print_joint_values()
{
  std::cout << std::setprecision(3);
  std::cout << "RIGHT ARM JOINTS: ";
  for(int i = 0; i < 7; i++)
  {
    if (right_arm_joint_positions(i) >= 0)
      std::cout << " ";
    std::cout << right_arm_joint_positions(i) << ", ";
  }
  std::cout << std::endl;
  std::cout << "LEFT ARM JOINTS:  ";
  for(int i = 0; i < 7; i++)
  {
    if (left_arm_joint_positions(i) >= 0)
      std::cout << " ";
    std::cout << left_arm_joint_positions(i) << ", ";
  }
  std::cout << std::endl;
}
