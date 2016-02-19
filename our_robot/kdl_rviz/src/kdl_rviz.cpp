#include <moveit/move_group_interface/move_group.h>
#include <math.h>
//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>
#include  <std_msgs/Float32MultiArray.h>//define joints
#include <iostream>
using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "our_kdl_rviz");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(20);
  spinner.start();
  
  sleep(10.0);//等rviz起来，如果没在一个lanuch中，可以省去这个

  moveit::planning_interface::MoveGroup group("joint_group");

  // Getting Basic Information
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());//打印坐标系
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());//打印末端链节

/*定义关节组位置数组，并得到当前位置，修改某些关节值，然后规划显示*/
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
   
  group_variable_values[0] = 0;
  group_variable_values[1] = 20*M_PI/180.0;
  group_variable_values[2] = -70*M_PI/180.0;
  group_variable_values[3] = 0;
  group_variable_values[4] = -90*M_PI/180.0;
  group_variable_values[5] = 0;
  group.setJointValueTarget(group_variable_values);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);


  ROS_INFO("Visualizing plan 1 (joint space goal) %s",success?"":"FAILED");
  

  //group.execute(my_plan);

  /*publish the point's position*/
  std_msgs::Float32MultiArray joints;
  joints.data.resize(6);

  ros::NodeHandle nh;
  ros::Publisher command_pub = nh.advertise<std_msgs::Float32MultiArray> ("moveJ", 1000);
  ros::Rate loop_rate(20);//Hz
  int j =0;
  while(ros::ok())
  {
      if( j != my_plan.trajectory_.joint_trajectory.points.size()) //or return and again
      {
          for(j=0; j<my_plan.trajectory_.joint_trajectory.points.size(); j++)
          {
              for(int i=0; i<6; i++)
             {
                  joints.data[i]=my_plan.trajectory_.joint_trajectory.points[j].positions[i];
                  ROS_INFO("send ponits[%d]positions[%d] %f",j,i,joints.data[i]);
             }
              command_pub.publish(joints);
              loop_rate.sleep();
          }
      }
  }

  ros::shutdown();  
  return 0;

 

}
