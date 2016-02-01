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


 group.stop();//don't research the meanning.
//  group.asyncExecute(my_plan);
  sleep(5.0); //Sleep to give Rviz time to visualize the plan.  

  ros::shutdown();  
  return 0;

  //moveit_msgs::DisplayTrajectory d;

//my_plan.trajectory_.joint_trajectory.points.


/*

//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>

//设置一个末端执行器的位置（这种方式就是表示末端位置？）
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;
  group.setPoseTarget(target_pose1);


//规划路径并在rviz中显示
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);//自动显示

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  
  sleep(5.0);

// 手动再显示一次,首先创建一个publisher在rviz中显示规划.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  if (1)
  {
    ROS_INFO("Visualizing plan 1 (again)");    
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);

    sleep(5.0);
  }*/

}
