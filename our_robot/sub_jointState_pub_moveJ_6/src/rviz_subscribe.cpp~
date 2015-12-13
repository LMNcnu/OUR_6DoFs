#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "config.h"
#include <moveit/move_group_interface/move_group.h>
//#include <jointcontrolcanbus.h>


//JointControlCanBus *jointContorlHandler = new JointControlCanBus();
 std::vector<double> group_variable_values;
moveit::planning_interface::MoveGroup *ptr_group;//指定一个MoveGroup组
moveit::planning_interface::MoveGroup::Plan my_plan;
int count=0;
void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{ 

  ROS_INFO("[%f,%f,%f,%f,%f,%f]",
           msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);
  for(int i=0; i<6; i++)
  {
    group_variable_values[i] = msg->data[i];
  }
  ptr_group->setJointValueTarget(group_variable_values);
ptr_group->asyncMove();
 //ptr_group->plan(my_plan);会进程阻塞
 // bool success = ptr_group->execute(my_plan);

  //ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
if(count==0)
{
//moveit::planning_interface::MoveGroup group("joint_group");//指定一个MoveGroup组
count=1;
}

//移动real robot

 // bool isSent = jointContorlHandler->setTagPosRadio(0x05, msg->data[5], MODEL_TYPE_J60, CMDTYPE_WR_NR);
 // ROS_INFO("Sent successfully? %d", isSent);


//    for(int i=0; i < ARM_DOF/2; ++i)
//    {
//        jointContorlHandler->setTagPosRadio(i+1, msg->data[i], MODEL_TYPE_J80, CMDTYPE_WR_NR);
//    }

//    for(int i=ARM_DOF/2; i < ARM_DOF; ++i)
//    {
//        jointContorlHandler->setTagPosRadio(i+1, msg->data[i], MODEL_TYPE_J60, CMDTYPE_WR_NR);
//    }
}



int main(int argc, char **argv)
{
//  jointContorlHandler->JointControlInit(DEFAULT_NODE);
  
  ros::init(argc, argv, "rviz_subscribe");

  moveit::planning_interface::MoveGroup group("joint_group");//指定一个MoveGroup组
  ptr_group = &group;
  ptr_group->getCurrentState()->copyJointGroupPositions(ptr_group->getCurrentState()->getRobotModel()->getJointModelGroup(ptr_group->getName()), group_variable_values);

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("moveJ", 1000, chatterCallback);
/*//while(ros::ok())会阻塞在这不会执行下面的 ros::spin();订阅也就不会接收数据了

{ 
  group.setJointValueTarget(group_variable_values);
 // bool success = group.plan(my_plan);

 // ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
}*/
 //int err = pthread_create(&tid, NULL, thread_caller, NULL);
 ros::spin(); 


  return 0;
}
