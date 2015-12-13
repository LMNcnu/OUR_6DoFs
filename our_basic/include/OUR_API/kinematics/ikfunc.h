/**
*@file ikfunc.h
*@brief Inverse kinematics
*
*Functions about inverse kinematics.
*
*@author Smokie Robotics, Inc. US.
*@version 1.0.0
*@date 10-09-2014
*/

#ifndef IKFUNC_H
#define IKFUNC_H

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include "../core/roadpoint.h"
#include "ikfast.h"

using namespace ikfast;
//using std::vector;
using namespace std;

//****************************************************************************
// DEFINITIONS
#define IKREAL_TYPE IkReal // for IKFast 56,61
#define POINTTOPOINT_RATIO  0.00872664 /**< Error of joint angle between the road points in radian (< 0.5 degree)*/
#define DEFAULT_POINTTOPOINT_RATIO  0.00872664 /**< Error of joint angle between the road points in radian (< 0.5 degree)*/
#define COMPUTE_CURVE_LINE_STEP 0.001745329 /**< Maximal distance of  free curve between the road points in radian (< 0.1 degree)*/
#define MAX_JOINT_STEP_FOR_JOINT 0.00518    /**< Maximal speed interval of joint angle */
#define MAX_COMPUTE_CURVE_LINE_STEP 0.01745329   /**< Maximal distance of joint control */
#define MIN_COMPUTE_CURVE_LINE_STEP 0.0001745329 /**< Minimal distance of joint control */
#define COMPUTE_LINE_STEP   0.0002          /**< Maximal distance between the road points. Unit: m*/
#define MIN_COMPUTE_LINE_STEP 0.00002     /**< Minimal distance between the road points. Unit: m*/
#define MAX_COMPUTE_LINE_STEP 0.002         /**< Maximal distance between the road points. Unit: m*/
#define JOINTMINRADIO  -2.967059678  /**< Minimal angle (-170 degree) */
#define JOINTMAXRADIO  2.967059678  /**< Maximal angle (170 degree) */

#define JOINT80_MAX_SPEED  3000    /**< 大关节速度rpm/s */
#define JOINT60_MAX_SPEED  3000    /**< 小关节速度rpm/s */

#define CONTROL_FREQUENCE  50.00      /**< 关节的控制频率 x/10000 秒*/

#define JOINT_ERROR_LIM_RAD  0.00001           /**< 关节最小误差弧度*/
#define JOINT_ERROR_LIM_ANG  0.000401070456     /**< 关节最小误差角度*/

#define JOINT60_RAD  303       /**< 小关节减速比 */
#define JOINT80_RAD  363      /**< 大关节减速比 */

#define JOINT60_MINDPI  0.00000506242    /**< 小关节分辨率 单位时rad*/
#define JOINT80_MINDPI  0.00000422584    /**< 大关节分辨率 单位时rad*/

#define RPM_TO_STEP_NUM   0.0000104719*CONTROL_FREQUENCE  /**< 速度转化成步长的转化系数 弧度步长*/

#define RELAT_COORD    0x01         /**< Relative coordinate */
#define WORLD_COORD    0x02      /**< World coordinate */

#define MIS_JOINT_DEG   0.015     /**< 关节定位角度误差*/

/**
 * @brief Moving pattern
 *
 */
enum moveway{MOV_X,MOV_Y,MOV_Z,ROT_X,ROT_Y,ROT_Z};

/**
 * @brief Functions on inverse kinematics
 *
 */
class Ikfunc
{
public:
    /**
     * @brief Default constructor
     *
     */
    explicit Ikfunc();

     /**
      * @brief Compute the solution of inverse kinematics.
      *
      * @param cur_stat Current road points
      * @param coordinate Coordinate type
      * @param move_mode moving pattern
      * @param step step length
      * @param result Resultant road point
      * @return bool Indication of successful computing
      */
     static bool ArmIk(RoadPoint &cur_stat, int coordinate, int move_mode, double step, RoadPoint &result);

     /**
      * @brief 计算这个路点是否为正常路点
      *
      * @param testpoint 测试路点
      * @return bool 这个路点是否正常
      */
     static bool ChekcArmPointok(RoadPoint testpoint);

     /**
      * @brief Compute the road point with the minimal change of joint angle.
      *
      * @param cur_stat Current road points
      * @param joint[] Resultant joint angles
      * @return bool Indication of successful computing
      */
     static bool ArmIk(RoadPoint &cur_stat, double joint[ARM_DOF]);

     /**
      * @brief Check the solution of inverse kinematics when the road points are given.
      *
      * @param cur_stat Current road points
      * @return bool Indication of successful computing
      */
     static bool ArmIk(RoadPoint &cur_stat);

     /**
      * @brief 用于检测当前位置是否具有逆解，如果没有逆解，则计算其近似解.
      *
      * @param roadpoint 待计算的解
      * @return bool 如果真实解和相似解都不存在，则返回错误
      */
     static bool ArmCheckAndCountSimilarIk(RoadPoint &cur_stat);

     /**
      * @brief Solution of inverse kinematics with the constraint of minimal distance between road points.
      *
      * @param cur_stat Current road point
      * @param joint[] Resultant joint angles
      * @param max_joint_step_range Maximal range of joint step
      * @return bool Indication of successful computing
      */
     static bool ArmJointIk(RoadPoint &cur_stat, double joint[ARM_DOF], double max_joint_step_range = DEFAULT_POINTTOPOINT_RATIO * 5);

     /**
      * @brief 基于一个原来的点的解计算一个新的点.
      *
      * @param cur_stat Current road point
      * @param joint[] Resultant joint angles
      * @param max_joint_step_range Maximal range of joint step
      * @return bool Indication of successful computing
      */
     static bool ArmJointIk(RoadPoint &cur_stat, RoadPoint &dis_stat, double max_joint_step_range = DEFAULT_POINTTOPOINT_RATIO * 5);


     /**
      * @brief Solution of inverse kinematics with the constraint of minimal distance between road points.
      *
      * @param cur_stat Current road point
      * @param dis     LineMoveDis
      * @param joint[] Resultant joint angles
      * @param max_joint_step_range Maximal range of joint step
      * @return bool Indication of successful computing
      */
     static bool ArmLineMoveIk(RoadPoint &cur_stat, const Pos &dis, double joint[ARM_DOF], double max_joint_step_range = DEFAULT_POINTTOPOINT_RATIO * 5);

     /**
      * @brief Compute the solution of forward kinematics.
      *
      * @param arm_pos Position of the end
      * @param arm_ori Orientation of the end
      * @param joint_solve[] Resultant joint angles
      * @return bool Indication of successful computing
      */
     static bool ArmFk(Pos &arm_pos, Ori &arm_ori, double joint_solve[ARM_DOF]);

     /**
      * @brief Self check of the road point.
      *
      * @param arm_pos Current road points
      * @return bool Indication of successful computing
      */
     static bool ArmFk(RoadPoint &arm_pos);

    /**
     * @brief Check the planning of straight line.
     *
     * @param s_point Starting point
     * @param e_point Ending point
     * @param speed Moving speed
     * @return int Check if the solution exists.
     * @brief                     -1: No solution;
     * @brief                     -2: The poses of two road points are different;
     * @brief                    >2: Multiple solutions exsit.
     */
    static int CountLineSize(RoadPoint s_point, RoadPoint e_point, int speed);

   /**
    * @brief Check the planning of straight line.
    *
    * @param s_point Starting point
    * @param e_point Ending point
    * @param precis Step length (1 by default, 0.001*precis, 0.001 is equal to 1 mm)
    * @param point_array Resultant road points
    * @return int Check if the solution exists.
     * @brief                     -1: No solution;
     * @brief                     -2: The poses of two road points are different;
     * @brief                    >2: Multiple solutions exsit;
     * @brief                    =1: s_point and e_point are equal.
    */
   static int CountLineToBuffer(RoadPoint s_point, RoadPoint e_point, double precis, RoadPoint *point_array);

  /**
   * @brief Check the planning of straight line.
   *
   * @param s_point Starting point in world frame
   * @param e_point Ending point in world frame
   * @param step_count Step length
   * @param point_array[] Resultant road points
   * @param speed Moving speed
   * @return bool  Indication of successful computing
   */
  static bool CountLineToBuffer(const RoadPoint s_point, const RoadPoint e_point, int step_count, RoadPoint point_array[], int speed);

  /**
   * @brief Compute the curve between road points
   *
   * @param s_point Starting point
   * @param e_point Ending point
   * @param step_count Step length
   * @param point_array[] Resultant road points
   * @return bool Indication of successful computing
   */
  static bool CountCurveLineToBuffer(RoadPoint s_point, RoadPoint e_point, int step_count, RoadPoint point_array[]);

  /**
   * @brief Compute the curve from the starting point and ending point.
   *
   * @param s_point Starting point in the world frame
   * @param e_point Ending point in the world frame
   * @param point_road Information of road points
   * @param speed Moving speed
   * @return bool Indication of successful computing
   */
  static bool CountCurvelLineToBuffer(RoadPoint s_point, RoadPoint e_point, vector<RoadPoint> *point_road, int speed);

  /**
   * @brief Compute the curve from the starting point and ending point.
   *
   * @param s_point Starting point in the world frame
   * @param e_point Ending point in the world frame
   * @param point_road Information of road points
   * @param speed  代表机械臂在运动过程中最大速度的百分比，也就是3000×speed/100
   * @param max_acc max acc of each joint
   * @return bool Indication of successful computing
   */
  static bool CountRunCurvelLineToBuffer(RoadPoint s_point, RoadPoint e_point, vector<RoadPoint> *point_road, int speed, int max_acc);

  /**
   * @brief Compute the curve from the starting point and ending point.
   *
   * @param s_point Starting point in the world frame
   * @param e_point Ending point in the world frame
   * @param point_road Information of road points
   * @param speed  如果有解，解从speed参数返回这段规划所能够使用的最大速度
   * @param max_v max speed of each joint RPM
   * @param max_acc max acc of each joint RPM/S
   * @param jerk jerk of each joint
   * @return bool Indication of successful computing
   */
  static int CountSRunCurvelLineToBuffer(RoadPoint s_point, RoadPoint e_point, vector<RoadPoint> *point_road,  float max_v, float max_acc, float max_jerk, int max_joint);

  /**
   * @brief 从四元素转换成欧拉角
   *
   * @param orientation 四元素姿态坐标
   * @param EulerAngle 转换出的欧拉角坐标 yaw pitch roll
   * @return void.
   */
   static void QuaternionToEulerAngle(Ori orientation, double EulerAngle[]);

   /**
    * @brief 欧拉角到四元素
    *
    * @param orientation 四元素姿态坐标
    * @param EulerAngle 转换出的欧拉角坐标 yaw pitch roll
    * @return void
    */
    static void EulerAngleToQuaternion(Ori &orientation, double EulerAngle[]);

    /**
     * @brief 计算两个姿态的欧拉角的差值 路点姿态2-路点姿态1
     *
     * @param roadpoint1 路点1
     * @param roadpoint2 路点2
     * @param EulerDis   欧拉角的差值
     * @return void
     */
    static void QuaternionToEulerDis(RoadPoint roadpoint1,RoadPoint roadpoint2, double EulerDis[]);

    /**
     * @brief 将一个路点的姿态旋转，差值为欧拉角度
     *
     * @param roadpoint1 路点1
     * @param roadpoint2 路点2
     * @param EulerDis   欧拉角的差值
     * @return void
     */
    static void QuaternionAddEuler(RoadPoint &roadpoint1,double EulerDis[]);

  /**
   * @brief Compute the point number on the curve trajectory.
   *
   * @param s_point Starting point in the world frame
   * @param e_point Ending point in the world frame
   * @param 代表机械臂在运动过程中最大速度的百分比，也就是3000×speed/100
   * @param max_acc max acc rmp/s
   * @return int Number of points.
   * @brief  <2: failure;
   * @brief  >=2: success
   */
  static int CountRunCurvelLineSize(RoadPoint s_point, RoadPoint e_point, int speed, int max_acc);

  /**
   * @brief Compute the curve from the starting point and ending point.
   *
   * @param s_point Starting point in the world frame
   * @param e_point Ending point in the world frame
   * @param point_road Information of road points
   * @param speed Moving speed
   * @return bool Indication of successful computing
   */
  static bool CountLineToBuffer(RoadPoint s_point, RoadPoint e_point, vector<RoadPoint> *point_road, int speed);

  /**
   * @brief Compute the point number on the curve trajectory.
   *
   * @param s_point Starting point in the world frame
   * @param e_point Ending point in the world frame
   * @param speed Moving speed
   * @return int Number of points.
   * @brief  <2: failure;
   * @brief  >=2: success
   */
  static int CountCurvelLineSize(RoadPoint s_point, RoadPoint e_point, int speed);

  /**
   * @brief Section the decimal data.
   *
   * @param num Input number
   * @return double Result withe precision of 0.000001
   */
  static double SectionNum(double num);

  /**
   * @brief 末端位置比较函数
   *
   * @param roadpoint1 第一个路点位置
   * @param roadpoint2 第二个路点位置
   * @param pos_range 点与点之间允许的距离误差
   * @param ori_range 点与点之间允许的姿态误差
   * @return 两个点之间的误差是否在误差范围内，如果true说明两个点是一个点
   */
  static bool PositionCompare(RoadPoint roadpoint1, RoadPoint roadpoint2, float pos_range, float ori_range);
  /**
   * @brief 关节位置比较函数，比较机器人是否到达相应位置
   *
   * @param roadpoint1 第一个路点位置
   * @param roadpoint2 第二个路点位置
   * @param pos_range 每个路点关节之间的允许误差
   * @return 两个点之间的误差是否在误差范围内，如果true说明两个点是一个点
   */
  static bool JointCompare(RoadPoint roadpoint1, RoadPoint roadpoint2, float joint_range);

private:
     /**
      * @brief Compute the sign of number.
      *
      * @param x Input number
      * @return double Sign of number
      */
     static double SIGN(double x);

     /**
      * @brief 检查机械臂两个相邻路点之间的步长是否符合规范
      *
      * @param point1 第一个路点位置
      * @param point2 第二个路点位置
      * @param max_joint_speed_rpm 机械臂关节的最大速度
      * 如果路点的关节的路点差能够小于关节的最大运行速度，说明路点是可以执行的
      */
     static bool CheckRobotJointStep(RoadPoint &point1, RoadPoint &point2, int max_joint_speed_rpm);

     /**
      * @brief Compute the distance
      *
      * @param a Input
      * @param b Input
      * @param c Input
      * @param d Input
      * @return double Distance
      */
     static double NORM(double a, double b, double c, double d);

     /**
      * @brief Section the number with the precision of 0.0001.
      *
      * @param num Input numebr
      * @return double Resultant value
      */
     static double SectionFourNum(double num);

     /**
      * @brief Test the pulse of each joint
      *
      * @param a[] First road point
      * @param b[] Second road point
      * @return bool If exceed the maximal angle
      */
     static bool PointAngTest(double a[ARM_DOF], double b[ARM_DOF]);

     static void CountEerotEetrans(RoadPoint &tempPoint, IKREAL_TYPE eerot[], IKREAL_TYPE eetrans[]);
     /**
      * @brief Compute the translation of the end in the world frame.
      *
      * @param end_ori Current pose (quaternion) of the end
      * @param mov_dst Distances along x-, y- and z-axis
      * @return Pos Moving distance of the end in the world frame
      */
     static Pos EndMoveXYZ(const Ori &end_ori, const Pos &mov_dst);

     /**
      * @brief 世界坐标系下面的平移
      *
      * @param ori_pos 出发点的三维姿态坐标（在世界坐标系下）
      * @param dis 在世界坐标系下需要移动的三维距离
      * @return 在世界坐标系下面的末端坐标
      */
     static Pos WorldMoveXYZ(const Pos &ori_pos, const Pos &dis);

     /**
      * @brief 计算两个末端位置的差值
      *
      * @param roadpoint1 第一个路点位置
      * @param roadpoint2 第二个路点位置
      * @return 两个路点的位置和角度差值
      */
     static RoadPoint PointDis(RoadPoint &roadpoint1, RoadPoint &roadpoint2);

     /**
      * @brief 计算两个坐标值相加的值
      *
      * @param roadpoint1 第一个路点位置
      * @param roadpoint2 需要增加的路点差值
      * @return 计算出的新的路点
      */
     static RoadPoint PointAdd(RoadPoint &roadpoint1, RoadPoint &roadpoint_dis);

     /**
      * @brief Compute the rotation of the end in the world frame.
      *
      * @param end_ori Current pose (quaternion) of the end
      * @param rel_ori Rotation relative to the x-, y- and z-axis
      * @return Ori Rotation (quaterion) of the end
      */
     static Ori EndRotXYZ(const Ori &end_ori, const Roz  &rel_ori);

      /**
       * @brief Compute the road point when the translation matrix is applied.
       *
       * @param end_pos Pose of the end (quaternion)
       * @param mov_dst Distances along x-, y- and z-axis
       * @return RoadPoint Resultant road point in the relative coordinate
       */
      static RoadPoint RelatMove(const RoadPoint &end_pos, const Pos &mov_dst);

      /**
       * @brief 计算机械臂末端移动
       *
       * @param cur_stat   当前的路点状态
       * @param coordinate 移动的方式，是针对末端坐标系还是世界坐标系
       * @param move_mode  说明移动的时轴还是角度
       * @param step       表示相对运动的距离
       * @return RoadPoint 经过移动以后输出的末端路点状态
       */
      static RoadPoint EndMove(const RoadPoint &cur_stat, int coordinate, int move_mode, double step);

      /**
       * @brief Compute the road point when the rotation matrix is applied.
       *
       * @param end_pos Pose of the end
       * @param rot_dst Angles w.r.t the coordinate of the end
       * @return RoadPoint Resultant road point
       */
      static RoadPoint RelatRot(const RoadPoint &end_pos, const Roz &rot_dst);

      /**
       * @brief 计算路径的最大运行时间.
       *
       * @param max_run_dis 运行的距离
       * @param joint_max_speed 关节的最大运行速度
       * @param joint_acc 关节的最大加速度
       * @param step_time 底层接口板的发送频率，现在时18/10000秒
       * @param radio 关节减速比
       * @return 关节轨迹运行需要发送的脉冲的次数
       */
      static int countmaxruntime(double max_run_dis, double joint_max_speed, int joint_acc, double step_time, int radio);

      /**
      * @brief 计算出关节每次的控制步长.
      *
      * @param distance  关节运动的距离
      * @param alljointruncount 关节控制次数n
      * @param max_speed_count 关节的最大运行速度
      * @param jointid 关节ID
      * @param result  存放结果的buffer
      * @param max_acc 最大加速度
      * @return void
      */
      static void getJointStepLength(float joint_distance,int alljointruncount, int max_speed_count, int joint_id, double result[], int max_acc);

      /**
       * @brief 计算轴之间的运动距离.
       *
       * @param s_point point begin
       * @param e_point point end
       * @param arm_run_joint dis
       * @return
       */
      static void countjointdis(RoadPoint s_point, RoadPoint e_point,  float arm_run_joint[]);

      /**
       * @brief caltimeslover 计算最大速度运行时间.
       *
       * @param n 运行的时间（脉冲的次数）
       * @param run_dis 运行的距离
       * @param step_time 底层接口板的发送频率，现在时18/10000秒
       * @param radio 关节减速比
       * @return 关节轨迹运行需要发送的脉冲的次数
       *         -1 无解
       */
      static int caltimeslover(int n, double run_dis, double step_time, int radio, int acc);
};

#endif // IKFUNC_H
