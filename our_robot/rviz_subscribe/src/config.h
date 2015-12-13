/**
*@file config.h
*@brief Global configuration
*
*Configure the communication node and number of DoFs.
*
*@author Smokie Robotics, Inc. US.
*@version 1.0.0
*@date 10-09-2014
*/

#ifndef CONFIG_H_
#define CONFIG_H_

#define DEFAULT_NODE "/dev/pcan0"  /**< Definition of CAN bus node*/

#define CAN_READ_TIMEOUT    5000  /**< Define the time delay max for CAN bus reading*/

#define ARM_DOF 6 /**< Number of DoFs */

#define ARM_RUN_TRACKFRQ    200        /**< hz */

#define JOINT_RUN_TRACKFRQ  200        /**< hz */

#define READ_ROBOTMSG_FRQ   40         /**< 25ms 40hz */

#define ERROR_CHECK_FRQ     40         /**< 40hz */

#define MIN_CONTROL_TIMER   5          /**< 5ms */

#define COMMUNICATE_TYPE    0          /**< 0---CAN 1---MAC */

//init position
#define ARM_INIT_JOINT_0 102.67
#define ARM_INIT_JOINT_1 60.79
#define ARM_INIT_JOINT_2 -64.91
#define ARM_INIT_JOINT_3 -123.47
#define ARM_INIT_JOINT_4 -88.58
#define ARM_INIT_JOINT_5 -5.38

#endif //CONFIG_H_
