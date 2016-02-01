#ifndef JOINTCONTROLBASE_H
#define JOINTCONTROLBASE_H
#include <fcntl.h>
#include "jointcmd.h"

//****************************************************************************
// DEFINES

#define JOINT_ANGLE		0x01                        /**< Use degree */
#define JOINT_RADIAN  	0x02                        /**< Use radian */
#define JOINT_UNITS     0x03                        /**< Use reading of encoder */
#define SMALL_MODULAR   0x00                        /**< Small module */
#define BIG_MODULAR     0x01                        /**< Large module*/
#define MOTOR_SPEED     0x00                        /**< Define angle velocity of motor */
#define JOINT_SPEED     0x01                        /**< Define angle velocity of joint */

#define JOINT_CONTROL_MINSTEP  5                    /**< Minimal control period of joint. Unit: ms */
#define JOINT_CONTROL_MAXSTEP  40                   /**< Maximal control period of joint. Unit: ms */

//#define JOINT_MAX_POS       170.0/180*M_PI          /**< Maximal position of joint */
//#define JOINT_MIN_POS       -170.0/180*M_PI         /**< Minimal position of joint */

#define RADIAN_CONTROL_STEP  0.1*M_PI/180.0         /**< Control step in radian, 0.1 degree*/
#define RELATE_RADIAN_CONTROL_STEP  0.01*M_PI/180.0 /**< Control step in radian relatively */
#define POS_MOVESTEP_PERCISION 0.0001               /**< Position precision of moving step */

#define DEVICE_NODE_NAME_LEN 32                     /**< Device node name length*/
//****************************************************************************


class JointControlBase
{
public:
    JointControlBase();


    /**
     * @brief Initialize the bus.
     *
     * @param szDeviceNode Device node
     * @return bool Indication of successful initialization
     */
    virtual bool JointControlInit(const char* szDeviceNode);

    /**
     * @brief Close the bus.
     *
     * @return bool Indication of successful operation
     */
    virtual bool JointControlUninit(void);



public:
    /**
     * @brief Read the error code of joint.
     *
     */
    virtual int   readJointError(int joint)=0;

    /**
     * @brief Read the current of driver.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Current of joint (mA)
     */
    virtual int   readCurrentI(int joint)=0;

    /**
     * @brief Read the motor speed of joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Speed, rpm
     */
    virtual int   readSpeed(int joint, u_int16_t jointType)=0;

    /**
     * @brief Read the parameter P in the velocity loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter P in the velocity loop
     */
    virtual int   readSpeedP(int joint)=0;

    /**
     * @brief Read the parameter I in the velocity loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter I in the velocity loop
     */
    virtual int   readSpeedI(int joint)=0;

    /**
     * @brief Read the parameter D in the velocity loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter D in the velocity loop
     */
    virtual int   readSpeedD(int joint)=0;

    /**
     * @brief Read the dead zone of speed.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return float Dead zone of speed. Unit: rpm
     */
    virtual float readSpeedDS(int joint)=0;

    /**
     * @brief Read the position of joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @param return float Position of joint in radian
     */
    virtual float readPosJ(int joint, u_int16_t jointType)=0;

    /**
     * @brief Read the output of potentiometer.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Output of potentiometer
     */
    virtual int   readPotenVal(int joint)=0;


    /**
     * @brief Obtain the zero drift.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Zero drift in radian
     */
    virtual float readZeroPosOffset(int joint, u_int16_t jointType)=0;

    /**
     * @brief Read the current voltage of joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return float Voltage of joint. Unit: mV
     */
    virtual float readJointVal(int joint)=0;

    /**
     * @brief Obtain the current temprature of joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return float temprature of joint
     */
    virtual float readJointTemp(int joint)=0;

    /**
     * @brief Obtain the target current.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Target current
     */
    virtual int   readTagCurI(int joint)=0;

    /**
     * @brief Obtain the target speed of motor.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Target speed of motor. Unit: rpm
     */
    virtual float readTagSpeedMoto(int joint, u_int16_t jointType)=0;

    /**
     * @brief Read the target position.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Target position in radian
     */
    virtual float readTagPos(int joint, u_int16_t jointType)=0;

    /**
     * @brief Obtain the maximal current.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Maximal current
     */
    virtual int   readMaxCurI(int joint)=0;

    /**
     * @brief Obtain the maximal speed of motor
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Maximal speed of motor. Unit: rpm
     */
    virtual float readMaxSpeedMoto(int joint, u_int16_t jointType)=0;

    /**
     * @brief Obtain the minimal position limit.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Minimal position limit in radian
     */
    virtual float readLimMinPosRadio(int joint, u_int16_t jointType)=0;

    /**
     * @brief Obtain the maximal position limit.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Maximal position limit in radian
     */
    virtual float readLimMaxPosRadio(int joint, u_int16_t jointType)=0;

    /**
     * @brief Obtain the parameter P in the current loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter P in the current loop
     */
    virtual int   readCurrentIP(int joint)=0;

    /**
     * @brief Obtain the parameter I in the current loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter I in the current loop
     */
    virtual int   readCurrentII(int joint)=0;

    /**
     * @brief Obtain the parameter D in the current loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter D in the current loop
     */
    virtual int   readCurrentID(int joint)=0;

    /**
     * @brief Obtain the parameter P in the position loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter P in the position loop
     */
    virtual int   readPosP(int joint)=0;

    /**
     * @brief Obtain the parameter I in the position loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter I in the position loop
     */
    virtual int   readPosI(int joint)=0;

    /**
     * @brief Obtain the parameter D in the position loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter D in the position loop
     */
    virtual int   readPosD(int joint)=0;

    /**
     * @brief Obtain the dead zone of position.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Dead zone of position
     */
    virtual int   readPosDS(int joint)=0;

    /**
     * @brief Set the current.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param I Assigned current
     * @return bool Indication of successful setting
     */
    virtual bool setCurrentI(int joint, int I)=0;

    /**
     * @brief Set the zero drift.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param zeroPosRadio Zero drift
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @param writeype CMDTYPE_WR or CMDTYPE_WR_NR
     * @return bool Indication of successful setting
     */
    virtual bool setZeroPosOffset(int joint, float zeroPos, u_int16_t jointType, u_int8_t jointWMode)=0;

    /**
     * @brief Set the target position.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param radio Target position in radian
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @param writeype CMDTYPE_WR or CMDTYPE_WR_NR
     * @return bool Indication of successful setting
     */
    virtual bool setTagPosRadio(int joint, float radio, u_int16_t jointType, u_int8_t jointWMode)=0;

    /**
     * @brief Set the target position.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param angle Target position in degree
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @param writeype CMDTYPE_WR or CMDTYPE_WR_NR
     * @return bool Indication of successful setting
     */
    virtual bool setTagPosAngle(int joint, float angle, u_int16_t jointType, u_int8_t jointWMode)=0;

    /**
     * @brief Set the maximal speed of motor.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param speedMoto maximal speed of motor
     * @return bool Indication of successful setting
     */
    virtual bool setMaxSpeedMoto(int joint, int speedMoto)=0;

    /**
     * @brief Set the maximal acceleration of motor.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param acc Maximal acceleration of motor. Unit: rmp/s
     * @return bool Indication of successful setting
     */
    virtual bool setMaxACC(int joint, int acc)=0;

    /**
     * @brief Set the maximal current.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param I Maximal current
     * @return bool Indication of successful setting
     */
    virtual bool setMaxCurI(int joint, int I)=0;

	 /**
     *  @brief send radio target position in radian
     *  @return bool the success or failure
     */
	virtual bool sendRobotControlCmd(void)=0;

public:
    /**
     * @brief Check if bus is working.
     *
     * @return bool Indication of open state
     */
    bool JointBusOpened(void) { return m_busOpened; }

protected:
    bool m_busOpened;
    char m_szDeviceNode[DEVICE_NODE_NAME_LEN+1];
};

#endif // JOINTCONTROLBASE_H
