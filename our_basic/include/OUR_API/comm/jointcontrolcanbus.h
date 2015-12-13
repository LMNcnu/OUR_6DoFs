#ifndef JOINTCONTROLCANBUS_H
#define JOINTCONTROLCANBUS_H
#include "jointcontrolbase.h"

class JointControlCanBus : public JointControlBase
{
public:
    JointControlCanBus();
    ~JointControlCanBus();

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
    virtual int   readJointError(int joint);

    /**
     * @brief Read the current of driver.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Current of joint (mA)
     */
    virtual int   readCurrentI(int joint);

    /**
     * @brief Read the motor speed of joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Speed, rpm
     */
    virtual int   readSpeed(int joint, u_int16_t jointType);

    /**
     * @brief Read the parameter P in the velocity loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter P in the velocity loop
     */
    virtual int   readSpeedP(int joint);

    /**
     * @brief Read the parameter I in the velocity loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter I in the velocity loop
     */
    virtual int   readSpeedI(int joint);

    /**
     * @brief Read the parameter D in the velocity loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter D in the velocity loop
     */
    virtual int   readSpeedD(int joint);

    /**
     * @brief Read the dead zone of speed.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return float Dead zone of speed. Unit: rpm
     */
    virtual float readSpeedDS(int joint);

    /**
     * @brief Read the position of joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @param return float Position of joint in radian
     */
    virtual float readPosJ(int joint, u_int16_t jointType);

    /**
     * @brief Read the output of potentiometer.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Output of potentiometer
     */
    virtual int   readPotenVal(int joint);


    /**
     * @brief Obtain the zero drift.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Zero drift in radian
     */
    virtual float readZeroPosOffset(int joint, u_int16_t jointType);

    /**
     * @brief Read the current voltage of joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return float Voltage of joint. Unit: mV
     */
    virtual float readJointVal(int joint);

    /**
     * @brief Obtain the current temprature of joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return float temprature of joint
     */
    virtual float readJointTemp(int joint);

    /**
     * @brief Obtain the target current.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Target current
     */
    virtual int   readTagCurI(int joint);

    /**
     * @brief Obtain the target speed of motor.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Target speed of motor. Unit: rpm
     */
    virtual float readTagSpeedMoto(int joint, u_int16_t jointType);

    /**
     * @brief Read the target position.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Target position in radian
     */
    virtual float readTagPos(int joint, u_int16_t jointType);

    /**
     * @brief Obtain the maximal current.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Maximal current
     */
    virtual int   readMaxCurI(int joint);

    /**
     * @brief Obtain the maximal speed of motor
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Maximal speed of motor. Unit: rpm
     */
    virtual float readMaxSpeedMoto(int joint, u_int16_t jointType);

    /**
     * @brief Obtain the minimal position limit.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Minimal position limit in radian
     */
    virtual float readLimMinPosRadio(int joint, u_int16_t jointType);

    /**
     * @brief Obtain the maximal position limit.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Maximal position limit in radian
     */
    virtual float readLimMaxPosRadio(int joint, u_int16_t jointType);

    /**
     * @brief Obtain the parameter P in the current loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter P in the current loop
     */
    virtual int   readCurrentIP(int joint);

    /**
     * @brief Obtain the parameter I in the current loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter I in the current loop
     */
    virtual int   readCurrentII(int joint);

    /**
     * @brief Obtain the parameter D in the current loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter D in the current loop
     */
    virtual int   readCurrentID(int joint);

    /**
     * @brief Obtain the parameter P in the position loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter P in the position loop
     */
    virtual int   readPosP(int joint);

    /**
     * @brief Obtain the parameter I in the position loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter I in the position loop
     */
    virtual int   readPosI(int joint);

    /**
     * @brief Obtain the parameter D in the position loop.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Parameter D in the position loop
     */
    virtual int   readPosD(int joint);

    /**
     * @brief Obtain the dead zone of position.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Dead zone of position
     */
    virtual int   readPosDS(int joint);

    /**
     * @brief Set the current.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param I Assigned current
     * @return bool Indication of successful setting
     */
    virtual bool setCurrentI(int joint, int I);

    /**
     * @brief Set the zero drift.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param zeroPosRadio Zero drift
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @param writeype CMDTYPE_WR or CMDTYPE_WR_NR
     * @return bool Indication of successful setting
     */
    virtual bool setZeroPosOffset(int joint, float zeroPos, u_int16_t jointType, u_int8_t jointWMode);

    /**
     * @brief Set the target position.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param radio Target position in radian
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @param writeype CMDTYPE_WR or CMDTYPE_WR_NR
     * @return bool Indication of successful setting
     */
    virtual bool setTagPosRadio(int joint, float radio, u_int16_t jointType, u_int8_t jointWMode);

    /**
     * @brief Set the target position.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param angle Target position in degree
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @param writeype CMDTYPE_WR or CMDTYPE_WR_NR
     * @return bool Indication of successful setting
     */
    virtual bool setTagPosAngle(int joint, float angle, u_int16_t jointType, u_int8_t jointWMode);

    /**
     * @brief Set the maximal speed of motor.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param speedMoto maximal speed of motor
     * @return bool Indication of successful setting
     */
    virtual bool setMaxSpeedMoto(int joint, int speedMoto);

    /**
     * @brief Set the maximal acceleration of motor.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param acc Maximal acceleration of motor. Unit: rmp/s
     * @return bool Indication of successful setting
     */
    virtual bool setMaxACC(int joint, int acc);

    /**
     * @brief Set the maximal current.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param I Maximal current
     * @return bool Indication of successful setting
     */
    virtual bool setMaxCurI(int joint, int I);

	/**
     *  @brief send radio target position in radian
     *  @return bool the success or failure
     */
	virtual bool sendRobotControlCmd(void);

private:
    /**
     * @brief Read the return value of joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param cmd Command identifier
     * @param index Command index
     * @return bool Indication of successful reading
     */
    virtual bool JointRecvack(u_int16_t joint, u_int8_t cmd, u_int8_t index);

    /**
     * @brief Send message to joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param cmd Command identifier
     * @param index Command index
     * @param data pointer to the message
     * @param data_length
     * @return bool Indication of successful operation
     */
    virtual bool JointSendMsg(u_int16_t joint, u_int8_t cmd, u_int8_t index, u_int8_t *data, int data_length);

    /**
     * @brief Read message of joint
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param addr Address index
     * @param length Data length
     * @param data Pointer to data
     * @return bool Indication of successful operation
     */
    virtual bool JointReadMsg(int joint, u_int8_t addr, int8_t length, u_int8_t *data);


    /**
     * @brief Read two-byte data (WORD)
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param addr Address index
     * @return u_int16_t Two-byte data
     */
    virtual u_int16_t JointReadWordData(int joint, u_int8_t addr);

    /**
     * @brief Write two-byte data (WORD)
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param addr Address index
     * @param num Written data
     * @return bool Indication of successful operation
     */
    virtual bool JointWriteWordData(int joint, u_int8_t addr, u_int16_t num);

    /**
     * @brief Read the reduction ratio.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Reduction radio
     */
    virtual int JointReadREDU_RATIO(int joint);

    /**
     * @brief Read position of joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param cmd Address index. 1: degree; 2: radian
     * @param opt Value at position address
     * @param model MODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Joint angle
     */
    virtual float JointReadPos(int joint, int cmd, int opt, u_int8_t model);

    /**
     * @brief Set the position of joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param angle Joint angle
     * @param cmd 1: degree; 2: radian
     * @param model 0x00: small module; 0x01: large module
     * @param writeype CMDTYPE_WR or CMDTYPE_WR_NR
     * @return bool Indication of successful operation
     */
    virtual bool JointSetPos(int joint, float angle, int cmd, u_int8_t model, u_int8_t writeype);

    /**
     * @brief Enable the joint driver.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param opt Enable/disable joint driver
     * @return bool Indication of successful operation
     */
    virtual bool JointEnable(int joint, bool opt);

    /**
     * @brief Set the power state of joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param opt Enable/disable the power of joint
     * @return bool Indication of successful operation
     */
    virtual bool JointPowerOnEnable(int joint, bool opt);

    /**
     * @brief Save data to flash.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return bool Indication of the successful operation
     */
    virtual bool JointSaveDataFlash(int joint);

    /**
     * @brief Set current position to zero point.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return bool Indication of the successful operation
     */
    virtual bool JointSetNowPosToZero(int joint);

    /**
     * @brief Clear the error code.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return bool Indication of the successful operation
     */
    virtual bool JointTryClearError(int joint);

    /**
     * @brief Read the temprature of joint.
     *
     * @param joint  Joint identifier (0x01-0x06)
     * @return float  Temprature of joint (degree)
     */
    virtual float JointReadTemperature(int joint);


    /**
     * @brief Read the voltage of joint.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param opt SYS_VOLTAGE or MOT_RATED_VOL
     * @return float Current voltage of joint (V)
     */
    virtual float JointReadVoltage(int joint, int opt);

    /**
     * @brief Read the speed.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param cmd MOTOR_SPEED or JOINT_SPEED
     * @param opt SYS_SPEED_L, TAG_SPEED_L or LIT_MAX_SPEED
     * @param modelMODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Speed, rpm
     */
    virtual float JointReadSpeed(int joint, int cmd, int opt, u_int8_t model);

    /**
     * @brief Read the current of joint
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param opt SYS_CURRENT_L, TAG_CURRENT_L, MOT_RATED_CUR or LIT_MAX_CURRENT
     * @return int Current of joint (mA)
     */
    virtual int JointReadElectric(int joint, int opt);

    /**
     * @brief Read the output of potentiometer
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Output of potentiometer (-2047-2047)
     */
    virtual int JointReadPotenVal(int joint);

    /**
     * @brief Read the zero drift
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param cmd JOINT_ANGLE, JOINT_RADIAN or JOINT_UNITS
     * @param model MODEL_TYPE_J60 or MODEL_TYPE_J80
     * @return float Zero drift (-2047-2047)
     */
    virtual float JointReadZeroPosOffsize(int joint, int cmd, u_int8_t model);

    /**
     * @brief Write the zero drift.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param angle Zero drift (-2047-2047)
     * @param cmd JOINT_ANGLE or JOINT_RADIAN
     * @param model MODEL_TYPE_J60 or MODEL_TYPE_J80
     * @param writeype CMDTYPE_WR or CMDTYPE_WR_NR
     * @return bool Indication of successful writing
     */
    virtual bool JointWriteZeroPosOffsize(int joint, float angle, int cmd, u_int8_t model, u_int8_t writeype);

    /**
     * @brief Read the internal resistance of motor.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Internal resistance of motor. Unit: mOhms
     */
    virtual int JointReadMotoR(int joint);

    /**
     * @brief Read the rated inductance of motor.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Rated inductance of motor. Unit: mH
     */
    virtual int JointReadMotoI(int joint);

    /**
     * @brief Read the maximal acceleration of motor.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @return int Maximal acceleration. Unit: rpm/s
     */
    virtual int JointReadMotoMAXACC(int joint);

    /**
     * @brief Write the maximal speed.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param speed Maximal speed. Unit: rpm
     * @return bool Indication of successful writing
     */
    virtual bool JointWriteMotoMAXSpeed(int joint, int speed);

    /**
     * @brief Write the maximal acceleration.
     *
     * @param joint Joint identifier (0x01-0x06)
     * @param acc Maximal acceleration. Unit: rpm/s
     * @return bool Indication of successful writing
     */
    virtual bool JointWriteMotoMAXACC(int joint, int acc);

private:
    /**
     * @brief Initialize the CAN bus.
     *
     * @param szDeviceNode Device node
     * @return bool Indication of successful initialization
     */
     bool JointCanInit(const char* szDeviceNode);

private:
    void *m_hCan; /**< Handle of CAN bus */

};

#endif // JOINTCONTROLCANBUS_H
