#ifndef JOINTBASE_H
#define JOINTBASE_H

#include <fcntl.h>

class JointBase
{
public:
    //();
    /**
     * @brief Initialize the Joint instance.
     *
     * @param id Identifier(0x01-0x06)
     * @param type Joint type
     * @param radio Reduction ratio
     * @param dir Rotation direction, true if clockwise; false if anticlockwise.
     * @param wmode Motor control mode
     */
    JointBase(int id, int type, int radio, bool dir, u_int8_t jointWMode);

    int getJointCurrentI() const;
    void setJointCurrentI(int value);

    int getJointSpeedMoto() const;
    void setJointSpeedMoto(int value);

    float getJointPosJ() const;
    void setJointPosJ(float value);

    float getJointCurVol() const;
    void setJointCurVol(float value);

    float getJointCurTemp() const;
    void setJointCurTemp(float value);

    int getJointTagCurrentI() const;
    void setJointTagCurrentI(int value);

    float getJointTagSpeedMoto() const;
    void setJointTagSpeedMoto(float value);

    float getJointTagPosJ() const;      //RADIO 孤独
    void setJointTagPosJ(float value);

    u_int16_t getJoint_error_num() const;
    void setJoint_error_num(const u_int16_t &value);

    int getJointID() const;

    u_int16_t getJointType() const;

    u_int16_t getJointRATIO() const;

    bool getJointRotDir() const;

    u_int8_t getJointWMode() const;

    void setJointWMode(const u_int8_t &value);

private:
    const int jointID;                      /**< Joint identifier */
    const u_int16_t jointType;       /**< Motor type     MODEL_TYPE_J60/80 */
    const u_int16_t jointRATIO;     /**< Reduction ratio of motor */
    const bool jointRotDir;            /**< Rotation direction of motor */
    u_int8_t jointWMode;              /**< Writing mode of motor control */
    int jointCurrentI;                    /**< Current of driver */
    int jointSpeedMoto;               /**< Speed of driver */
    float jointPosJ;                        /**< Current position in radian */
    float jointCurVol;                     /**< Rated voltage of motor. Unit: mV */
    float jointCurTemp;                 /**< Current temprature of joint */
    int jointTagCurrentI;               /**< Target current of motor */
    float jointTagSpeedMoto;      /**< Target speed of motor */
    float jointTagPosJ;                   /**< Target position of joint in radian */
    u_int16_t joint_error_num;            /**< Joint error of joint num */

};

#endif // JOINTBASE_H
