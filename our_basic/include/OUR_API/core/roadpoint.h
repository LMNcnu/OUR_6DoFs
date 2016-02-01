/**
*@file roadpoint.h
*@brief Road point definition
*
*Define the rotation, orientation and roadpoint.
*
*@author Smokie Robotics, Inc. US.
*@version 1.0.0
*@date 10-09-2014
*/

#ifndef LOADPOINT_H
#define LOADPOINT_H

#include "config.h"
#include "pos.h"

//****************************************************************************
// DEFINITIONS
#define SINGL_POINT    0x00
#define STRAIGHT_LINE  0x01
#define CURVED_LINE    0x02

/**
 * @brief Rotation representation in degree w.r.t x-y-z coordinate.
 *
 */
class Roz
{
public:
/**
 * @brief Default constructor
 *
 */
    Roz(){ x=0; y=0; z=0;}
    double x; /**< rotation w.r.t x axis */
    double y; /**< rotation w.r.t y axis */
    double z; /**< rotation w.r.t z axis */
};

/**
 * @brief Quaternion representation of the end.
 *
 */
class Ori
{
public:
/**
 * @brief Default constructor
 *
 */
    Ori(){ w=0; x=0; y=0; z=0;}
    double w; /**< W = cos (0.5 × a),  angle a*/
    double x; /**< x = X × sin (0.5 × a), axis (X Y Z) and angle a*/
    double y; /**< y = Y × sin (0.5 × a), axis (X Y Z) and angle a */
    double z; /**< z = Z × sin (0.5 × a), axis (X Y Z) and angle a */
    /**
     * @brief Operator ==
     *
     * @param ori
     * @return bool operator
     */
    bool operator== (const Ori &ori) const{
        if(w!=ori.w)
            return false;
        if(x!=ori.x)
            return false;
        if(y!=ori.y)
            return false;
        if(z!=ori.z)
            return false;
        return true;
    }
};

/**
 * @brief Define the road point for the kinematics control.
 *
 */
class RoadPoint
{
public:
    /**
     * @brief Default constructor
     *
     */
    explicit RoadPoint();
    Pos position;                 /**<Coordinate in the world frame  */
    Ori orientation;             /**<Pose of the end */
    double jointpos[ARM_DOF];    /**<Pose of all joints */
    bool ik_flag;                  /**<Indication of reversibility */
    /**
     * @brief print the information of road point.
     *
     */
    void printfPoint();
    /**
     * @brief  Section the data of node
     *
     */
    void SectionRoadPoint();

   /**
    * @brief Assignment operator
    *
    * @param org input RoadPoint
    * @return RoadPoint &operator
    */
   RoadPoint &operator = (const RoadPoint &org);
   /**
    * @brief Operator ==
    *
    * @param p input RoadPoint
    * @return bool operator
    */
   bool operator== (const RoadPoint &p);
   /**
    * @brief auto fill the ikflage
    *
    * @param  
    * @return void
    */
};

#endif // LOADPOINT_H
