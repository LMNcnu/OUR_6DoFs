/**
*@file pos.h
*@brief Position definition
*
*Define the position.
*
*@author Smokie Robotics, Inc. US.
*@version 1.0.0
*@date 10-10-2014
*/

#ifndef POS_H
#define POS_H

/**
 * @brief Position definition
 *
 */
class Pos
{
public:
/**
 * @brief Default constructor
 *
 */
    Pos();
/**
 * @brief Copy constructor
 *
 * @param p Input Pos
 */
    Pos(const Pos & p);
    double x; /**< x coordinate*/
    double y; /**< y coordinate */
    double z; /**< z coordinate */

    /**
     * @brief Operator +
     *
     * @param p Input Pos
     * @return Pos operator
     */
    Pos operator+ (const Pos & p) const;
    /**
     * @brief Operator -
     *
     * @param p Input Pos
     * @return Pos operator
     */
    Pos operator- (const Pos & p) const;
    /**
     * @brief Operator ==
     *
     * @param p Input Pos
     * @return bool operator
     */
    bool operator== (const Pos &p) const;
};

#endif // POS_H
