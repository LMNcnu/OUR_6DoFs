/**
*@file jointcmd.h
*@brief Joint command
*
*Macro definitions about joint command.
*
*@author Smokie Robotics, Inc. US.
*@version 1.0.0
*@date 10-09-2014
*/

#ifndef CAN_ROBOT_ARM_CMD_H
#define CAN_ROBOT_ARM_CMD_H 

#define BROADCAST_ID		0xFF /**< Broadcast Identifier */

/// Macro definition of command type
#define CMDTYPE_RD			0x01		/**< Reading command type */
#define CMDTYPE_WR			0x02		/**< Writing command type */
#define CMDTYPE_WR_NR		0x03	/**< Writing command type. No reutrn */
#define CMDTYPE_WR_REG		0x04	/**< Asynchronous writing command type (Reserved) */
#define CMDTYPE_SCP			0x05		/**< Command for data return of oscilloscope (Reserved) */
#define CMDTYPE_RST			0x011		/**< Reset the control table (Reset) */

/// Macro definition of model type
#define MODEL_TYPE_J60		0x02	/**< Joint with the rated torque of 60 Nm */
#define MODEL_TYPE_J80		0x03   /**< Joint with the rated torque of 80  Nm */

/// Macro definition of modular reduction ratio
#define GEAR_RATIO_J60		303    /**< Reduction ratio of J60 */
#define GEAR_RATIO_J80		363    /**< Reduction ratio of J80 */

/// Macro definition of memory control table
#define CMDMAP_LEN			128			 /**< Total length of memory control table */
#define CMDMAP_INDLEN		8 			 /**< Index number of memory control table */
#define CMDMAP_SUBLEN		16 		 /**< Subindex number of memory control table */

/// Macro definition of driver mode
#define MODE_OPEN			0			/**< Open-loop mode */
#define MODE_CURRENT		1			/**< Current mode */
#define MODE_SPEED			2			/**< Speed mode */
#define MODE_POSITION		3		/**< Position mode */

/// System state
#define SYS_ID					0x01                /**< Driver identifier */
#define SYS_MODEL_TYPE			0x02	/**< Driver type */
#define SYS_FW_VERSION			0x03	/**< Fireware version */
#define SYS_ERROR				0x04		    /**< Error code */
#define SYS_VOLTAGE				0x05		/**< System voltage */
#define SYS_TEMP				0x06                /**< System temprature */
#define SYS_REDU_RATIO			0x07		/**< Reduction ratio */
#define SYS_BAUDRATE_232		0x08	/**< Baudrate of RS232 */
#define SYS_BAUDRATE_CAN		0x09	/**< Baudrate of CAN Bus */
#define SYS_ENABLE_DRIVER		0x0a	/**< Enabled flag of driver */
#define SYS_ENABLE_ON_POWER		0x0b		/**< Enabled on power flag of driver */
#define SYS_SAVE_TO_FLASH		0x0c		/**< Flag of data saving */
#define SYS_DEMA_ABSPOS			0x0d	/**< Automatically calibrate the absolute position */
#define SYS_SET_ZERO_POS		0x0e		/**< Set current position to zero */
#define SYS_CLEAR_ERROR			0x0f		/**< Clear error flag */

#define SYS_CURRENT_L			0x10		/**< Lower 16 bits of current (mA) */
#define SYS_CURRENT_H			0x11		/**< Higher 16 bits of current (mA) */
#define SYS_SPEED_L				0x12		/**< Lower 16 bits of current speed (units/s) */
#define SYS_SPEED_H				0x13		/**< Higher 16 bits of current speed (units/s) */
#define SYS_POSITION_L			0x14		/**< Lower 16 bits of current position (units) */
#define SYS_POSITION_H			0x15		/**< Higher 16 bits of current position (units) */
#define SYS_POTEN_VALUE			0x16	/**< Reading of potentiometer */
#define SYS_ZERO_POS_OFFSET_L	0x17		/**< Lower 16 bits of zero position offset (units) */
#define SYS_ZERO_POS_OFFSET_H	0x18		/**< Higher 16 bits of zero position offset (units) */

/// Motor information
#define MOT_RES					0x20		/**< Internal resistance of motor */
#define MOT_INDUC				0x21		/**< Inductance of motor */
#define MOT_RATED_VOL			0x22		/**< Rated voltage of motor */
#define MOT_RATED_CUR			0x23	/**< Rated current of motor */
#define MOT_ENC_LINES			0x400		/**< Lines of encoder */
#define MOT_HALL_VALUE			0x25	/**< HALL state */

/// Control target
#define TAG_WORK_MODE			0x30	/**< Work mode. 0: open-loop mode; 1: current mode; 2: speed mode; 3: position mode */
#define TAG_OPEN_PWM			0x31		/**< PWM in open-loop mode (0~100) */
#define TAG_CURRENT_L			0x32		/**< Lower 16 bits of target current (mA) */
#define TAG_CURRENT_H			0x33		/**< Higher 16 bits of target current (mA) */
#define TAG_SPEED_L				0x34		/**< Lower 16 bits of target speed (units/s) */
#define TAG_SPEED_H				0x35		/**< Higher 16 bits of target speed (units/s) */
#define TAG_POSITION_L			0x36		/**< Lower 16 bits of target poistion (units) */
#define TAG_POSITION_H			0x37	/**< Higher 16 bits of target poistion (units) */

/// Control threshold
#define LIT_MAX_CURRENT			0x40	/**< Maximal current (mA) */
#define LIT_MAX_SPEED			    0x41	/**< Maximal speed (rmp) */
#define LIT_MAX_ACC				    0x42	/**< Maximal acceleration (rpm/s) */
#define LIT_MIN_POSITION_L		0x43	/**< Lower 16 bits of minimal position */
#define LIT_MIN_POSITION_H		0x44	/**< Higher 16 bits of minimal position */
#define LIT_MAX_POSITION_L		0x45	/**< Lower 16 bits of maximal position */
#define LIT_MAX_POSITION_H		0x46	/**< Higher 16 bits of maximal position */

/// Closed loop
#define SEV_PARAME_LOCKED		0x50		/**< Flag of parameter lock in three closed loops */
#define SEV_CURRENT_P			0x51		/**< Parameter P in current loop */
#define SEV_CURRENT_I			0x52		/**< Parameter I in current loop */
#define SEV_CURRENT_D			0x53		/**< Parameter D in current loop */
#define SEV_SPEED_P				0x54		/**< Parameter P in speed loop */
#define SEV_SPEED_I				0x55		/**< Parameter I in speed loop */
#define SEV_SPEED_D				0x56		/**< Parameter D in speed loop */
#define SEV_SPEED_DS             0x57            /**< Dead zone in speed loop */
#define SEV_POSITION_P			0x58		/**< Parameter P in position loop */
#define SEV_POSITION_I			0x59		/**< Parameter I in position loop */
#define SEV_POSITION_D			0x5a		/**< Parameter D in position loop */
#define SEV_POSITION_DS		0x5b	/**< Dead zone in position loop */

/// Address definition of subindex
#define SCP_MASK				0x70		/**< Record MASK */
#define SCP_TRI_SOC				0x71		/**< Triger source. 0: open-loop triger; 1: current triger; 2: speed triger; 3: position triger; 4: user triger */
#define SCP_TRI_MOD				0x72		/**< Triger mode. 0: raising edge; 1: trailing edge; 2: continuous sampling */
#define SCP_TRI_FLG				0x73		/**< Flag of user triger */
#define SCP_REC_TIM				0x74		/**< Record the time interval */
#define SCP_REC_OFS				0x75		/**<  Record the time offset */
#define SCP_TAGCUR				0x76		/**< Data set of target current */
#define SCP_MEACUR				0x77		/**< Data set of measured current */
#define SCP_TAGSPD				0x78		/**< Data set of target speed */
#define SCP_MEASPD				0x79		/**< Data set of measured speed */
#define SCP_TAGPOS				0x7A		/**< Data set of target position */
#define SCP_MEAPOS				0x7B		/**< Data set of measured position */

/// Macro definition of baudrate
#define BAUD_232_19200		0x0000		/**< 19200 */
#define BAUD_232_115200		0x0001		/**< 115200 */
#define BAUD_232_500000		0x0002		/**< 500k */
#define BAUD_232_1000000	0x0003		/**< 1M */
#define BAUD_CAN_250000		0x0000		/**< 250K */
#define BAUD_CAN_500000		0x0001		/**< 500K */
#define BAUD_CAN_1000000	0x0002		/**< 1M */

/// Macro definition of oscilloscope triger
#define TRIGER_UP		0x0000		/**< Rising edge */
#define TRIGER_DOWN		0x0001		/**< Trailing edge */
#define TRIGER_USER		0x0002		/**< Manual triger */
#define TRIGER_OTHER	0x0003		/**< Continuous sampling */

/// MASK definition of oscilloscope record
#define MASK_TAGCUR		0x0001		/**< Target current MASK */
#define MASK_MEACUR		0x0002		/**< Measured current of MASK */
#define MASK_TAGSPD		0x0004		/**< Target speed MASK */
#define MASK_MEASPD		0x0008		/**< Measured speed of MASK */
#define MASK_TAGPOS		0x0010		/**< Target position MASK */
#define MASK_MEAPOS		0x0020		/**< Measured position of MASK */

/// MASK definition of errors
#define ERROR_MASK_OVER_CURRENT		0x0001		/**< Error code of over current */
#define ERROR_MASK_OVER_VOLTAGE		0x0002		/**< Error code of over voltage */
#define ERROR_MASK_UNDER_VOLTAGE	0x0004		/**< Error code of under voltage */
#define ERROR_MASK_OVER_TEMP		0x0008		/**< Error code of over temprature */
#define ERROR_MASK_HALL				0x0010		/**< Error code of HALL  */
#define ERROR_MASK_ENCODER			0x0020	/**< Error code of encoder */
#define ERROR_MASK_POTEN			0x0040		/**< Error code of potentiometer */
#define ERROR_MASK_CURRENT_INIT		0x0080		/**< Error code of current detection */
#define ERROR_MASK_FUSE				0x0100		/**< Error code of fuse */

/// MASK defintion of bus errors
#define ERROR_MASK_BUS_INIT         0x0001		/**< 总线初始化错误*/
#define ERROR_MASK_BUS_READ         0x0002		/**< 总线读操作超时*/
#define ERROR_MASK_BUS_WRITE        0x0004		/**< 总线写操作失败*/
#define ERROR_MASK_BUS_FORMAT       0x0008		/**< 数据格式错误*/
#define ERROR_MASK_BUS_CMD          0x0010		/**< 未启用的指令*/
#define ERROR_MASK_BUS_UNKNOWN		0x0100		/**< 总线其他错误*/

#endif //CAN_ROBOT_ARM_CMD_H
