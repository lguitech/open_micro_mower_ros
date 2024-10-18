/*********************************************************************
 *
 *  This file is part of the [OPEN_MICRO_MOWER_ROS] project.
 *  Licensed under the MIT License for non-commercial purposes.
 *  Author: Brook Li
 *  Email: lguitech@126.com
 *
 *  For more details, refer to the LICENSE file or contact [lguitech@126.com].
 *
 *  Commercial use requires a separate license.
 *
 *  This software is provided "as is", without warranty of any kind.
 *
 *********************************************************************/

#include "mr_behav_base.h"

void mr_behav_base::putter_up()
{
	mower_msgs::MowerChassisControl msg;
	msg.control_type = CHASSIS_CONTROL_TYPE_PUTTER;
	msg.param1 = CHASSIS_CONTROL_PUTTER_UP;
	getChassisControlPublisher().publish(msg);
}

void mr_behav_base::putter_down()
{
	mower_msgs::MowerChassisControl msg;
	msg.control_type = CHASSIS_CONTROL_TYPE_PUTTER;
	msg.param1 = CHASSIS_CONTROL_PUTTER_DOWN;
	getChassisControlPublisher().publish(msg);
}

void mr_behav_base::cutter_rotate()
{
	mower_msgs::MowerChassisControl msg;
	msg.control_type = CHASSIS_CONTROL_TYPE_CUTTER;
	msg.param1 = CHASSIS_CONTROL_CUTTER_ROTATE;
	getChassisControlPublisher().publish(msg);
}

void mr_behav_base::cutter_stop()
{
	mower_msgs::MowerChassisControl msg;
	msg.control_type = CHASSIS_CONTROL_TYPE_CUTTER;
	msg.param1 = CHASSIS_CONTROL_CUTTER_STOP;
	getChassisControlPublisher().publish(msg);
}

