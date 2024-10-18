#include <tf/tf.h>
#include "mower_msgs/MowerChassisControl.h"
#include "mr_mower_controller.h"
#include "float.h"
#include "mr_util.h"


mr_mower_controller::mr_mower_controller()
{
    time_t now;
    struct tm *local_time;
    char formatted_time[20];

    time(&now);
    local_time = localtime(&now);
    strftime(formatted_time, sizeof(formatted_time), "%y%m%d-%H%M%S", local_time);

    char strFileSpec[128];
    sprintf(strFileSpec, "/var/tmp/logger-ctl-%s.csv", formatted_time);
                                        
    if (!log_file.is_open()) {
        log_file.open(strFileSpec, std::ios::app);
        if (!log_file.is_open()) {
            ROS_ERROR("Failed to open log file.");
            return;
        }
        log_file << "time, total, lastx, lasty, last_angle, waypoint, waypoint_x, waypoint_y, waypoint_angle, lookpoint, lookpoint_x, lookpoint_y, alpha_look, alpha, behav, left_speed, right_speed" << std::endl;
        log_file.flush();
    }

    
    reset();
}

mr_mower_controller::~mr_mower_controller()
{

}

mr_mower_controller* mr_mower_controller::getInstance()
{
    static mr_mower_controller instance;
    return &instance;
}



void mr_mower_controller::set_path(nav_msgs::Path& path)
{
    vec_trajectory = path.poses;
    int size = vec_trajectory.size();

    reset();
}

void mr_mower_controller::reset_path()
{
	vec_trajectory.clear();
}

void mr_mower_controller::reset()
{
   
    isFinished = false;

    last_pose.x = 0;
    last_pose.y = 0;
    last_pose.angle = 0;

    look_waypoint = goal_waypoint = 0;
}

void mr_mower_controller::set_goal_waypoint(int index)
{
	look_waypoint = goal_waypoint = index;
}


bool mr_mower_controller::get_process_info()
{
    return isFinished;
}

void mr_mower_controller::skip_to_final()
{
	isFinished = true;
}

void mr_mower_controller::set_parameter(double max_linear_speed, double max_angular_speed, double wheel_distance)
{
	this->max_linear_speed = max_linear_speed;
	this->max_angular_speed = max_angular_speed;
	this->wheel_distance = wheel_distance;

    controllerTool.set_parameter(max_linear_speed, max_angular_speed, wheel_distance);
}


void mr_mower_controller::set_pose(const geometry_msgs::Pose& msg)
{
    last_pose.x = msg.position.x;
    last_pose.y = msg.position.y;

    double yaw = tf::getYaw(msg.orientation);
    
    if(yaw < 0) {
        last_pose.angle = yaw + M_PI * 2;
    }
    else if (yaw >= M_PI * 2) {
        last_pose.angle = yaw - M_PI * 2;
    }
    else {
        last_pose.angle = yaw;
    }
}

bool mr_mower_controller::has_arrived_goal()
{
	assert(look_waypoint >= goal_waypoint);

	for (int i=goal_waypoint; i<=look_waypoint; i++) {
		geometry_msgs::PoseStamped& next_waypoint = vec_trajectory.at(i);
		double disNext = std::hypot(last_pose.x - next_waypoint.pose.position.x,
									last_pose.y - next_waypoint.pose.position.y);
		if (disNext < THRESHOLD_ARRIVED) {
			return true;
		}
	}

return false;

}


int mr_mower_controller::compute_control_command()
{
	if (isFinished) {
		return 0;
	}	
    mr_cmd cmd;
    if (has_arrived_goal()) {
        if (goal_waypoint == vec_trajectory.size() -1) {
            isFinished = true;
            cmd.param1 = 0;
            cmd.param2 = 0;
        }
        else {
            goal_waypoint ++;
            
            compute_control_command_normal(cmd);
        }
    }
    else {
        compute_control_command_normal(cmd);
    }

    double linear_speed, angular_speed;
    convert_cmd_vel(cmd.param1, cmd.param2, linear_speed, angular_speed);
    pub_control(linear_speed, angular_speed);

    return goal_waypoint;
}

std::string mr_mower_controller::getFormatTime() 
{
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::tm now_tm = *std::localtime(&now_time_t);

    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%H%M%S") << '.' << std::setw(3) << std::setfill('0') << now_ms.count();
    
    return oss.str();
}

void mr_mower_controller::compute_control_command_normal(mr_cmd& cmd)
{
    int tIndex;
    double tx;
    double ty;

    std::string strFormatTime = getFormatTime();
    
    cal_look_forward_position(tIndex, tx, ty);


    double dx = tx - last_pose.x;
    double dy = ty - last_pose.y;

    double ld = sqrt(dx * dx + dy * dy);

    double alpha_goal = atan2(dy, dx);

    if (alpha_goal < 0) {
        alpha_goal += 2 * M_PI;
    }
    double alpha = alpha_goal - last_pose.angle;

    if (alpha > M_PI) {
        alpha -= 2 * M_PI;
    }
    else if (alpha < -M_PI) {
        alpha += 2 * M_PI;
    }
    double abs_alpha = fabs(alpha);

    mr_pose pose_goal;
    getMRPose(vec_trajectory.at(goal_waypoint), pose_goal);
    char buffer[256];
    sprintf(buffer, "%s, %d, %.4f, %.4f, %.1f, %d, %.4f, %.4f, %.1f, %d, %.4f, %.4f, %.1f, %.1f, ", 
        strFormatTime.c_str(), (int)vec_trajectory.size(), 
        last_pose.x, last_pose.y, last_pose.angle * 180 / M_PI, 
        goal_waypoint, pose_goal.x, pose_goal.y, pose_goal.angle * 180 / M_PI, 
        tIndex, tx, ty, alpha_goal * 180 / M_PI, alpha * 180 / M_PI
    );


    log_file << buffer;

    
    if (alpha < (-1 * ROTATE_THRESHOLD)) {
        controllerTool.rotate_right_max_speed(abs_alpha, cmd);

        char buffer[64];
        sprintf(buffer, "rmax, %.2f, %.2f\r\n", cmd.param1, cmd.param2);
        log_file << buffer;
        log_file.flush();

        return;
    }
    else if (alpha > ROTATE_THRESHOLD) {
        controllerTool.rotate_left_max_speed(abs_alpha, cmd);

        char buffer[64];
        sprintf(buffer, "lmax, %.2f, %.2f\r\n", cmd.param1, cmd.param2);
        log_file << buffer;
        log_file.flush();

        return;
    }

    double ey = ld * sin(alpha); 

    if (fabs(ey) < 0.01) {
        controllerTool.straight_forward_max_speed(cmd);

        char buffer[64];
        sprintf(buffer, "fmax, %.2f, %.2f\r\n", cmd.param1, cmd.param2);
        log_file << buffer;
        log_file.flush();

        return;
    }

    double radius = ld * ld / (2 * fabs(ey));
    if (ey > 0) { 
        controllerTool.rotate_left(abs_alpha, radius, cmd);

        char buffer[64];
        sprintf(buffer, "lrot, %.2f, %.2f\r\n", cmd.param1, cmd.param2);
        log_file << buffer;
        log_file.flush();

    }
    else {
        controllerTool.rotate_right(abs_alpha, radius, cmd);

        char buffer[64];
        sprintf(buffer, "rrot, %.2f, %.2f\r\n", cmd.param1, cmd.param2);
        log_file << buffer;
        log_file.flush();

    }
}


double mr_mower_controller::toNormalRegion(double angle)
{
    while(angle < 0) {
        angle += 2 * M_PI;
    }
    while (angle >= 2 * M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}

double mr_mower_controller::normalizeDiff(double diff)
{
    while (diff < -M_PI) {
        diff += 2 * M_PI;
    }
    while (diff > M_PI) {
        diff -= 2 * M_PI;
    }
    return diff;
}

double mr_mower_controller::getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);
    return yaw;
}

void mr_mower_controller::getMRPose(const geometry_msgs::PoseStamped& input, mr_pose& output)
{
    output.x = input.pose.position.x;
    output.y = input.pose.position.y;
    output.angle = getYawFromQuaternion(input.pose.orientation);
}


void mr_mower_controller::cal_look_forward_position(int& tIndex, double& tx, double& ty) 
{
    mr_pose pose_goal, pose_look, pose_prev;

    getMRPose(vec_trajectory.at(goal_waypoint), pose_goal);
    getMRPose(vec_trajectory.at(look_waypoint), pose_look);
    pose_prev = last_pose;

    double dis_sum = 0;

    double look_distance = LOOK_FORWARD_DISTANCE;
    
    while(
          (look_waypoint + 1 < vec_trajectory.size())
    )
    {   
        dis_sum += std::hypot(pose_look.x - pose_prev.x, 
                              pose_look.y - pose_prev.y);
        if (dis_sum > look_distance) {
            break;
        }

        double diffAngle = std::fabs(normalizeDiff(pose_look.angle - pose_goal.angle));
        if (diffAngle > LOOKAHEAD_THRESHOLD) {
            break;
        }
        
        pose_prev = pose_look;
        
        look_waypoint++;
        getMRPose(vec_trajectory.at(look_waypoint), pose_look);
    }

    tIndex = look_waypoint;
    tx = pose_look.x;
    ty = pose_look.y;
}

void mr_mower_controller::convert_cmd_vel(double left_speed, double right_speed,
										  double& linear_speed, double& angular_speed)
{
    double linear_velocity = (left_speed + right_speed) / 2;
    double angular_velocity = (right_speed - left_speed) / wheel_distance;

    linear_speed = linear_velocity;
    angular_speed = angular_velocity;
}