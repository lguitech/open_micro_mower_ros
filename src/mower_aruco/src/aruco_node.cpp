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


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>

#define DOCK_IMAGE_ID 42

ros::Publisher pub_aruco_detect;

bool g_pubDetect = false;
void arucoCallback(const std_msgs::Int32ConstPtr& msg)
{
	if (msg->data == 1) {
		g_pubDetect = true;
	}
	else {
		g_pubDetect = false;
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

        if (!markerIds.empty()) {
            cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);
			cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
				381.36246688113556, 0, 320.5, 
				0, 381.36246688113556, 240.5, 
				0, 0, 1);

			cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.2, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (size_t i = 0; i < markerIds.size(); ++i) {
				if (markerIds.at(i) != DOCK_IMAGE_ID) {
					continue;
				}
                cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

                cv::Mat rotationMatrix;
                cv::Rodrigues(rvecs[i], rotationMatrix);

                tf2::Matrix3x3 tf_rotationMatrix(
                    rotationMatrix.at<double>(0, 0), rotationMatrix.at<double>(0, 1), rotationMatrix.at<double>(0, 2),
                    rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(1, 1), rotationMatrix.at<double>(1, 2),
                    rotationMatrix.at<double>(2, 0), rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 2));

                tf2::Quaternion quaternion;
                tf_rotationMatrix.getRotation(quaternion);

				geometry_msgs::Pose pose;

				pose.orientation.x = quaternion.getX();
				pose.orientation.y = quaternion.getY();
				pose.orientation.z = quaternion.getZ();
				pose.orientation.w = quaternion.getW();

				pose.position.x = tvecs[i][0];
				pose.position.y = tvecs[i][1];
				pose.position.z = tvecs[i][2];
				
				pub_aruco_detect.publish(pose);

                std::stringstream ss;
                ss << "ID: " << markerIds[i];
                cv::putText(image, ss.str(), markerCorners[i][0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

                ss.str("");
                ss << "Pos: [" << tvecs[i][0] << ", " << tvecs[i][1] << ", " << tvecs[i][2] << "]";
                cv::putText(image, ss.str(), markerCorners[i][1], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

            }
        }

        cv::imshow("ArUco Detection", image);
        cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "aruco_detector");

    ros::NodeHandle handle;
    ros::NodeHandle nh_private;

	std::string camera_image_topic;
	std::string mower_aruco_detect_topic;
	std::string mower_aruco_control_topic;

	

	nh_private.param<std::string>("camera_image_topic",  camera_image_topic,  "/camera/image_raw");
	nh_private.param<std::string>("mower_aruco_detect_topic",  mower_aruco_detect_topic,  "/mower/aruco_detect");
	nh_private.param<std::string>("mower_aruco_control_topic",  mower_aruco_control_topic,  "/mower/aruco_control");

    ros::Subscriber sub_image = handle.subscribe(camera_image_topic, 1, imageCallback);
	ros::Subscriber sub_aruco = handle.subscribe(mower_aruco_control_topic, 1, arucoCallback);

	pub_aruco_detect = handle.advertise<geometry_msgs::Pose>(mower_aruco_detect_topic, 10, true);

    ros::spin();

    return 0;
}
