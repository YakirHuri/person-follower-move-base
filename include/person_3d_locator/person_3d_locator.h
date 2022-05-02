/*=========================================================================
Filename : include/person_3d_locator/person_3d_locator.h
Project  : Person Follower
Purpose  : ROS node to publish the 3D pose of a targeted person by
           detecting the person from image captured using camera and also
           by locating the coordinates of the respective person
           in realworld
---------------------------------------------------------------------------
---------------------------------------------------------------------------
Version Number : 1.0
Last Updated   : October 20, 2021
Updated By     : Manodhayan K
---------------------------------------------------------------------------
Copyright (c) 2012 Mobiveil. All rights reserved.
---------------------------------------------------------------------------
This file contains trade secrets of Mobiveil.
No part may be reproduced or transmitted in any form by any means or for
any purpose without the express written permission of Mobiveil.
---------------------------------------------------------------------------
Revision History
---------------------------------------------------------------------------


---------------------------------------------------------------------------
Known Issues
---------------------------------------------------------------------------

---------------------------------------------------------------------------
To Do List
---------------------------------------------------------------------------
*/

#ifndef PERSON_3D_LOCATOR_H
#define PERSON_3D_LOCATOR_H

#include <ros/ros.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <queue>
#include "ros_compat.h"
#include <image_transport/image_transport.h>



//yakir
#include <depth_image_proc/depth_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/stereo_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <string>

#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include "../opencv/cv-helpers.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#define PUBLISH_IMAGE_ENCODING      "bgr8"
#define CENTER_OFFSET_DEPTH          (2)  // Number of pixels
#define CENTER_FAR_OFFSET_DEPTH      (5)  // Number of pixels

#define CLIPPING_DISTANCE            (5)
#define CAMERA_FRAME_ID              ("camera_link")
#define COLOR_IMAGE_FRAME_ID         ("color_image")


#include "../include/detectnet.h"

using namespace std;
using namespace cv;

#define DEBUG (1)
#define NODE_NAME "person_3d_locator"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Possible Target regions
enum TargetRegion
{
    NEAR,
    FOLLOW,
    FAR
};

class CameraTarget
{
public:
    float x{};
    float y{};
    float rawDepthRayDistance;
    float depthRayDistance{};
    float angleX{};

    CameraTarget() {}
};

class PersonCameraTarget : public CameraTarget
{
public:
    TargetRegion targetRegion;
    PersonCameraTarget() {}
};

/*============================================================================
Name    :   Person3DLocator
------------------------------------------------------------------------------
Purpose :   Class to implement the person 3d locator
============================================================================*/

class Person3DLocator
{

private:
    // Private node handler to read the params
    ros::NodeHandle *nodeHandlerPrivate;

    // To store the detections and the centeral depth of bounding box
    std::vector<std::pair<float, vision_msgs::Detection2D>> detectedPersons;

    // Nearest person bounding box and depth
    std::pair<float, vision_msgs::Detection2D> nearestPerson;

    // Target person bounding box and depth
    std::pair<float, vision_msgs::Detection2D> targetedPerson;

    // Frame in which the goal has to be sent
    std::string sGoalFrameName;

    // Topic to fetch rgb image
    std::string sInputImageTopic;

    // Topic to fetch detections
    std::string sDetectionsTopic;

    // Pose message of target from Camera
    geometry_msgs::PoseStamped targetFromCameraPoseMsg;

    // Temporary pose message of target
    geometry_msgs::PoseStamped poseMsg;

    // Tracking image message
    cv_bridge::CvImage trackingMsg;

    // RBG Image publisher
    image_transport::Publisher pub_ImagePublisher;

    // ROS publisher to publish target pose
    ros::Publisher posePublisher;

    // ROS Publisher to publish the image that has the overlay bounding box of tracking person
    image_transport::Publisher trackingImagePublisher;

    // OpenCV Tracker
    cv::Ptr<cv::Tracker> tracker;

    // Flag to disable person 3d locator
    bool bPerson3dLocateDisable = false;
    // Flag to indicate the target locked status
    bool bIsTargetLocked;

    // Flag to indicate whether the tracker found the locked target person or not
    bool bIsTargetFound;

    // Flag to indicate the search rotation
    bool bIsRotationInitiated;

    // Rotation angle in radians
    float fRotation;

    // Width of the RGB image
    int iImageWidth;

    // Height of the RGB image
    int iImageHeight;

    // Dwell time to initiate the search, if the target is missed instantly
    float fSearchDwellDuration;

    // Warning IOU threshold between target detection bounding box from detectnet and bounding box from the OpenCV tracker
    float fWarningIOUThreshold;

    // Critical IOU threshold between target detection bounding box from detectnet and bounding box from the OpenCV tracker
    float fCriticalIOUThreshold;

    // Minimum distance between robot and the target person, to start/continue following
    float fMinFollowRegion;

    // Minimum distance between robot and the target person, to start/continue following
    float fMaxFollowRegion;

    // Orientation of target in Quaternion
    tf2::Quaternion qQuaternionTarget, qQuaternionTargetFromCam;

    // ROS Transform buffer
    tf2_ros::Buffer tfBuffer;

    // ROS Transform listener
    tf2_ros::TransformListener *tfListener;

    ros::Time prevGoalTime;
    ros::Time prevRotationEndTime;

    geometry_msgs::PoseStamped lastKnownCameraPoseFromOdom;

    MoveBaseClient *actionClient;

    std::queue<geometry_msgs::PoseStamped> qRotationGoalQueue;

    PersonCameraTarget personCameraTarget{};

    geometry_msgs::PoseStamped recoveryStartPoseMsg;

    //yakir
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
    image_geometry::PinholeCameraModel pinholeCameraModel_;
    ros::Subscriber imgBgrSubscriber_;
    bool cameraInfoInited_ = false;
    bool bgrInited_ = false;
    cv::Mat curretDepthImg_;
    cv::Mat currentBgrImg_;


   

    
    
    /*============================================================================
    Name    :   Person3DLocator::PublishGoal()
    ------------------------------------------------------------------------------
    Purpose :   To publish goal relative to user defined goal frame
    Input   :   n/a
    Output  :   n/a
    Notes   :   n/a
    ============================================================================*/

    void PublishGoal(geometry_msgs::PoseStamped goalMsg, bool bUseActionServer = false);

    /*============================================================================
    Name    :   Person3DLocator::PublishRotationGoal()
    ------------------------------------------------------------------------------
    Purpose :   To compute and publish rotation goal
    Input   :   n/a
    Output  :   n/a
    Notes   :   n/a
    ============================================================================*/

    void PublishRotationGoal();

    /*============================================================================
    Name    :   Person3DLocator::CheckTargetRegion
    ------------------------------------------------------------------------------
    Purpose :   To identify the region in which the target is located
    Input   :   dDepth - distance of target person from the camera
    Output  :   TargetRegion - Region in which the target is located
    Notes   :   n/a
    ============================================================================*/

    TargetRegion CheckTargetRegion(double dDepth);

    void PrepareTargetPoseFromDepthRay(geometry_msgs::PoseStamped &poseMsg, float fTargetDepthRayDistance, float fTargetAngle, int iTargetPersonXCenterInImage);

    void getCameraPose();
    void transformPoseToOdom(geometry_msgs::PoseStamped &toTransformPose);
    void emptyRotationGoalQueue();

public:
    ros::NodeHandle nodeHandler;

#ifndef USE_DETECT_NET
    DetectnetWrapper detectnetWrapper;
#endif

    // RealsenseCamera realsense_camera;
    cv::Rect2d rBoundingBox;
    sensor_msgs::ImagePtr trackingImageMessage;

    /*============================================================================
    Name    :   Person3DLocator()
    ------------------------------------------------------------------------------
    Purpose :   Constructor for person 3D locator
    Input   :   nodeHandler - a ros node handler
    Output  :   n/a
    Notes   :   n/a
    ============================================================================*/

    Person3DLocator(ros::NodeHandle &nodeHandler) : nodeHandler(nodeHandler){

         //yakir
        image_sub.subscribe(nodeHandler, "/camera/depth/image_rect_raw", 1);
        image_sub.registerCallback(&Person3DLocator::depthCallback, this);

        info_sub.subscribe(nodeHandler, "/camera/depth/camera_info", 1);
        info_sub.registerCallback(&Person3DLocator::cameraInfoCallback, this); 

        imgBgrSubscriber_ = nodeHandler.subscribe("/camera/color/image_raw", 1,
                        &Person3DLocator::imageCallback, this);     


    };

    /*============================================================================
    Name    :   ~Person3DLocator()
    ------------------------------------------------------------------------------
    Purpose :   Destructor for person 3D locator
    Input   :   n/a
    Output  :   n/a
    Notes   :   n/a
    ============================================================================*/

    ~Person3DLocator();

    /*============================================================================
    Name    :   Person3DLocator::Init
    ------------------------------------------------------------------------------
    Purpose :   Init function for person follower class
    Input   :   n/a
    Output  :   n/a
    Notes   :   n/a
    ============================================================================*/

    void Init();

    /*============================================================================
    Name    :   Person3DLocator::Run
    ------------------------------------------------------------------------------
    Purpose :   To implement the control flow/operation of person follower
    Input   :   n/a
    Output  :   n/a
    Notes   :   n/a
    ============================================================================*/

    void Run();

    /*============================================================================
    Name    :   Person3DLocator::doneWithGoalCallback()
    ------------------------------------------------------------------------------
    Purpose :   DCalled once the move base is done with the current goal
    Input   :   state - State of the goal
                result - Result of the goal
    Output  :   n/a
    Notes   :   n/a
    ============================================================================*/

    void doneWithGoalCallback(const actionlib::SimpleClientGoalState &, 
        const move_base_msgs::MoveBaseResult::ConstPtr &);


    //yakir
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &cam_info) {

        if (cameraInfoInited_ == false)
        {
            if (pinholeCameraModel_.fromCameraInfo(*cam_info))
            {
                cameraInfoInited_ = true;
            }
        }
    }


    float yakirGetDistance(int x, int y, const Mat& depthImg){

        if( cameraInfoInited_){
            
            float cx = pinholeCameraModel_.cx();
            float cy = pinholeCameraModel_.cy();

            float fx = pinholeCameraModel_.fx();
            float fy = pinholeCameraModel_.fy(); 

            cv::Point2d centerObject( x, y );

            float d = depthImg.at<float>(centerObject.y, centerObject.x) / 1000; /// IN METERS
            

            return d;
        }

        return -1;
        
    }


    //yakir
     void depthCallback(const sensor_msgs::ImageConstPtr &image) {

        if(cameraInfoInited_){

            cv_bridge::CvImagePtr cvBridgePtr =
                cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);

            curretDepthImg_ = cvBridgePtr->image;

        }
        
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {   

        if( !bgrInited_ ){
            
            bgrInited_ = true;          
           
        }
        try
        {
           currentBgrImg_ = cv_bridge::toCvShare(msg, "bgr8")->image;           
          
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }




};

#endif
