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
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>


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


struct YakirPerson {

    vision_msgs::Detection2D box_;

    float distnace_;

    geometry_msgs::PointStamped location_;
};
class Person3DLocator
{

private:
    // Private node handler to read the params
    ros::NodeHandle *nodeHandlerPrivate;

    // To store the detections and the centeral depth of bounding box
    // std::vector<std::pair<float, vision_msgs::Detection2D>> detectedPersons;

    // // Nearest person bounding box and depth
    // std::pair<float, vision_msgs::Detection2D> nearestPerson;

    // // Target person bounding box and depth
    // std::pair<float, vision_msgs::Detection2D> targetedPerson;


    std::vector<YakirPerson> detectedPersonsYakir_;

    // Nearest person bounding box and depth
    YakirPerson nearestPersonYakir_;

    // Target person bounding box and depth
    YakirPerson targetedPersonYakir_;

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

    //yakir params
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;

    image_transport::Publisher  debug_depth_pub_;

    image_geometry::PinholeCameraModel pinholeCameraModel_;
    
    ros::Subscriber imgBgrSubscriber_;
    bool cameraInfoInited_ = false;
    bool bgrInited_ = false;
    cv::Mat curretDepthImg_;
    cv::Mat currentBgrImg_;
    tf::TransformListener yakir_tfListener_;
    tf::StampedTransform transform_;

    bool initTransform_ = false;
    float cx_ ;
    float cy_ ;
    float fx_ ;
    float fy_;

    ros::Publisher targets_marker_pub_;
        ros::Publisher goal_marker_pub_;




   

    
    
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
        image_sub.subscribe(nodeHandler, "/camera/aligned_depth_to_color/image_raw", 1);
        image_sub.registerCallback(&Person3DLocator::depthCallback, this);

        info_sub.subscribe(nodeHandler, "/camera/aligned_depth_to_color/camera_info", 1);
        info_sub.registerCallback(&Person3DLocator::cameraInfoCallback, this); 

        imgBgrSubscriber_ = nodeHandler.subscribe("/camera/color/image_raw", 1,
                        &Person3DLocator::imageCallback, this); 

        targets_marker_pub_ = nodeHandler.advertise<visualization_msgs::MarkerArray>("/persons_markers", 10);

        goal_marker_pub_ = nodeHandler.advertise<visualization_msgs::Marker>("/goal_marker", 10);


        image_transport::ImageTransport it(nodeHandler);
        debug_depth_pub_ = it.advertise("/debug_depth", 1);


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
            
            cx_ = pinholeCameraModel_.cx();
            cy_ = pinholeCameraModel_.cy();

            fx_ = pinholeCameraModel_.fx();
            fy_ = pinholeCameraModel_.fy(); 

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

    


    bool extractDepthFromBboxObject( cv::Point2d pix, float d,
           geometry_msgs::PointStamped& pose , string targetFrame);

      geometry_msgs::PointStamped transformToByFrames(
        Point3d objectPoint3d, string base_Frame, string child_Frame)  {


        geometry_msgs::PointStamped pointStampedIn;
        geometry_msgs::PointStamped pointStampedOut;

        pointStampedIn.header.frame_id = child_Frame;
        pointStampedIn.header.stamp = ros::Time(0);
        pointStampedIn.point.x = objectPoint3d.x;
        pointStampedIn.point.y = objectPoint3d.y;
        pointStampedIn.point.z = objectPoint3d.z;

        if( !initTransform_){

            try{
                

                if  (yakir_tfListener_.waitForTransform(base_Frame, child_Frame, 
                    ros::Time(0), ros::Duration(0.005))){
                        
                    initTransform_ = true;

                    yakir_tfListener_.lookupTransform(base_Frame, child_Frame,  
                                        ros::Time(0), transform_);

                    yakir_tfListener_.transformPoint(base_Frame, pointStampedIn, pointStampedOut);

                    return pointStampedOut;
                    
                }else {

                   //cerr<<"Failed to find transform between "<<base_Frame<<" and "<<child_Frame<<endl;
                   return pointStampedOut;
                }

            }   
                catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
        }
        else {


            yakir_tfListener_.transformPoint(base_Frame, pointStampedIn, pointStampedOut);


            return pointStampedOut;
        }  

      
    }

    void publishTargetsMarkers(){

        visualization_msgs::MarkerArray Markerarr;
        Markerarr.markers.resize(detectedPersonsYakir_.size());

        cerr<<" detectedPersonsYakir_.size() "<<detectedPersonsYakir_.size()<<endl;
        for(int i = 0; i < detectedPersonsYakir_.size(); i++){

            Markerarr.markers[i].header.frame_id = detectedPersonsYakir_[i].location_.header.frame_id;
            Markerarr.markers[i].header.stamp = ros::Time::now();
            Markerarr.markers[i].ns = "points_and_lines";
            Markerarr.markers[i].id = i+1;
            Markerarr.markers[i].action = visualization_msgs::Marker::ADD;
            Markerarr.markers[i].type = visualization_msgs::Marker::SPHERE;
            Markerarr.markers[i].pose.position.x = detectedPersonsYakir_[i].location_.point.x;
            Markerarr.markers[i].pose.position.y =  detectedPersonsYakir_[i].location_.point.y;
            Markerarr.markers[i].pose.position.z =  0;//detectedPersonsYakir_[i].location_.point.z;
            Markerarr.markers[i].pose.orientation.x = 0.0;
            Markerarr.markers[i].pose.orientation.y = 0.0;
            Markerarr.markers[i].pose.orientation.z = 0.0;
            Markerarr.markers[i].pose.orientation.w = 1.0;
            Markerarr.markers[i].scale.x = 0.3;
            Markerarr.markers[i].scale.y = 0.3;
            Markerarr.markers[i].scale.z = 0.3;
            Markerarr.markers[i].color.a = 1.0;
            Markerarr.markers[i].color.r = 0.0;
            Markerarr.markers[i].color.g = 1.0;
            Markerarr.markers[i].color.b = 0.0;

            //Markerarr.markers[i].lifetime = ros::Duration(1.0);


        }

        if( detectedPersonsYakir_.size() > 0)
            targets_marker_pub_.publish(Markerarr);
    }


    void updateGoalByShift(float x, float y, float shiftM, cv::Point2d& newP ){


        float currentDisnace = sqrt(pow(x,2) + pow(y, 2));

        float diff = currentDisnace - shiftM;

        if( diff <=0){
            newP = cv::Point2d(x,y);
            return;
        }

        float scalar = diff / currentDisnace;

        newP = cv::Point2d(x * scalar ,y * scalar);

    }

    void calculateGoalHeading(geometry_msgs::PoseStamped& currentGoal){


        //get current robot pose
        geometry_msgs::PoseStamped robotPose;


        try
        {   
            tf::StampedTransform transform;
            //get current robot pose
            yakir_tfListener_.lookupTransform(PERSON_GOAL_DEFAULT_FRAME_ID, ROBOT_FRAME_ID,
                                        ros::Time(0), transform);

            robotPose.pose.position.x = transform.getOrigin().x();
            robotPose.pose.position.y = transform.getOrigin().y();
            robotPose.pose.position.z = 0;
            robotPose_.pose.orientation.x = transform.getRotation().x();
            robotPose.pose.orientation.y = transform.getRotation().y();
            robotPose.pose.orientation.z = transform.getRotation().z();
            robotPose.pose.orientation.w = transform.getRotation().w();

            float angleFromTarget =  atan2(robotPose.pose.position.y - currentGoal.pose.position.y,
                robotPose.pose.position.x - currentGoal.pose.position.x);

            tf2::Quaternion orientation;
            orientation.setRPY( 0, 0, angle); 


            //set the new orientation
            currentGoal.pose.orientation.w =  orientation.getW(); 
            currentGoal.pose.orientation.x =  orientation.getX(); 
            currentGoal.pose.orientation.y =  orientation.getY(); 
            currentGoal.pose.orientation.z =  orientation.getZ(); 




            return true;
        }

        catch (...)
        {
            cerr << " error between " << mapFrame_ << " to " << baseLinkFrame_ << endl;
            return false;
        }
        
        



    }

    void publishMarkerGoal(const geometry_msgs::PoseStamped& currentGoal){

        visualization_msgs::Marker targetFromCameraPoseMsg;
        targetFromCameraPoseMsg.header.frame_id = currentGoal.header.frame_id;
        targetFromCameraPoseMsg.header.stamp = ros::Time::now();
        targetFromCameraPoseMsg.ns = "points_and_lines";
        targetFromCameraPoseMsg.id = 1;
        targetFromCameraPoseMsg.action = visualization_msgs::Marker::ADD;
        targetFromCameraPoseMsg.type = visualization_msgs::Marker::ARROW;
        targetFromCameraPoseMsg.pose.position.x = currentGoal.pose.position.x;
        targetFromCameraPoseMsg.pose.position.y = currentGoal.pose.position.y;
        targetFromCameraPoseMsg.pose.position.z =  currentGoal.pose.position.z;
        targetFromCameraPoseMsg.pose.orientation.x = currentGoal.pose.orientation.x;
        targetFromCameraPoseMsg.pose.orientation.y = currentGoal.pose.orientation.y;
        targetFromCameraPoseMsg.pose.orientation.z = currentGoal.pose.orientation.z;
        targetFromCameraPoseMsg.pose.orientation.w = currentGoal.pose.orientation.w;
        targetFromCameraPoseMsg.scale.x = 0.3;
        targetFromCameraPoseMsg.scale.y = 0.3;
        targetFromCameraPoseMsg.scale.z = 0.3;
        targetFromCameraPoseMsg.color.a = 1.0;
        targetFromCameraPoseMsg.color.r = 0.0;
        targetFromCameraPoseMsg.color.g = 0.0;
        targetFromCameraPoseMsg.color.b = 1.0;

        goal_marker_pub_.publish(targetFromCameraPoseMsg);

    }
};

#endif
