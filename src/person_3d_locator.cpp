/*=========================================================================
Filename : src/person_3d_locator.cpp
Project  : Person Follower
Purpose  : ROS node to publish the 3D pose of a targeted person by
           detecting the person from image captured using camera and also
           by locating the coordinates of the respective person
           in realworld
---------------------------------------------------------------------------
---------------------------------------------------------------------------
Version Number : 1.0
Last Updated   : September 15, 2021
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

#include "../include/person_3d_locator/person_3d_locator.h"
#include "../include/person_3d_locator/person_3d_locator_int.h"

float CalculateIOU(cv::Rect2d rBoxA, cv::Rect2d rBoxB);

#if TIME_MEASURE
ros::Duration recoveryElaspedTime;
ros::Time recoveryElaspedTimeStart;
ros::Time recoveryElaspedTimeEnd;
#endif

/*============================================================================
Name    :   Person3DLocator::Init
------------------------------------------------------------------------------
Purpose :   Init function for person follower class
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/

void Person3DLocator::Init()
{
    nodeHandler.setParam(PERSON_FOLLOWER_STATUS_PARAM, PERSON_FOLLOWER_INITALIZING);

    nodeHandlerPrivate = new ros::NodeHandle("~");
    // Static transform broadcaster
    tf2_ros::StaticTransformBroadcaster staticTranformBroadcaster;

    // Static transform pose
    geometry_msgs::TransformStamped static_transformStamped;

    // Goal pose publisher
    posePublisher = nodeHandler.advertise<geometry_msgs::PoseStamped>(NEAREST_PERSON_TOPIC, NEAREST_PERSON_MSG_QUEUE_SIZE);

    // Tracking image transporter
    image_transport::ImageTransport imageTransporter(nodeHandler);
    trackingImagePublisher = imageTransporter.advertise(TRACKING_IMAGE_TOPIC, TRACKING_IMAGE_QUEUE_SIZE);

    // Search rotation dwell time
    prevRotationEndTime = ros::Time::now();

    // CSRT tracker to track the target person
    // tracker = cv::TrackerCSRT::create();
    tracker = cv::TrackerGOTURN::create();

    if (!tracker)
    {
        ROS_DEBUG("Failed to create tracker");
    }

    // Load detection model
    detectnetWrapper.LoadModel();

    // Initalize (or) reset detected pesons
    detectedPersonsYakir_.clear();

    // Initial goal is relative to color camera frame
    poseMsg.header.frame_id = CAMERA_FRAME_ID;

    // Initialize person follower flags & variables
    bIsTargetFound = false;
    bIsTargetLocked = false;
    bIsRotationInitiated = false;
    fRotation = ROTATION_DEFAULT;
    fCriticalIOUThreshold = CRITICAL_IOU_THRESHOLD;
    fWarningIOUThreshold = WARNING_IOU_THRESHOLD;
    fSearchDwellDuration = SEARCH_DWELL_DURATION;
    fMinFollowRegion = MIN_FOLLOW_REGION_DEFAULT;
    fMaxFollowRegion = MAX_FOLLOW_REGION_DEFAULT;
    sGoalFrameName = PERSON_GOAL_DEFAULT_FRAME_ID;

    // Initalize tranform between base_link of the robot to camera
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = ROBOT_FRAME_ID;
    static_transformStamped.child_frame_id = CAMERA_FRAME_ID;
    static_transformStamped.transform.translation.x = BASE_LINK_CAMERA_TRANSFORM_X;
    static_transformStamped.transform.translation.y = BASE_LINK_CAMERA_TRANSFORM_Y;
    static_transformStamped.transform.translation.z = BASE_LINK_CAMERA_TRANSFORM_Z;
    static_transformStamped.transform.rotation.w = true;

   

    // Get frame id to tranform the goal from the camera frame
    if (nodeHandlerPrivate->hasParam(GOAL_FRAME_PARAM_NAME))
    {
        nodeHandlerPrivate->getParam(GOAL_FRAME_PARAM_NAME, sGoalFrameName);
    }

    ROS_INFO("Goal frame: %s", sGoalFrameName.c_str());

    // Get frame id to tranform the goal from the camera frame
    if (nodeHandlerPrivate->hasParam(MIN_FOLLOW_REGION_PARAM))
    {
        nodeHandlerPrivate->getParam(MIN_FOLLOW_REGION_PARAM, fMinFollowRegion);
    }

    ROS_INFO("Min follow region : %f", fMinFollowRegion);

    // Get frame id to tranform the goal from the camera frame
    if (nodeHandlerPrivate->hasParam(MAX_FOLLOW_REGION_PARAM))
    {
        nodeHandlerPrivate->getParam(MAX_FOLLOW_REGION_PARAM, fMaxFollowRegion);
    }

    ROS_INFO("Max follow region : %f", fMaxFollowRegion);

    ROS_INFO("Max follow region : %f", fMaxFollowRegion);

    // Initialize transform listener to listen the transforms
    tfListener = new tf2_ros::TransformListener(tfBuffer);

    // Start the move base action client
    actionClient = new MoveBaseClient(moveBaseClientName, true);

    if (actionClient == NULL)
    {
        ROS_INFO("Failed to start the action client");
    }

    // wait for the action server to come up
    while (!actionClient->waitForServer(ros::Duration(moveBaseClientServerWaitDuration)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
}

/*============================================================================
Name    :   Person3DLocator::Run
------------------------------------------------------------------------------
Purpose :   To implement the control flow/operation of person follower
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/

void Person3DLocator::Run()
{

    float fDepth;
    float fHorizontalViewXPosition, fHorizontalViewYPosition;
    float fVerticalViewYDepthToCenterYDepthProjection;
    tf2::Quaternion qQuaternionBase;
    double dPixelDifference;
    double dPixelAngleFromCenter;
    cv::Rect2d rBBoxTemp, rDetectionTargetBBox;
    float fIOU = false;
    float fTargetIOU = false;
    TargetRegion eTargetRegion;
    int iTrackerRetryAttempts = static_cast<int>(false);

    
  

    nodeHandler.setParam(PERSON_FOLLOWER_STATUS_PARAM, PERSON_FOLLOWER_RUNNING);

    // Ros node loop
    while (ros::ok())
    {
        // Check for ros interupts
        ros::spinOnce();

        //yakir
        if( !cameraInfoInited_  || !bgrInited_ ){
            continue;
        }

        iImageWidth = currentBgrImg_.cols;
        iImageHeight = currentBgrImg_.rows;



        // Reset target person IOU
        fTargetIOU = false;

       

        if( !currentBgrImg_.data || !curretDepthImg_.data){
            cerr<<" bad imgs "<<endl;
            continue;
        }

        Mat bgrWorkImg = currentBgrImg_.clone();
        Mat depthImg = curretDepthImg_.clone();


        sensor_msgs::ImagePtr image_msg_detect = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgrWorkImg).toImageMsg();
        detectnetWrapper.Detect(image_msg_detect);

        // Publish Detections
        detectnetWrapper.PublishDetections();



        // Disable the person 3d locator, if disable is set
        if (nodeHandlerPrivate->hasParam(PERSON_3D_LOCATE_DISABLE_PARAM))
        {   
            nodeHandlerPrivate->getParam(PERSON_3D_LOCATE_DISABLE_PARAM, bPerson3dLocateDisable);


            if (bPerson3dLocateDisable)
            {   

                bIsTargetLocked = false;

                nodeHandler.setParam(PERSON_FOLLOWER_STATUS_PARAM, PERSON_FOLLOWER_STOPPED);

                while (qRotationGoalQueue.size() > static_cast<int>(false))
                {
                    qRotationGoalQueue.pop();
                }


                ROS_WARN("Person 3D Locator is Disabled. Set person_3d_locate_disable parameter to false to enable it.");
                continue;
            }

            nodeHandler.setParam(PERSON_FOLLOWER_STATUS_PARAM, PERSON_FOLLOWER_RUNNING);
        }
        // End of disable section

        // When target is locked
        if (bIsTargetLocked)
        {   
            ROS_INFO("updating position of tracker");

#if TIME_MEASURE
            recoveryElaspedTimeStart = ros::Time::now();
#endif

            // Fetch the current location of tracked object in image
            
            //yakir
            bIsTargetFound = tracker->update(bgrWorkImg, rBoundingBox);

#if TIME_MEASURE
            recoveryElaspedTimeEnd = ros::Time::now();
            recoveryElaspedTime = recoveryElaspedTimeEnd - recoveryElaspedTimeStart;
            ROS_DEBUG("Tracker update took : %d seconds", recoveryElaspedTime.toSec());
#endif

            // Calculate IOU for tracked person with the detected persons
            for (auto detection : detectnetWrapper.detection2DArray.detections)
            {
                if ((detection.results[FALSE].id == PERSON_CLASS_ID) & (detection.results[FALSE].score >= PERSON_THRESHOLD))
                {
                    rBBoxTemp = cv::Rect2d(static_cast<int>(detection.bbox.center.x - (detection.bbox.size_x / 2)),
                                           static_cast<int>(detection.bbox.center.y - (detection.bbox.size_y / 2)),
                                           static_cast<int>(detection.bbox.size_x), static_cast<int>(detection.bbox.size_y));

                    // Get IOU
                    fIOU = CalculateIOU(rBoundingBox, rBBoxTemp);

                    // Find the detection having maximum IOU with the tracker
                    if (fIOU > fTargetIOU)
                    {
                        fTargetIOU = fIOU;
                        rDetectionTargetBBox = rBBoxTemp;
                    }
                }
            }


            ROS_DEBUG("Tracker has IOU of %f", fTargetIOU);

            // Tracker lost the target
            if (fTargetIOU < fCriticalIOUThreshold)
            {
                bIsTargetFound = false;
            }

            // Tracker located the target with some offset, re-initialize the tracker with detection that has maximum IOU
            else if ((fTargetIOU > fCriticalIOUThreshold) && (fTargetIOU < fWarningIOUThreshold))
            {   
                tracker.release();
                // tracker = cv::TrackerCSRT::create();
                tracker = cv::TrackerGOTURN::create();

                if (tracker->init(bgrWorkImg, rDetectionTargetBBox))
                {   
                    ROS_DEBUG("Tracker Re-initialised!");
                    tracker->update(bgrWorkImg, rBoundingBox);
                }
                else
                {
                    ROS_DEBUG("Tracker failed to  reinitialise!");
                }


            }

            // Prepare tracking image to publish
            // cv::rectangle(realsense_camera.rgbImage, rBoundingBox, cv::Scalar(255, 0, 0), 2, 1);
            // cv::rectangle(realsense_camera.rgbImage, rDetectionTargetBBox, cv::Scalar(0, 255, 0), 2, 1);

            
            cv::rectangle(bgrWorkImg, rBoundingBox, cv::Scalar(255, 0, 0), 2, 1);
            cv::rectangle(bgrWorkImg, rDetectionTargetBBox, cv::Scalar(0, 255, 0), 2, 1);

            
            trackingImageMessage = cv_bridge::CvImage(std_msgs::Header(),
                 PUBLISH_IMAGE_ENCODING, bgrWorkImg).toImageMsg();

            trackingImageMessage->header.frame_id = TRACKING_IMAGE_FRAME_ID;
            trackingImagePublisher.publish(trackingImageMessage);

            // Target is located by the tracker
            if (bIsTargetFound)
            {
                ROS_INFO("***************Target Located******************");

                if (bIsRotationInitiated)
                {
                    ROS_DEBUG("Cancelling all the previous goals");

                    while (qRotationGoalQueue.size() > iZero)
                    {
                        qRotationGoalQueue.pop();
                    }

                    actionClient->cancelAllGoals();
                }

                // Reset target search flags
                bIsRotationInitiated = false;

                // Update the target person location
                targetedPersonYakir_.box_.bbox.center.x = rBoundingBox.x + (rBoundingBox.width / 2);
                targetedPersonYakir_.box_.bbox.center.y = rBoundingBox.y + (rBoundingBox.height / 2);
                targetedPersonYakir_.box_.bbox.size_x = rBoundingBox.width;
                targetedPersonYakir_.box_.bbox.size_y = rBoundingBox.height;

                //yakir
                targetedPersonYakir_.distnace_ = yakirGetDistance(targetedPersonYakir_.box_.bbox.center.x,
                     targetedPersonYakir_.box_.bbox.center.y, depthImg);

            } // if (bIsTargetFound)

            else
            {

                ROS_INFO("Target Lost!");
                ROS_INFO("Time: %f - %f", (ros::Time::now() - prevRotationEndTime).toSec(), fSearchDwellDuration);
                if (iTrackerRetryAttempts < TRACKER_RETRY_ATTEMPTS)
                {
                    ROS_INFO("Retrying...");
                    iTrackerRetryAttempts++;
                    continue;
                }

                else if ((bIsTargetLocked == true) && (bIsRotationInitiated == false))
                {
                    bIsRotationInitiated = true;
                    // actionClient->cancelAllGoals();
                    getCameraPose();
                    ROS_INFO("Initiating rotation");

                    // Reset rotation variables
                    fRotation = ROTATION_DEFAULT;
                    prevRotationEndTime = ros::Time::now();
                    PublishRotationGoal();
                }

                else if ((ros::Time::now() - prevRotationEndTime).toSec() > fSearchDwellDuration)
                {
                    // Initiate target search
                    // Note : Target locked status can get modified in move base done goal callback, hence it's necessary to check for the target locked status
                    // before initiating the rotation, otherwise, it might end up in infinite rotation behaviour

                    bIsTargetLocked = false;

                    ROS_INFO("Delay in executing recovery behaviour, hence cancelling it and returning to original pose");

                    while (qRotationGoalQueue.size() > iZero)
                    {
                        qRotationGoalQueue.pop();
                    }

                    actionClient->cancelAllGoals();
                    prevRotationEndTime = ros::Time::now();
                    PublishGoal(recoveryStartPoseMsg, true);
                }

                continue;

            } // else (bIsTargetFound)

        } // if (bIsTargetLocked)

        else
        {
            nearestPersonYakir_.distnace_ = NEAREST_PERSON_DEFAULT;
            detectedPersonsYakir_.clear();


            // Mat depthGrayscale;
            // double minVal;
            // double maxVal;
            // minMaxLoc(depthImg, &minVal, &maxVal);
            // depthImg.convertTo(depthGrayscale, CV_8UC1, (255 / (maxVal - minVal)));
            // cv::Mat distanceTransformImg;
            // distanceTransform(depthGrayscale, distanceTransformImg, DIST_L2, 3);
            // normalize(distanceTransformImg, distanceTransformImg, 0, 1.0, NORM_MINMAX);   

            // cvtColor(depthGrayscale, depthGrayscale, COLOR_GRAY2BGR);

            int count = 0;
            cerr<<" num of detection "<<detectnetWrapper.detection2DArray.detections.size()<<endl;
            for (auto detection : detectnetWrapper.detection2DArray.detections)
            {
                count++;
                if ((detection.results[FALSE].id == PERSON_CLASS_ID) && (detection.results[FALSE].score >= PERSON_THRESHOLD))
                {
                    cv::Rect depth_bounding_box = cv::Rect2d(static_cast<int>(detection.bbox.center.x - (detection.bbox.size_x / 2)),
                                           static_cast<int>(detection.bbox.center.y - (detection.bbox.size_y / 2)),
                                           static_cast<int>(detection.bbox.size_x), static_cast<int>(detection.bbox.size_y));                  

                    auto centerDepth = cv::Point2d(depth_bounding_box.x + (depth_bounding_box.width /2),
                        depth_bounding_box.y + (depth_bounding_box.height /2));

                    //yakir 
                    fDepth = yakirGetDistance(centerDepth.x, centerDepth.y, depthImg);      
                    cerr<<" fDepth "<<fDepth<<" index "<<count<<endl;
                    if (fDepth  < 0)
                    {
                        ROS_INFO("Unable to get depth of person or the person is too far");
                        continue;
                    }     

                    // cv::rectangle(depthGrayscale, depth_bounding_box, cv::Scalar(0,255,0), 2);
                    // cv::circle(depthGrayscale, centerDepth, 5 , cv::Scalar(0,255,0), -1);

                    YakirPerson person;
                    person.box_ = detection;
                    person.distnace_ = fDepth;
                    extractDepthFromBboxObject(centerDepth, 
                        fDepth, person.location_, "odom");

                    detectedPersonsYakir_.push_back(person);

                    if (person.distnace_ < nearestPersonYakir_.distnace_)
                    {
                        nearestPersonYakir_.distnace_ = fDepth;
                        nearestPersonYakir_.box_ = detection;
                        nearestPersonYakir_.location_ = person.location_;
                    }

                }

            }

            // auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depthGrayscale).toImageMsg();
            // debug_depth_pub_.publish(msg);

            if( true){
                
                cerr<<" publishTargetsMarkers "<<detectedPersonsYakir_.size()<<endl;
                publishTargetsMarkers();
            }
            

            if (nearestPersonYakir_.distnace_ != NEAREST_PERSON_DEFAULT)
            {   
                targetedPersonYakir_ = nearestPersonYakir_;

                if (CheckTargetRegion(targetedPersonYakir_.distnace_) == FOLLOW)
                {
                    rBoundingBox = cv::Rect2d(static_cast<int>(targetedPersonYakir_.box_.bbox.center.x - (targetedPersonYakir_.box_.bbox.size_x / 2)),
                                              static_cast<int>(targetedPersonYakir_.box_.bbox.center.y - (targetedPersonYakir_.box_.bbox.size_y / 2)),
                                              static_cast<int>(targetedPersonYakir_.box_.bbox.size_x),
                                              static_cast<int>(targetedPersonYakir_.box_.bbox.size_y));


                    tracker.release();
                    // tracker = cv::TrackerCSRT::create();
                    tracker = cv::TrackerGOTURN::create();


                    //yakir
                    if (tracker->init(bgrWorkImg, rBoundingBox))
                    {
                        ROS_DEBUG("Tracker Initialised!");
                    }

                    cerr<<" yakir 6 bIsTargetLocked"<<endl;

                    bIsTargetLocked = true; // NEED TO SET TRUE!!
                }
            }

        } // else (bIsTargetLocked)

        if (bIsTargetLocked)
        {           
            
            //yakir
            targetFromCameraPoseMsg.header.frame_id = "odom";
            targetFromCameraPoseMsg.pose.position.x = targetedPersonYakir_.location_.point.x;
            targetFromCameraPoseMsg.pose.position.y = targetedPersonYakir_.location_.point.y;
            targetFromCameraPoseMsg.pose.position.z = 0;
            targetFromCameraPoseMsg.pose.orientation.x = 0;
            targetFromCameraPoseMsg.pose.orientation.y = 0;
            targetFromCameraPoseMsg.pose.orientation.z = 0;
            targetFromCameraPoseMsg.pose.orientation.w = 1;



            cerr<<" the goal is :"<<targetFromCameraPoseMsg.pose.position.x<<", "<<targetFromCameraPoseMsg.pose.position.y<<endl;

            // cv::Point2d newP;
            // updateGoalByShift(targetFromCameraPoseMsg.pose.position.x,targetFromCameraPoseMsg.pose.position.y,
            //     0.2, newP );
            // targetFromCameraPoseMsg.pose.position.x = newP.x;
            // targetFromCameraPoseMsg.pose.position.y = newP.y;
            // targetFromCameraPoseMsg.pose.position.z = 0;          


            if (eTargetRegion != FOLLOW)
            {
                poseMsg.pose.position.x = iZero;
                poseMsg.pose.position.y = iZero;
            }

            // Publish the goal pose of target
            PublishGoal(targetFromCameraPoseMsg);

            iTrackerRetryAttempts = iZero;
        }
    }
}

/*============================================================================
Name    :   Person3DLocator::PublishGoal
------------------------------------------------------------------------------
Purpose :   To publish goal relative to user defined goal frame
Input   :   n/a
Output  :   n/a
Notes   :   Reference https://answers.ros.org/question/323075/transform-the-coordinate-frame-of-a-pose-from-one-fixed-frame-to-another/
============================================================================*/


bool Person3DLocator::extractDepthFromBboxObject( cv::Point2d pix, float d,
           geometry_msgs::PointStamped& pose, string targetFrame ) {
               
        string source_frame = "camera_depth_optical_frame";
       
        if( isinf(d) || d == 0){
            return false;
        }

        float dM = d ;
        cv::Point3d p = cv::Point3d(((pix.x - cx_) * dM / fx_), ((pix.y - cy_) * dM / fy_), dM);

        pose = transformToByFrames(p, targetFrame , source_frame );
           

        return true;
    }



void Person3DLocator::PublishGoal(geometry_msgs::PoseStamped goalPoseMsg, bool bUseActionServer)
{
    move_base_msgs::MoveBaseGoal goal;

    ROS_DEBUG("Pose from %s - (%f, %f, %f), (%f, %f, %f, %f)", goalPoseMsg.header.frame_id.c_str(), goalPoseMsg.pose.position.x,
              goalPoseMsg.pose.position.y,
              goalPoseMsg.pose.position.z,
              goalPoseMsg.pose.orientation.x,
              goalPoseMsg.pose.orientation.y,
              goalPoseMsg.pose.orientation.z,
              goalPoseMsg.pose.orientation.w);

    if (goalPoseMsg.header.frame_id == CAMERA_FRAME_ID)
    {
        goalPoseMsg.pose.position.x += (BASE_LINK_CAMERA_TRANSFORM_X > 0) ? (-BASE_LINK_CAMERA_TRANSFORM_X) : (BASE_LINK_CAMERA_TRANSFORM_X);
        goalPoseMsg.pose.position.y += (BASE_LINK_CAMERA_TRANSFORM_Y > 0) ? (-BASE_LINK_CAMERA_TRANSFORM_Y) : (BASE_LINK_CAMERA_TRANSFORM_Y);
    }

    transformPoseToOdom(goalPoseMsg);

    goalPoseMsg.pose.position.z = iZero;

    ROS_DEBUG("Goal - (%f, %f, %f), (%f, %f, %f, %f)", goalPoseMsg.pose.position.x,
              goalPoseMsg.pose.position.y,
              goalPoseMsg.pose.position.z,
              goalPoseMsg.pose.orientation.x,
              goalPoseMsg.pose.orientation.y,
              goalPoseMsg.pose.orientation.z,
              goalPoseMsg.pose.orientation.w);

    // Publish
    if (bUseActionServer)
    {
        goal.target_pose = goalPoseMsg;
        ROS_INFO("Sending goal using action server");
        actionClient->sendGoal(goal, boost::bind(&Person3DLocator::doneWithGoalCallback, this, _1, _2), NULL, NULL);
    }

    else
    {
        posePublisher.publish(goalPoseMsg);
    }
}

/*============================================================================
Name    :   Person3DLocator::PublishRotationGoal
------------------------------------------------------------------------------
Purpose :   To compute and publish rotation goal
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/

void Person3DLocator::PublishRotationGoal()
{
    tf2::Quaternion qQuaternionCam, qQuaternionTargetFromCam, qQuaternionTarget;
    int iRotationSteps;
    float fTargetDepthRayDistance{personCameraTarget.depthRayDistance - fRecoveryTargetOffset};
    geometry_msgs::PoseStamped poseMsg;

    PrepareTargetPoseFromDepthRay(poseMsg, fTargetDepthRayDistance, personCameraTarget.angleX, personCameraTarget.x);

    transformPoseToOdom(poseMsg);
    recoveryStartPoseMsg = poseMsg;

    PublishGoal(poseMsg, true);

    for (iRotationSteps = static_cast<int>(true); iRotationSteps <= (FULL_ROTATION_DEGREE / ROTATION_STEP); iRotationSteps++)
    {

        if (targetedPersonYakir_.box_.bbox.center.x > (iImageWidth / 2))
        {
            ROS_INFO("Target is on right side");
            qQuaternionTargetFromCam.setRPY(FALSE, FALSE, -DEG_TO_RAD(ROTATION_STEP));
        }
        else
        {
            ROS_INFO("Target is on left side");
            qQuaternionTargetFromCam.setRPY(FALSE, FALSE, DEG_TO_RAD(ROTATION_STEP));
        }

        ROS_INFO("Rotating %d degree (or) %f radians", iRotationSteps * ROTATION_STEP, (DEG_TO_RAD(ROTATION_STEP * iRotationSteps)));

        tf2::convert(poseMsg.pose.orientation, qQuaternionCam);

        qQuaternionTarget = qQuaternionTargetFromCam * qQuaternionCam;
        qQuaternionTarget.normalize();

        tf2::convert(qQuaternionTarget, poseMsg.pose.orientation);
        // poseMsg.header.frame_id = sGoalFrameName;

        qRotationGoalQueue.push(poseMsg);
    }
}

/*============================================================================
Name    :   Person3DLocator::CheckTargetRegion
------------------------------------------------------------------------------
Purpose :   To identify the region in which the target is located
Input   :   dDepth - distance of target person from the camera
Output  :   TargetRegion - Region in which the target is located
Notes   :   n/a
============================================================================*/

TargetRegion Person3DLocator::CheckTargetRegion(double dDepth)
{
    if (dDepth <= fMinFollowRegion)
    {
        return NEAR;
    }

    else if (dDepth > fMaxFollowRegion)
    {
        return FAR;
    }

    else
    {
        return FOLLOW;
    }
}

/*============================================================================
Name    :   CalculateIOU
------------------------------------------------------------------------------
Purpose :   To compute and publish rotation goal
Input   :   rBoxA - 2D Bounding box
            rBoxB - 2D Bounding box

Output  :   IOU value between two input boxes
Notes   :   References :
            https://www.pyimagesearch.com/2016/11/07/intersection-over-union-iou-for-object-detection/
============================================================================*/

float CalculateIOU(cv::Rect2d rBoxA, cv::Rect2d rBoxB)
{
    float fIOU;
    double iIntersectionArea;
    double iBoxAArea, iBoxBArea;

    // determine the (x, y)-coordinates of the intersection rectangle
    double iIntersectionX1 = std::max(rBoxA.x, rBoxB.x);
    double iIntersectionY1 = std::max(rBoxA.y, rBoxB.y);
    double iIntersectionX2 = std::min(rBoxA.x + rBoxA.width, rBoxB.x + rBoxB.width);
    double iIntersectionY2 = std::min(rBoxA.y + rBoxA.height, rBoxB.y + rBoxB.height);

    if (iIntersectionX2 < iIntersectionX1)
    {
        return false;
    }

    // compute the area of intersection rectangle
    iIntersectionArea = std::max((double)false, iIntersectionX2 - iIntersectionX1 + true) * std::max((double)false, iIntersectionY2 - iIntersectionY1 + true);

    iBoxAArea = rBoxA.width * rBoxA.height;
    iBoxBArea = rBoxB.width * rBoxB.height;

    // compute the intersection over union by taking the intersection
    // area and dividing it by the sum of prediction + ground-truth
    // areas - the interesection area
    fIOU = iIntersectionArea / float(iBoxAArea + iBoxBArea - iIntersectionArea);

    // return the intersection over union value
    return fIOU;
}

/*============================================================================
Name    :   Person3DLocator::doneWithGoalCallback()
------------------------------------------------------------------------------
Purpose :   DCalled once the move base is done with the current goal
Input   :   state - State of the goal
            result - Result of the goal
Output  :   n/a
Notes   :   n/a
============================================================================*/

void Person3DLocator::doneWithGoalCallback(const actionlib::SimpleClientGoalState &state,
                                           const move_base_msgs::MoveBaseResult::ConstPtr &result)
{
    geometry_msgs::PoseStamped poseMsg;

    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Pending Goals: %d", static_cast<int>(qRotationGoalQueue.size()));
    ROS_INFO("Goal reached in %f", (ros::Time::now() - prevRotationEndTime).toSec());

    if (qRotationGoalQueue.size() >= static_cast<int>(true))
    {
        prevRotationEndTime = ros::Time::now();
        if ((state == actionlib::SimpleClientGoalState::ABORTED) || (state == actionlib::SimpleClientGoalState::REJECTED) ||
            (state == actionlib::SimpleClientGoalState::LOST) || (qRotationGoalQueue.size() == static_cast<int>(true)))
        {
            ROS_DEBUG("Rotation is about to be completed, hence releasing the target");
            bIsTargetLocked = false;
            bIsRotationInitiated = false;

            emptyRotationGoalQueue();
            PublishGoal(recoveryStartPoseMsg, true);

            return;
        }

        poseMsg = qRotationGoalQueue.front();
        qRotationGoalQueue.pop();

        PublishGoal(poseMsg, true);
    }
}

void Person3DLocator::PrepareTargetPoseFromDepthRay(geometry_msgs::PoseStamped &poseMsg, float fTargetDepthRayDistance, float fTargetAngle, int iTargetPersonXCenterInImage)
{

    tf2::Quaternion qQuaternionCam;
    tf2::Quaternion QuaternionTargetFromCam;
    tf2::Quaternion qQuaternionTarget;
    float fXPosition;
    float fYPosition;
    float fRotation;
    TargetRegion eTargetRegion;

    fXPosition = cos(fTargetAngle) * fTargetDepthRayDistance;
    fYPosition = sin(fTargetAngle) * fTargetDepthRayDistance;

    poseMsg.header.stamp = ros::Time::now();
    poseMsg.header.frame_id = CAMERA_FRAME_ID;
    poseMsg.pose.position.x = fXPosition;
    poseMsg.pose.position.y = (HALF_OF(iImageWidth) - iTargetPersonXCenterInImage) > iZero ? fYPosition : -fYPosition;
    poseMsg.pose.position.z = iZero;

    // Rotation in Roll, Pitch & Yaw
    qQuaternionTargetFromCam.setRPY(FALSE, FALSE, fTargetAngle);
    qQuaternionTarget = qQuaternionTargetFromCam;
    qQuaternionTarget.normalize();
    tf2::convert(qQuaternionTarget, poseMsg.pose.orientation);
}

void Person3DLocator::getCameraPose()
{
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped poseStamped{};
    poseStamped.pose.orientation.w = true;
    poseStamped.header.frame_id = CAMERA_FRAME_ID;

    try
    {
        // ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        transformStamped = tfBuffer.lookupTransform(sGoalFrameName, poseStamped.header.frame_id, ros::Time(false));

        // Tranform the pose from color camera frame to user defined frame
        tf2::doTransform(poseStamped, lastKnownCameraPoseFromOdom, transformStamped);
    }

    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
}

void Person3DLocator::transformPoseToOdom(geometry_msgs::PoseStamped &toTransformPose)
{
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped poseStamped{};

    try
    {
        // ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        transformStamped = tfBuffer.lookupTransform(sGoalFrameName, toTransformPose.header.frame_id, ros::Time(false));

        // Tranform the pose from color camera frame to user defined frame
        tf2::doTransform(toTransformPose, poseStamped, transformStamped);
    }

    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    toTransformPose = poseStamped;
}

void Person3DLocator::emptyRotationGoalQueue()
{
    while (qRotationGoalQueue.size() > iZero)
    {
        qRotationGoalQueue.pop();
    }
}

/*============================================================================
Name    :   Person3DLocator()
------------------------------------------------------------------------------
Purpose :   Destructor for person 3D locator
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/

Person3DLocator::~Person3DLocator()
{
    nodeHandler.setParam(PERSON_FOLLOWER_STATUS_PARAM, PERSON_FOLLOWER_STOPPED);
}
