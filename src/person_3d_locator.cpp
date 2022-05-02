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
    detectedPersons.clear();

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

    // Publish tranform between base_link of the robot to camera
    // staticTranformBroadcaster.sendTransform(static_transformStamped);

    // // Publish transforms of realsense camera
    // realsense_camera.PublishTransforms(staticTranformBroadcaster);
    //yakir ??


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

    // realsense_camera.CaptureFrames();
    // iImageWidth = realsense_camera.rgbImage.cols;
    // iImageHeight = realsense_camera.rgbImage.cols;

  

    nodeHandler.setParam(PERSON_FOLLOWER_STATUS_PARAM, PERSON_FOLLOWER_RUNNING);

    // Ros node loop
    while (ros::ok())
    {
        // Check for ros interupts
        ros::spinOnce();

        //yakir
        if( !cameraInfoInited_  || !bgrInited_){
            continue;
        }

        iImageWidth = currentBgrImg_.cols;
        iImageHeight = currentBgrImg_.rows;

        // Reset target person IOU
        fTargetIOU = false;

        // Capture color and depth frame from camera
        // realsense_camera.CaptureFrames();

        // Publish captured frames
        // realsense_camera.PublishImage();
        // ROS_DEBUG("Frame Captured (%d, %d)", realsense_camera.rgbImage.cols, realsense_camera.rgbImage.rows);

        // Run Detections on captured frame
        //detectnetWrapper.Detect(realsense_camera.imageMessage);
        
        //yakir

        if( !currentBgrImg_.data){
            continue;
        }
        sensor_msgs::ImagePtr image_msg_detect = cv_bridge::CvImage(std_msgs::Header(), "bgr8", currentBgrImg_).toImageMsg();
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
            // bIsTargetFound = tracker->update(realsense_camera.rgbImage, rBoundingBox);
            
            //yakir
            bIsTargetFound = tracker->update(currentBgrImg_, rBoundingBox);


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

                // if (tracker->init(realsense_camera.rgbImage, rDetectionTargetBBox))
                // {
                //     ROS_DEBUG("Tracker Re-initialised!");
                //     tracker->update(realsense_camera.rgbImage, rBoundingBox);
                // }
                // else
                // {
                //     ROS_DEBUG("Tracker failed to  reinitialise!");
                // }

                //yakir
                if (tracker->init(currentBgrImg_, rDetectionTargetBBox))
                {
                    ROS_DEBUG("Tracker Re-initialised!");
                    tracker->update(currentBgrImg_, rBoundingBox);
                }
                else
                {
                    ROS_DEBUG("Tracker failed to  reinitialise!");
                }

            }

            // Prepare tracking image to publish
            // cv::rectangle(realsense_camera.rgbImage, rBoundingBox, cv::Scalar(255, 0, 0), 2, 1);
            // cv::rectangle(realsense_camera.rgbImage, rDetectionTargetBBox, cv::Scalar(0, 255, 0), 2, 1);

            //yakir
            cv::rectangle(currentBgrImg_, rBoundingBox, cv::Scalar(255, 0, 0), 2, 1);
            cv::rectangle(currentBgrImg_, rDetectionTargetBBox, cv::Scalar(0, 255, 0), 2, 1);

            // trackingImageMessage = cv_bridge::CvImage(std_msgs::Header(), PUBLISH_IMAGE_ENCODING, realsense_camera.rgbImage).toImageMsg();
            //yakir
            trackingImageMessage = cv_bridge::CvImage(std_msgs::Header(),
                 PUBLISH_IMAGE_ENCODING, currentBgrImg_).toImageMsg();

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
                targetedPerson.second.bbox.center.x = rBoundingBox.x + (rBoundingBox.width / 2);
                targetedPerson.second.bbox.center.y = rBoundingBox.y + (rBoundingBox.height / 2);
                targetedPerson.second.bbox.size_x = rBoundingBox.width;
                targetedPerson.second.bbox.size_y = rBoundingBox.height;
                // targetedPerson.first = realsense_camera.GetDistance(targetedPerson.second.bbox.center.x, targetedPerson.second.bbox.center.y);

                //yakir
                targetedPerson.first = yakirGetDistance(targetedPerson.second.bbox.center.x, targetedPerson.second.bbox.center.y);

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
            nearestPerson.first = NEAREST_PERSON_DEFAULT;
            detectedPersons.clear();

            for (auto detection : detectnetWrapper.detection2DArray.detections)
            {

                if ((detection.results[FALSE].id == PERSON_CLASS_ID) && (detection.results[FALSE].score >= PERSON_THRESHOLD))
                {
                    //fDepth = realsense_camera.GetDistance(detection.bbox.center.x, detection.bbox.center.y);      

                    //yakir 
                    fDepth = yakirGetDistance(detection.bbox.center.x, detection.bbox.center.y);      

                    if (fDepth <= FALSE)
                    {
                        ROS_INFO("Unable to get depth of person or the person is too far");
                        continue;
                    }

                    detectedPersons.push_back(std::make_pair(fDepth, detection));

                    if (fDepth < nearestPerson.first)
                    {
                        nearestPerson = std::make_pair(fDepth, detection);
                    }
                }

            } // for (auto detection : detectnetWrapper.detection2DArray.detections)
            if (nearestPerson.first != NEAREST_PERSON_DEFAULT)
            {
                targetedPerson = nearestPerson;

                if (CheckTargetRegion(targetedPerson.first) == FOLLOW)
                {
                    rBoundingBox = cv::Rect2d(static_cast<int>(targetedPerson.second.bbox.center.x - (targetedPerson.second.bbox.size_x / 2)),
                                              static_cast<int>(targetedPerson.second.bbox.center.y - (targetedPerson.second.bbox.size_y / 2)),
                                              static_cast<int>(targetedPerson.second.bbox.size_x),
                                              static_cast<int>(targetedPerson.second.bbox.size_y));

                    tracker.release();
                    // tracker = cv::TrackerCSRT::create();
                    tracker = cv::TrackerGOTURN::create();

                    // if (tracker->init(realsense_camera.rgbImage, rBoundingBox))
                    // {
                    //     ROS_DEBUG("Tracker Initialised!");
                    // }

                    //yakir
                    if (tracker->init(currentBgrImg_, rBoundingBox))
                    {
                        ROS_DEBUG("Tracker Initialised!");
                    }

                    bIsTargetLocked = true;
                }
            }

        } // else (bIsTargetLocked)

        if (bIsTargetLocked)
        {
            personCameraTarget.x = targetedPerson.second.bbox.center.x;
            personCameraTarget.y = targetedPerson.second.bbox.center.y;
            personCameraTarget.rawDepthRayDistance = targetedPerson.first;

            // tangentialDepth = targetDepth * cos(angle of bbox center from center of image)
            // angle of bbox center from center of image =
            ROS_DEBUG("Nearest person at %f, %f, %f", targetedPerson.second.bbox.center.x, targetedPerson.second.bbox.center.y, targetedPerson.first);

            // Compute the 3D coordinate of target person

            // Vertical View
            dPixelDifference = abs(HALF_OF(iImageHeight) - targetedPerson.second.bbox.center.y);
            dPixelAngleFromCenter = DEG_TO_RAD((dPixelDifference / HALF_OF(iImageHeight)) * (double)(HALF_OF(VERTICAL_FIELD_OF_VIEW)));
            fVerticalViewYDepthToCenterYDepthProjection = cos(dPixelAngleFromCenter) * targetedPerson.first;

            // Horizontal View
            dPixelDifference = abs(HALF_OF(iImageWidth) - targetedPerson.second.bbox.center.x);
            dPixelAngleFromCenter = DEG_TO_RAD((dPixelDifference / HALF_OF(iImageWidth)) * (double)(HALF_OF(HORIZONTAL_FIELD_OF_VIEW)));

            personCameraTarget.depthRayDistance = fVerticalViewYDepthToCenterYDepthProjection;
            personCameraTarget.angleX = dPixelAngleFromCenter;

            PrepareTargetPoseFromDepthRay(targetFromCameraPoseMsg, personCameraTarget.depthRayDistance, personCameraTarget.angleX, personCameraTarget.x);

            // Check the target region
            eTargetRegion = CheckTargetRegion(fVerticalViewYDepthToCenterYDepthProjection);

            // To maintain a offset distance between the target person and the robot while following
            if (eTargetRegion == FOLLOW)
            {
                fVerticalViewYDepthToCenterYDepthProjection -= fMinFollowRegion;
            }

            PrepareTargetPoseFromDepthRay(poseMsg, fVerticalViewYDepthToCenterYDepthProjection, personCameraTarget.angleX, personCameraTarget.x);

            if (eTargetRegion != FOLLOW)
            {
                poseMsg.pose.position.x = iZero;
                poseMsg.pose.position.y = iZero;
            }

            // Publish the goal pose of target
            PublishGoal(poseMsg);

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

        if (targetedPerson.second.bbox.center.x > (iImageWidth / 2))
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

