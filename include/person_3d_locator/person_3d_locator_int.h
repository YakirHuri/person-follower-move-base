/*=========================================================================
Filename : include/person_3d_locator/person_3d_locator_int.h
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

#include<iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection2D.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <vector>
#include <utility> 
#include <cstring>
#include <math.h>

// Param names
#define PARAM_NAMESPACE				            "person_3d_locator"
#define GOAL_FRAME_PARAM_NAME                   "goal_frame_id"
#define IMAGE_TOPIC_PARAM                       "color_image_topic"
#define DETECTIONS_TOPIC_PARAM                  "detections"
#define MIN_FOLLOW_REGION_PARAM			        "min_follow_distance"
#define MAX_FOLLOW_REGION_PARAM			        "max_follow_distance"
#define PERSON_3D_LOCATE_DISABLE_PARAM		        "person_3d_locate_disable"

// Frame IDs
#define TRACKING_IMAGE_FRAME_ID                 ("tracking_frame")
#define MAP_FRAME_ID                            ("map")
#define ROBOT_FRAME_ID                          ("base_footprint")
#define PERSON_GOAL_DEFAULT_FRAME_ID            ("odom")

//Topics
#define NEAREST_PERSON_TOPIC                    ("/move_base_simple/goal")
#define TRACKING_IMAGE_TOPIC                    ("tracking")

// Queue size
#define NEAREST_PERSON_MSG_QUEUE_SIZE           (1)
#define TRACKING_IMAGE_QUEUE_SIZE               (1)

// Default values
#define ROTATION_DEFAULT                        (-FULL_ROTATION_DEGREE)
#define NEAREST_PERSON_DEFAULT                  (100)

// Distance in meters
#define TARGET_OFFSET_DISTANCE                  (0.8)
#define MIN_FOLLOW_REGION_DEFAULT               (TARGET_OFFSET_DISTANCE)
#define MAX_FOLLOW_REGION_DEFAULT               (4.5)

#define RAD_TO_DEG(X)                           ((X) * (180 / 3.14159265))
#define DEG_TO_RAD(X)                           ((X) * (3.14159265 / 180))

#define HORIZONTAL_FIELD_OF_VIEW                (87)  // In Degrees
#define VERTICAL_FIELD_OF_VIEW                  (58)  // In Degrees
#define HALF_OF(X)                              ((X) / 2)

// IOU Thresholds
#define WARNING_IOU_THRESHOLD                    (0.60)
#define CRITICAL_IOU_THRESHOLD                   (0.40)

// Search and Rotation
#define FULL_ROTATION_DEGREE                    (360) // In Degrees
#define ROTATION_STEP                           (30)  // In Degrees
#define SEARCH_DWELL_DURATION                   (30)   // In Seconds

#define TRACKER_RETRY_ATTEMPTS			(5)

//Base link to Camera Tranform
#define BASE_LINK_CAMERA_TRANSFORM_X            (0.3)
#define BASE_LINK_CAMERA_TRANSFORM_Y            (0)
#define BASE_LINK_CAMERA_TRANSFORM_Z            (1)


#define PERSON_CLASS_ID                         (1)
#define PERSON_THRESHOLD                        (0.7)
#define TRUE                                    (1)
#define FALSE                                   (0)



// Status
#define PERSON_FOLLOWER_STATUS_PARAM            ("status")
#define PERSON_FOLLOWER_INITALIZING             ("INITIALIZING")
#define PERSON_FOLLOWER_RUNNING                 ("RUNNING")
#define PERSON_FOLLOWER_STOPPED                 ("STOPPED")


// For Debug
#define TIME_MEASURE                            (TRUE)

constexpr char moveBaseClientName[] { "move_base" };
constexpr float moveBaseClientServerWaitDuration { 5.0 } ;
constexpr int iZero {0};
constexpr float fRecoveryTargetOffset {BASE_LINK_CAMERA_TRANSFORM_X + 0.15}; //In meters

