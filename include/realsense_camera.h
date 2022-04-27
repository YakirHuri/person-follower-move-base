
#ifndef REALSENSE_CAMERA_H
#define REALSENSE_CAMERA_H

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


class RealsenseCamera
{
    private:
        // Camera pipeline
        rs2::pipeline pipeline;

        // Profile for the pipeline
        rs2::pipeline_profile profile;

        // Config for the pipeline
        rs2::config config;

        // Frameset (has both depth and rgb frame)
        rs2::frameset frameset;
        rs2::frameset current_frameset;

        // Depth Frame
        rs2::depth_frame *depthFrame;
        
        // To align depth frame to the size of RGB image
        rs2::align *align_to_color;
        
        float depthScale;
        rs2::decimation_filter dec;
        rs2::disparity_transform depth2disparity;
        rs2::spatial_filter spat;
        rs2::temporal_filter temp;
        ros::NodeHandle* pNodeHandlerCamera;
        bool bIsFrameChanged = false;
        rs2::frame_queue postprocessed_frames;
        ros::Subscriber robotPoseSubscriber;
        
    public:
        image_transport::Publisher rbgImagePublisher;
        sensor_msgs::ImagePtr imageMessage;
        cv::Mat rgbImage;

        /*============================================================================
        Name    :   RealsenseCamera()
        ------------------------------------------------------------------------------
        Purpose :   Constructor for realsense camera to init the camera
        Input   :   n/a
        Output  :   n/a
        Notes   :   n/a
        ============================================================================*/
        RealsenseCamera();

        /*============================================================================
        Name    :   ~RealsenseCamera()
        ------------------------------------------------------------------------------
        Purpose :   Destructor to release the camera access
        Input   :   n/a
        Output  :   n/a
        Notes   :   n/a
        ============================================================================*/

        ~RealsenseCamera();

        /*============================================================================
        Name    :   CaptureFrames()
        ------------------------------------------------------------------------------
        Purpose :   To capture image from the camera
        Input   :   n/a
        Output  :   n/a
        Notes   :   n/a
        ============================================================================*/

        void CaptureFrames();

        /*============================================================================
        Name    :   PublishImage()
        ------------------------------------------------------------------------------
        Purpose :   To publish captured image as ros message
        Input   :   n/a
        Output  :   n/a
        Notes   :   n/a
        ============================================================================*/

        void PublishImage();
        
        /*============================================================================
        Name    :   PublishTransforms()
        ------------------------------------------------------------------------------
        Purpose :   To publish the camera transforms
        Input   :   tranformBroadcaster - a static transform broadcaster object
        Output  :   n/a
        Notes   :   n/a
        ============================================================================*/

        void PublishTransforms(tf2_ros::StaticTransformBroadcaster &tranformBroadcaster);
        
        /*============================================================================
        Name    :   GetDistance()
        ------------------------------------------------------------------------------
        Purpose :   To get the depth of a particular pixel
        Input   :   iPixelX - X coordintate of pixel
                    iPixelY - Y coordintate of pixel
        Output  :   n/a
        Notes   :   n/a
        ============================================================================*/

        float GetDistance(int iPixelX, int iPixelY);
        
        /*============================================================================
        Name    :   DepthTo3D()
        ------------------------------------------------------------------------------
        Purpose :   Convert depth to 3D co-ordinatnes using the api provided in
                    realsense SDK
        Input   :   fPoint - outparam to store the projected 3D coordinates
                    iPixelX - X coordintate of pixel
                    iPixelY - Y coordintate of pixel
                    fDepth - depth in meters of particular pixel

        Output  :   n/a
        Notes   :   n/a
        ============================================================================*/
        
        void DepthTo3D(float fPoint[], int iPixelX, int iPixelY, float fDepth);
        
};

#endif
