#pragma once
#include <stdio.h>
#include <fcntl.h>               // open, O_RDWR
#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

#include "telemetry_sample.hpp"


// Types of sensors supported
enum sensor_types {
  Boson320, Boson640
};

enum image_types {
	rawImage, yuvImage
};

struct camera_ref{
	int ret;
	int fd;
	int i;
	struct v4l2_capability cap;
	long frame=0;                   // First frame number enumeration
	char video[20];                 // To store Video Port Device
	char label[50];                 // To display the information
	char thermal_sensor_name[20];  // To store the sensor name
	char filename[60];              // PATH/File_count
	char folder_name[30];           // To store the folder name
    char video_frames_str[30];
	// Default Program options
	int  video_frames=0;
	int  zoom_enable=0;
	int  record_enable=0;
	sensor_types my_thermal	=	Boson640;
	image_types my_image 	= 	image_types::yuvImage;
	struct v4l2_buffer bufferinfo;
	void * buffer_start;
	int type;
	int width;
	int height;
	cv::Mat thermal16;
	cv::Mat *t_16;
	cv::Mat thermal16_linear;
	cv::Mat thermal_luma;
	cv::Mat *t_luma;
	cv::Mat thermal_rgb;
	bool cameraSetup;
	int imageCounter;
	uint32_t last_time_log_;
};
#define v_major 1
#define v_minor 0

// Define COLOR CODES
#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"

#define YUV   0
#define RAW16 1

void AGC_Basic_Linear(cv::Mat input_16, cv::Mat output_8, int height, int width);
void cameraSetup(camera_ref &cameraSettings);
void updateCamera(camera_ref &cameraSettings);
void saveImage(camera_ref &cameraSettings, customOA_ref &in);
void saveImage(camera_ref &cameraSettings, customOA_v2_ref &in);

void closeCamera(camera_ref &cameraSettings);