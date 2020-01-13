

#include "boson.h"

using namespace cv;

/* ---------------------------- 16 bits Mode auxiliary functions ---------------------------------------*/

// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void AGC_Basic_Linear(cv::Mat input_16, cv::Mat output_8, int height, int width) {
	int i, j;  // aux variables

	// auxiliary variables for AGC calcultion
	unsigned int max1=0;         // 16 bits
	unsigned int min1=0xFFFF;    // 16 bits
	unsigned int value1, value2, value3, value4;

	// RUN a super basic AGC
	for (i=0; i<height; i++) {
		for (j=0; j<width; j++) {
			value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
			value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
			value3 = ( value1 << 8) + value2;
			if ( value3 <= min1 ) {
				min1 = value3;
			}
			if ( value3 >= max1 ) {
				max1 = value3;
			}
			//printf("%X.%X.%X  ", value1, value2, value3);
		}
	}
	//printf("max1=%04X, min1=%04X\n", max1, min1);

	for (int i=0; i<height; i++) {
		for (int j=0; j<width; j++) {
			value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
			value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
			value3 = ( value1 << 8) + value2;
			value4 = ( ( 255 * ( value3 - min1) ) ) / (max1-min1)   ;
			// printf("%04X \n", value4);

			output_8.at<uchar>(i,j)= (uchar)(value4&0xFF);
		}
	}

}


/* ---------------------------- Other Aux functions ---------------------------------------*/
void cameraSetup(camera_ref &cameraSettings){

	sprintf(cameraSettings.video, "/dev/video1");

	// Printf Sensor defined
	printf(WHT ">>> " YEL "%s" WHT " selected\n", cameraSettings.thermal_sensor_name);

	// We open the Video Device
	printf(WHT ">>> " YEL "%s" WHT " selected\n", cameraSettings.video);
	if((cameraSettings.fd = open(cameraSettings.video, O_RDWR)) < 0){
		perror(RED "Error : OPEN. Invalid Video Device" WHT "\n");
		exit(1);
	}

	// Check VideoCapture mode is available
	if(ioctl(cameraSettings.fd, VIDIOC_QUERYCAP, &cameraSettings.cap) < 0){
	    perror(RED "ERROR : VIDIOC_QUERYCAP. Video Capture is not available" WHT "\n");
	    exit(1);
	}

	if(!(cameraSettings.cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
		fprintf(stderr, RED "The device does not handle single-planar video capture." WHT "\n");
		exit(1);
	}

	struct v4l2_format format;

	// Two different FORMAT modes, 8 bits vs RAW16
	if (cameraSettings.my_image==image_types::rawImage) {
		printf(WHT ">>> " YEL "16 bits " WHT "capture selected\n");

		// I am requiring thermal 16 bits mode
		format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;

		// Select the frame SIZE (will depend on the type of sensor)
		switch (cameraSettings.my_thermal) {
			case Boson320:  // Boson320
			          	cameraSettings.width=320;
				        cameraSettings.height=256;
				        break;
		        case Boson640:  // Boson640
				        cameraSettings.width=640;
				        cameraSettings.height=512;
				        break;
			default:  // Boson320
				        cameraSettings.width=320;
				        cameraSettings.height=256;
				        break;
		 }

	} else { // 8- bits is always 640x512 (even for a Boson 320)
		 printf(WHT ">>> " YEL "8 bits " WHT "YUV selected\n");
	         format.fmt.pix.pixelformat = V4L2_PIX_FMT_YVU420; // thermal, works   LUMA, full Cr, full Cb
		 	cameraSettings.width = 640;
		 	cameraSettings.height = 512;
	}

	// Common varibles
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width 	= cameraSettings.width;
	format.fmt.pix.height 	= cameraSettings.height;

	// request desired FORMAT
	if(ioctl(cameraSettings.fd, VIDIOC_S_FMT, &format) < 0){
		perror(RED "VIDIOC_S_FMT" WHT);
		exit(1);
	}


	// we need to inform the device about buffers to use.
	// and we need to allocate them.
	// we’ll use a single buffer, and map our memory using mmap.
	// All this information is sent using the VIDIOC_REQBUFS call and a
	// v4l2_requestbuffers structure:
	struct v4l2_requestbuffers bufrequest;
	bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufrequest.memory = V4L2_MEMORY_MMAP;
	bufrequest.count = 1;   // we are asking for one buffer

	if(ioctl(cameraSettings.fd, VIDIOC_REQBUFS, &bufrequest) < 0){
		perror(RED "VIDIOC_REQBUFS" WHT);
		exit(1);
	}
		// Now that the device knows how to provide its data,
	// we need to ask it about the amount of memory it needs,
	// and allocate it. This information is retrieved using the VIDIOC_QUERYBUF call,
	// and its v4l2_buffer structure.

	memset(&cameraSettings.bufferinfo, 0, sizeof(cameraSettings.bufferinfo));

	cameraSettings.bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cameraSettings.bufferinfo.memory = V4L2_MEMORY_MMAP;
	cameraSettings.bufferinfo.index = 0;

	if(ioctl(cameraSettings.fd, VIDIOC_QUERYBUF, &cameraSettings.bufferinfo) < 0){
		perror(RED "VIDIOC_QUERYBUF" WHT);
		exit(1);
	}


	// map fd+offset into a process location (kernel will decide due to our NULL). lenght and
	// properties are also passed
	printf(WHT ">>> Image width  =" YEL "%i" WHT "\n", cameraSettings.width);
	printf(WHT ">>> Image height =" YEL "%i" WHT "\n", cameraSettings.height);
	printf(WHT ">>> Buffer lenght=" YEL "%i" WHT "\n", cameraSettings.bufferinfo.length);

	cameraSettings.buffer_start = mmap(NULL, cameraSettings.bufferinfo.length, PROT_READ | PROT_WRITE,MAP_SHARED, cameraSettings.fd, cameraSettings.bufferinfo.m.offset);

	if(cameraSettings.buffer_start == MAP_FAILED){
		perror(RED "mmap" WHT);
		exit(1);
	}

	// Fill this buffer with ceros. Initialization. Optional but nice to do
	memset(cameraSettings.buffer_start, 0, cameraSettings.bufferinfo.length);

	// Activate streaming
	cameraSettings.type = cameraSettings.bufferinfo.type;
	if(ioctl(cameraSettings.fd, VIDIOC_STREAMON, &cameraSettings.type) < 0){
		perror(RED "VIDIOC_STREAMON" WHT);
		exit(1);
	}

	cameraSettings.cameraSetup = true;


}

/* ---------------------------- Main Function ---------------------------------------*/
// ENTRY POINT
int gerp(int argc, char** argv )
{
	#if 0
	camera_ref cameraSettings;
	printf(WHT ">>> " YEL "Hello moto" WHT " ------------\n");

	// Video device by default
	cameraSetup(cameraSettings);

	
	// Declarations for RAW16 representation
    // Will be used in case we are reading RAW16 format
	// Boson320 , Boson 640
	Mat thermal16(height, width, CV_16U, cameraSettings.buffer_start);   // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
	Mat thermal16_linear(height,width, CV_8U, 1);         				// OpenCV output buffer : Data used to display the video

	// Declarations for Zoom representation
    // Will be used or not depending on program arguments
	Size size(640,512);
	Mat thermal16_linear_zoom;   // (height,width, CV_8U, 1);    // Final representation
	Mat thermal_rgb_zoom;   	 // (height,width, CV_8U, 1);    // Final representation

	int luma_height ;
	int luma_width ;
	int color_space ;

	// Declarations for 8bits YCbCr mode
    // Will be used in case we are reading YUV format
	// Boson320, 640 :  4:2:0
	luma_height = height+height/2;
	luma_width = width;
	color_space = CV_8UC1;
 	Mat thermal_luma(luma_height, luma_width,  color_space, cameraSettings.buffer_start);  // OpenCV input buffer
	Mat thermal_rgb(height, width, CV_8UC3, 1);    // OpenCV output buffer , BGR -> Three color spaces (640 - 640 - 640 : p11 p21 p31 .... / p12 p22 p32 ..../ p13 p23 p33 ...)

	// Reaad frame, do AGC, paint frame
	for (;;) {

		// Put the buffer in the incoming queue.
		if(ioctl(cameraSettings.fd, VIDIOC_QBUF, &cameraSettings.bufferinfo) < 0){
			perror(RED "VIDIOC_QBUF" WHT);
			exit(1);
		}

		// The buffer's waiting in the outgoing queue.
		if(ioctl(cameraSettings.fd, VIDIOC_DQBUF, &cameraSettings.bufferinfo) < 0) {
			perror(RED "VIDIOC_QBUF" WHT);
			exit(1);
		}


		// -----------------------------
		// RAW16 DATA
		if ( cameraSettings.my_image==image_types::rawImage ) {
			AGC_Basic_Linear(thermal16, thermal16_linear, height, width);

			// Display thermal after 16-bits AGC... will display an image
			if (cameraSettings.zoom_enable==0) {
                		sprintf(cameraSettings.label, "%s : RAW16  Linear", cameraSettings.thermal_sensor_name);
                    		imshow(cameraSettings.label, thermal16_linear);
          		} else {
			        resize(thermal16_linear, thermal16_linear_zoom, size);
                     		sprintf(cameraSettings.label, "%s : RAW16  Linear Zoom", cameraSettings.thermal_sensor_name);
                     		imshow(cameraSettings.label, thermal16_linear_zoom);
                	}
          	}


		// ---------------------------------
		// DATA in YUV
		else {  // Video is in 8 bits YUV
            	cvtColor(thermal_luma, thermal_rgb, COLOR_YUV2RGB_I420, 0 );   // 4:2:0 family instead of 4:2:2 ...
        		sprintf(cameraSettings.label, "%s : 8bits", cameraSettings.thermal_sensor_name);
		        imshow(cameraSettings.label, thermal_rgb);
		}

		// Press 'q' to exit
		if( waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
			printf(WHT ">>> " RED "'q'" WHT " key pressed. Quitting !\n");
			break;
		}
		// Stop if frame limit reached.
		if (cameraSettings.video_frames>0 && cameraSettings.frame+1 > cameraSettings.video_frames) {
			printf(WHT ">>>" GRN "'Done'" WHT " Frame limit reached, Quitting !\n");
			break;
		}
	}
	// Finish Loop . Exiting.

	// Deactivate streaming
	if( ioctl(cameraSettings.fd, VIDIOC_STREAMOFF, &cameraSettings.type) < 0 ){
		perror(RED "VIDIOC_STREAMOFF" WHT);
		exit(1);
	};

	close(cameraSettings.fd);
	return EXIT_SUCCESS;

	#endif
}
