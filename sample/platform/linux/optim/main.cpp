/*! @file telemetry/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Telemetry API usage in a Linux environment.
 *  Shows example usage of the new data subscription API.
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "telemetry_sample.hpp"
#include "boson.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace cv;


int
main(int argc, char** argv)
{

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();

  //camera settings
	camera_ref cameraSettings;
  cameraSettings.cameraSetup = false;

  customOA_ref oa_data_;
  oa_data_.log_file_running_ = false; //log hasn't started
  oa_data_.log_file_name_ = "";
  bool log_init                 = false;
  oa_data_.last_time_log_       = 0;
  oa_data_.missed_gps_counter_  = 0;

  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  ////////////////////////////////////////////////////////////////////////
  //setup camera
  //cameraSetup(cameraSettings);
  
  if(cameraSettings.cameraSetup){
    // Declarations for RAW16 representation
    // Will be used in case we are reading RAW16 format
    // Boson320 , Boson 640
    Mat thermal16(cameraSettings.height, cameraSettings.width, CV_16U, cameraSettings.buffer_start);   // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
    //Mat thermal16_linear(cameraSettings.height,cameraSettings.width, CV_8U, 1);         				// OpenCV output buffer : Data used to display the video
    cameraSettings.t_16 = &thermal16;
    cameraSettings.thermal16_linear.create(cameraSettings.height,cameraSettings.width, CV_8U);

    int luma_height ;
    int luma_width ;
    int color_space ;

    // Declarations for 8bits YCbCr mode
    // Will be used in case we are reading YUV format
    // Boson320, 640 :  4:2:0
    luma_height = cameraSettings.height+cameraSettings.height/2;
    luma_width  = cameraSettings.width;
    color_space = CV_8UC1;
    Mat thermal_luma(luma_height, luma_width,  color_space, cameraSettings.buffer_start);  // OpenCV input buffer
    //Mat thermal_rgb(cameraSettings.height, cameraSettings.width, CV_8UC3, 1);    // OpenCV output buffer , BGR -> Three color spaces (640 - 640 - 640 : p11 p21 p31 .... / p12 p22 p32 ..../ p13 p23 p33 ...)
    cameraSettings.t_luma = &thermal_luma;
    cameraSettings.thermal_rgb.create(cameraSettings.height, cameraSettings.width, CV_8UC3);
  }else{
    printf("camera skipped \n");
  }
  
  ////////////////////////////////////////////////////////////////////////

  while(1){

    /////////////////////////////////////////////////////////////////////////////////////////////
    //update dji
    updateDJI_DATA(vehicle,1,oa_data_);

    if(!log_init && oa_data_.log_file_running_){
      //first loop thru when log file name has been populated
      // create a log file with data/time name
      printf("log init\n");
      log_init = true;
      oa_data_.log_to_file_.open ("example.txt");
      init_log(oa_data_);
    }

    //update dji log
    if(log_init){
      if( abs(oa_data_.timestamp.time_ms - oa_data_.last_time_log_) > 2000 ){
        oa_data_.last_time_log_ = oa_data_.timestamp.time_ms;
        //log dji data
        logDJI_DATA(oa_data_);
        printf("logged\n");
      }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //  update camera
    if(cameraSettings.cameraSetup)
    {
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
        AGC_Basic_Linear(*cameraSettings.t_16, cameraSettings.thermal16_linear, cameraSettings.height, cameraSettings.width);

        // Display thermal after 16-bits AGC... will display an image
        if (cameraSettings.zoom_enable==0) {
                      sprintf(cameraSettings.label, "%s : RAW16  Linear", cameraSettings.thermal_sensor_name);
                          imshow(cameraSettings.label, cameraSettings.thermal16_linear);
                }
      }


      // ---------------------------------
      // DATA in YUV
      else {  // Video is in 8 bits YUV
                cvtColor(*cameraSettings.t_luma, cameraSettings.thermal_rgb, COLOR_YUV2RGB_I420, 0 );   // 4:2:0 family instead of 4:2:2 ...
              sprintf(cameraSettings.label, "%s : 8bits", cameraSettings.thermal_sensor_name);
              imshow(cameraSettings.label, cameraSettings.thermal_rgb);
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

    }//end of camera setup

    //update camera log 
  }

  oa_data_.log_to_file_.close();

  if(cameraSettings.cameraSetup){
    // Deactivate streaming
    if( ioctl(cameraSettings.fd, VIDIOC_STREAMOFF, &cameraSettings.type) < 0 ){
      perror(RED "VIDIOC_STREAMOFF" WHT);
      exit(1);
    };
    
    close(cameraSettings.fd);

  }
  return 0;
}
