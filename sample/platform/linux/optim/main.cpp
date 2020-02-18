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

#include <ctime>
#include <ratio>
#include <chrono>

#define USE_DJI_SOFTWARE true

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace cv;
using namespace std::chrono;

struct oa_timeRef {
    high_resolution_clock::time_point dji_log_time;
    high_resolution_clock::time_point image_log_time;
    high_resolution_clock::time_point current_time;
};

void time_checker(oa_timeRef &timeRef, camera_ref &camera, customOA_ref &oa){
    
  //grab current time
  timeRef.current_time  = high_resolution_clock::now();

  milliseconds ms_dji   = std::chrono::duration_cast<milliseconds>(timeRef.current_time - timeRef.dji_log_time);
  milliseconds ms_image = std::chrono::duration_cast<milliseconds>(timeRef.current_time - timeRef.image_log_time);
  //std::cout << ms.count() << "ms\n";

  if(ms_dji.count() >= 1000){
    //update log
    updateLog(oa);
    timeRef.dji_log_time = timeRef.current_time;
  }

  if(ms_image.count() >= 2000)
  {
    //camera.logNow = true;
    timeRef.image_log_time = timeRef.current_time;

    //update camera log 
    if(camera.cameraSetup && oa.logInitalized_)
    {
      // we want to either save based on DJI remote...or change rate of camera recording based on DJI remote
      saveImage(camera, oa);
      printf("save image \n");
    }
  }
    
}

void time_checker(oa_timeRef &timeRef, camera_ref &camera, customOA_v2_ref &oa){
    
  //grab current time
  timeRef.current_time  = high_resolution_clock::now();
  //std::cout << ms.count() << "ms\n";

  if(oa.rc.gear > -6000)
  {
    milliseconds ms_dji   = std::chrono::duration_cast<milliseconds>(timeRef.current_time - timeRef.dji_log_time);
    milliseconds ms_image = std::chrono::duration_cast<milliseconds>(timeRef.current_time - timeRef.image_log_time);
    if(ms_dji.count() >= 200){
      //update log
      updateLog(oa);
      timeRef.dji_log_time = timeRef.current_time;
    }

    if(ms_image.count() >= 200)
    {
      //camera.logNow = true;
      timeRef.image_log_time = timeRef.current_time;

      //update camera log 
      if(camera.cameraSetup && oa.logInitalized_)
      {
        // we want to either save based on DJI remote...or change rate of camera recording based on DJI remote
        saveImage(camera, oa);
        printf("save image \n");
      }
    }
  }    
}

int
main(int argc, char** argv)
{
    
  oa_timeRef timer;
  timer.dji_log_time    = high_resolution_clock::now();
  timer.image_log_time  = high_resolution_clock::now();
  
  // Setup OSDK.
  #if USE_DJI_SOFTWARE == true
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle*   vehicle = linuxEnvironment.getVehicle();
  #endif

  //camera settings
	camera_ref cameraSettings;
  cameraSettings.cameraSetup      = false;
  cameraSettings.imageCounter     = 0;
  cameraSettings.last_time_log_   = 0;

  #if 0
  customOA_ref oa_data_;
  oa_data_.log_file_running_    = false; //log hasn't started
  oa_data_.log_folder_name_     = "";
  oa_data_.logInitalized_       = false;
  oa_data_.last_time_log_       = 0;
  oa_data_.missed_gps_counter_  = 0;
  #else
  customOA_v2_ref oa_data_;
  oa_data_.log_file_running_    = false; //log hasn't started
  oa_data_.log_folder_name_     = "";
  oa_data_.logInitalized_       = false;
  oa_data_.last_time_log_       = 0;
  oa_data_.missed_gps_counter_  = 0;
  #endif

  #if USE_DJI_SOFTWARE == true
    if (vehicle == NULL)
    {
      std::cout << "Vehicle not initialized, exiting.\n";
      return -1;
    }
  #endif

  ////////////////////////////////////////////////////////////////////////
  //setup camera
  cameraSetup(cameraSettings);
  if(!cameraSettings.cameraSetup){
    printf("camera skipped \n");
  }
  ////////////////////////////////////////////////////////////////////////

/////////////////////////////////////
// DJI init
printf("init DJI\n");
optimInitializeSubscribe(vehicle, 1);

//////////////////////////////////////



  /* initialize random seed: */
  srand (time(NULL));

  while(1){

    /////////////////////////////////////////////////////////////////////////////////////////////
    //update dji
    #if USE_DJI_SOFTWARE == true
      updateDJI_DATA(vehicle,1,oa_data_);
    #endif

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //  update camera
    if(cameraSettings.cameraSetup)
    {
      updateCamera(cameraSettings);
    }//end of camera setup

    //time check for logging purposes
    time_checker(timer,cameraSettings,oa_data_);
  
  }//end of while

  oa_data_.log_to_file_.close();

  if(cameraSettings.cameraSetup){
    closeCamera(cameraSettings);
  }

  return 0;
}