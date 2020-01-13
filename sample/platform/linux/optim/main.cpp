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

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;


int
main(int argc, char** argv)
{

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();

  customOA_ref oa_data_;
  oa_data_.log_file_running_ = false; //log hasn't started
  oa_data_.log_file_name_ = "";
  FILE * pFile;
  bool log_init = false;
  oa_data_.last_time_log_ = 0;

  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  while(1){

    //update dji
    updateDJI_DATA(vehicle,1,oa_data_);

    if(!log_init && oa_data_.log_file_running_){
      //first loop thru when log file name has been populated
      // create a log file with data/time name
      log_init = true;
      pFile = fopen (oa_data_.log_file_name_.c_str(),"w");
      init_log(oa_data_,pFile);
    }

    //update dji log
    if(log_init){
      if( abs(oa_data_.timestamp.time_ms - oa_data_.last_time_log_) > 2000 ){
        oa_data_.last_time_log_ = oa_data_.timestamp.time_ms;
        //log dji data
        logDJI_DATA(oa_data_, pFile);
      }
    }
    
    //update camera


    //update camera log

    
  }

  fclose (pFile);

  return 0;
}
