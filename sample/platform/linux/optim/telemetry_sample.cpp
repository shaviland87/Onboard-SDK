/*! @file telemetry_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Telemetry API usage in a Linux environment.
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
#include  <signal.h>
#include  <stdlib.h>
#include  <stdio.h>
#include <sys/stat.h>


//using namespace DJI::OSDK;
//using namespace DJI::OSDK::Telemetry;

void init_log(customOA_ref &in){
  //file has been created
  /* Time, location(lat/lon/alt), attitude, temperature? */
  if(in.log_to_file_.is_open()){
    in.log_to_file_ << "time ms,";
    in.log_to_file_ << "time ns,";
    in.log_to_file_ << "status gear,";
    in.log_to_file_ << "status mode,";
    in.log_to_file_ << "status flight,";
    in.log_to_file_ << "rc gear,";
    in.log_to_file_ << "rc mode,";
    in.log_to_file_ << "lat,";
    in.log_to_file_ << "lon,";
    in.log_to_file_ << "alt,";
    in.log_to_file_ << "q0,";
    in.log_to_file_ << "q1,";
    in.log_to_file_ << "q2,";
    in.log_to_file_ << "q3\n";
  }
  in.log_to_file_.flush();

}

void logDJI_DATA(customOA_ref &in){
  //file_in has been created

  if(in.log_to_file_.is_open()){
    in.log_to_file_ << in.timestamp.time_ms;
    in.log_to_file_ << ", ";
    in.log_to_file_ << in.timestamp.time_ns;
    in.log_to_file_ << ", ";
    in.log_to_file_ << std::fixed << (int)in.status.gear;
    in.log_to_file_ << ", ";
    in.log_to_file_ << std::fixed << (int)in.status.mode;
    in.log_to_file_ << ", ";
    in.log_to_file_ << std::fixed << (int)in.status.flight;
    in.log_to_file_ << ", ";
    in.log_to_file_ << std::fixed << (int)in.status.error;
    in.log_to_file_ << ", ";
    in.log_to_file_ << std::fixed << (int)in.rc.gear;
    in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << (int)in.rc.mode;
        in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.globalPosition.latitude;
        in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.globalPosition.longitude;
        in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.globalPosition.altitude;
        in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.quat.q0;
        in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.quat.q1;
        in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.quat.q2;
        in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.quat.q3;
    in.log_to_file_ << "\n";    
  }
  in.log_to_file_.flush();
}

void updateDJI_DATA(DJI::OSDK::Vehicle* vehicle, int responseTimeout,  customOA_ref &in){


  const int TIMEOUT = 20;
  // Re-set Broadcast frequencies to their default values
  ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreqDefaults(TIMEOUT); ///not sure if this needs to happen on every loop

  // Matrice 100 broadcasts only flight status
  /*status         = vehicle->broadcast->getStatus();
  globalPosition = vehicle->broadcast->getGlobalPosition();
  rc             = vehicle->broadcast->getRC();
  velocity       = vehicle->broadcast->getVelocity();
  */

  in.globalPosition = vehicle->broadcast->getGlobalPosition();
  in.status         = vehicle->broadcast->getStatus();
  in.rc             = vehicle->broadcast->getRC();
  in.velocity       = vehicle->broadcast->getVelocity();
  in.timestamp      = vehicle->broadcast->getTimeStamp();
  in.gpsInfo        = vehicle->broadcast->getGPSInfo();
  in.quat           = vehicle->broadcast->getQuaternion();

  
  /*gpsInfo.time...
    uint32_t date;     !< yyyymmdd E.g.20150205 means February 5th,2015 (GMT+8)
    uint32_t time;     !< hhmmss E.g. 90209 means 09:02:09 (GMT+8)
  */
  // if(in.gpsInfo.detail.fix) //should check fix and sats --> if both good set date/time
  if(!in.log_file_running_){ 
  }
}

void updateLog(customOA_ref &in){


  if(!in.logInitalized_ && !in.log_file_running_){
    // log not initalized and not yet running

    //we haven't created a log file yet
    //check to see if gps good
    if(in.gpsInfo.detail.fix > 0.5){
      //assume some number for now that says a gps data/time has been started

      in.log_file_running_ = true;  //set to true for future ..this will create a log
      std::string date = std::to_string(in.gpsInfo.time.date);
      std::string time = std::to_string(in.gpsInfo.time.time);
      in.log_folder_name_ = date + time ; //log file will be stamped with date/time
    }else
    {
      in.missed_gps_counter_++;
      printf("cnt %u \n",in.missed_gps_counter_);

      if(in.missed_gps_counter_ > 10){
        //we skip log name based on gps
        std::string rando = std::to_string(rand());
        in.log_folder_name_ = "log" + rando ;
        in.log_file_running_ = true;
      }

    }

  }

  if(!in.logInitalized_ && in.log_file_running_){
      //first loop thru when log file name has been populated
      // create a log file with data/time name
      printf("log init\n");
      in.logInitalized_ = true;

      const int dir_err = mkdir(in.log_folder_name_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      if (-1 == dir_err)
      {
          printf("Error creating directory!n");
          exit(1);
      }

      //TBD need to make this variable -- want a new directory to be made for each power up
      std::string folder_and_file = in.log_folder_name_ + "/DJI_data.txt";
      in.log_to_file_.open (folder_and_file);  
      init_log(in);
    }

    //update dji log
    if(in.logInitalized_){
        //log dji data
        logDJI_DATA(in);
        printf("logged\n");
    }
}

bool
getBroadcastData(DJI::OSDK::Vehicle* vehicle, int responseTimeout)
{
  // Counters
  int elapsedTimeInMs = 0;
  int timeToPrintInMs = 2000;

  // We will listen to five broadcast data sets:
  // 1. Flight Status
  // 2. Global Position
  // 3. RC Channels
  // 4. Velocity
  // 5. Quaternion
  // 6. Avoid obstacle data

  // Please make sure your drone is in simulation mode. You can
  // fly the drone with your RC to get different values.

  Telemetry::Status         status;
  Telemetry::GlobalPosition globalPosition;
  Telemetry::RC             rc;
  Telemetry::Vector3f       velocity;
  Telemetry::Quaternion     quaternion;
  Telemetry::RelativePosition avoidData;

  const int TIMEOUT = 20;

  // Re-set Broadcast frequencies to their default values
  ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreqDefaults(TIMEOUT);

  // Print in a loop for 2 seconds
  while (elapsedTimeInMs < timeToPrintInMs)
  {
    // Matrice 100 broadcasts only flight status
    status         = vehicle->broadcast->getStatus();
    globalPosition = vehicle->broadcast->getGlobalPosition();
    rc             = vehicle->broadcast->getRC();
    velocity       = vehicle->broadcast->getVelocity();
    quaternion     = vehicle->broadcast->getQuaternion();
    avoidData      = vehicle->broadcast->getRelativePosition();

    std::cout << "Counter = " << elapsedTimeInMs << ":\n";
    std::cout << "-------\n";
    std::cout << "Flight Status                         = "
              << (unsigned)status.flight << "\n";
    std::cout << "Position              (LLA)           = "
              << globalPosition.latitude << ", " << globalPosition.longitude
              << ", " << globalPosition.altitude << "\n";
    std::cout << "RC Commands           (r/p/y/thr)     = " << rc.roll << ", "
              << rc.pitch << ", " << rc.yaw << ", " << rc.throttle << "\n";
    std::cout << "Velocity              (vx,vy,vz)      = " << velocity.x
              << ", " << velocity.y << ", " << velocity.z << "\n";
    std::cout << "Attitude Quaternion   (w,x,y,z)       = " << quaternion.q0
              << ", " << quaternion.q1 << ", " << quaternion.q2 << ", "
              << quaternion.q3 << "\n";
    std::cout << "Avoid obstacle data  (down,front,right,back,left,up) ="
              << avoidData.down  << ", "<< avoidData.front << ", "
              << avoidData.right << ", "<< avoidData.back  << ", "
              << avoidData.left  << ", "<< avoidData.up    << "\n";
    std::cout << "-------\n\n";

    usleep(5000);
    elapsedTimeInMs += 5;
  }

  std::cout << "Done printing!\n";
  return true;
}

bool
subscribeToData(Vehicle* vehicle, int responseTimeout)
{
  // RTK can be detected as unavailable only for Flight controllers that don't support RTK
  bool rtkAvailable = false;
  // Counters
  int elapsedTimeInMs = 0;
  int timeToPrintInMs = 20000;

  // We will subscribe to six kinds of data:
  // 1. Flight Status at 1 Hz
  // 2. Fused Lat/Lon at 10Hz
  // 3. Fused Altitude at 10Hz
  // 4. RC Channels at 50 Hz
  // 5. Velocity at 50 Hz
  // 6. Quaternion at 200 Hz

  // Please make sure your drone is in simulation mode. You can fly the drone
  // with your RC to
  // get different values.

  // Telemetry: Verify the subscription
  ACK::ErrorCode subscribeStatus;
  subscribeStatus = vehicle->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }

  // Package 0: Subscribe to flight status at freq 1 Hz
  int       pkgIndex        = 0;
  int       freq            = 1;
  TopicName topicList1Hz[]  = { TOPIC_STATUS_FLIGHT };
  int       numTopic        = sizeof(topicList1Hz) / sizeof(topicList1Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList1Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Package 1: Subscribe to Lat/Lon, and Alt at freq 10 Hz
  pkgIndex                  = 1;
  freq                      = 10;
  TopicName topicList10Hz[] = { TOPIC_GPS_FUSED, TOPIC_ALTITUDE_FUSIONED};
  numTopic                  = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  enableTimestamp           = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Package 2: Subscribe to RC Channel and Velocity at freq 50 Hz
  pkgIndex                  = 2;
  freq                      = 50;
  TopicName topicList50Hz[] = { TOPIC_RC, TOPIC_VELOCITY };
  numTopic                  = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  enableTimestamp           = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Package 3: Subscribe to Quaternion at freq 200 Hz.
  pkgIndex                   = 3;
  freq                       = 200;
  TopicName topicList200Hz[] = { TOPIC_QUATERNION };
  numTopic        = sizeof(topicList200Hz) / sizeof(topicList200Hz[0]);
  enableTimestamp = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
          pkgIndex, numTopic, topicList200Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
      return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
      ACK::getErrorCodeMessage(subscribeStatus, __func__);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
  }

  // Package 4: Subscribe to RTK at freq 5 Hz.
  pkgIndex                   = 4;
  freq                       = 5;
  TopicName topicListRTK5Hz[] = {TOPIC_RTK_POSITION, TOPIC_RTK_YAW_INFO,
                                  TOPIC_RTK_POSITION_INFO, TOPIC_RTK_VELOCITY,
                                  TOPIC_RTK_YAW};
  numTopic        = sizeof(topicListRTK5Hz) / sizeof(topicListRTK5Hz[0]);
  enableTimestamp = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicListRTK5Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  else {
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if(subscribeStatus.data == ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE)
    {
      std::cout << "RTK Not Available" << "\n";
      rtkAvailable = false;
    }
    else
    {
      rtkAvailable = true;
      if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        return false;
      }
    }
  }

  // Wait for the data to start coming in.
  sleep(1);

  // Get all the data once before the loop to initialize vars
  TypeMap<TOPIC_STATUS_FLIGHT>::type     flightStatus;
  TypeMap<TOPIC_GPS_FUSED>::type         latLon;
  TypeMap<TOPIC_ALTITUDE_FUSIONED>::type altitude;
  TypeMap<TOPIC_RC>::type                rc;
  TypeMap<TOPIC_VELOCITY>::type          velocity;
  TypeMap<TOPIC_QUATERNION>::type        quaternion;
  TypeMap<TOPIC_RTK_POSITION>::type      rtk;
  TypeMap<TOPIC_RTK_POSITION_INFO>::type rtk_pos_info;
  TypeMap<TOPIC_RTK_VELOCITY>::type      rtk_velocity;
  TypeMap<TOPIC_RTK_YAW>::type           rtk_yaw;
  TypeMap<TOPIC_RTK_YAW_INFO>::type      rtk_yaw_info;

  // Print in a loop for 2 sec
  while (elapsedTimeInMs < timeToPrintInMs)
  {
    flightStatus = vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>();
    latLon       = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    altitude     = vehicle->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
    rc           = vehicle->subscribe->getValue<TOPIC_RC>();
    velocity     = vehicle->subscribe->getValue<TOPIC_VELOCITY>();
    quaternion   = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    if(rtkAvailable) {
      rtk = vehicle->subscribe->getValue<TOPIC_RTK_POSITION>();
      rtk_pos_info = vehicle->subscribe->getValue<TOPIC_RTK_POSITION_INFO>();
      rtk_velocity = vehicle->subscribe->getValue<TOPIC_RTK_VELOCITY>();
      rtk_yaw = vehicle->subscribe->getValue<TOPIC_RTK_YAW>();
      rtk_yaw_info = vehicle->subscribe->getValue<TOPIC_RTK_YAW_INFO>();
    }
    std::cout << "Counter = " << elapsedTimeInMs << ":\n";
    std::cout << "-------\n";
    std::cout << "Flight Status                         = " << (int)flightStatus
              << "\n";
    std::cout << "Position              (LLA)           = " << latLon.latitude
              << ", " << latLon.longitude << ", " << altitude << "\n";
    std::cout << "RC Commands           (r/p/y/thr)     = " << rc.roll << ", "
              << rc.pitch << ", " << rc.yaw << ", " << rc.throttle << "\n";
    std::cout << "Velocity              (vx,vy,vz)      = " << velocity.data.x
              << ", " << velocity.data.y << ", " << velocity.data.z << "\n";
    std::cout << "Attitude Quaternion   (w,x,y,z)       = " << quaternion.q0
              << ", " << quaternion.q1 << ", " << quaternion.q2 << ", "
              << quaternion.q3 << "\n";
    if(rtkAvailable) {
      std::cout << "RTK if available   (lat/long/alt/velocity_x/velocity_y/velocity_z/yaw/yaw_info/pos_info) ="
                << rtk.latitude << "," << rtk.longitude << "," << rtk.HFSL << "," << rtk_velocity.x << ","
                << rtk_velocity.y
                << "," << rtk_velocity.z << "," << rtk_yaw << "," << rtk_yaw_info << rtk_pos_info << "\n";
    }
    std::cout << "-------\n\n";
    usleep(5000);
    elapsedTimeInMs += 5;
  }

  std::cout << "Done printing!\n";
  vehicle->subscribe->removePackage(0, responseTimeout);
  vehicle->subscribe->removePackage(1, responseTimeout);
  vehicle->subscribe->removePackage(2, responseTimeout);
  vehicle->subscribe->removePackage(3, responseTimeout);
  vehicle->subscribe->removePackage(4, responseTimeout);

  return true;
}

void optimInitializeSubscribe(Vehicle* vehicle, int responseTimeout){
  // We will subscribe to six kinds of data:
  // 1. Flight Status at 1 Hz , gps time
  // 2. Fused Lat/Lon at 10Hz
  // 3. Fused Altitude at 10Hz
  // 4. RC Channels at 10 Hz
  // 6. Quaternion at 10 Hz

  // Telemetry: Verify the subscription
  ACK::ErrorCode subscribeStatus;
  subscribeStatus = vehicle->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    printf("ISSUE WITH SUBSCRIPTION\n");
  }

  // Package 0: Subscribe to flight status at freq 1 Hz
  int       pkgIndex        = 0;
  int       freq            = 1;
  TopicName topicList1Hz[]  = { TOPIC_STATUS_FLIGHT, TOPIC_GPS_TIME, TOPIC_GPS_DATE, TOPIC_GPS_DETAILS };
  int       numTopic        = sizeof(topicList1Hz) / sizeof(topicList1Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList1Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    printf("ISSUE WITH PACKAGE STATUS\n");
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    printf("CLEANUP - remove package status flight \n");
  }
///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
  // Package 2: Subscribe to RC Channel and Velocity at freq 10 Hz
  pkgIndex                  = 1;
  freq                      = 10;
  TopicName topicList10Hz[] = { TOPIC_RC, TOPIC_VELOCITY, TOPIC_QUATERNION, TOPIC_GPS_FUSED, TOPIC_ALTITUDE_FUSIONED };
  numTopic                  = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  enableTimestamp           = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    printf("ISSUE WITH PACKAGE STATUS RC \n");
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    printf("CLEAN UP PACKAGE --> RC\n");
  }
//////////////////////////////////////////////////////////////////////////////////////
}

void updateOptimSubscription(Vehicle* vehicle, int responseTimeout, customOA_v2_ref &in){

    //TOPIC_HARD_SYNC -- contains time
    // 
    in.flightStatus = vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>();
    in.gps_time     = vehicle->subscribe->getValue<TOPIC_GPS_TIME>();
    in.gps_date     = vehicle->subscribe->getValue<TOPIC_GPS_DATE>();
    in.gps_details  = vehicle->subscribe->getValue<TOPIC_GPS_DETAILS>();
    in.latLon       = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    in.altitude     = vehicle->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
    in.rc           = vehicle->subscribe->getValue<TOPIC_RC>();
    in.velocity     = vehicle->subscribe->getValue<TOPIC_VELOCITY>();
    in.quaternion   = vehicle->subscribe->getValue<TOPIC_QUATERNION>();

}

void quitOptimSubscription(Vehicle* vehicle, int responseTimeout){  
  vehicle->subscribe->removePackage(0, responseTimeout);
  vehicle->subscribe->removePackage(1, responseTimeout);
}

void     INThandler(int);
static bool keepRunning = true;

bool
subscribeToDataForInteractivePrint(Vehicle* vehicle, int responseTimeout)
{
  // RTK can be detected as unavailable only for Flight controllers that don't support RTK


  // Please make sure your drone is in simulation mode. You can fly the drone
  // with your RC to
  // get different values.

  // Telemetry: Verify the subscription
  ACK::ErrorCode subscribeStatus;
  subscribeStatus = vehicle->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }

  // Package 0: Subscribe to flight status at freq 1 Hz
  int       pkgIndex        = 0;
  int       freq            = 50;
  TopicName topicList50Hz[]  = {TOPIC_RC
                                ,TOPIC_RC_FULL_RAW_DATA
                                ,TOPIC_RC_WITH_FLAG_DATA
                                ,TOPIC_ESC_DATA
                                ,TOPIC_RTK_CONNECT_STATUS
                                ,TOPIC_GIMBAL_CONTROL_MODE
                                ,TOPIC_FLIGHT_ANOMALY
                                ,TOPIC_POSITION_VO
                                ,TOPIC_AVOID_DATA
                                ,TOPIC_HOME_POINT_SET_STATUS
                                ,TOPIC_HOME_POINT_INFO
  };

  int       numTopic        = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
          pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);

  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Wait for the data to start coming in.
  sleep(1);

  while(true)
  {
    int userInput;

    std::cout << "Select variable you are interested in printing:\n"
              << "1. rcFuncFull\n"
              << "2. rcWithFlag\n"
              << "3. escData\n"
              << "4. rtkConnect\n"
              << "5. gimbalCtrlMode\n"
              << "6. flyAnomaly\n"
              << "7. local position vo\n"
              << "8. avoid obstacle data\n"
              << "9. home location set status\n"
              << "10. home location information\n"
              << "0. exit\n";


    std::cin >> userInput;

    if(userInput == 0)
      break;

    // Get all the data once before the loop to initialize vars
    TypeMap<TOPIC_RC>::type rc;

    TypeMap<TOPIC_RC_FULL_RAW_DATA>::type rcFuncFull;
    TypeMap<TOPIC_RC_WITH_FLAG_DATA>::type rcWithFlag;
    TypeMap<TOPIC_ESC_DATA>::type escData;
    TypeMap<TOPIC_RTK_CONNECT_STATUS>::type rtkConnect;
    TypeMap<TOPIC_GIMBAL_CONTROL_MODE>::type gimbalCtrlMode;
    TypeMap<TOPIC_FLIGHT_ANOMALY>::type flyAnomaly;
    TypeMap<TOPIC_POSITION_VO>::type    localPos;
    TypeMap<TOPIC_AVOID_DATA>::type     avoidData;
    TypeMap<TOPIC_HOME_POINT_SET_STATUS>::type homePointSetStatus;
    TypeMap<TOPIC_HOME_POINT_INFO>::type  homeLocationInfo;

    // Counters
    int printFrequency          = 50; //Hz
    int printIntervalInMicroSec = 1e6/printFrequency;
    int totalPrintTimeInSec     = 10;
    int totalSample             = totalPrintTimeInSec * printFrequency;
    // Print in a loop for 2 sec
    while(totalSample--)
    {
      switch(userInput)
      {
        case 1: //rcFuncFull
          rcFuncFull = vehicle->subscribe->getValue<TOPIC_RC_FULL_RAW_DATA>();
          printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                 rcFuncFull.lb2.roll,
                 rcFuncFull.lb2.pitch,
                 rcFuncFull.lb2.yaw,
                 rcFuncFull.lb2.throttle,
                 rcFuncFull.lb2.mode,
                 rcFuncFull.lb2.gear,
                 rcFuncFull.lb2.camera,
                 rcFuncFull.lb2.video,
                 rcFuncFull.lb2.videoPause,
                 rcFuncFull.lb2.goHome,
                 rcFuncFull.lb2.leftWheel,
                 rcFuncFull.lb2.rightWheelButton,
                 rcFuncFull.lb2.rcC1,
                 rcFuncFull.lb2.rcC2);
          break;
        case 2:
          rcWithFlag = vehicle->subscribe->getValue<TOPIC_RC_WITH_FLAG_DATA>();
          printf("%f, %f, %f, %f, %d, %d, %d, %d,\n",
                 rcWithFlag.roll,
                 rcWithFlag.pitch,
                 rcWithFlag.yaw,
                 rcWithFlag.throttle,
                 rcWithFlag.flag.logicConnected,
                 rcWithFlag.flag.groundConnected,
                 rcWithFlag.flag.skyConnected,
                 rcWithFlag.flag.appConnected
          );
          break;
        case 3:
          escData = vehicle->subscribe->getValue<TOPIC_ESC_DATA>();
          printf("Speeds: %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
                 escData.esc[0].speed,
                 escData.esc[1].speed,
                 escData.esc[2].speed,
                 escData.esc[3].speed,
                 escData.esc[4].speed,
                 escData.esc[5].speed,
                 escData.esc[6].speed,
                 escData.esc[7].speed
          );
          break;
        case 4:
          rtkConnect = vehicle->subscribe->getValue<TOPIC_RTK_CONNECT_STATUS>();
          printf("RTKConnected = %d\n", rtkConnect.rtkConnected);
          break;
        case 5:
          gimbalCtrlMode = vehicle->subscribe->getValue<TOPIC_GIMBAL_CONTROL_MODE>();
          printf("Gimbal Control Mode = %d\n", gimbalCtrlMode);
          break;
        case 6:
          flyAnomaly = vehicle->subscribe->getValue<TOPIC_FLIGHT_ANOMALY>();
          printf("FlyAnomaly: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                 flyAnomaly.impactInAir,
                 flyAnomaly.randomFly,
                 flyAnomaly.heightCtrlFail,
                 flyAnomaly.rollPitchCtrlFail,
                 flyAnomaly.yawCtrlFail,
                 flyAnomaly.aircraftIsFalling,
                 flyAnomaly.strongWindLevel1,
                 flyAnomaly.strongWindLevel2,
                 flyAnomaly.compassInstallationError,
                 flyAnomaly.imuInstallationError,
                 flyAnomaly.escTemperatureHigh,
                 flyAnomaly.atLeastOneEscDisconnected,
                 flyAnomaly.gpsYawError);
          break;
        case 7:
          localPos = vehicle->subscribe->getValue<TOPIC_POSITION_VO>();
              printf("PX=%.2f, PY=%.2f, PZ=%.2f\n",
                     localPos.x,
                     localPos.y,
                     localPos.z
              );
              break;
        case 8:
          avoidData =  vehicle->subscribe->getValue<TOPIC_AVOID_DATA>();
          printf("down = %.2f, down health = %d, front = %.2f,  front health = %d, right = %.2f, right health = %d,"
                 " back = %.2f, back health = %d, left = %.2f, left health = %d, up = %.2f, up health = %d\n",
                 avoidData.down , avoidData.downHealth ,
                 avoidData.front, avoidData.frontHealth,
                 avoidData.right, avoidData.rightHealth,
                 avoidData.back , avoidData.backHealth ,
                 avoidData.left , avoidData.leftHealth ,
                 avoidData.up   , avoidData.upHealth);
          break;
        case 9:
          homePointSetStatus =  vehicle->subscribe->getValue<TOPIC_HOME_POINT_SET_STATUS>();
          printf("home location statas is %d\n", homePointSetStatus.status);
          break;
        case 10:
          homeLocationInfo = vehicle->subscribe->getValue<TOPIC_HOME_POINT_INFO>();
          printf("home location latitude is :%.15f, home location longitude is %.15f\n",homeLocationInfo.latitude, homeLocationInfo.longitude);
          break;
        case 0:
          break;
        default:
          break;

      }

      usleep(printIntervalInMicroSec);
    }
  }
  std::cout << "Done printing!\n";
  vehicle->subscribe->removeAllExistingPackages();
  return true;
}

bool
subscribeToDataAndSaveLogToFile(Vehicle* vehicle, int responseTimeout)
{
    signal(SIGINT, INThandler);
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        return false;
    }

    int       pkgIndex        = 0;
    int       freq            = 50;
    TopicName topicList50Hz[]  = {
             TOPIC_VELOCITY
            ,TOPIC_RC_WITH_FLAG_DATA
            ,TOPIC_RTK_CONNECT_STATUS
            ,TOPIC_POSITION_VO
            ,TOPIC_ALTITUDE_FUSIONED
            ,TOPIC_ALTITUDE_BAROMETER
            ,TOPIC_HEIGHT_FUSION
            ,TOPIC_GPS_FUSED
            ,TOPIC_STATUS_DISPLAYMODE
    };

    int       numTopic        = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
    bool      enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
    std::cout <<"\n1. InitPackageFromTopicList" <<std::endl;

    if (!(pkgStatus))
    {
      std::cout <<"1. InitPackageFromTopicList failed" <<std::endl;
      return pkgStatus;
    }
    std::cout <<"1. StartPackage" <<std::endl;

    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        return false;
    }

///////////////////////////////////////////
    pkgIndex                   = 1;
    freq                       = 1;
    TopicName topicList1Hz[] = { TOPIC_HEIGHT_HOMEPOINT
                                 ,TOPIC_GPS_POSITION
                                 ,TOPIC_GPS_VELOCITY};
    numTopic        = sizeof(topicList1Hz) / sizeof(topicList1Hz[0]);
    enableTimestamp = false;

    std::cout <<"2. initPackageFromTopicList" <<std::endl;

    pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topicList1Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      std::cout <<"2.1 initPackageFromTopicList" <<std::endl;
      return pkgStatus;
    }
    std::cout <<"2.startPackage" <<std::endl;
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        return false;
    }

    // Wait for the data to start coming in.
    sleep(1);

    // Get all the data once before the loop to initialize vars
    TypeMap<TOPIC_VELOCITY>::type           velocity;
    TypeMap<TOPIC_RC_WITH_FLAG_DATA>::type  rc_with_flag_data;
    TypeMap<TOPIC_RTK_CONNECT_STATUS>::type rtk_connect_status;
    TypeMap<TOPIC_POSITION_VO>::type        position_vo;
    TypeMap<TOPIC_ALTITUDE_FUSIONED>::type  altitude_fusioned;
    TypeMap<TOPIC_ALTITUDE_BAROMETER>::type altitude_barometer;
    TypeMap<TOPIC_HEIGHT_HOMEPOINT>::type   height_homepoint;
    TypeMap<TOPIC_HEIGHT_FUSION>::type      height_fusion;
    TypeMap<TOPIC_GPS_FUSED>::type          gps_fused;
    TypeMap<TOPIC_STATUS_DISPLAYMODE>::type status_displaymode;
    TypeMap<TOPIC_STATUS_FLIGHT>::type      status_flight;
    TypeMap<TOPIC_GPS_POSITION>::type       gpsPostion;
    TypeMap<TOPIC_GPS_VELOCITY>::type       gpsVelocity;

  // Counters
    int printFrequency          = 50; //Hz
    int printIntervalInMicroSec = 1e6/printFrequency;
    int totalPrintTimeInSec     = 1000;  // 1000 : 16min
    int totalSample             = totalPrintTimeInSec * printFrequency;
    int notifyInterval          = 50;
    int notifyCount             = notifyInterval;
    FILE * pFile;
    pFile = fopen ("telemetryLogFile.txt","w");
    fprintf ( pFile,
            "velocity_data_x,"
            "velocity_data_y,"
            "velocity_data_z,"
            "rc_with_flag_data_roll,"
            "rc_with_flag_data_pitch,"
            "rc_with_flag_data_yaw,"
            "rc_with_flag_data_throttle,"
            "position_vo_x,"
            "position_vo_y,"
            "position_vo_z,"
            "altitude_fusioned,"
            "altitude_barometer,"
            "height_homepoint,"
            "height_fusion,"
            "gps_fused_latitude,"
            "gps_fused_longitude,"
            "gps_fused_altitude,"
            "gps_longitude,"
            "gps_latitude,"
            "gps_altitude,"
            "velocity_x,"
            "velocity_y,"
            "velocity_z,"
            "status_displaymode,"
            "status_flight,"
            "rtk_connect_status_rtkConnected\n"
    );
    // Print in a loop for 2 sec
    while(totalSample--)
    {
        velocity           = vehicle->subscribe->getValue<TOPIC_VELOCITY>();
        rc_with_flag_data  = vehicle->subscribe->getValue<TOPIC_RC_WITH_FLAG_DATA>();
        rtk_connect_status = vehicle->subscribe->getValue<TOPIC_RTK_CONNECT_STATUS>();
        position_vo        = vehicle->subscribe->getValue<TOPIC_POSITION_VO>();
        altitude_fusioned  = vehicle->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
        altitude_barometer = vehicle->subscribe->getValue<TOPIC_ALTITUDE_BAROMETER>();
        height_homepoint   = vehicle->subscribe->getValue<TOPIC_HEIGHT_HOMEPOINT>();
        height_fusion      = vehicle->subscribe->getValue<TOPIC_HEIGHT_FUSION>();
        gps_fused          = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        gpsPostion         = vehicle->subscribe->getValue<TOPIC_GPS_POSITION>();
        gpsVelocity        = vehicle->subscribe->getValue<TOPIC_GPS_VELOCITY>();
        status_displaymode = vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>();

        fprintf ( pFile, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%lf,%lf,%f,%d,%d,%d,%f,%f,%f,%d,%d,%d\n"
                 ,velocity.data.x
                 ,velocity.data.y
                 ,velocity.data.z
                 ,rc_with_flag_data.roll
                 ,rc_with_flag_data.pitch
                 ,rc_with_flag_data.yaw
                 ,rc_with_flag_data.throttle
                 ,position_vo.x
                 ,position_vo.y
                 ,position_vo.z
                 ,altitude_fusioned
                 ,altitude_barometer
                 ,height_homepoint
                 ,height_fusion
                 ,gps_fused.latitude
                 ,gps_fused.longitude
                 ,gps_fused.altitude
                 ,gpsPostion.x
                 ,gpsPostion.y
                 ,gpsPostion.z
                 ,gpsVelocity.x
                 ,gpsVelocity.y
                 ,gpsVelocity.z
                 ,status_displaymode
                 ,status_flight
                 ,rtk_connect_status.rtkConnected
        );

        if(!keepRunning)
        {
            std::cout << "Ctrl-C pressed, quit loop" << std::endl;
            break;
        }

        if( (--notifyCount) == 0 )
        {
            std::cout << "Printing to file ...\n";
            notifyCount = notifyInterval;
        }
        usleep(printIntervalInMicroSec);
    }

    std::cout << "Done printing!\n";
    fclose (pFile);
    return true;
}



/////////////////
//// M600
void init_log(customOA_v2_ref &in){
  //file has been created
  if(in.log_to_file_.is_open()){
    in.log_to_file_ << "gps time,";
    in.log_to_file_ << "status flight,";
    in.log_to_file_ << "rc gear,";
    in.log_to_file_ << "rc mode,";
    in.log_to_file_ << "gps-lat,";
    in.log_to_file_ << "gps-lon,";
    in.log_to_file_ << "gps-alt,";
    in.log_to_file_ << "gps-num,";
    in.log_to_file_ << "alt,";
    in.log_to_file_ << "q0,";
    in.log_to_file_ << "q1,";
    in.log_to_file_ << "q2,";
    in.log_to_file_ << "q3\n";
  }
  in.log_to_file_.flush();

}

void logDJI_DATA(customOA_v2_ref &in){
  //file_in has been created
  
  // in.latLon.altitude
  // in.latLon.latitude
  // in.latLon.longitude
  // in.latLon.visibleSatelliteNumber
  // in.altitude
  // in.rc.gear
  // in.rc.mode
  // in.velocity.data.x
  // in.velocity.data.y
  // in.velocity.data.z
  // in.quaternion.q0
  // in.quaternion.q1
  // in.quaternion.q2
  // in.quaternion.q3
  // in.gps_time

  //in.rc.lb2.video
  //in.rc.lb2.videoPause
//  printf("video = %d, videoPause is %d \n",in.rc.lb2.video, in.rc.lb2.videoPause);
//if gear is -5000 --gear is up...record ...-10000 is down

  /*(if(in.rc.gear > -6000){
  printf("mode is %d, gear is %d \n",in.rc.mode, in.rc.gear); 
  }*/

  if(in.log_to_file_.is_open()){
    in.log_to_file_ << in.gps_time;
    in.log_to_file_ << ", ";
    in.log_to_file_ << std::fixed << (int)in.flightStatus;
    in.log_to_file_ << ", ";
    in.log_to_file_ << std::fixed << (int)in.rc.gear;
    in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << (int)in.rc.mode;
    in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.latLon.latitude;
    in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.latLon.longitude;
    in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.latLon.altitude;
    in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.latLon.visibleSatelliteNumber;
    in.log_to_file_ << ", ";  
    in.log_to_file_ << std::fixed << in.altitude;
    in.log_to_file_ << ", ";        
    in.log_to_file_ << std::fixed << in.quaternion.q0;
    in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.quaternion.q1;
    in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.quaternion.q2;
    in.log_to_file_ << ", ";    
    in.log_to_file_ << std::fixed << in.quaternion.q3;
    in.log_to_file_ << "\n";    
    
  }
  in.log_to_file_.flush();
}
void updateDJI_DATA(DJI::OSDK::Vehicle* vehicle, int responseTimeout,  customOA_v2_ref &in){
  updateOptimSubscription( vehicle, responseTimeout, in);
}

void updateLog(customOA_v2_ref &in){

  if(!in.logInitalized_ && !in.log_file_running_){
    // log not initalized and not yet running
    //we haven't created a log file yet
    //check to see if gps good
    //printf("gps fix is %f \n",in.gps_details.fix);
    if( in.gps_details.fix > 0.5){
      //assume some number for now that says a gps data/time has been started
      in.log_file_running_  = true;                         //set to true for future ..this will create a log
      std::string date      = std::to_string(in.gps_date);  //in.gpsInfo.time.date
      std::string time      = std::to_string(in.gps_time);  //in.gpsInfo.time.time
      in.log_folder_name_   = date + time ;                 //log file will be stamped with date/time
    }else
    {
      in.missed_gps_counter_++;
      printf("cnt %u \n",in.missed_gps_counter_);

      if(in.missed_gps_counter_ > 10){
        //we skip log name based on gps
        std::string rando     = std::to_string(rand());
        in.log_folder_name_   = "log" + rando ;
        in.log_file_running_  = true;
      }

    }

  }

  if(!in.logInitalized_ && in.log_file_running_){
      //first loop thru when log file name has been populated
      // create a log file with data/time name
      printf("log init\n");
      in.logInitalized_ = true;

      const int dir_err = mkdir(in.log_folder_name_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      if (-1 == dir_err)
      {
          printf("Error creating directory!n");
          exit(1);
      }

      //TBD need to make this variable -- want a new directory to be made for each power up
      std::string folder_and_file = in.log_folder_name_ + "/DJI_data.txt";
      in.log_to_file_.open (folder_and_file);  
      init_log(in);
    }

    //update dji log
    if(in.logInitalized_){
        //log dji data
        logDJI_DATA(in);
        printf("logged\n");
    }
}

void log_printf( customOA_v2_ref &in, std::string to_be_logged){

  if(in.logInitalized_)
  {
    if(in.log_to_file_.is_open())
    {
      in.log_to_file_ << to_be_logged;
      in.log_to_file_ << "\n";
    }
      in.log_to_file_.flush();
  }
}

////////////////

void  INThandler(int sig)
{
    keepRunning = false;
}