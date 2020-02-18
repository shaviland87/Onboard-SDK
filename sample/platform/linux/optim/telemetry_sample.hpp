/*! @file telemetry_sample.hpp
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

#ifndef DJIOSDK_TELEMETRYSAMPLE_HPP
#define DJIOSDK_TELEMETRYSAMPLE_HPP

// System Includes
#include <iostream>
#include <fstream>

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>
#include <dji_telemetry.hpp>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

  /* Time, location(lat/lon/alt), attitude, temperature? */
struct customOA_ref {
  Telemetry::Status         status;
  Telemetry::GlobalPosition globalPosition; // has lat/long/alt/health
  Telemetry::RC             rc;
  Telemetry::Vector3f       velocity;
  Telemetry::TimeStamp      timestamp;
  Telemetry::GPSInfo        gpsInfo;
  Telemetry::Quaternion     quat; //attitude
  Telemetry::SBUSFullRawData raw_rc; // raw rc -- might be useful
  std::string               log_folder_name_;
  bool                      log_file_running_;
  bool                      logInitalized_;
  uint32_t                  last_time_log_;
  uint32_t                  missed_gps_counter_;
  std::ofstream             log_to_file_;
};

struct customOA_v2_ref {

  DJI::OSDK::Telemetry::TypeMap<TOPIC_STATUS_FLIGHT>::type      flightStatus;
  DJI::OSDK::Telemetry::TypeMap<TOPIC_GPS_FUSED>::type          latLon;
  DJI::OSDK::Telemetry::TypeMap<TOPIC_ALTITUDE_FUSIONED>::type  altitude;
  DJI::OSDK::Telemetry::TypeMap<TOPIC_RC>::type   rc;
  DJI::OSDK::Telemetry::TypeMap<TOPIC_VELOCITY>::type           velocity;
  DJI::OSDK::Telemetry::TypeMap<TOPIC_QUATERNION>::type         quaternion;
  DJI::OSDK::Telemetry::TypeMap<TOPIC_GPS_TIME>::type           gps_time; 
  DJI::OSDK::Telemetry::TypeMap<TOPIC_GPS_DATE>::type           gps_date; 
  DJI::OSDK::Telemetry::TypeMap<TOPIC_GPS_DETAILS>::type        gps_details; 

  std::string                             log_folder_name_;
  bool                                    log_file_running_;
  bool                                    logInitalized_;
  uint32_t                                last_time_log_;
  uint32_t                                missed_gps_counter_;
  std::ofstream                           log_to_file_;
};

bool subscribeToData(DJI::OSDK::Vehicle* vehiclePtr, int responseTimeout = 1);
bool subscribeToDataForInteractivePrint(DJI::OSDK::Vehicle* vehiclePtr, int responseTimeout = 1);
bool subscribeToDataAndSaveLogToFile(DJI::OSDK::Vehicle* vehiclePtr, int responseTimeout = 1);

// Broadcast data implementation for Matrice 100
bool getBroadcastData(DJI::OSDK::Vehicle* vehicle, int responseTimeout = 1);

//update custom
void updateLog(customOA_ref &in);
void updateDJI_DATA(DJI::OSDK::Vehicle* vehicle, int responseTimeout ,  customOA_ref &in);
void init_log(customOA_ref &in);
void logDJI_DATA(customOA_ref &in);
void optimInitializeSubscribe(Vehicle* vehicle, int responseTimeout);
void updateOptimSubscription(Vehicle* vehicle, int responseTimeout, customOA_v2_ref &in);

//m600 calls
void init_log(customOA_v2_ref &in);
void logDJI_DATA(customOA_v2_ref &in);
void updateLog(customOA_v2_ref &in);
void updateDJI_DATA(DJI::OSDK::Vehicle* vehicle, int responseTimeout ,  customOA_v2_ref &in);

//debug calls
void log_printf( customOA_v2_ref &in, std::string to_be_logged);


void quitOptimSubscription(Vehicle* vehicle, int responseTimeout);



#endif // DJIOSDK_TELEMETRYSAMPLE_HPP
