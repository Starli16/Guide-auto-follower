#include "guide_filter.h"

#include <iostream>
#include <cstdio>
using std::cout;
using std::endl;
using namespace std;

bool guide_Filter::Init() {


  chassis_detail_writer_ =
      node_->CreateWriter<ChassisDetail>("guide/ChassisDetail");
  // Create Writer

  chassis_detail_origin_reader_ = node_->CreateReader<ChassisDetail>(
      "guide/ChassisDetailOrig",
      [this](const std::shared_ptr<ChassisDetail>& msg) {UpdateChassisDetail(msg);});

  //read config;
  
  FILE* f;
  f=fopen("/apollo/modules/guide_filter/Filter.config","r");
  if(f!=NULL) {
    fscanf(f,"%d",&FilterLength);
    AINFO<<"FilterLength= "<<FilterLength;
    fclose(f);
  }
  else AERROR << "Filter.config Missing";
  return true;
}

bool guide_Filter::Proc() {  // Timer callback;
  AINFO<<"Distance Filter distance "<<chassis_detail_.uwb_distance();
  chassis_detail_writer_->Write(chassis_detail_);
  return true;
}
void guide_Filter::Clear() {  // shutdown

}
void guide_Filter::UpdateChassisDetail(const std::shared_ptr<ChassisDetail>& msg) {
  chassis_detail_=*msg;
  //uwb_azimuth
  static AverageFilter<double> azimuth_filter(FilterLength, -180, 180);
  azimuth_filter.AddElement(msg->uwb_azimuth());
  chassis_detail_.set_uwb_azimuth( azimuth_filter.GetValue());
  //uwb_distance
  static AverageFilter<double> distance_filter(FilterLength,-10,100);
  distance_filter.AddElement(msg->uwb_distance());
  chassis_detail_.set_uwb_distance( distance_filter.GetValue());

  //steer_angle
  static AverageFilter<double> steer_angle_filter(FilterLength,-50000,50000);
  steer_angle_filter.AddElement(msg->steer_angle());
  chassis_detail_.set_steer_angle( steer_angle_filter.GetValue());
  //follower_speed
  static AverageFilter<double> follower_speed_filter(FilterLength,-10,7200);
  follower_speed_filter.AddElement(msg->x_speed());
  chassis_detail_.set_x_speed( follower_speed_filter.GetValue());
  //Follower Yaw rate
  static AverageFilter<double> follower_yaw_rate_filter(FilterLength,-180,180);
  follower_yaw_rate_filter.AddElement(msg->follower_yaw_rate());
  chassis_detail_.set_follower_yaw_rate( follower_yaw_rate_filter.GetValue());
  return;
}
