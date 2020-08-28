#include "guide_control.h"
#include <cstdio>
#define ACC_LIMIT 0.5
#define DEACC_LIMIT -4

float Desired_speed = 0;
float Desired_distance = 20;
constexpr float L = 5.0;
constexpr float r = 1.0;
constexpr float k_a = 0.1;
constexpr float k_v = 0.05;
constexpr float k_d = 0.05;

bool guide_Control::Init() {
  using namespace std;
  AINFO << "Guide_Control init";
  FILE* f;
  f=fopen("/apollo/modules/guide_control/SteerPID.config","r");
  if(f!=NULL) {
    fscanf(f,"%f%f%f",&steer_PID_kp,&steer_PID_ki,&steer_PID_kd);
    AINFO<<"steerKP= "<<steer_PID_kp<<" steerKI=" << steer_PID_ki<<" steerKP=" << steer_PID_kd;
    fclose(f);
  }
  else AERROR << "Steer PID config Missing";

  writer = node_->CreateWriter<ControlCommand>("guide/ControlCommand");
  // Init ControlCommand Writer
  return true;
}

/*
  Reader Callback function
*/

bool guide_Control::Proc(const std::shared_ptr<ChassisDetail>& msg0,
                         const std::shared_ptr<TrajInfo>& msg1) {
  DealTraj(msg1);

  // calculate steer
  float distance = msg0->uwb_distance();
  float azimuth = msg0->uwb_azimuth();
  AINFO << "reveiced Chassis Detail UWBdistance:" << distance
        << "  azimuth:" << azimuth;

  control_steer = Caculate_steer(msg0,msg1);
  controlcmd.set_control_steer(control_steer);

  // calculate acc
  control_acc = Caculate_acc(msg0);
  controlcmd.set_control_acc(control_acc);
  AINFO << controlcmd.DebugString();

  writer->Write(controlcmd);
  return true;
}

void guide_Control::DealTraj(const std::shared_ptr<TrajInfo>& msg1) {
  using namespace std;
  // save Traj
  int number = msg1->rel_x_size();
  FILE* f;
  f = fopen("/apollo/modules/traj.record", "w");
  if (f != NULL) {
    fprintf(f, "%d\n", number);
    for (int i = 0; i < number; i++) {
      fprintf(f, "%f %f\n", msg1->rel_x(i), msg1->rel_y(i));
    }
    fclose(f);
  }

  // Bezier Curve
  BezierFitting(msg1);
  // Lateral error
  int index_la = FindLookAheadPointBezier(2);
  err_lat = BezierX[index_la];
  AINFO << "Lateral Error is: " << err_lat;
}

/* Find Bezier Curve.*/

void guide_Control::BezierFitting(const std::shared_ptr<TrajInfo>& msg1) {
  //double BezierX[251], BezierY[251];//TODO: points number
  for (int i = 0; i <= 250; i++) BezierX[i] = BezierY[i] = 0;
  int number = msg1->rel_x_size();
  // 4 control point
  int ControlPointIndex[6];
  float Px[6], Py[6];
  for (int i = 0; i < 6; i++) {
    ControlPointIndex[i] = floor((number - 1) * i / 5);
    Px[i] = msg1->rel_x(ControlPointIndex[i]);
    Py[i] = msg1->rel_y(ControlPointIndex[i]);
  }
  // Cal Curve
  for (int i = 0; i < 251; i++) {
    BezierX[i] = CalBezierLoc(6, i * 0.004, Px);
    BezierY[i] = CalBezierLoc(6, i * 0.004, Py);
  }
  FILE* f;
  f = fopen("/apollo/modules/bezier.record", "w");
  if (f != NULL) {
    fprintf(f, "%d\n", 251);
    for (int i = 0; i < 251; i++) {
      fprintf(f, "%f %f\n", BezierX[i], BezierY[i]);
    }
    fclose(f);
  }
}

float guide_Control::CalBezierLoc(int n, float t, float p[]) {
  float loc = 0;
  // X(t)=sigma( nchoosek(n-1,k)*t^k*(1-t)^(n-1-k)*Px)
  for (int j = 0; j < n; j++) {
    float ret = 1;
    for (int i = 0; i < j; i++) ret *= t;
    for (int i = 0; i < n - 1 - j; i++) ret *= (1 - t);
    // nchoosek
    for (int i = n - 1; i >= n - j; i--) ret *= (i);
    for (int i = 1; i <= j; i++) ret /= i;
    ret *= p[j];
    loc += ret;
  }

  return loc;
}

/*
  Input ChassisDetail message
  Output Control_steer degree
*/

float guide_Control::Caculate_steer(const std::shared_ptr<ChassisDetail>& msg0,
    const std::shared_ptr<TrajInfo>& msg1) {
  // cout << "lat_distance = "<< to_string(lat_distance) << endl;


  //Calculate the longitudinal and the lateral distance of the lookahead point
  
  /* Lookahead point is uwb point
  float distance = msg0->uwb_distance();
  float azimuth_angle = msg0->uwb_azimuth();

  if (distance == 0) return 4.8505;
  float long_distance;
  long_distance = distance * cos(azimuth_angle / 180 * M_PI);  // m
  float lat_distance;
  lat_distance = distance * sin(azimuth_angle / 180 * M_PI);  // m
    */
  
  /* Specify a lookahead point*/
  int index_la = FindLookAheadPointBezier(10);
  float long_distance;
  long_distance = BezierX[index_la];
  float lat_distance;
  lat_distance = BezierY[index_la];

  AINFO << "lookahead x is: " << long_distance;
  AINFO << "lookahead y is: " << lat_distance;


  PID pid_steer(steer_PID_kp, steer_PID_ki, steer_PID_kd);
  float frontwheel_steer_angle =
      -0.6 * atan(2 * L * lat_distance / (long_distance * long_distance)) * 180 /
          M_PI +
      0.4*pid_steer.pid_control(0, err_lat) ;
  if (frontwheel_steer_angle > 20)
    frontwheel_steer_angle = 20;
  else if (frontwheel_steer_angle < -20)
    frontwheel_steer_angle = -20;  // steer angle limit
  // cout << "Frontwheel_steer_angle = "<< to_string(frontwheel_steer_angle) <<
  // endl;

  // Saturation
  float steer_wheel_angle =
      24.1066 * frontwheel_steer_angle + 4.8505;  // Caculate from steer map
  return steer_wheel_angle;
}

/*
  Input ChassisDetail message
  Output Control_acc m/s^2
*/
float guide_Control::Caculate_acc(const std::shared_ptr<ChassisDetail>& msg0) {
  float v1 = msg0->leader_speed();
  float v2 = msg0->x_speed();
  float a1 = msg0->leader_acc();
  float distance = msg0->uwb_distance();
  float azimuth_angle = msg0->uwb_azimuth();
  float long_distance = distance * cos(azimuth_angle / 180 * M_PI);  // m
  float Leader_Brake_pedal = msg0->leader_brake_pedal();
  float Leader_Acc_pedal=msg0->leader_acc_pedal();
  float control_acc = 0;

  /******Distance Keeping Control*****/

  // cout << "a = " << a1 << endl;
  // cout << "delta v = " << v1-v2 << endl;
  // cout << "delta x = " << long_distance - Desired_distance << endl;
  float distance_error = distance - float(Desired_distance);
  AINFO<<"distance_error= "<<distance_error;
  if (distance_error < 10 && distance_error > -5)
    control_acc = k_a * a1 + k_v * (v1 - v2) + k_d * distance_error;
  else
    control_acc = k_a * a1 / 2 + k_v * (v1 - v2) / 2 + k_d * distance_error * 2;

  if (control_acc < 0) control_acc = control_acc * 4;
  // cout << "Leader Brake Pedal = " << Leader_Brake_pedal << endl;
  if (Leader_Brake_pedal > 45) control_acc = control_acc - 2.5;

  if(Leader_Brake_pedal>0){
    double pedal=Leader_Brake_pedal/100;
    if(pedal>0.1 && pedal <0.5) {
      double k=3*(0.6+0.8*pedal);
      control_acc=control_acc-k*pedal;
    }
    else if (pedal >=0.5){
      double k=3;
      control_acc=control_acc-k*pedal;
    }
  }
  else if(Leader_Acc_pedal>0){
    double pedal=Leader_Acc_pedal/100;
    if(pedal>0.1) { 
      double k=3*(0.4+0.4*pedal);
      control_acc=control_acc+k*pedal;
    }
  }

  // control_acc = 0.05 * (long_distance - Desired_distance);

  // Saturation
  if (control_acc > ACC_LIMIT)
    control_acc = ACC_LIMIT;  // acc limit
  else if (control_acc < DEACC_LIMIT)
    control_acc = DEACC_LIMIT;  // deacc limit
  // cout << "Control_acceleration = "<< to_string(control_acc) << endl;
  return control_acc;
}

int guide_Control::FindLookAheadPoint(float LookAheadDis, const std::shared_ptr<TrajInfo>& msg1){
  int index_max = 0; //The possible max index of LA point
  while (msg1->rel_x(index_max) < LookAheadDis )
  {
    index_max++;
  }

  int index_la = index_max; //The index of LA point
  while (msg1->rel_x(index_la)*msg1->rel_x(index_la)+msg1->rel_y(index_la)*msg1->rel_y(index_la)-
          LookAheadDis*LookAheadDis > 0)
  {
    index_la--;
  }
  return(index_la);
}
/******************************************************************
 * Function: FindLookAheadPoint;
 * Description: Find the index of the point nearest of the LA point,
 * and return the index;
 * Input: 
 *      float LookAheadDis: Lookahead Distance;
 *      const std::shared_ptr<TrajInfo>& msg1: the pointer to the 
 *                    trajectory infomation;
 * Output:
 *      int index_la: the index of the la point;
 *****************************************************************/

int guide_Control::FindLookAheadPointBezier(float LookAheadDis){
    float DisSum=0;
    int i;
    for(i=1;i<251;i++){
        float dis=sqrt( (BezierX[i]-BezierX[i-1])*(BezierX[i]-BezierX[i-1]) +
                (BezierY[i]-BezierY[i-1])*(BezierY[i]-BezierY[i-1]) );
        DisSum+=dis;
        if(DisSum>LookAheadDis) break;
    }
    return i;
}
