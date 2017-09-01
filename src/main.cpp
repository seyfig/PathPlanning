#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"


using namespace std;

// for convenience
using json = nlohmann::json;

const double mphtoms = 0.44704;
const double tint = 0.02;
const double tpint = 0.1;

const int DANGER = pow(10.0, 3);
const int NEAR = pow(10.0, 4);
const int COMFORT = pow(10.0, 1);
const int EFFICIENCY = pow(10.0, 4);
const int CENTER = pow(10.0,1);
const int OCCUPIED_LANE = 5 * pow(10.0,2);
const int CANCEL_COST = pow(10.0, 2);
const int PREPARE_WO_CHANGE = pow(10.0, 1);;
const double MAXCOST = pow(10.0, 6);

const double DESIRED_BUFFER = 0.5;

const int TIME_DISTANCE = 200;
const int PLANNING_HORIZON = 5;
const int PLANNING_DISTANCE = 30;
const int PLANNING_STEPS = 20;

const double aint = 0.185;
const double amax = aint / tint;

const double max_v = 49.84 * mphtoms;

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

const int LANE_SIZE = 3;
const double LANE_WID = 4.0;
const double CAR_LEN = 10.0;
const double CAR_WID = 2.0;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

struct StepObject{
    double a;
    double lowest_time;
    double closest_approach;
};


// VEHICLE PART START
class Vehicle {
public:
    struct TrajectoryObject{
        vector<Vehicle> trajectory;
        vector<double> a_list;
        double v_sum = 0;
        double t_sum = 0;
        int step_count = 0;
        int cancel_count = 0;
        vector<double> closest_approach_list;
        vector<double> lowest_time_list;
        vector<double> lowest_time_front_list;
        int total_lane_changes = 0;

        string state;
        vector<string> state_list;
        double cost;
    };


  double preferred_buffer = 13.0; // impacts "keep lane" behavior.

  int prep_lane;

  int target_lane;

  int current_lane;

  double s;

  double d;

  double target_d;

  double time;

  double v;

  double a;

  vector<double> a_list;

  double target_speed;

  string state;

  bool lane_changing;
  bool lane_change_finish;

  bool plcl_lcl;
  bool plcr_lcr;

  int step;


  /**
  * Constructor
  */
  Vehicle(int lane, double s, double v, double a, double target_speed);


  /**
  * Destructor
  */
  virtual ~Vehicle();

  Vehicle copy_vehicle();

  void restore_vehicle(Vehicle snapshot);

  vector<string> get_available_states();

  void update_state(vector<vector<double>> sensor_fusion);

  TrajectoryObject get_next_state_recursive(vector<vector<double>> predictions, TrajectoryObject to, int horizon = 5);

  void configure(vector<int> road_data);

  void realize_state(vector<vector<double>> predictions);

  void realize_lane_change(string direction);

  void realize_prep_lane_change(string direction);

  StepObject acc_for_d(vector<vector<double>> predictions);

  double get_lowest_time_front(vector<vector<double>> predictions);

  void update_current_a(double time);
};

vector<vector<double>>  filter_predictions_by_s(
                        vector<vector<double>> predictions,
                        double s,
                        double range,
                        double t){

    vector<vector<double>> filtered;
    for (int i = 0; i < predictions.size(); i++) {
        vector<double> ocar = predictions[i];
        double ocar_s = ocar[1];
        double ocar_v = ocar[3];
        ocar_s += ocar_v * t;
        if (fabs(ocar[1] - s) <= range) {
            filtered.push_back(ocar);
        }
    }
    return filtered;
}

vector<vector<double>>  filter_predictions_by_d_range(
                        vector<vector<double>> predictions,
                        double d1, double d2){
    double d_small, d_big;
    if (d1 < d2) {
        d_small = d1;
        d_big = d2;
    }
    else {
        d_small = d2;
        d_big = d1;
    }
    vector<vector<double>> filtered;
    for (int i = 0; i < predictions.size(); i++) {
        vector<double> ocar = predictions[i];
        double ocar_d = ocar[2];
        if((ocar_d >= d_small - CAR_WID) && (ocar_d <= d_big + CAR_WID))
        {
            filtered.push_back(ocar);
        }
    }
    return filtered;
}

vector<vector<double>>  filter_predictions_by_d(
                        vector<vector<double>> predictions,
                        double d){
    return filter_predictions_by_d_range(predictions, d - CAR_WID, d + CAR_WID);
}

double time_to_collision(double ocar_s, double car_s, double ocar_v, double car_v) {
    double s_dif = ocar_s - car_s;
    double v_dif = ocar_v - car_v;
    double time_to = -1.0;
    if (v_dif != 0.0) {
        time_to = -s_dif / v_dif;
    }
    return time_to;
}


double cancel_previous_cost(int cancel_count){
    return cancel_count * CANCEL_COST;
}


double prepare_without_change_cost(bool plcl_lcr, bool plcr_lcr){
    int pwc = 0;
    if (plcl_lcr) {
        pwc++;
    }
    if (plcr_lcr) {
        pwc++;
    }
    return (double)pwc * PREPARE_WO_CHANGE;
}

double inefficiency_cost(double speed_sum, int step_count, double target_speed){
    double avg_speed = speed_sum / (double)step_count;
    double diff = fabs(target_speed - avg_speed);
    double pct = diff / target_speed;
    double cost = 0.0;

    if (abs(diff) > 0.01) {
        cost += 20.0;
    }

    return cost + (pct * EFFICIENCY);
}

double near_cost(vector<double> closest_approach_list) {
    double cost = 0.0;
    for (int i = 0; i < closest_approach_list.size(); i++) {
        double closest_approach = closest_approach_list[i];
        if (closest_approach < CAR_LEN) {
            double multiplier = 1.0 - pow(closest_approach / CAR_LEN,2);
            cost += multiplier * NEAR;
        }
    }
    return cost;
}

double occupied_cost(vector<double> lowest_time_front_list) {
    // lowest_time_front should be calculated with target velocity, not current vehicle velocity
    double cost = 0.0;
    for (int i = 0; i < lowest_time_front_list.size(); i++) {
      double lowest_time_front = lowest_time_front_list[i];
      if (lowest_time_front <= 0.0) {
          cost += 1.0 * DANGER;
      }
      else if (lowest_time_front < TIME_DISTANCE) {
          double multiplier = 1.0 / lowest_time_front;
          cost += multiplier * OCCUPIED_LANE;
      }
    }
    return cost;
}

double buffer_cost(vector<double> lowest_time_list){
    double cost = 0.0;
    for (int i = 0; i < lowest_time_list.size(); i++) {
        double lowest_time = lowest_time_list[i];
        if (lowest_time < DESIRED_BUFFER) {
          double multiplier = 1.0 - pow(lowest_time / DESIRED_BUFFER,2);
          cost += multiplier * DANGER;
      }
    }
    return cost;
}

// Add cost for each lane change
double change_lane_cost(int total_lane_changes){
    double cost = COMFORT * total_lane_changes;
    return cost;
}

double at_lane_cost(vector<Vehicle> trajectory){
    double cost = 0.0;
    double center = double(LANE_SIZE - 1) / 2.0;
    for (int i = 1; i < trajectory.size(); i++) {
        Vehicle vehicle = trajectory[i];
        double dif_center = abs((double)vehicle.current_lane - center);
        cost += dif_center * CENTER;
    }
    cost /= PLANNING_HORIZON;
    return cost;
}

double calculate_cost(Vehicle& vehicle,
                      Vehicle::TrajectoryObject to){
    double cost = 0.0;
    cost += inefficiency_cost(to.v_sum, to.step_count, vehicle.target_speed);
    cost += near_cost(to.closest_approach_list);
    cost += occupied_cost(to.lowest_time_front_list);
    cost += buffer_cost(to.lowest_time_list);
    cost += change_lane_cost(to.total_lane_changes);
    cost += at_lane_cost(to.trajectory);
    cost += cancel_previous_cost(to.cancel_count);
    cost += prepare_without_change_cost(vehicle.plcl_lcl, vehicle.plcr_lcr);
    return cost;
}

int get_lane(double d) {
  for (int l = 0; l < LANE_SIZE; l++) {
    if ((d >= l * LANE_WID) && (d < (l + 1) * LANE_WID)) {
      return l;
    }
  }
  return -1;
}

double get_d(int lane) {
    if (lane < 0 || lane >= LANE_SIZE) {
        return 0.0;
    }
    return LANE_WID * (0.5 + (double)lane);
}

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, double s, double v, double a, double target_speed) {

    this->target_lane = lane;
    this->current_lane = lane;
    this->prep_lane = lane;
    this->target_d = get_d(lane);
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = "CS";
    this->step = 0;
    this->target_speed = target_speed;
    this->time = 0.0;
    this->lane_changing = false;
    this->lane_change_finish = false;
    this->plcr_lcr = false;
    this->plcl_lcl = false;
}

Vehicle::~Vehicle() {}

vector<string> Vehicle::get_available_states(){
    vector<string> available_states;
    // 3.1. previous lane change still continues
    if (this->lane_changing) {
        // 3.1.1. accept new state (lane)
        available_states.push_back("KL");
        // 3.1.2. turn back to previous state (lane) if not already passed to the target_lane
        if (fabs(this->d - this->target_d) > 2.5) {
            available_states.push_back("CC");
        }
    }
    // 3.2. no lane change is taking place now
    else {
        // prevent lane change from 2 (current_lane) to 0 (target_lane)
        bool not_in_leftmost = this->target_lane > 0;
        bool not_in_rightmost = this->target_lane < LANE_SIZE - 1;
        bool left_available = (!(this->target_lane < this->current_lane));
        bool right_available = (!(this->target_lane > this->current_lane));
        available_states.push_back("KL");
        if (this->state.compare("KL") == 0 || this->state.compare("LCL") == 0 || this->state.compare("LCR") == 0) {
            if ((not_in_leftmost) &&
                left_available) {
                available_states.push_back("PLCL");
                available_states.push_back("LCL");
            }
            if ((not_in_rightmost) &&
                right_available) {
                available_states.push_back("PLCR");
                available_states.push_back("LCR");
            }
        }
        else if(this->state.compare("PLCL") == 0 && not_in_leftmost){
            if (left_available) {
                available_states.push_back("LCL");
            }
            available_states.push_back("PLCL");
        }
        else if(this->state.compare("PLCR") == 0 && not_in_rightmost){
            if (right_available) {
                available_states.push_back("LCR");
            }
            available_states.push_back("PLCR");
        }

    }
    return available_states;
}

void Vehicle::update_state(vector<vector<double>> predictions) {
    this->plcl_lcl = false;
    this->plcr_lcr = false;
    Vehicle::TrajectoryObject to;

    to = this->get_next_state_recursive(predictions, to, PLANNING_HORIZON);

    std::cout<<this->step<<" state_list:";
    for(int i = 0; i < to.state_list.size(); i++) {
        std::cout<<" "<<to.state_list[i]<<" ";
    }
    std::cout<<"; cost:"<<to.cost<<"!"<<std::endl;

    this->state = to.state;
    this->a_list = to.a_list;
    this->a = this->a_list[0];
}

// collect TrajectoryData for inner steps (horizon = 1) and outer steps (horizon = 5)
Vehicle::TrajectoryObject Vehicle::get_next_state_recursive(vector<vector<double>> predictions, Vehicle::TrajectoryObject trajectory_object, int horizon) {
    TrajectoryObject to;
    double d_const = 0.1;

    double vi = this->v;
    double si = this->s;
    double di = this->d;
    double df = this->target_d;

    // 0. find d and target_d d_dif
    double dd = df - di;
    double dd_abs = fabs(dd);


    // End of lane changeing
    if (this->lane_changing) {
        if (this->lane_change_finish) {
            this->lane_changing = false;
            this->lane_change_finish = false;
        }
        else if (dd_abs < d_const) {
            this->lane_change_finish = true;
        }
    }
    // 2. Filter near vehicles s0 +- PLANNING_DISTANCE
    vector<vector<double>> filtered_s = filter_predictions_by_s(predictions, si, (double)PLANNING_DISTANCE, this->time);

    // 3. get available states.
    Vehicle snapshot = this->copy_vehicle();
    vector<string> available_states = this->get_available_states();
    vector<Vehicle::TrajectoryObject> to_list;

    vector<double> costs;
    for (auto st: available_states) {
        to = trajectory_object;
        this->state = st;
        to.state = st;
        to.trajectory.push_back(*this);
        to.state_list.push_back(st);

        this->realize_state(predictions);
        if(state.compare("CC") == 0) {
            to.cancel_count++;
        }
        if (this->target_lane != this->current_lane) {
            to.total_lane_changes++;
        }

        vector<vector<double>> filtered_s_d_range = filter_predictions_by_d_range(filtered_s, this->d, this->target_d);

        double d_int = 0.0;
        double ds = (double)PLANNING_DISTANCE;

        df = this->target_d;
        dd = df - di;
        // change d if target_d not reached
        d_int = dd / (double)PLANNING_DISTANCE;

        double sf = si + ds;
        double lowest_time = DESIRED_BUFFER;
        double closest_approach = CAR_LEN;
        while (this->s < sf ) {
            // during iteration, construct the helper_data, calculated nearest, lowest_time, and lowest_time_front and others
            // filters should be used, not all ocars should be considered, filter by s, filter by ...., filter by d,
            // it is assumed that ocars wont change d!!! this assumption may be used. with notification.
            StepObject so = this->acc_for_d(filtered_s_d_range);
            this->a = so.a;
            if (so.lowest_time < lowest_time) {
                lowest_time = so.lowest_time;
            }
            if (so.closest_approach < closest_approach) {
                closest_approach = so.closest_approach;
            }
            this->time += tpint;
            this->s += tpint * this->v;
            if (d_int != 0.0) {
                double d_step = tpint * this->v * d_int;

                if (fabs(this->d - df) <= fabs(d_step)) {
                    this->d = df;
                }
                else {
                    this->d += d_step;
                }
            }
            this->current_lane = get_lane(this->d);
            to.v_sum += this->v;
            to.step_count++;
            to.t_sum += tpint;
            to.a_list.push_back(this->a);
            this->v += tpint * this->a;
        }
        to.closest_approach_list.push_back(closest_approach);
        to.lowest_time_list.push_back(lowest_time);
        // occupied, lowest_time_front
        to.lowest_time_front_list.push_back(get_lowest_time_front(predictions));

        double cost = MAXCOST;
        if (closest_approach < (CAR_LEN / 2.0)) {
          to.trajectory.push_back(this->copy_vehicle());

          to.cost = MAXCOST;
          to_list.push_back(to);
        }
        else if (horizon > 1) {
            Vehicle::TrajectoryObject child_to = this->get_next_state_recursive(predictions, to, horizon - 1);
            cost = child_to.cost;
            to_list.push_back(child_to);
        }
        else {
            to.trajectory.push_back(this->copy_vehicle());
            cost = calculate_cost(*this, to);
            to.cost = cost;
            to_list.push_back(to);
        }
        costs.push_back(cost);
        this->restore_vehicle(snapshot);
        if (cost < 5.0) {
            break;
        }
    }
    double min_cost = MAXCOST;
    for (int i = 0; i < costs.size(); i++) {
        if (costs[i] < min_cost){
            min_cost = costs[i];
            to = to_list[i];
            to.state = available_states[i];
        }
    }
    return to;
}

void Vehicle::restore_vehicle(Vehicle snapshot){
    this->target_lane = snapshot.target_lane;
    this->s = snapshot.s;
    this->v = snapshot.v;
    this->a = snapshot.a;
    this->state = snapshot.state;
    this->target_speed = snapshot.target_speed;
    this->d = snapshot.d;
    this->target_d = snapshot.target_d;
    this->current_lane = snapshot.current_lane;
    this->prep_lane = snapshot.prep_lane;
    this->time = snapshot.time;
    this->lane_changing = snapshot.lane_changing;
    this->lane_change_finish = snapshot.lane_change_finish;
    this->plcl_lcl = snapshot.plcl_lcl;
    this->plcr_lcr = snapshot.plcr_lcr;
}

Vehicle Vehicle::copy_vehicle(){
    Vehicle snapshot = Vehicle(this->target_lane,this->s,this->v,this->a,this->target_speed);
    snapshot.d = this->d;
    snapshot.target_d = this->target_d;
    snapshot.state = this->state;
    snapshot.current_lane = this->current_lane;
    snapshot.prep_lane = this->prep_lane;
    snapshot.time = this->time;
    snapshot.lane_changing = this->lane_changing;
    snapshot.lane_change_finish = this->lane_change_finish;
    snapshot.plcl_lcl = this->plcl_lcl;
    snapshot.plcr_lcr = this->plcr_lcr;
    return snapshot;
}

void Vehicle::realize_state(vector<vector<double>> predictions) {

    /*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CC") == 0)
    {
        if (this->target_d < this->d)
        {
            realize_lane_change("R");
        }
        else if (this->target_d > this->d)
        {
            realize_lane_change("L");
        }
    }
    else if(state.compare("LCL") == 0)
    {
        this->plcl_lcl = false;
        realize_lane_change("L");
    }
    else if(state.compare("LCR") == 0)
    {
        this->plcr_lcr = false;
        realize_lane_change("R");
    }
    else if(state.compare("PLCL") == 0)
    {
        this->plcl_lcl = true;
        realize_prep_lane_change("L");
    }
    else if(state.compare("PLCR") == 0)
    {
        this->plcr_lcr = true;
        realize_prep_lane_change("R");
    }

}

// combined to return acc, buffer_cost (lowest,time), near_cost (closest_approach)
StepObject Vehicle::acc_for_d(vector<vector<double>> predictions) {
    StepObject so;
    double delta_v_til_target = this->target_speed - this->v;
    double max_acc = min(amax, delta_v_til_target);

    double car_s = this->s;
    double car_d = this->d;
    double car_td = get_d(this->prep_lane);
    double car_v = this->v;
    double t_step = this->time;

    double min_s = 0.0;
    double min_s_dif = 100.0;
    double min_v = 0.0;
    int min_id = -1;

    double lowest_time = DESIRED_BUFFER;
    double closest_approach = CAR_LEN;

    // Filter by s and d-range
    double d_small, d_big;
    if (car_d < car_td) {
        d_small = car_d - CAR_WID;
        d_big = car_td + CAR_WID;
    }
    else {
        d_small = car_td - CAR_WID;
        d_big = car_d + CAR_WID;
    }
    for (int i = 0; i < predictions.size(); i++) {
        vector<double> ocar = predictions[i];
        double ocar_s = ocar[1];
        double ocar_d = ocar[2];
        double ocar_v = ocar[3];
        ocar_s += ocar_v * t_step;
        if((ocar_d >= d_small) && (ocar_d <= d_big))
        {
            double s_dif = ocar_s - car_s;
            if (s_dif >= 0.0 && s_dif < min_s_dif) {
                //in_front.push_back(ocar);
                min_s_dif = s_dif;
                min_s = ocar_s;
                min_v = ocar_v;
                min_id = (int)ocar[0];
            }
            if (fabs(ocar_d - car_d) <= CAR_WID) {
                double dist_abs = fabs(ocar_s - car_s);
                double time_to = DESIRED_BUFFER;
                if (dist_abs <= CAR_LEN) {
                    time_to = 0.0;
                }
                else if (ocar_s > car_s) {
                    time_to = time_to_collision(ocar_s, car_s + CAR_LEN, ocar_v, car_v);
                }
                else {
                    time_to = time_to_collision(ocar_s + CAR_LEN, car_s, ocar_v, car_v);
                }

                // buffer lowest_time
                if (time_to >= 0.0) {
                    if (time_to < lowest_time){
                        lowest_time = time_to;
                    }
                }

                //near closest_approach
                if (dist_abs < closest_approach) {
                    closest_approach = dist_abs;
                }
            }
        }
    }

    // If there is a leading vehicle
    if (min_s_dif < 100.0) {
      double next_pos = min_s + (min_v * tpint);
      double my_next = car_s + (car_v * tpint);
      double separation_next = next_pos - my_next;
      double available_room = separation_next - this->preferred_buffer;
      max_acc = min(max_acc, available_room);
      max_acc = max(max_acc, - amax);


    }

    so.a = max_acc;
    so.lowest_time = lowest_time;
    so.closest_approach = closest_approach;

    this->a = max_acc;

    return so;
}


double Vehicle::get_lowest_time_front(vector<vector<double>> predictions) {
    double lowest_time_front = TIME_DISTANCE;

    double car_s = this->s;
    double car_d = this->d;
    double car_td = this->target_d;
    double car_v = this->v;
    double t_step = this->time;

    for (int i = 0; i < predictions.size(); i++) {
        vector<double> ocar = predictions[i];
        double ocar_s = ocar[1];
        double ocar_d = ocar[2];
        double ocar_v = ocar[3];
        double time_to = TIME_DISTANCE;
        ocar_s += ocar_v * t_step;
        if(fabs(ocar_d - car_td) <= CAR_WID && ocar_s >= car_s)
        {
            time_to = time_to_collision(ocar_s, car_s, ocar_v, max_v);

            if (time_to > 0.0 && time_to < lowest_time_front) {
                lowest_time_front = time_to;
            }
        }
    }
    return lowest_time_front;
}

void Vehicle::update_current_a(double time) {
    double t_step = (int)(ceil(time / tpint));
    this->a = this->a_list[t_step];
}

void Vehicle::realize_lane_change(string direction) {
    int delta = -1;
    if (direction.compare("R") == 0)
    {
        delta = 1;
    }
    this->target_lane = this->target_lane + delta;
    this->prep_lane = this->target_lane;
    this->target_d = get_d(this->target_lane);
    this->lane_changing = true;
}

void Vehicle::realize_prep_lane_change(string direction) {
    int delta = -1;
    if (direction.compare("R") == 0)
    {
        delta = 1;
    }
    this->prep_lane = this->target_lane + delta;
}
// VEHICLE PART END





// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  //string map_file_ = "../data/highway_map_bosch1.csv";

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  double pre_s = -1.0;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    if ( s > pre_s) {
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
        pre_s = s;
    }
    else {
        break;
    }
  }

  int lane = 1;
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
  double target_speed = max_v;

  Vehicle vehicle = Vehicle(lane,s,v,a,target_speed);

  h.onMessage([&vehicle, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"];
            double car_speed = j[1]["speed"];

            // Previous path data given to the Planner
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];
            int pre_size = previous_path_x.size();

            car_speed *= mphtoms;

            if (pre_size > 0) {
              vehicle.s = end_path_s;
              vehicle.d = end_path_d;
              if (vehicle.d < 1.0) {
                vehicle.d = 1.0;
              }
              else if (vehicle.d > (LANE_WID * LANE_SIZE) - 1.0) {
                vehicle.d = (LANE_WID * LANE_SIZE) - 1.0;
              }

              vector<vector<double>> filtered_sensor_fusion;
              for (int i = 0; i < sensor_fusion.size(); i++) {
                vector<double> ocar = sensor_fusion[i];
                int ocar_id = (int)ocar[0];
                double ocar_s = ocar[5];
                double ocar_d = ocar[6];
                double ocar_v = sqrt(pow(ocar[3],2) + pow(ocar[4],2));
                double ocar_sn = ocar_s + ocar_v * pre_size * tint;
                int ocar_l = get_lane(ocar_d);

                if ((ocar_l < 0 || ocar_l >= LANE_SIZE) ||
                   ((ocar_s - car_s) <= -70.0)) {
                }
                else {
                    ocar[5] = ocar_sn;
                    filtered_sensor_fusion.push_back({(double)ocar_id,ocar_sn, ocar_d, ocar_v});
                }
              }
              sensor_fusion = filtered_sensor_fusion;
            }
            else {
              vehicle.s = car_s;
              vehicle.d = car_d;
            }
            vehicle.step++;
            vehicle.update_state(sensor_fusion);
            vehicle.realize_state(sensor_fusion);
            vehicle.current_lane = get_lane(vehicle.d);

            json msgJson;

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
            // Later we will interoplate these waypoints with a spline and fill it in with more points that control sp ...
            vector<double> ptsx;
            vector<double> ptsy;

            // reference x,y, yaw states
            // either we will reference the starting point as where the car is or at the previous path end point
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            double car_yaw_rad = deg2rad(car_yaw);

            // If previous path is almost empty, use the car as starting reference
            if (pre_size < 2) {
              // use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);


            }
            // use the previous path's end point as starting reference
            else {

              // redefine reference state as previous path end point
              ref_x = previous_path_x[pre_size - 1];
              ref_y = previous_path_y[pre_size - 1];

              double ref_x_prev = previous_path_x[pre_size - 2];
              double ref_y_prev = previous_path_y[pre_size - 2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // use two points that make the path tangent to the previous path's end point
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);

            }

            // In Frenet add evenly 30m spaced points ahead of the starting reference.
            for (int i = 0; i < 3; i++){
              double target_s = vehicle.s + PLANNING_DISTANCE * (i + 1);
              if (target_s > max_s) {
                target_s -= max_s;
              }
              double target_d = vehicle.target_d;
              if (i == 0) {
                if (target_d - vehicle.d < - LANE_WID) {
                    target_d = vehicle.d - LANE_WID;
                }
                else if (target_d - vehicle.d > LANE_WID) {
                    target_d = vehicle.d + LANE_WID;
                }
              }

              vector<double> next_wp = getXY(target_s, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(next_wp[0]);
              ptsy.push_back(next_wp[1]);
            }

            // Convert to the cars coordinates like MPC project.
            for (int i = 0; i < ptsx.size(); i++) {

              //shift car reference angle to 0 degrees
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              double a = cos(0 - ref_yaw);
              double b = sin(0 - ref_yaw);

              ptsx[i] = (shift_x * a - shift_y * b);
              ptsy[i] = (shift_x * b + shift_y * a);
            }

            // create a spline
            tk::spline s;

            // set(x, y) points to the spline
            s.set_points(ptsx, ptsy);

            // Start with all of the previous path points from last time
            for (int i = 0; i < pre_size; i++){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate how to break up spline points so that we travel at our desired reference velocity
            double target_x = PLANNING_DISTANCE;
            double target_y = s(target_x);
            double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));

            double x_add_on = 0;

            for (int i = 1; i <= PLANNING_STEPS - pre_size; i++) {
              vehicle.update_current_a(tint * i);
              if (vehicle.a > amax) {
                vehicle.a = amax;
              }
              else if (vehicle.a < - amax) {
                vehicle.a = -amax;
              }

              if (vehicle.target_speed - vehicle.v < 0.0) {
                vehicle.v += (vehicle.a * tint);
              }
              else if (vehicle.target_speed - vehicle.v > 0.0) {
                vehicle.v += (vehicle.a * tint);
              }
              if (vehicle.v > max_v) {
                vehicle.v = max_v;
              }

              double N = (target_dist/(tint * vehicle.v));
              double x_point = x_add_on + (target_x / N);
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              //rotate back to normal after rotating it earlier
              double a = cos(ref_yaw);
              double b = sin(ref_yaw);

              x_point = (x_ref * a - y_ref * b);
              y_point = (x_ref * b + y_ref * a);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"control\","+ msgJson.dump()+"]";

            //this_thread::sleep_for(chrono::milliseconds(1000));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

