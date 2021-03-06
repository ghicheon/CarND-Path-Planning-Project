#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <map>
#include "json.hpp"

#include "./spline.h"

#include <cmath>

/*
 * PRE_LEFT,PRE_RIGHT states have time limit.
 * If time is exceeded,It must be better to consider an another way.
 */
#define PRE_COUNTER_INIT    (50*5)    //5 seconds  since the car moves 50 times a second.

/*
 * In pre states, the car is slowing down. but It has time limit, if not, the car will stop.
 */
#define PRE_COUNTER_DEC_SPEED_COUNT     50   //1 second 

/*
 * Well, spline is used for changing lane. It took some time to change the lane in the state LEFT/RIGHT.
 * After it, the state will be KEEP.
 */
#define DOING_ACTION_COUNT_INIT  (50*4)       //4 seconds

/*
 * When changing the lane, there must be no car when my car is on the way to the next lane.
 */
#define FRONT_MARGIN 35
#define BACK_MARGIN 10


/*
 * In PRE_LEFT/PRE_RIGHT, 
 * If the speed keeps decreasing. the car will stop.this variable pre_counter is trying to avoid it!
 * when the limit reaches, the state will be KEEP.
 */
int pre_counter = 0;

/*  
 * In the state LEFT/RIGHT, this variable is set to DOING_ACTION_COUNT_INIT.
 * This value is decreasing as time goes on. When it reaches 0, the state will be KEEP.
 */
int doing_action_count=0;

enum state_type { KEEP, 
                  LEFT,
                  RIGHT,
                  PRE_LEFT,
                  PRE_RIGHT,
                  STATE_LEN};


// (char*) : to avoid compiler warning messages.
char * i_to_s[STATE_LEN] =  { (char*)"KEEP", (char*)"LEFT", (char*)"RIGHT", 
                              (char*)"PRE_LEFT" ,(char*)"PRE_RIGHT" };

//get the string using index.
char * itos(int i) 
{
    return i_to_s[i];
}

int state = KEEP;

//it's just for finding out if the state is changed(for debugging).
int prev_state = PRE_RIGHT; //PRE_RIGHT:for printing at first.

using namespace std;

//#define MAX_SPEED 200         //just for test~~
#define MAX_SPEED 49.7       //hard requirement

float max_speed= MAX_SPEED;  //soft requirement

int lane=1;
float speed=0; //state speed(reference velocity).

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
    return M_PI;
}
double deg2rad(double x) {
    return x * pi() / 180;
}
double rad2deg(double x) {
    return x * 180 / pi();
}

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
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);

    if(angle > pi()/4)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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



/*
 * For changing the state, this must be used. it prints debug messages.
 * Don't do like this:   state = KEEP
 */
#define CHANGE_STATE(s)                             \
        {                                                                    \
                if( state != s ){                                            \
                  cout << endl;                                              \
                  cout << "state :" << itos(state) << " ---> " ;             \
                  state = s;                                                 \
                  cout << itos(state) << "        " << __LINE__ << endl;     \
                }                                                            \
        }

/*
 * next_state: will be filled with a next state.
 * consider_left: consider left lane 
 * consider_right: consider right lane 
 *
 * return; next state
 *
 * - this function make a choice by using left lane speed and right lane speed.
 *   When there is 2 options, It must better to choose a speedy lane!.
 * - lane speed is measured by the closest car.
 *
 * 
 * FYI, It must be better to write a normal function not a macro one.
 * But I couldn't make it because auto arguments are not allowed in default compiler setting.
 * It can be done by -std=c++14 .
 * (https://stackoverflow.com/questions/29944985/is-there-a-way-to-pass-auto-as-an-argument-in-c)
 * But I am a lazy guy & I don't want to bother reviwers.I decided to write it as a macro function.
 */
#define CONSIDER_ALTERNATIVES( next_state , consider_left,consider_right)                   \
        do                                                                                  \
        {                                                                                   \
            int left_ok = (lane != 0) ? 1 : 0;                                              \
            int right_ok= (lane != 2) ? 1 : 0;                                              \
            double left_speed=0;                                                            \
            double right_speed=0;                                                           \
                                                                                            \
            double dist;                                                                    \
                                                                                            \
            double closest_right_dist=999999;                                               \
            double closest_left_dist=999999;                                                \
            double closest_right_speed=0;                                                   \
            double closest_left_speed=0;                                                    \
            double left_margin_max  = 0;                                                    \
            double right_margin_max = 0;                                                    \
                                                                                            \
            for(int i=0; i< sensor_fusion.size();i++)                                       \
            {                                                                               \
                double  x = sensor_fusion[i][1];                                            \
                double  y = sensor_fusion[i][2];                                            \
                double vx = sensor_fusion[i][3];                                            \
                double vy = sensor_fusion[i][4];                                            \
                double  s = sensor_fusion[i][5];   /*check_car_s*/                          \
                double  d = sensor_fusion[i][6];                                            \
                                                                                            \
                if( consider_right == 1 )                                                   \
                {                                                                           \
                                                                                            \
                        if( (lane == 0) || (lane == 1) )                                    \
                        {                                                                   \
                                right_speed = sqrt(vx*vx+vy*vy);                            \
                                s +=((double)prev_size*0.02*right_speed) ;                  \
                                if( (d < (2+4*(lane+1)+2)) && (d > (2+4*(lane+1)-2)) )      \
                                {                                                           \
                                    if(( s > car_s) && (( s - car_s) < 30) )                \
                                    {                                                       \
                                            right_ok = 0;                                   \
                                    }                                                       \
                                    if((s < (car_s+FRONT_MARGIN)) && (s > (car_s-BACK_MARGIN)) ) \
                                    {                                                       \
                                            right_ok = 0;                                   \
                                    }                                                       \
                                                                                            \
                                }                                                           \
                                                                                            \
                                dist = distance(car_x,car_y, x,y);                          \
                                if( closest_right_dist   > dist )                           \
                                {                                                           \
                                    closest_right_dist  = dist;                             \
                                    closest_right_speed = right_speed;                      \
                                }                                                           \
                        }                                                                   \
                }                                                                           \
                                                                                            \
                if( consider_left == 1 )                                                    \
                {                                                                           \
                        if( (lane == 2) || (lane == 1) )                                    \
                        {                                                                   \
                                left_speed = sqrt(vx*vx+vy*vy);                             \
                                s +=((double)prev_size*0.02*left_speed) ;                   \
                                if( (d < (2+4*(lane-1)+2)) && (d > (2+4*(lane-1)-2)) )      \
                                {                                                           \
                                    if(( s > car_s) && (( s - car_s) < 30) )                \
                                    {                                                       \
                                            left_ok = 0;                                    \
                                    }                                                       \
                                    if( (s < (car_s+FRONT_MARGIN)) && (s > (car_s-BACK_MARGIN)))\
                                    {                                                       \
                                            left_ok = 0;                                    \
                                    }                                                       \
                                }                                                           \
                                dist = distance(car_x,car_y, x,y);                          \
                                if( closest_left_dist   > dist )                            \
                                {                                                           \
                                    closest_left_dist  = dist;                              \
                                    closest_left_speed = left_speed;                        \
                                }                                                           \
                        }                                                                   \
                 }                                                                          \
            }                                                                               \
            right_speed = closest_right_speed;      /*result!!!*/                           \
            left_speed = closest_left_speed;        /*result!!!*/                           \
            /*car_speed =1;  change lane as many as possible.. */                           \
            assert( lane >= 0 && lane <= 2 );                                               \
            cout << "[lok,rok,lspd,rspd,carspd:" <<  left_ok << "," << right_ok << "," <<   \
                      left_speed << "," << right_speed << "," << car_speed  << "]" ;        \
            /* find the best choice! */                                                     \
            if( (left_ok == 1) && (right_ok == 0) )                                         \
            {                                                                               \
                next_state =  LEFT;                                                         \
                /*next_state =  (car_speed < left_speed) ? LEFT : KEEP; */                  \
            }                                                                               \
            else if( (left_ok == 0) && (right_ok == 1) )                                    \
            {                                                                               \
                next_state =  RIGHT ;                                                       \
                /*next_state =  (car_speed < right_speed) ? RIGHT : KEEP;*/                 \
            }                                                                               \
            else if( (left_ok == 1) && (right_ok == 1) )    /*best one will be selected*/   \
            {                                                                               \
                if( left_speed > right_speed &&  left_speed > car_speed )                   \
                {                                                                           \
                    next_state =   LEFT;                                                    \
                }                                                                           \
                else  if( left_speed < right_speed &&  right_speed > car_speed )            \
                {                                                                           \
                    next_state =  RIGHT;                                                    \
                }                                                                           \
                else                                                                        \
                {                                                                           \
                    /* the lane that has longer margin,it will be better! */                \
                    next_state = (closest_right_dist > closest_left_dist) ? RIGHT:LEFT;     \
                }                                                                           \
            }                                                                               \
            else if( (left_ok == 0) && (right_ok == 0) )                                    \
            {                                                                               \
                if( left_speed > right_speed )                                              \
                {                                                                           \
                    next_state =  (car_speed < left_speed) ? PRE_LEFT : KEEP;               \
                }                                                                           \
                else                                                                        \
                {                                                                           \
                    next_state =  (car_speed < right_speed) ? PRE_RIGHT : KEEP;             \
                }                                                                           \
            }                                                                               \
                                                                                            \
            CHANGE_STATE(next_state);                                                       \
        } while(0)                                                                                                     




                                                                                                                       
/* 
 * speed must not be exceeding speed limit!
 *
 *  max_speed: soft requirement. when a blocking car is front and there is no options,
 *             it will be better to follow the front car!!!
 *  MAX_SPEED: hard requirement!
 */
double get_proper_speed(double spd, double speed_change) 
{

    if( spd   > max_speed) 
        spd -= 0.224 ; //slow down slowly to avoid jerk.
    else if((spd + speed_change ) < max_speed  ) 
        spd += speed_change;
    else if( spd > MAX_SPEED) //hard requirement
    {
        spd -= 0.224 ; //slow down slowly to avoid jerk.
    }
    
    return spd;
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
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
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
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
    uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
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

                    int prev_size = previous_path_x.size();
                    double speed_change = 0;

                    json msgJson;

                    vector<double> ptsx;
                    vector<double> ptsy;

                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);
                    if(prev_size < 2)
                    {
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);
                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                    }
                    else
                    {
                        ref_x = previous_path_x[prev_size-1];
                        ref_y = previous_path_y[prev_size-1];

                        double ref_x_prev = previous_path_x[prev_size-2];
                        double ref_y_prev = previous_path_y[prev_size-2];

                        ref_yaw = atan2( ref_y-ref_y_prev,
                                         ref_x-ref_x_prev);

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }

                    //TODO:define a path made up of (x,y) points 
                    //that the car will visit sequentially every .02 seconds

                    if(prev_size > 0 )
                    {
                        car_s = end_path_s;
                    }

////////////////////CORE START/////////////////////////////////////////////////////////////////

                    //PRE_LEFT/PRE_RIGHT -> KEEP
                    pre_counter--;
                    if( (state == PRE_LEFT || state == PRE_RIGHT) && (pre_counter == 0) )
                    {
                        CHANGE_STATE(KEEP);                             \
                    }
                   
                    //LEFT/RIGHT -> KEEP
                    doing_action_count--;
                    if( (state == LEFT || state == RIGHT) && (doing_action_count == 0))
                    {
                        CHANGE_STATE(KEEP);                             \
                    }

                    //debug message!
                    if( prev_state  != state )
                    {
                        cout << "[state changed : " << itos(state) <<   "]" ;
                    }
                    prev_state  = state;


                    int next_state=-1;

                    for(int i=0; i< sensor_fusion.size();i++)
                    {
                        double d = sensor_fusion[i][6];
                        if( (d < (2+4*lane+2)) && (d > (2+4*lane-2)) ) //is it on the same lane?
                        {
                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            double check_speed = sqrt(vx*vx+vy*vy);
                            double check_car_s = sensor_fusion[i][5];

                            check_car_s +=((double)prev_size*0.02*check_speed) ;

                            //blocked!
                            if((check_car_s > car_s) && ((check_car_s-car_s) < 30) )  
                            {
                                    cout << "[B]" ; //block detected!

                                    //max_speed= check_speed; //XXX it might better? not tested yet.

                                    if( state == KEEP ) 
                                    {
                                            //next_state will be set inside this function!
                                            //consider both lanes. 
                                            CONSIDER_ALTERNATIVES(next_state,1,1); 
                                            
                                            switch(next_state)
                                            {
                                                case LEFT:
                                                     cout << "[left success:" << "old:" << lane << "  new:" << lane-1 << "]";
                                                     lane -=1;
                                                     doing_action_count = DOING_ACTION_COUNT_INIT;
                                                     break;

                                                case RIGHT:
                                                     doing_action_count = DOING_ACTION_COUNT_INIT;
                                                     cout << "[right success:" << "old:" << lane << "  new:" << lane+1 << "]";
                                                     lane +=1;
                                                     break;

                                                case PRE_LEFT:
                                                case PRE_RIGHT:
                                                     pre_counter =  PRE_COUNTER_INIT ;
                                                     break;

                                                case KEEP:
                                                     max_speed= check_speed;//try to follow the front car!
                                                     cout << "[max_speed changed:" << max_speed << "]";
                                                     break;
                                                default:
                                                     assert(0);
                                                     break;
                                                
                                            } //switch

                                    } //if KEEP

                                    speed_change += -0.224;
            
                                    break; //goto out_of_for_loop;

                            } //if blocked
                        } //if( (d < (2+4*lane+2)) && (d > (2+4*lane-2)) ) //is it on the same lane?
                    }//for loop

//out_of_for_loop:
                    //if next state is not KEEP, max_speed need to be turning back to original value.
                    if( next_state != KEEP ) 
                        max_speed= MAX_SPEED;

                    if( state == PRE_LEFT )    
                    {
                            int next_state=-1;

                            CONSIDER_ALTERNATIVES( next_state , 1,0); //only left
                            switch(next_state)
                            {
                                case LEFT:
                                     cout << "[left success(pre_left -> left):" <<  
                                                     "old:" << lane << "  new:" << lane-1 << "]";
                                     lane -=1;
                                     doing_action_count = DOING_ACTION_COUNT_INIT;
                                     break;

                                case PRE_LEFT:
                                     if( (PRE_COUNTER_INIT - PRE_COUNTER_DEC_SPEED_COUNT) < pre_counter )
                                     {
                                        speed_change += -0.001 ; //slow down.
                                     } 
                                     break;

                                case KEEP:
                                     break;

                                default:
                                     assert(0);
                                     break;

                            }
                    }
                    else if( state == PRE_RIGHT )    
                    {
                            int next_state;

                            CONSIDER_ALTERNATIVES( next_state , 0,1); //only right
                            
                            switch(next_state)
                            {
                                case RIGHT:
                                     cout << "[right success(pre_right -> right):" <<  
                                                     "old:" << lane << "  new:" << lane+1 << "]";
                                     lane +=1;
                                     doing_action_count = DOING_ACTION_COUNT_INIT;
                                     break;

                                case PRE_RIGHT:
                                     if( (PRE_COUNTER_INIT - PRE_COUNTER_DEC_SPEED_COUNT) < pre_counter )
                                     {
                                        speed_change += -0.001 ; //slow down.
                                     } 
                                     break;

                                case KEEP:  //there is no hope....
                                     break;

                                default:
                                     assert(0);
                                     break;
                            }
                    }


                    speed = get_proper_speed(speed, speed_change == 0 ? 0.224 : speed_change);

////////////////////CORE END//////////////////////////////////////////////////////////////////////////

                    vector<double> next_wp0 = 
                            getXY(car_s+30, 4*lane+2 ,map_waypoints_s, map_waypoints_x,map_waypoints_y);
                    vector<double> next_wp1 = 
                            getXY(car_s+60, 4*lane+2 ,map_waypoints_s, map_waypoints_x,map_waypoints_y);
                    vector<double> next_wp2 = 
                            getXY(car_s+90, 4*lane+2  ,map_waypoints_s, map_waypoints_x,map_waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    for(int i=0; i < ptsx.size(); i++)
                    {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;
                        ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                        ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
                    }

                    tk::spline s;
                    s.set_points(ptsx,ptsy);

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    for(int i=0; i < previous_path_x.size(); i++)
                    {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    double target_x=30.0;
                    double target_y = s(target_x);
                    double target_dist = sqrt(target_x*target_x + target_y*target_y);


                    double x_add_on=0;
                    for(int i=1; i<= 50-previous_path_x.size();i++)
                    {
                        double N = (target_dist/(.02*speed/2.24));
                        double x_point = x_add_on+(target_x)/N;
                        double y_point = s(x_point);
                        x_add_on = x_point;
                        double x_ref=x_point;
                        double y_ref=y_point;

                        x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
                        y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

                        x_point += ref_x;
                        y_point += ref_y;
                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }

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



#if 0

double dist_inc = 0.5;
    for(int i = 0; i < 50; i++)
    {
          next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
          next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
    }
#endif


