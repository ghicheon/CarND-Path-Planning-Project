
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"

#include "./spline.h"

#include <cmath>

//#define HERE_DEBUG() printf("%s   %d\n", __func__ , __LINE__ );
#define HERE_DEBUG() 

enum state { KEEP , 
             FIND,  //find good lane from their speed.  cost calculation!
             PRE_RIGHT,
             PRE_LEFT, 
             LEFT, 
             RIGHT };

int current = KEEP;

double    g_prev_s_;
double    g_prev_d_;

//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;



int cnt=0;

//#define MAX_SPEED 200
#define MAX_SPEED 50

int target_lane[]={0,1,2,1};
int counter = 0;

int lane=1;
float speed=0; //current speed(reference velocity).
float speed_multiplier=1; //not used now.

using namespace std;

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



//consider ONLY around s and ONLY at d !!
//return true or false
//it's based on ClosestWaypoint().
int IsSafe(double x, double y,  const vector<double> &maps_x, const vector<double> &maps_y,
           double s, double d)
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

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
class Vehicle {
    public:

    vector<double> state;

    Vehicle( double s, double s_ , double s__ , double d, double d_, double d__ )
    {
        state.push_back(s);
        state.push_back(s_);
        state.push_back(s__);
        state.push_back(d);
        state.push_back(d_);
        state.push_back(d__);
    }
    ~Vehicle() {}

    vector<double>  state_in(double t) 
    {
        //vector<double> s = start_state[:3]
        //vector<double> d = start_state[3:]
        //state[0+]: s
        //state[3+]: d
        state.push_back( state[0] + (state[1] * t) + state[2] * t*t / 2.0 );
        state.push_back( state[1] + state[2] * t );
        state.push_back( state[2] );
        state.push_back( state[3+0] + (state[3+1] * t) + state[3+2] * t*t / 2.0 );
        state.push_back( state[3+1] + (state[3+2] * t) );
        state.push_back( state[3+2] );

        return state;
    }

};


map<unsigned int, Vehicle*> carMap; //manage all cars!

//6 coefficient & time
int getResult(vector<double> xx , unsigned int t)
{

    return xx[0] + xx[1]*t + xx[2]*t*t  + xx[3]*t*t*t + xx[4]*t*t*t*t + xx[5]*t*t*t*t*t ;
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

                    json msgJson;

                    vector<double> ptsx;
                    vector<double> ptsy;

                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);
HERE_DEBUG();
                    if(prev_size < 2)
                    {
HERE_DEBUG();
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);
                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                    }
                    else
                    {
HERE_DEBUG();
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
HERE_DEBUG();

                    for(int i=0; i< sensor_fusion.size();i++)
                    {
                        unsigned int  id = sensor_fusion[i][0];
                        double  x = sensor_fusion[i][1];
                        double  y = sensor_fusion[i][2];
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];

                        double  s = sensor_fusion[i][5];
                        double  d = sensor_fusion[i][6];
                        //double check_speed = sqrt(vx*vx+vy*vy);

                        if( cnt == 0 ) //first sensor data!
                        {
                            delete carMap[id]; //XXX use it again...
                            carMap[id] = new Vehicle(s,s,s,s,s,s); //0.0,0.0,d,0.0,0.0);
                        }
                        else
                        {
                            double prev_s = carMap[id]->state[0];
                            double prev_s_ = carMap[id]->state[1];
                            double prev_d = carMap[id]->state[4];
                            double prev_d_ = carMap[id]->state[5];
                            double s_  = abs(prev_s - s) /0.02;
                            double s__ = abs(prev_s_ - s_ )/0.02;
                            double d_  = abs(prev_d - d)/0.02;
                            double d__ = abs(prev_d_ - d_)/0.02;
                            carMap[id] = new Vehicle(s ,s_ ,s__ ,d ,d_ ,d__ );
                        }


HERE_DEBUG();
                    }

HERE_DEBUG();

                    //TODO:define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

                    if(prev_size > 0 )
                    {
                        car_s = end_path_s;
                    }

                    unsigned int too_close = 0;

HERE_DEBUG();
                
                    if( current == KEEP )     //too_close  == 0 )
                    {
                            for(int i=0; i< sensor_fusion.size();i++)
                            {
                                float d = sensor_fusion[i][6];
                                if( (d < (2+4*lane+2)) && (d > (2+4*lane-2)) )
                                {
                                    double vx = sensor_fusion[i][3];
                                    double vy = sensor_fusion[i][4];
                                    double check_speed = sqrt(vx*vx+vy*vy);
                                    double check_car_s = sensor_fusion[i][5];

                                    check_car_s +=((double)prev_size*0.02*check_speed) ;

                                    if((check_car_s > car_s) &&
                                       (check_car_s-car_s) < 30) 
                                    {
                                            //consider chainging line for following 10 times.
                                            //too_close = 10; 
                                            current = FIND; 
                                            break;

                                    }
                                }
                            }
                    }
HERE_DEBUG();

                    if(current == FIND )      //too_close)
                    {
                        //first, find close left car and right one.
                        int left_ok = 1;
                        int right_ok= 1;
                        double dist;
                        double closest_right=-1;
                        double closest_right_dist=999999;
                        double closest_left=-1;
                        double closest_left_dist=999999;
                        for(int i=0; i< sensor_fusion.size();i++)
                        {
                            double  x = sensor_fusion[i][1];
                            double  y = sensor_fusion[i][2];
                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            double  s = sensor_fusion[i][5];
                            double  d = sensor_fusion[i][6];


                            if( (lane == 0) || (lane == 1) )  //right
                            {
                                    if( (d < (2+4*(lane+1)+2)) && (d > (2+4*(lane+1)-2)) )
                                    {
                                        if( (s < (car_s+30)) && (s > (car_s-35)) ) // +/- something!
                                        {
                                                right_ok = 0;
                                        }
                                        else
                                        {
                                                dist = distance(car_x,car_y, x,y);
                                                if( closest_right_dist   > dist )
                                                {
                                                    closest_right = i;
                                                    closest_right_dist = dist;
                                                }
                                        }
                                    }

                            }

                            if( (lane == 2) || (lane == 1) ) // left
                            {
                                    if( (d < (2+4*(lane-1)+2)) && (d > (2+4*(lane-1)-2)) )
                                    {
                                        if( (s < (car_s+30)) && (s > (car_s-35)) ) // +/- something!
                                        {
                                                left_ok = 0;
                                        }
                                        else
                                        {
                                                dist = distance(car_x,car_y, x,y);
                                                if( closest_left_dist   > dist )
                                                {
                                                    closest_left = i;
                                                    closest_left_dist = dist;
                                                }
                                        }
                                    }

                            }
                        }
HERE_DEBUG();

                        //second, change lane if it's possible. 
                        //        when there are 2 options, select the best one after calculating costs!

HERE_DEBUG();
                        if( left_ok == 1 && right_ok == 0 )
                        {
                            //XXX check speed.
HERE_DEBUG();
                            lane -=1;
                            assert( lane >= 0 && lane <= 2 );
                        }
                        else if( left_ok == 0 && right_ok == 1 ) 
                        {
HERE_DEBUG();
                            //XXX check speed.
                            lane +=1;
                            assert( lane >= 0 && lane <= 2 );
                        }
                        else
                        {
HERE_DEBUG();
                            //select the best one! currently, speed is cost.
                            double vx ;
                            double vy ;
                            double left_speed=0;
                            double right_speed=0;

                            if(closest_left != -1  )
                            {
                                    vx = sensor_fusion[closest_left][3];
                                    vy = sensor_fusion[closest_left][4];
                                    left_speed = sqrt(vx*vx+vy*vy);
                            }

                            if(closest_right != -1  )
                            {
                                    vx = sensor_fusion[closest_right][3];
                                    vy = sensor_fusion[closest_right][4];
                                    right_speed = sqrt(vx*vx+vy*vy);
                            }
HERE_DEBUG();

                            if( left_speed > right_speed ) 
                            {
HERE_DEBUG();
                                current = LEFT;
                                cout << "change line success!!! old:" << lane << "  new:" << lane-1 << endl;
                                lane -=1;
HERE_DEBUG();
                            }
                            else
                            {
HERE_DEBUG();
                                current = RIGHT;
                                cout << "change line success!!! old:" << lane << "  new:" << lane+1 << endl;
                                lane +=1;
HERE_DEBUG();
                            }
                            current = KEEP;
HERE_DEBUG();
                        }

HERE_DEBUG();
                        speed -= 0.224 * speed_multiplier;   //slowing down little bit.
                        if(speed_multiplier != 1 )
                            speed_multiplier -=1;
HERE_DEBUG();

                    }
                    else if((speed +0.224 * speed_multiplier) < MAX_SPEED)
                    {
                        speed += 0.224 * speed_multiplier;

                        if(speed_multiplier < 3 )
                            speed_multiplier +=1;
HERE_DEBUG();
                        
                    }

HERE_DEBUG();
                    counter++;

                    too_close--;


HERE_DEBUG();


                    vector<double> next_wp0 = 
                            getXY(car_s+30, 4*lane+2 ,map_waypoints_s, map_waypoints_x,map_waypoints_y);
                    vector<double> next_wp1 = 
                            getXY(car_s+60, 4*lane+2 ,map_waypoints_s, map_waypoints_x,map_waypoints_y);
                    vector<double> next_wp2 = 
                            getXY(car_s+90, 4*lane+2  ,map_waypoints_s, map_waypoints_x,map_waypoints_y);
HERE_DEBUG();

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
HERE_DEBUG();

                    tk::spline s;
HERE_DEBUG();
                    s.set_points(ptsx,ptsy);
HERE_DEBUG();

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
HERE_DEBUG();


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

HERE_DEBUG();
                    //std::cout << "cnt-----------------------:" <<  cnt   << std::endl;
                    cnt++;
//////////////////////////////////////////////////////////////////////////////////////////////

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
