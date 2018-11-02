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
#include "json.hpp"

#include "./spline.h"

int cnt=0;

double    g_prev_s_;
double    g_prev_d_;
//random.seed(0)

int N_SAMPLES = 10;
double SIGMA_S[3] = {10.0, 4.0, 2.0}; // s, s_dot_coeffs, s_double_dot
double SIGMA_D[3] = {1.0, 1.0, 1.0};
double SIGMA_T = 2.0;

double MAX_JERK = 10; // m/s/s/s
double MAX_ACCEL= 10; // m/s/s

double EXPECTED_JERK_IN_ONE_SEC = 2;//  m/s/s
double EXPECTED_ACC_IN_ONE_SEC = 1; //  m/s

double SPEED_LIMIT = 30;
double VEHICLE_RADIUS = 1.5; // model vehicle as circle to simplify collision detection

#define MAX_SPEED 200

int target_lane[4]={0,1,2,1};
int counter = 0;

int lane=1;
float speed=0; //current speed(reference velocity).
float speed_multiplier=1;

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
/////////////////////////////////////////////////////////////////////////

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

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"

//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    
    MatrixXd A = MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
                3*T*T, 4*T*T*T,5*T*T*T*T,
                6*T, 12*T*T, 20*T*T*T;
        
    MatrixXd B = MatrixXd(3,1);     
    B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
                end[1]-(start[1]+start[2]*T),
                end[2]-start[2];
                
    MatrixXd Ai = A.inverse();
    
    MatrixXd C = Ai*B;
    
    vector <double> result = {start[0], start[1], .5*start[2]};
    for(int i = 0; i < C.size(); i++)
    {
        result.push_back(C.data()[i]);
    }
    
    return result;
    
}


vector<vector<double>> 
PTG( vector<double> start_s, vector<double> start_d, int target_vehicle,vector<double> delta,
          int T,  map<unsigned int, Vehicle*>  predictions )
{
        vector<vector<double>>  tr ;

        //for(;;)
        vector<double> s_goal;
        s_goal.push_back(start_s[0]+200); //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
        s_goal.push_back(start_s[1]);
        s_goal.push_back(start_s[2]);

        vector<double> d_goal;
        d_goal.push_back( target_lane[(int)start_d[0]] );
        d_goal.push_back(start_d[1]);
        d_goal.push_back(start_d[2]);

        vector<double>  s_coefficients = JMT(start_s, s_goal, T);
        vector<double>  d_coefficients = JMT(start_d, d_goal, T);

        tr.push_back(s_coefficients);
        tr.push_back(d_coefficients);
        return tr;

#if 0
    vector<Vehicle> target = predictions[target_vehicle];

    //generate alternative goals
    vectorall_goals = []
    timestep = 0.5
    t = T - 4 * timestep
    while t <= T + 4 * timestep:
        target_state = np.array(target.state_in(t)) + np.array(delta)
        goal_s = target_state[:3]
        goal_d = target_state[3:]
        goals = [(goal_s, goal_d, t)]
        for _ in range(N_SAMPLES):
            perturbed = perturb_goal(goal_s, goal_d)
            goals.append((perturbed[0], perturbed[1], t))
        all_goals += goals
        t += timestep
    
    minimum = 9999999999
    best = None
    for goal in all_goals:
        s_goal, d_goal, t = goal
        s_coefficients = JMT(start_s, s_goal, t)
        d_coefficients = JMT(start_d, d_goal, t)
        tr = tuple([s_coefficients, d_coefficients, t])
        curr = cost(tr, target_vehicle, delta, T, predictions )
        if minimum > curr:
            minimum = curr
            best  = tr

    return best
#endif

}

map<unsigned int, Vehicle*> carMap; //manage all cars!


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

                    double car_last_s=car_s;//init

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
                    //[id,x,y,vx,vy,s,d]
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    int prev_size = previous_path_x.size();

                    json msgJson;


                    //generate trj
                    //cal cost
                    //find best
                    //do it

                    //adjust carMap dataset from current sensor data.
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


                    }

                    //vector<double> ttt = getXY(end_path_s,end_path_d ,map_waypoints_s, map_waypoints_x,map_waypoints_y);
                    //std::cout << "XXXXXXXXXXXXXXXstart--------------end_path_xy:" << ttt[0] << ttt[1] <<
                    //                                    "car_xy:" << car_x << car_y << std::endl;

                    //for(int i=0;i<previous_path_x.size();i++)
                    //{
                    //     cout << "previous_path_" << i << ": " << previous_path_x[i] << endl;
                    //}

                    //std::cout << "XXXXXXXXXXXXXXXend--------------end_path_xy:" << ttt[0] << ttt[1] <<
                    //                                        "car_xy:" << car_x << car_y << std::endl;




                    //TODO:define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

                    if(prev_size > 0 )
                    {
                        car_last_s = end_path_s; //the last waypoint of received end_path_s
                    }

                    bool too_close = false;

                    for(int i=0; i< sensor_fusion.size();i++)
                    {
                        float d = sensor_fusion[i][6];
                        if( (d < (2+4*lane+2)) && (d > (2+4*lane-2)) )
                        {
                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            double check_speed = sqrt(vx*vx+vy*vy);
                            double other_car_last_s = sensor_fusion[i][5];

                            other_car_last_s +=((double)prev_size*0.02*check_speed) ;



                            if((other_car_last_s > car_last_s) &&
                               (other_car_last_s-car_last_s) < 30) 
                            {
                                    too_close=true;
                                    lane =  counter++ % 4; // target_lane[(counter++)%4];
                            }
                        }
                    }

                    if(too_close)
                    {
                        speed -= 0.224 * speed_multiplier;
                        speed_multiplier -=2;
                        
                    }
                    else if(speed < MAX_SPEED)
                    {
                        speed += 0.224 * speed_multiplier;
                        speed_multiplier +=2;
                    }

//////////////////////////////////////////////////////////////////////////////////////////////
#if 1
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

                    vector<double> next_wp0 = 
                            getXY(car_last_s+30,(2+4*lane) ,map_waypoints_s, map_waypoints_x,map_waypoints_y);
                    vector<double> next_wp1 = 
                            getXY(car_last_s+60,(2+4*lane) ,map_waypoints_s, map_waypoints_x,map_waypoints_y);
                    vector<double> next_wp2 = 
                            getXY(car_last_s+90,(2+4*lane) ,map_waypoints_s, map_waypoints_x,map_waypoints_y);

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

                    vector<vector<double>> pts;

                    //car_s   end_path_s   car_d   end_path_d

                    vector<double>  start_s;
                    vector<double>  start_d;

                    double s_ = abs(end_path_s - car_s)/0.02;
                    double s__;
                    double d_ = abs(end_path_d - car_d ) /0.02; 
                    double d__;
                    if( cnt == 0 )
                    {
                        s__ = 0; //assume
                        d__ = 0; //assume
                    }
                    else
                    {
                        s__ = abs(g_prev_s_ - s_ )/0.02;
                        d__ = abs(g_prev_d_ - d_ )/0.02;
                    }
                    g_prev_s_ = s_;

                    start_s.push_back(car_s);
                    start_s.push_back(s_);
                    start_s.push_back(s__);
                    start_d.push_back(car_d);
                    start_d.push_back(d_);
                    start_d.push_back(d__);

                    vector<double> delta ; 
                    delta.push_back(0);
                    delta.push_back(0);
                    delta.push_back(0);
                    delta.push_back(0);
                    delta.push_back(0);
                    delta.push_back(0);
                    //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
                    pts = PTG( start_s, start_d, 0 , delta, 2,   carMap); //T is assumed as 2.....

                    tk::spline s;
                    s.set_points(pts[0],pts[1]);   //ptsx , ptsy

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    for(int i=0; i < previous_path_x.size(); i++)
                    {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);

                        std::cout << "    prev: added... " << previous_path_x[i] << "  " << previous_path_y[i];
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
                        std::cout << "next:added... " << x_point  << "   "  << y_point  << std::endl;
                    }
#endif
                    std::cout << "cnt-----------------------:" <<  cnt   << std::endl;
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
