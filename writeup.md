# **Path Planning** 


## The goals / steps of this project are the following:
* In this project, your goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. 
* A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.


[//]: # (Image References)

[state_diagram]: ./state_diagram.png "STATE DIAGRAM"

## Rubric Points
I've been considered the [rubric points](https://review.udacity.com/#!/rubrics/1971/view)

---
## Writeup 

### Compilation
#### The code compiles correctly.  Code must compile without errors with cmake and make.  Given that we've made CMakeLists.txt as general as possible, it's recommend that you do not change it unless you can guarantee that your changes will still compile on any platform.
Yes. My code compiles without neither erros or warning.


### Valid Trajectories
#### The car is able to drive at least 4.32 miles without incident..  The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.  
Sure.  Actually more than that. My car drives 26.23 miles without any in incident.    
("Outside of Lane" occured  at around 26.23 miles. However,It doesn't seem to be a big deal because it is just off the line a little bit for a while. )    
I plan to upload the video. You will find it below link.
[Path_plainning_project_by Ghicheon](youtube link)





#### The car drives according to the speed limit.  The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.
Yes. It's implimented in the line 413 of main.cpp    
get_proper_speed is responsible for returning a proper speed.

* max_speed: soft requirement. when a blocking car is in front of my car, there is no options.It will be better to folow the front car!

* MAX_SPEED: it's set to 49.7  (hard requirement!)


#### Max Acceleration and Jerk are not Exceeded.  The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
Yes. Actually, I used the spline of the lecture and followed the formula of the video in order to avoid jerk and acceleration limit.

#### Car does not have collisions.  The car must not come into contact with any of the other cars on the road.
Definitelly. The car drives 32 miles without any collisions(I stopped it).     
line 589 ~ 636  :  if my car is blocked. slow down and consider lane change.

Line 273: CONSIDER_ALTERNATIVES() is core routine for selecting the lane without collisions.   

Line 308 ~ 311: check if it blocked on the right lane,
Line 312 ~ 315: check if there is enough free space on the right lane,

#### The car stays in its lane, except for the time between changing lanes.  The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.
Yes. My car drives 26.23 miles without "Outside of lane".

#### The car is able to change lanes.  The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.
Yes.   

In line 361 ~ 397, I considered free space of the left lane and the right lane.  Also, speed is important. If there are 2 options(left&right), speedy lane is selected.     

In blocking situation,when changing lane can't be done due to adjacent cars, the state is changed to PRE\_LEFT or PRE\_RIGHT.   

### Reflection
#### The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation".

![alt text][state_diagram]  

In Blocking situation.
* KEEP can be LEFT when the left lane is clear.   
* KEEP can be RIGHT when the right lane is clear.   
* if Both options are possible, more free space & speedy lane will be selected.
* When there is no hope..., my car is trying to follow the front car by setting max\_speed to the speed of the front car.   
  The state changes from KEEP to PRE_LEFT or PRE_RIGHT. 
* It's also possible to be KEEP to LEFT/RIGHT without PRE states!

In PRE_LEFT/PRE_RIGHT,
* try to turn left. But It's not a best option to maintain the state PRE_LEFT/PRE_RIGHT.    after some time, the state will be KEEP again.

In LEFT/RIGHT,
* turn left or right by using spline.



### Suggestions to Make Your Project Stand Out!
#### Create a path planner that performs optimized lane changing, this means that the car only changes into a lane that improves its forward progress.
* I get the left/right lane speed using the closest car.  
  If I consider some range from my car, It could be better.
* In line 384, I consider the cloest car for free space.
  If considered the front space from my car, the result will be more accurate.
* One lane change is only considered, If considered more than 2 lanes, the path will be optimized. For example,even if there is a car next to me, the next after next lane might be free.



