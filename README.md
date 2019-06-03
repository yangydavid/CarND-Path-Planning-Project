# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Reflections: 

The project involves three major parts of the self-driving car technology: Sensor Fusion/Predictino, Behavior Planning, Trajectory Generation. In the following paragraphs, I will go through each of them and discuss how I approach to them. 

### Part I: Sensor Fusion & Prediction (line 110 - 173)
As introduced in Udaicty's original repo, the longitudinal(s) and lateral (d) position of the surrouding vehicle can be easily obtained through the "sensor_fusion" matrix. Therefore, using their relative localizaiton infromation, we can easily locate if the vehicle is in front of, to the left, or to the right of the EGO vehicle. 

Basically, three relative position flags were created: ```car_front, car_left, car_right```; These flags were set as ```false``` unless there are vehicles appears near the EGO vehicle and they are within a dangerous zone (front vehicle: less than 30 meters from us, left/right: within a 60 meters long passage next to us). 

### Part II: Behavior Planning (line 175 - 197)
Based on the location of surrounding vehicles, the EGO vehicle needs to make some decision such that the forward motion can be improved (e.g., reaching the target speed / drive smooth as much as possible). To do that, I create a lane switching mechanism so that we can somewhat improve the quality of the forward motion. 

As we know from the traffic laws, overtaking a front vehicle on the left is legal in most countries. Hence, we first considering make a left lane change if it is safe and the front vehicle is too slow. Then, if the left lane is occupied, we consider a lane change to the right. If both left and right lanes are occupied, we stay in the current lane and slow down to avoid collision. 

### Part III: Trajectory Generation (line 199 - 304)
Finally, let's discuss how we generate a jerkless polynomial whenever we make a behavior decision. One of the biggest trick here is that we always use part of the trajectory generated from the previous cycle. This is really helpful in order to keep the smothness of the trajecory. To do so, we created a vector to store previous generated trajecory and only pop out the used the ones and then pass it to the next cycle. 

Another trick here is that we used the ```spline.h``` library as suggested by the Udacity instructors. The best advantage is that we don't need to worry about the polynomial coefficients but just throw waypoints into it. Meanwhile, Keep in mind that we have to convert waypoints coordinates into vehicle's local frame. It makes my life a lot easier. 

Finally, I was able to satisfy the project rubrics and thanks so much for the help provided by the Udacity community during the wonderful learning experience. 