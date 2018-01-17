
# Getting Started


Great! With just a couple of lines of code we were able to get the car to drive forward at constant velocity.

Notice, however, that the car went from 0 MPH to 56 MPH in a single 20 ms frame, causing a spike in acceleration.

Acceleration is calculated by comparing the rate of change of average speed over .2 second intervals. In this case total acceleration at one point was as high as 75 m/s^2. Jerk was also very high. The jerk is calculated as the average acceleration over 1 second intervals. In order for the passenger to have an enjoyable ride both the jerk and the total acceleration should not exceed 10 m/s^2.

Part of the total acceleration is the normal component, AccN which measures the centripetal acceleration from turning. The tighter and faster a turn is made, the higher the AccN value will be.

In our simple test we were not turning at all, so the value of AccN was zero.

Going forward, consider how to minimize total acceleration and jerk by gradually increasing and decreasing point path spacing based on the car_speed variable.

To get a better idea of how movement affects the acceleration of the car, click the Manual Mode check box in the top left of the simulator screen to drive the car around yourself.



# TODO

# spline.h

- http://kluge.in-chemnitz.de/opensource/spline/
- http://kluge.in-chemnitz.de/opensource/spline/spline.h
- https://github.com/ttk592/spline/

## Usage
```
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "spline.h"

int main(int argc, char** argv) {

   std::vector<double> X(5), Y(5);
   X[0]=0.1; X[1]=0.4; X[2]=1.2; X[3]=1.8; X[4]=2.0;
   Y[0]=0.1; Y[1]=0.7; Y[2]=0.6; Y[3]=1.1; Y[4]=0.9;

   tk::spline s;
   s.set_points(X,Y);    // currently it is required that X is already sorted

   double x=1.5;

   printf("spline at %f is %f\n", x, s(x));

   return EXIT_SUCCESS;
}
```

# Rubrics

## Valid Trajectories

- The car is able to drive at least 4.32 miles without incident..

  - The top right screen of the simulator shows the current/best miles
    driven without incident. Incidents include exceeding
    acceleration/jerk/speed, collision, and driving outside of the
    lanes. Each incident case is also listed below in more detail.

- The car drives according to the speed limit.

  - The car doesn't drive faster than the speed limit. Also the car
    isn't driving much slower than speed limit unless obstructed by
    traffic.

- Max Acceleration and Jerk are not Exceeded.

  - The car does not exceed a total acceleration of 10 m/s^2 and a
    jerk of 10 m/s^3.

- Car does not have collisions.

  - The car must not come into contact with any of the other cars on
    the road.

- The car stays in its lane, except for the time between changing lanes.

  - The car doesn't spend more than a 3 second length out side the
    lane lanes during changing lanes, and every other time the car
    stays inside one of the 3 lanes on the right hand side of the
    road.

- The car is able to change lanes

  - The car is able to smoothly change lanes when it makes sense to do
    so, such as when behind a slower moving car and an adjacent lane
    is clear of other traffic.


## Reflection

- There is a reflection on how to generate paths.

  - The code model for generating paths is described in detail. This
    can be part of the README or a separate doc labeled "Model
    Documentation".
