
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
