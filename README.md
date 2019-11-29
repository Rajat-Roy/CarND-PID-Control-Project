# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Goal
Control the throttle and steering using two PID controllers to drive the car around the track.

The starter code for this project is provided by Udacity and can be found [here](https://github.com/udacity/CarND-PID-Control-Project).

## What Is PID?

[PID](https://en.wikipedia.org/wiki/PID_controller) is a closed loop feedback controller. It produces a control value based on the error between the current state of the system and the desired state.

## PID Algorithm
The PID controller is implemented using the PID class. 


The `p_error`, `i_error` and `d_error` are calculated below.
The `p_error` is the straight value taken from the feedback loop. It represents the difference between the desired system state and its current state. 
`d_error`, or the derivate, is simply the substraction between the previous error and the current one.  
`i_error`, or the integral, is the sum of errors over time.

These are calculated by the following code:

```
void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  i_error += cte;
  d_error = cte - p_error; //it can be initialized with CTE value because the simulator is responsive only after 2 cycles
  p_error = cte;
}
```

Total error is used to calculate the control value in a similar way as the equation from above.

```
double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return Kp * p_error + Kd * d_error + Ki * i_error;
}
```
## Controlling The Direction
The first controller in this project is the one that, through the steering angle, makes the car drive along the road as close as possible to its center.

At the beginning, I set the steering angle to a hardcoded value of zero and launched the simulator to see if the car goes straight. I noticed that the steering angle was correctly set to zero, with no deviation, and the car seemed (visually) to be driving straight. This is how I decided that, for the direction control, the integral component is not needed. I chose a PD controller.


I changed the vehicle's length to 4 meters and used the motion model in the class to tune the `Kp` and `Kd` paramaters.


`Kp = 0.5` and `Kd = 2.5` seem to provide a good vehicle trajectory in following a reference line.

I plugged these values into the implementation and started the simulator. The vehicle was very unstable, the oscilations were strong, and the car was not even able to drive on a straight line.

This behavior was very surprising since the results looked good on the graph. Then, I realized that the car was steering with an angle in between -25deg and +25deg when the control values were given between -1rad and 1rad corresponding to -57deg and +57deg.


And here is the problem. The car does not steer with the controller provided value and it has a 2/3 reducing factor.

With this updated vehicle model, I chose to increase the `Kd` value to compensate the overshooting.
I chose `Kp = 0.5` as before and increased `Kd = 3.5`.


I got the simulator running and this time the car was able to follow a straight line trajectory. That's a great result!

The remaining problem is that it becomes very unstable in curves. I took a look at how the control value is calculated and realized that such a big `Kd` induces the instability. When driving into a curve, the error grows very fast making the `Kd` component built a lot on top of `Kp` and results in oversteering.

I then used the vehicle model to find a `Kp` `Kd` combination that doesn't overshoot, but allows for `Kd` to be smaller.

I chose `Kp = 0.04` and lowered `Kd = 1`.

In the simulator the vehicle is still able to drive on a straight line, but it is too slow to turn into curves and the car gets off the track.

I kept tuning the parameters from this point by trial and error. I got a pair `Kp = 0.1` and `Kd = 1.2` that seemed like a good compromise. These are my final values.


## Controlling The Velocity

For the velocity controller the values were tuned manually since I don't have the car's velocity dynamics equations.  Choosing a hardcoded value of 0.3 for the throttle gets the vehicle running at 37 MPH in a pretty constant manner. The only thing to notice is that the vehicle takes some time to get up the speed. 

I decided to use a PI controller. Here the integral component is obviously useful. When the error is zero, the throttle value is maintained to keep the car moving at the desired velocity. I could have included the Kd component as well. It could serve in case the error decreases too fast. Then, the Kd would reduce the throttle anticipating that the car will go over the set velocity.

I set the desired speed at 30MPH and chose `Kp = 0.01` and `Ki = 0.001` for my first trial. With these values the simulated car starts from 0MPH, increases speed, gets up to 50MPH and then slows down and stabilizes at 30MPH.

Even without a graph to look at, it is obvious that the `Ki` factor is too large. The error gets intergated over a long period of time. I didn't want to increase the `Kp` factor to make the initial acceleration faster since: `0.01(Kp) * 30(initial error) = 0.3` (the approximate throttle to drive at desired speed). So increasing `Kp` would lead to instability in reaching the desired velocity.

With `Ki = 0.001` the integrated error factor is too large. When the error reaches zero, the `Ki * sum(e(t))` is larger than 0.3, the approximated desired throttle. Because of this the car accelerates and increases the velocity up to 50MPH. During this time the error is negative and keeps being intergated. `Kp` component slows the vehicle down and when the 30MPH is reached again, the `sum(e(t))` is smaller than it was before, and manages to provide the desired throttle to maintain the speed.

I kept `Kp = 0.01` and lowered the integral factor ten times to `Ki = 0.0001`.
These are my final values.
