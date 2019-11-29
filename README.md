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

The `p_error` is the difference between the desired state and current state. 

`d_error` is simply the difference between the previous error and the current one.  

`i_error` is the sum of errors over time.

These are calculated by the following code:

```
void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = cte;
  i_error += cte;
  d_error = cte - p_error;
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
## PID Steering
`pid_steering` controller tries to keep the car to the center.

By hand tuning I set variable `Kp = 0.5` and `Kd = 3.5`.

But there was a problem in the turnings where it was very unstable. 

Then I tried small `Kp` values which reduced the instability but also I had to adjust the `Kd` value again.

This time it became `Kp = 0.04` and lowered `Kd = 1`.

Then again to increase the speed at turnings I kept tuning and finally chose `Kp = 0.1` and `Kd = 1.2`.


## PID Speed

For the velocity controller the values were tuned manually since I don't have the car's velocity dynamics equations.  Choosing a hardcoded value of 0.3 for the throttle gets the vehicle running at 37 MPH in a pretty constant manner. The only thing to notice is that the vehicle takes some time to get up the speed. 

I decided to use a PI controller. Here the integral component is obviously useful. When the error is zero, the throttle value is maintained to keep the car moving at the desired velocity. I could have included the Kd component as well. It could serve in case the error decreases too fast. Then, the Kd would reduce the throttle anticipating that the car will go over the set velocity.

I set the desired speed at 30MPH and chose `Kp = 0.01` and `Ki = 0.001` for my first trial. With these values the simulated car starts from 0MPH, increases speed, gets up to 50MPH and then slows down and stabilizes at 30MPH.

Even without a graph to look at, it is obvious that the `Ki` factor is too large. The error gets intergated over a long period of time. I didn't want to increase the `Kp` factor to make the initial acceleration faster since: `0.01(Kp) * 30(initial error) = 0.3` (the approximate throttle to drive at desired speed). So increasing `Kp` would lead to instability in reaching the desired velocity.

With `Ki = 0.001` the integrated error factor is too large. When the error reaches zero, the `Ki * sum(e(t))` is larger than 0.3, the approximated desired throttle. Because of this the car accelerates and increases the velocity up to 50MPH. During this time the error is negative and keeps being intergated. `Kp` component slows the vehicle down and when the 30MPH is reached again, the `sum(e(t))` is smaller than it was before, and manages to provide the desired throttle to maintain the speed.

I kept `Kp = 0.01` and lowered the integral factor ten times to `Ki = 0.0001`.
These are my final values.
