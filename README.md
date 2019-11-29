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


## PID Speed

`pid_speed` controller tries to maintain a steady speed.

I set the desired speed at 30MPH and chose `Kp = 0.85` and `Ki = 0.00009` by trial and error.

Setting the desired speed at first is very important because `pid_steering` parameters depend on it as different velocity will require different steering response.

## PID Steering
`pid_steering` controller tries to keep the car to the center.

At 30MPH it is required to notice how much steering speed is required to steer at the maximum curvature of the road.

By hand tuning I set variable `Kp = 0.18` and then I kept on finding the approriate value `Kd = 3.67` to mitigate the instability.
Also, I set `Ki=00005` to center the vehicle by slowly reducing the error to 0 in case of straight path.
