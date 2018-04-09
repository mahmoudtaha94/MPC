# Model Predictive Controller
Udacity Self-Driving Car Nanodegree Term 2, Project 5

### Results
See video of the results from my implementation [here](out.mp4).

## Discussion
The code starts by receiving some inputs from the simulator:
* ptsx: x-position of the mid line of the track in global coordinates
* ptsy: y-position of the mid line of the track in global coordinates
* px: x-position of the car in global coordinates
* py: y-position of the car in global coordinates
* psi: the orientation angle of the car
* v: the velocity of the car
* delta: current steering angle of the car
* a: the throttle
### 1) Fitting a polynomial
First we need to transform the mid line coordinates from the global coordinates to the vehicle's coordinates (main: lines 99-104).
Then we use polyfit function to fit these transformed points into a third degree polynomial equation, creating the ideal path for the car. cross track error is calculated from evaluating the polynomial (Polyeval function) at 0 which is the position of the car in the car coordinates (always 0) and error psi , epsi= psi - psi(desired), since psi is now 0 because we're talking from the vehicle coordinates and psi is the orientation, epsi= -1* psi(desired), psi desired is the tangential angle of the derivative of polynomial evaluated at xt arctan(f'(xt)) (main:: 108).
### 2) Handle the simulator delay
As advised in the lesson, in the MPC we can handle the 100ms delay by modeling it, so I used the model equations and used dt=0.1 :
x1 = x0 + v * cos(psi) * dt
y1 = y0 + v * sin(psi) * dt
psi1 = psi0 - v * delta / Lf * dt
v1 = v + a * dt
cte1 = (cte + (v * sin(epsi) * dt))
epsi1 = (epsi - v * delta / Lf * dt)
since the init state is zeros (x0,y0,psi0), the equations simplified to be (main ::118-124)
Then passed to the Solver in the MPC.cpp
### 3) MPC.cpp
in the Mpc Solve function we set all the limits of al the controlable outputs like delta from -25 to 25 degrees and acceleration -1 to 1. and we set all the state variables at all timesteps to zero except the variables at the first timestep to be the input. then FG_eval class calculated the costs for (epsi , cte, v, delta, a) because we want the epsi and cte to be zeros and we don't want the car to stop so we want the v to be as close as possible to a ref velocity, and we panelize the high change of the delta and the acceleration to make the as smooth as possible. but we use wights to panelize the important parameters more than the others (MPC:: 49 - 65). The updated cost constraints are calculated by first calculating the states at time t and time + 1. These states are then put through the update equations(MPC:: 67-109).
Then everything (the variables and the constrains) are fed to the Ipopt solver. This solver takes in all the information and will calculate the future predicted states
### 4)Tuning Timesteps (N) and Timestep Duration (dt) in MPC.cpp
the computation complexity is related to the parameter N. we want to maximize the N and minimize dt without affecting the response, so I chose the dt to be the simulator delay 0.1 and increased the N step by step until I reached 15 which means I'm looking at 1.5 seconds into the future 0.1*15 which is good to get the needed response.
