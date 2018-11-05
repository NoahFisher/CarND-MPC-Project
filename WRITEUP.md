## Dependencies
===============

To build the docker image with all the required dependencies:

```sh
./build.sh
```

To run the docker image:

```sh
./run.sh
```

## Project Goal
===============

The goal of this project is to use a model predictive controller to drive a car around the track.
A model predictive controller uses a predicted path (yellow), motion model equations, and a cost
function to generate a predicted path. The predicted path is the solution of an optimization problem
to minimize the cost function.

## The Model
============

The model consists of the following six parameters.
  - the car's `x` and `y` positions
  - the car's orientation (`psi`)
  - the car's cross track error (`cte`)
  - the car's direction error (`ePsi`)

The motion of the car is governed by the following equations, where `t` is the value at the current
timestep and `t+1` is the value at the next timestep.

```cpp
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = y[t] - f(x[t]) + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

## Timestep Length and Elapsed Duration (Horizon Model)
============

During experimentation, it was found that the time horizon was correlated to the speed of the car,
the faster I wanted the car to go, the further out the time horizon had to be. This intuitively
makes sense as the car should be forced to take actions based on events further into the future
instead of those right in front of it. I settled on a time horizon of `20` and a timestep of `0.2`.


## Polynomial Fitting and MPC Preprocessing
============

A third order polynomial was used to model the track. Prior to fitting to the polynomial, the track's
waypoints (given by the simulator) were transformed to a coordinate system such that the car was
centered at the origin (0, 0).


## Model Predictive Control with Latency
============

To account fr latency, I chose to predict the car's state 100ms into the future. I used the
predicted state as the initial state and passed this to the optimizer.



#### Further info on set-up (cpp issues)
==============

To properly get the dependencies set up, I created a docker container.

The docker container was originally built with the contents of the Dockerfile using the following
command:

```
// runs in the same directory as the Dockerfile
docker build .
```

After the docker image is built, it can be run with the following command:

```
docker run -it -v $(pwd):/home/ubuntu -p 4567:4567 nfisher89/carnd-mpc bash
```
To explain...
  - `docker run` run the docker container
  - `-it` run interactively, and without a timeout
  - `-v` share the volume between the host and the docker image (allows for code editing with local
      tools like vim)
  - `-w` working directory
  - `-p` port forwarding
  - `nfisher89/carnd-mpc` the name of the container
  - `bash` after building, leave me in bash mode (optional)
