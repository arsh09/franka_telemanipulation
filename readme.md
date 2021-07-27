### Motivation 

- Using libfranka without ROS or Simulation lab. Minimizing the delays in UDP communication using multi-threaded, asynchoronity receptions with synchoronous send calls. 

- The need for an easy to use, easy to setup unilateral and bilateral teleoperation control library for two franka panda (or any follower-leader scenario reall) 


### Dependencies 

Requried are: 

1) Libfranka 0.7.0 (available in third_party folder as deb package) 
2) Boost 1.58 ( sudo apt-get install libboost1.58-all-dev )
  
### How to compile
```bash
$ git clone https://github.com/arsh09/franka_panda_experiments.git
$ cd franka_panda_experiements
$ mkdir build && cd build 
$ cmake .. 
$ cmkae --build .
```


### Structure Explanation  
To run the leader-follower examples in unilateral or bilateral control mode, first setup two franka pandas, and then open <i> follower_franka.cpp </i> and <i> leader_franka.cpp </i>. 

You will notice that these files include two header for <i>follower</i> and <i>leader</i> class. 

Please note that in this library, I have always assigned follower a UDP-server status and leader a UDP-client status. 

After the initialization, each of the classes (follower and server) takes a callback function. Depending on if you want to control the robot or just read the states, you can call the respective function (i.e. Read or Control) and pass in the callback function (i.e. read_loop or control_loop). Please note that Control and Read are both blocking calls. 

The callback function definitions are: 

```C++
// read callback
bool read_loop (
    franka::RobotState  _fstate /* follower full state */,
    franka::RobotState  _lstate /* leader full state */,
    franka::Duration    _duration /* duration, 0 in read loop case */,
    bool is_state /* if true, means that other side has sent its full state. */
    
) 

// control callback
franka::Torques control_loop (
    franka::RobotState  _fstate /* follower full state */,
    franka::RobotState  _lstate /* leader full state */,
    franka::Duration    _duration /* control loop timer/period */,
    bool is_state /* if true, means that other side has sent its full state. */
    
) 

```


Please note that as a user, you basically have the power to return any joint torques (of course under torque limits) using the control loop which receives leader and follower full states. This torques do get execuated by the lib franka control loop. Please always make sure that <b>is_state</b> is true before you use the <b>_fstate </b> or <b> _lstate </b> variables


#### Example

On leader side, run: 

```bash
$ cd /path/to/franka_panda_expeirments/build/
$ ./leader_franka <server-port> <follower-pc-hostname> <leader-robot-host>
```

On follower side, run: 

```bash
$ cd /path/to/franka_panda_expeirments/build/
$ ./follower_franka <server-port>  <follower-robot-host>
```