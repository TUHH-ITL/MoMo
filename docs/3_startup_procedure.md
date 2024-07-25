# Startup Procedure

Before proceeding further please make sure everything is installed correctly by following the [Installation](2_software_setup.md) section.
Follow these steps to start the MoMo:

## Power On

Press the main power button to start the board computer on the robot and then press the power button on the NUC to turn on the NUC.

## Check connection

From the board computer, open a terminal and ping the NUC to verify if it is ON using:

```bash
ping 192.168.1.1 # change IP address if different
```

Replace the IP address with your own configured IP address for the NUC. Follow [this](https://www.youtube.com/watch?v=l2hC59n5M60&t=2s) tutorial to set static ip for both computers.

## Start command

Open up a terminal on the main computer and run the following command:

```bash
wakeupmomo
```

**wakeupmomo** is an alias which starts a catmux session.

**wakeupmomo** command opens up four terminals, three of them SSHs into the NUC and one of them starts the joystick node in ROS 2. 

More details about catmux can be found [here](https://github.com/fmauch/catmux).

Now you can move the robot using joystick connected to the main board computer in ROS 2. The velocity commands are sent to the robot over the `/cmd_vel` topic.

If it does not work, kill the session using the below command and try again: 
```bash
stopmomo
```
**Note:** Just closing the terminals is not enough, you need to kill the catmux session using the above command.

## Shutdown and Cleanup

To completely shutdown the robot including both the computers, enter the following command on the main board computer terminal:

```bash
sleepmomo
```


