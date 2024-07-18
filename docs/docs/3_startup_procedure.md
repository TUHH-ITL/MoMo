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

**wakeupmomo** is an alias which runs the ./wakeupmomo.sh bash script on the Main PC.

**wakeupmomo** command opens up four terminals, three of them SSHs into the NUC and one of them starts the joystick node in ros2.

## Start bash scripts

Enter the following commands in the opened terminals in sequence.

In terminal 1:

```bash
./start_momo1.sh
```

In terminal 2:

```bash
./start_momo2.sh
```

and in terminal 3:

```bash
./start_momo3.sh
```

Now you can move the robot using joystick connected to the main board computer in ROS 2. The velocity commands are sent to the robot over the `/cmd_vel` topic.


## Shutdown and Cleanup

To completely shutdown the robot including both the computers, enter the following command on the main board computer terminal:

```bash
stopmomo
```

**stopmomo** is an alias which runs the /stopmomo.sh on the Main PC.
