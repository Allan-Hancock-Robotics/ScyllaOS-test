# ScyllaOS
pc_ws - pc workspace (Ground control pc)

pi_ws - pi workspace (Onboard rasperry pi)

Disclaimer: The following setup instructions are done with a Windows pc so things will be different for different OS(Lunix, etc.).

## Setup ethernet connection to the robot's Raspberry Pi 4
We need to configure an ethernet port for the computer we are using to make sure we can connect with BlueOS on the pi (Connection might stil work without this step)
* **Linux:**
    1. Go to Network Conenctions and click on the ethernet port (Usually called "Wired Connection")
    2. Go to IPv4 Settings. Under "Additional static addresses" add the following information:
        * Address: 192.168.2.1
        * Netmask 255.255.255.0
        * Gateway: Leave empty
        And press Add
* **Windows:**
    To have your pc talk to the robot you need to change your ethernet settings, as the robot uses a tethered ethernet connection.
    1. Open control panel -> Network/sharing
    2. Open settings for your ethernet port. Either go to "change adapter settings" and click "Ethernet" or click on "Ethernet" in the active networks view
    3. Go to properties -> Internet Protocal Version 4 (TCP/IPv4)
    4. Change from "Obtain an IP address automatically" to "Use the following IP address" and enter the following:
    ```
    IP address: 192.168.2.1
    Subjet mask: 255.255.255.0
    ```
    This sets the IP for your ethernet port manually and this is the IP address that the robot will look for when connecting
    5. Press OK and exit



## Connection instructions

### 1. make sure pi and pc can talk to each other.

On pc:
```
ping 192.168.2.2
```
On pi:
```
ping 192.168.2.1
```

### 2. (Windows only) Open firewall for ROS2 DDS traffic
Because ROS2 uses UDP ports and multicast, we want to make sure the pc's firewall doesn't stop us. 

**Windows:** Allow inbound/outbound UDP for our ROS2 executables to talk. You can also disable the firewall temporarily if needed.
    1. Open Windows Firewall -> Advanced settings
    2. Inbound Rules -> New Rules -> 
    3. Protocal: UDP
    4. Specific local ports: 7400-7500 (should be enough for our setup, if not, widen to 7400-7600)
    5. Allow connection
    6. Apply to domain/private/public as needed
    7. Give it a name
Outbound Rules are usually allowed by default so you shouldn't need to set a rule for it. If you're having problems connecting, try setting the same rule for Outbound Rules.

### 3. Install ROS2 Jazzy on your pc
Follow these steps to install ros2 jazzy: 
* **Windows:**https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html
* **Linux:** https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

### 4. Clone this repo and add teleop_node workspace
If you followed the installation instructions correctly (and are using Windows) you should have a pixi_ws workspace folder (or you might've named it something else, here' I'll refer to the workspace as pixi_ws). Here you can clone this repo to have all our Scylla code available.

In addition, you will want to add another workspace to install ros2 packages for reading and using controller inputs:
1. Make another directory in pixi_ws called teleop_ws (or something memorable), and a src directory inside that, teleop_ws\src. 
2. Use ```pixi shell``` to enter the pixi environment so the colcon command is available
3. Source the ROS2 Jazzy environment: ```call C:\pixi_ws\ros2-windows\local_setup.bat``` 
4. In teleop_ws\src:
 * clone the teleop repo: ```call C:\pixi_ws\ros2-windows\local_setup.bat```
 * enter the repo and make sure it's in verion 2.6.5: 
    ```
    cd teleop_twist_joy
    git checkout 2.6.5
    ```
5. Back in teleop_ws\src, clone the joy repo: ```git clone -b ros2 https://github.com/ros-drivers/joystick_drivers.git```
6. Go back to teleop_ws\ and build the workspace: ```colcon build --merge-install```
7. Overlay the workspace: ```call C:\pixi_ws\teleop_ws\install\local_setup.bat```
8. Test to see if the packages are available:
```
ros2 pkg executables teleop_twist_joy
ros2 pkg executables joy
```
You should see ```teleop_twist_joy: teleop_node``` and ```joy: joy_node, joy_enumerate_devices, game_controller_node```


### helpful commands to check environment variables
On pi side (and also on linux pc):

**Check domain id:** ```echo $ROS_DOMAIN_ID```

**check if local host only is on/off:** ```echo $ROS_LOCALHOST_ONLY```

**check middleware implementation:** ```echo $RMW_IMPLEMENTATION```

On pc side (windows):
**Check domain id:** ```$env:$ROS_DOMAIN_ID```

**check if local host only is on/off:** ```$env:$ROS_LOCALHOST_ONLY```

**check middleware implementation:** ```$env:$RMW_IMPLEMENTATION```

if these variables show nothing they they are set to default:

**Domain ID:** ROS2 uses domain 0 by default. If both sides are on domain 0, they can connect just fine 

**Local Host Only:** Set to off by default. We usually want it set to off, otherwise ROS 2 won't communicate on other devices

**Middleware Implementation:** Ros2 uses the default middleware installed on the system (usually Fast DDS, Connext DDS, or Eclipse Cyclone DDS)


### important scripts for windows
1. ```call C:\pixi_ws\ros2-windows\local_setup.bat```
2. ```call C:\pixi_ws\teleop_ws\install\local_setup.bat```

### other helpful commands:
Call a service:```ros2 service call <service_name> <service_type> <arguments>```

List types of all active services:```ros2 service list -t```

Get info on a service:```ros2 service info <service_name>```

Save a node's parameter configuration to a file:```ros2 param dump <node_name> > <filename>```

Load a node's parameter from a file:```ros2 param load <node_name> <filename>```


## Code Layout
Still in progress