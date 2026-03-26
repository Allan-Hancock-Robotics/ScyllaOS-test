# ScyllaOS
pc_ws - pc workspace (Ground control pc)
pi_ws - pi workspace (Onboard rasperry pi)
The following setup instructions are done with a Windows pc.

## Setup ethernet connection to the robot's Raspberry Pi 4
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

## Code Layout

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

### 2. Match ROS2 "domain" and allow network discovery.
On both sides set ```ROS_DOMAIN_ID``` to the same number (anything between 0 and 232) and set ```ROS_LOCALHOST_ONLY``` to false so ros2 can communicate with a separate device:
```
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
```

If we run ```echo $ROS_DOMAIN_ID``` and ```echo $ROS_LOCALHOST_ONLY``` we should see 0 and 1 to check if they were set correctly.

### 3. (Windows only) Open firewall for ROS2 DDS traffic
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


### other helpful commands:
List sypes of all active services:```ros2 service list -t```

Get info on a service:```ros2 service info <service_name>```

Save a node's parameter configuration to a file:```ros2 param dump <node_name> > <filename>```

Load a node's parameter from a file:```ros2 param load <node_name> <filename>```
