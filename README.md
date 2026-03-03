# ScyllaOS
pc_ws - pc workspace (Ground control pc)
pi_ws - pi workspace (Onboard rasperry pi)

## Code Layout
On the Pi we have BlueOS as the Operating System. Within BlueOS there is the ROS2 extension. This is where we are making connections to the pc and the controller.


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
On both sides set ```ROS_DOMAIN_ID``` to the same number (anything between 0 and 232):
```
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
```

If we run ```echo $ROS_DOMAIN_ID``` and ```echo $ROS_DOMAIN_ID``` we should see 0 and 1 to check if they were set correctly.

### 3. Open firewal for ROS2 DDS traffic
Because ROS2 uses UDP ports and multicast, we want to make sure the pc's firewall doesn't stop us. 

**Windows:** Allow inbound/outbound UDP for our ROS2 executables to talk. You can also disable the firewall temporarily if needed.