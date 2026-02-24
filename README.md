# ScyllaOS-test
pc_ws - pc workspace (Ground control pc)
pi_ws - pi workspace (Onboard rasperry pi)

## Conenction instructions

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
On both sides set ``` ROS_DOMAIN_ID ``` to the same number (anything between 0 and 232):
```
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
```

