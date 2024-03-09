# Project SWARM

Created: February 19, 2024 6:18 AM
Last edited time: March 8, 2024 9:14 PM
Author: إبراهيم الهلالي
Tags: PSDSRARC, Work

To establish a governing way of connection we need to have multipoint network to connect and control drones with a script.

scripting the drone movements allow planned maneuvers, formation, smart spread, collision avoidance and more swarming applications. 

To connect multiple drones to one core we will need to have a center that gathers information from the drones, then calculate and broadcast the commands to the aerial units.

The center can be one of the flying units, a master, or the ground station.

The data flowing out and from the flight controller ‘FC’, is in the mavlink format, a standard format or protocol supported by many flight-controllers software, such as Ardupilot and PX4,

# Connections

## FC telemetry port:

The telemetry port found in the Cube FC, transmits and receives the mavlink messages through UART port in which can be connected to telemetry devices, such as RFD or 3DR radios. 

![FC Cube ports diagram](Project%20SWARM%20dcc951d03ecc4ff3872c6056a538c850/Untitled.png)

FC Cube ports diagram

Telemery (TELEM1|2) port pinouts**:**

| Pin # | Name | I/O | Voltage | Description |
| --- | --- | --- | --- | --- |
| 1 | VCC_5V | OUT | 5 V | Supply from AP |
| 2 | MCU_TX | OUT | 3.3V-5.0 V TTL | TX of AP |
| 3 | MCU_RX | IN | 3.3V-5.0 V TTL | RX of AP |
| 4 | MCU_CTS (TX) | OUT | 3.3V-5.0 V TTL | CTS (Clear To Send) |
| 5 | MCU_RTS (RX) | IN | 3.3V-5.0 V TTL | RTS (Request To Send) |
| 6 | GND | - | GND | GND connection |

## Multipoint methods:

there’re many methods to stablish a mesh network, listing here two available ways to establish a network suitable for swarms:

1. **Radio Telemetry:** *RFD900+ through SiK Multipoint* firmware*.*
2. **WiFi:** *ESP32 through with DroneBridge* firmware*.*

### 1- RFD900+ Radios

RFD900+ pinouts datasheet:

![Untitled](Project%20SWARM%20dcc951d03ecc4ff3872c6056a538c850/Untitled%201.png)

pins 4 and 6 are shorted by default, pins: 1, 5,7,9 are connected to the FC.

After flashing the multipoint firmware, change the following setting:

NODECOUNT = *(1-255) (number of units in the network, must be the same on all units)*

TXPOWER = (20-30) *mdb, range*

MAVLINK = 1

*Airborne platforms as follows:*
NODEID = (1-255)
NODEDESTINATION = 0 *(to the ground station)*

*Set the ground station as follows:*
NODEID = 0
NODEDESTINATION = 65535 *(Broadcast mode)*

To do so, you’d need to use the AT commands in the device terminal, can be done with RFD tools software, or any serial monitoring software.

First enter into dev mode in terminal with the command: +++ 

Dev mode exist itself frequently, make sure to renter the command to renter the dev mode to set and save settings.

Modify the setting with AT commands following the manual: 

[http://files.rfdesign.com.au/Files/documents/Software manual.pdf](http://files.rfdesign.com.au/Files/documents/Software%20manual.pdf)

with the RFD multi-point setup, you may need a ground station software, like MAVProxy or mavlink-router to filter each vehicle messages based on their SYSID parameter, and to forward the separated connections to UDP or TCP port, to be easily handled by the script. 

---

## 2- WiFi with ESP32 DroneBridge

### ESP32

![Untitled](Project%20SWARM%20dcc951d03ecc4ff3872c6056a538c850/Untitled%202.png)

the ESP32 board comes with WiFi chip, its relatively cheap, reliable, and easy to configure with drone bridge, but it comes with short range 20m-50m, suitable for testing.

Connect pins L1, L2, R6, R7 to the FC.

### Drone Bridge

[https://github.com/DroneBridge/DroneBridge](https://github.com/DroneBridge/DroneBridge)

DroneBridge is a system based on the WifiBroadcast approach. A bidirectional digital radio link between two endpoints is established using standard WiFi hardware and a custom protocol. 

DroneBridge is optimized for use in UAV applications and is a complete system. It is intended be a real alternative to other similar systems, such as DJI Lightbridge or MAVLink.

![Untitled](Project%20SWARM%20dcc951d03ecc4ff3872c6056a538c850/Untitled%203.png)

[https://github.com/DroneBridge/ESP32](https://github.com/DroneBridge/ESP32)

A DroneBridge enabled firmware for the popular ESP32 modules from Espressif Systems. Probably the cheapest way to communicate with your drone, UAV, UAS, ground based vehicle or whatever you may call them.

It also allows for a fully transparent serial to WiFi pass through link with variable packet size (Continuous stream of data required).

![Untitled](Project%20SWARM%20dcc951d03ecc4ff3872c6056a538c850/Untitled%204.png)

---

### MAVProxy

mavproxy is ground station command-line based software, it can route mavlink message from different end points.

Make sure to allow the ports though the fire wall.

mavproxy example:

```bash
[mavproxy.py](http://mavproxy.py) --master=udpout:192.168.8.131:14550 --out=udp:localhost:14541
# connect with --master and rerout with --out to local port used by a script
# 'udpout' for external ports, 'udp' for internel ports
```

---

### Mavlink-Router

MAVLink Router is an application to distribute [MAVLink](https://mavlink.io/en/) messages between multiple endpoints (connections). It distributes packets to a single port or multiple endpoints depending on the target address.

Connections, can be made via UART, UDP or TCP.

the configuration file :

```bash
[UartEndpoint alpha]
Device = /dev/ttyUSB0
Baud = 57600

[UdpEndpoint bravo]
Mode = Normal
Address = 0.0.0.0
Port = 10000
AllowSrcSysOut = 1  # swarm SYSID_THISMAV parameter

[UdpEndpoint charlie]
Mode = Normal
Address = 0.0.0.0
Port = 20000
AllowSrcSysOut = 2  # swarm SYSID_THISMAV parameter 
```

Start the mavlink router, pointing with -c option to the configuration file:

```bash
mavlink-routerd -c ~/path/to/config.conf
```

### Linking with QGroundControl:

configure the routing to set an end point that is mainly used by the ground station using internal ports

mavlink router configuration:

```bash
[UdpEndpoint bravo]
Mode = Normal
Address = 0.0.0.0
Port = 10000        # Goes to the script

[UdpEndpoint charlie]
Mode = Normal
Address = 0.0.0.0
Port = 14551        # Goes to the QGS
```

or mavproxy:

```bash
[mavproxy.py](http://mavproxy.py) --master=/dev/tty --out=udp:localhost:10000 --out=udp:localhost:14541
```

then in QGC application setting, configure ‘comm links’ to listen to these ports,

in swarm application when having multiple ports, make sure to add comm links for each drone as follows:

![Untitled](Project%20SWARM%20dcc951d03ecc4ff3872c6056a538c850/Untitled%205.png)

![Untitled](Project%20SWARM%20dcc951d03ecc4ff3872c6056a538c850/Untitled%206.png)

This allows you to see and control the vehicles in the QGC, and a drop down menu to switch between vehicles.

![Untitled](Project%20SWARM%20dcc951d03ecc4ff3872c6056a538c850/Untitled%207.png)

# Testing

### Ardupilot SITL

The SITL (software in the loop) simulator allows you to run Plane, Copter or Rover without any hardware. It is a build of the autopilot code using an ordinary C++ compiler, giving you a native executable that allows you to test the behaviour of the code without hardware.

To start swarm drone simulator using SITL:

```bash
sim_vehicle.py -v copter --out=udp:localhost:14550 --out=udp:localhost:10000 -I0
```

*-I0* & *-I1* is an essential option to simulate swarms by creating multiple sitl drone instances.

Make sure to change SYSID_THISMAV parameter of each drone

can be done with the following commands in the simulation terminal:

```bash
param set SYSID_THISMAV 1
```

same for the second vehicle in other terminal: 

```bash
sim_vehicle.py -v copter --out=udp:localhost:14551 --out=udp:localhost:20000 -I1
```

```bash
param set SYSID_THISMAV 2
```

# Scripting

### Connection with python script

Using the multipoint RFD radios with mavlink router to bind each drone link to a drone instance inside the python script with pymavlink library

![Untitled](Project%20SWARM%20dcc951d03ecc4ff3872c6056a538c850/Untitled%208.png)

## PyMavLink

Pymavlink is a *low level* and *general purpose* MAVLink message processing library, written in Python. It has been used to implement MAVLink communications in many types of MAVLink systems.

### Pymavlink UDP connection:

```python
from pymavlink import mavutil

drone1 = mavutil.mavlink_connection('udpin:0.0.0.0:14541')
drone1.wait_heartbeat()

drone2 = mavutil.mavlink_connection('udpin:0.0.0.0:14542')
drone2.wait_heartbeat()
```

---

### Distance between drones

```python
mav = drone1.recv_match(type='AHRS2', blocking=True)
elev1 = mav.altitude
lat1 = mav.lat * 1e-7
lon1 = mav.lng * 1e-7

mav = drone2.recv_match(type='AHRS2', blocking=True)
elev = mav.altitude
lat = mav.lat * 1e-7
lon = mav.lng * 1e-7

lat1_rad, lon1_rad = np.deg2rad(lat1), np.deg2rad(lon1)
lat2_rad, lon2_rad = np.deg2rad(lat), np.deg2rad(lon)

R = 6371000 # Earth radius in meters
dlat = lat2_rad - lat1_rad
dlon = lon2_rad - lon1_rad

# Haversine
a = np.sin(dlat/2)**2 + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(dlon/2)**2
c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
distance = R * c

bearing = np.arctan2(np.sin(dlon) * np.cos(lat2_rad),
                    np.cos(lat1_rad) * np.sin(lat2_rad) -
                    np.sin(lat1_rad) * np.cos(lat2_rad) * np.cos(dlon))

x_distance = distance * np.cos(bearing)
y_distance = distance * np.sin(bearing)
z_distance = elev1 - elev

t_distance = sqrt(x_distance**2+y_distance**2+z_distance**2)
```

### Arm and take off action

```python
def request_message_interval(message_id: int, frequency_hz: float,master):

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id,  # The MAVLink message ID
        1e6 / frequency_hz,  # The interval between two messages in microseconds.
        # set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0,  # Unused parameters
        0,  # Target address of message stream (if message has target address fields).
        # 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

def flight_mode(mode: str):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

def arm_command(arm: int,master):
    master.mav.command_long_send(master.target_system, master.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            arm,  # 1 for arm, 0 for disarm
                            0,0,0,0,0,0)
    
def takeoff_command(alt: int,master):
    master.mav.command_long_send(master.target_system, master.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0,0,0,0,0,0,
                            alt)  # altitude target

def take_off_action(meters,master):
    print("arming")master
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,1,drone1)
    arm_command(1,drone1)
    time.sleep(1)
    print("armed")
    done = False
    taking_off = False

    while not done:
        mav = drone1.recv_match(type='EXTENDED_SYS_STATE', blocking=True)

        if mav.landed_state == 1 and not taking_off :
            flight_mode('GUIDED',drone1)
            time.sleep(1)
            mav = drone1.recv_match(type='HEARTBEAT', blocking=True)
            mode = mavutil.mode_string_v10(mav)

            if mode != 'GUIDED':
                continue

            takeoff_command(meters,drone1)
            mav = drone1.recv_match(type='EXTENDED_SYS_STATE', blocking=True)
            print('take off commanded, target height: ',meters)
            taking_off = True

        elif mav.landed_state == 3 and taking_off:
            mav = drone1.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            print('taking off, hight: ', np.round(-1*mav.z,2),'m , target: ',meters,'m')

        elif mav.landed_state == 2:
            print('vehicle',drone1.target_system,'on air')
            done = True

take_off_action(10,drone1)
```

### Movement

A script of two drones orbiting around each other: 

![Untitled](Project%20SWARM%20dcc951d03ecc4ff3872c6056a538c850/Untitled%209.png)

```python
def flight_mode(mode: str, master):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

def set_vel_glob(vel_x:float, vel_y:float, master):
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        10, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # refrance frame option
        (0b110111000111),  # control option blocks all and takes vel float type, and sets yaw rate 0 rad/s 
        0,  # Latitude
        0 ,  # Longitude
        0,  # in meters, +postive+ for ^upward^ height
        vel_x,  # velocity in x direction type masking ingore this
        vel_y,  # velocity in y direction
        0,  # velocity in z direction
        0, 0, 0, # accelartions (not supported)
        0,  # yaw or heading in radians, 0 for front or north (depends on refrance frame)
        0  # yaw rate in rad/s , 0 for no yaw during movment
    ))

def lock(lat, lon, master):
    mav = master.recv_match(type='AHRS2', blocking=True)
    lat1 = mav.lat * 1e-7
    lon1 = mav.lng * 1e-7

    lat1_rad, lon1_rad = np.deg2rad(lat1), np.deg2rad(lon1)
    lat2_rad, lon2_rad = np.deg2rad(lat), np.deg2rad(lon)

    R = 6371 # Earth radius in kilometers
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    # Haversine
    a = np.sin(dlat/2)**2 + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(dlon/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    distance = R * c

    bearing = np.arctan2(np.sin(dlon) * np.cos(lat2_rad),
                        np.cos(lat1_rad) * np.sin(lat2_rad) -
                        np.sin(lat1_rad) * np.cos(lat2_rad) * np.cos(dlon))

    return distance*1000, bearing

def heemo_orbit(lat,lon, R_s, K, master):
    R, bearing = lock(lat,lon,master)

    if R < R_s:
        Rd = R_s - R
        Rd = R_s + Rd
        a = np.arccos(R_s/Rd)
        drv_ang = bearing + a + np.pi/2
        b = 'in'

    else:
        a = np.arcsin(R_s/R)
        drv_ang = bearing + a
        b = 'out'

    x = K*np.cos((drv_ang))
    y = K*np.sin((drv_ang))

		set_vel_glob(x, y, master)

flight_mode('GUIDED',drone1)
flight_mode('GUIDED',drone2)

while True:

    mav1 = drone1.recv_match(type='AHRS2', blocking=True)
    lat1 = mav1.lat * 1e-7
    lon1 = mav1.lng * 1e-7

    mav2 = drone2.recv_match(type='AHRS2', blocking=True)
    lat2 = mav2.lat * 1e-7
    lon2 = mav2.lng * 1e-7

    heemo_orbit(lat1, lon1, 10,3,drone2)
    heemo_orbit(lat2, lon2, 10,3,drone1)
```