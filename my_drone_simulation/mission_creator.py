# Docs - https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/file_formats/plan.html

# MAVLink Mission Commands Reference (common.xml)
# Source: https://mavlink.io/en/messages/common.html#MAV_CMD
# ================================================


# MAV_CMD_NAV_WAYPOINT (16)
# ─────────────────────────
# Navigate to waypoint. Intended for use in missions.

#   Param 1  (Hold)           Hold time in seconds (ignored by fixed wing, used by rotary wing)     min: 0
#   Param 2  (Accept Radius)  Acceptance radius in meters (waypoint reached when inside this sphere) min: 0
#   Param 3  (Pass Radius)    0: pass through WP. >0: radius to pass by. Positive=CW, Negative=CCW
#   Param 4  (Yaw)            Desired yaw angle in degrees. NaN = use current heading mode
#   Param 5  (Latitude)       Latitude
#   Param 6  (Longitude)      Longitude
#   Param 7  (Altitude)       Altitude in meters


# MAV_CMD_NAV_RETURN_TO_LAUNCH (20)
# ──────────────────────────────────
# Return to launch location. All params empty.


# MAV_CMD_NAV_LAND (21)
# ─────────────────────
# Land at location.

#   Param 1  (Abort Alt)      Minimum target altitude if landing is aborted (0 = use system default)
#   Param 2  (Land Mode)      Precision land mode (0=disabled, 1=opportunistic, 2=required)
#   Param 3                   Empty
#   Param 4  (Yaw Angle)      Desired yaw angle in degrees. NaN = use current heading mode
#   Param 5  (Latitude)       Latitude
#   Param 6  (Longitude)      Longitude
#   Param 7  (Altitude)       Landing altitude (ground level in current frame) in meters


# MAV_CMD_NAV_TAKEOFF (22)
# ────────────────────────
# Takeoff from ground / hand.

#   Param 1  (Pitch)          Minimum pitch in degrees (if airspeed sensor present)
#   Param 2                   Empty
#   Param 3  (Flags)          Bitmask of option flags (NAV_TAKEOFF_FLAGS)
#   Param 4  (Yaw)            Yaw angle in degrees (if magnetometer present). NaN = use current heading mode
#   Param 5  (Latitude)       Latitude
#   Param 6  (Longitude)      Longitude
#   Param 7  (Altitude)       Altitude in meters


# MAV_CMD_NAV_LOITER_UNLIM (17)
# ──────────────────────────────
# Loiter around this waypoint indefinitely.

#   Param 1                   Empty
#   Param 2                   Empty
#   Param 3  (Radius)         Loiter radius in meters. Positive=CW, Negative=CCW (not for multicopters)
#   Param 4  (Yaw)            Desired yaw angle in degrees. NaN = use current heading mode
#   Param 5  (Latitude)       Latitude
#   Param 6  (Longitude)      Longitude
#   Param 7  (Altitude)       Altitude in meters


# MAV_CMD_NAV_LOITER_TIME (19)
# ─────────────────────────────
# Loiter at location for a specified time. Multicopters stop at the point.
# Fixed-wing circles around the point.

#   Param 1  (Time)           Loiter time in seconds (starts once position reached)                  min: 0
#   Param 2  (Heading Req)    Leave only when heading toward next WP (0=leave on time, 1=wait for heading)
#   Param 3  (Radius)         Loiter radius in meters. Positive=CW, Negative=CCW (not for multicopters)
#   Param 4  (Xtrack)         Exit location / path to next WP for fixed-wing
#   Param 5  (Latitude)       Latitude
#   Param 6  (Longitude)      Longitude
#   Param 7  (Altitude)       Altitude in meters


# MAV_CMD_DO_CHANGE_SPEED (178)
# ──────────────────────────────
# Change speed and/or throttle. Persists until overridden or mode change.

#   Param 1  (Speed Type)     0=airspeed, 1=groundspeed, 2=climb speed, 3=descent speed
#   Param 2  (Speed)          Speed in m/s (-1=no change, -2=return to default)
#   Param 3  (Throttle)       Throttle in % (-1=no change, -2=return to default)


# MAV_CMD_DO_SET_ROI_LOCATION (195)
# ──────────────────────────────────
# Set Region of Interest to a location (camera/gimbal will point there).

#   Param 1  (Gimbal ID)      Component ID (0=all gimbals, 1-6 for specific)
#   Param 5  (Latitude)       Latitude of ROI
#   Param 6  (Longitude)      Longitude of ROI
#   Param 7  (Altitude)       Altitude of ROI in meters


# MAV_CMD_DO_SET_ROI_NONE (197)
# ──────────────────────────────
# Cancel any previous ROI command. Gimbal returns to default.

#   Param 1  (Gimbal ID)      Component ID (0=all gimbals)


# MAV_CMD_CONDITION_YAW (115)
# ────────────────────────────
# Reach a certain target yaw angle.

#   Param 1  (Angle)          Target angle 0-360 degrees (0=north for absolute)
#   Param 2  (Angular Speed)  Angular speed in deg/s
#   Param 3  (Direction)      -1=CCW, 0=shortest, 1=CW
#   Param 4  (Relative)       0=absolute angle, 1=relative offset


# MAV_CMD_CONDITION_DELAY (112)
# ──────────────────────────────
# Delay mission state machine.

#   Param 1  (Delay)          Delay in seconds                                                       min: 0


# MAV_CMD_DO_JUMP (177)
# ─────────────────────
# Jump to a mission item. Repeat specified number of times.

#   Param 1  (Number)         Sequence number to jump to
#   Param 2  (Repeat)         Repeat count


# COORDINATE FRAMES (used in "frame" field)
# ──────────────────────────────────────────
#   0   MAV_FRAME_GLOBAL                  WGS84 + altitude relative to mean sea level (MSL)
#   2   MAV_FRAME_MISSION                 Not a coordinate frame, indicates a mission command
#   3   MAV_FRAME_GLOBAL_RELATIVE_ALT    WGS84 + altitude relative to home position
#   10  MAV_FRAME_GLOBAL_TERRAIN_ALT     WGS84 + altitude relative to ground level (AGL)

# When you upload a mission from QGC to the drone, only specific things are sent
# UPLOADED TO AUTOPILOT (affects flight)
# ───────────────────────────────────────
# items[].command        → MAVLink command (22=takeoff, 16=waypoint, 20=RTL, etc.)
# items[].frame          → Coordinate reference frame (3=relative alt, 2=mission)
# items[].params         → The 7 command parameters (lat, lon, alt, hold time, etc.)
# items[].autoContinue   → Whether to auto-advance to the next item
# items[].doJumpId       → ID used by DO_JUMP commands to reference this item
# geoFence               → Geofence circles/polygons (uploaded separately as a fence)
# rallyPoints            → Rally/safe return points (uploaded separately)


# NOT UPLOADED — QGC planner/UI only
# ───────────────────────────────────────
# fileType                        → Tells QGC how to parse the file
# groundStation                   → Identifies which GCS created it
# version                         → File format version
# mission.firmwareType            → Tells QGC which command set to show in the UI
# mission.globalPlanAltitudeMode  → Default altitude mode for QGC's planning display
# mission.hoverSpeed              → ETA/time estimates in QGC for multirotors
# mission.cruiseSpeed             → ETA/time estimates in QGC for fixed-wing
# mission.vehicleType             → Tells QGC which vehicle icon/options to show
# mission.version                 → Mission object format version
# mission.plannedHomePosition     → Map marker for home when no vehicle is connected
# items[].AMSLAltAboveTerrain     → Altitude display value in QGC's UI
# items[].Altitude                → QGC altitude display (actual alt is in params[6])
# items[].AltitudeMode            → QGC altitude mode display
# items[].type                    → Tells QGC if it's SimpleItem or ComplexItem for rendering

# When you upload a mission from QGC to the drone, only specific things are sent
#1. The mission plan (i.e. the list of waypoints and their parameters)
#2. The planned home position
#3. The vehicle type

# How to add configurations in SITL

# A quadcopter accelerates horizontally by tilting — and ArduCopter limits how far it can tilt (default 30 degrees). This creates a physical speed ceiling regardless of what WP_SPD is set to.
# Forward speed - param show WP_SPD | param set WP_SPD X
# Tilt angle - param show ATC_ANGLE_MAX | param set ATC_ANGLE_MAX X

# Waypoint acceptance radius - param show WP_RADIUS_M | param set WP_RADIUS_M X

# Starting points 
    # param set SIM_OPOS_LAT 40.193254
    # param set SIM_OPOS_LNG 44.504462
    # param set SIM_OPOS_ALT 1100
    # param set SIM_OPOS_HDG 0

import json

# -----------------------------
# Coordinates
# -----------------------------

HOME_LAT = 40.193254 # AUA lat
HOME_LON = 44.504462 # AUA lon
HOME_ALT_AMSL = 1100

# SHORT MISSION
WP2_LAT = 40.194682
WP2_LON = 44.504019
WP3_LAT = 40.193507
WP3_LON = 44.503119

# LONG MISSION
# WP2_LAT = 40.196765
# WP2_LON = 44.502891
# WP3_LAT = 40.192994
# WP3_LON = 44.499088

MISSION_ALT = 50

plan = {
    "fileType": "Plan", # tells QGC this is a mission plan file. Must be "Plan"
    "geoFence": { # (Optional) defines virtual boundaries the drone shouldn't cross
        "circles": [],
        "polygons": [],
        "version": 2
    },
    "groundStation": "QGroundControl", # identifies which ground control software created the file.

    "mission": { # The mission associated with this flight plan.
        # "cruiseSpeed": 15, # The default forward speed for Fixed wing or VTOL vehicles (i.e. when moving between waypoints).
        # Firmware types
        # 3 → MAV_AUTOPILOT_ARDUPILOTMEGA = ArduPilot
        # 12 → PX4
        # 0 → generic autopilot
        # 8 → invalid / not a flight controller
        "firmwareType": 3, 
        # globalPlanAltitudeMode = default for the whole mission
            # 0 = Relative    → Altitude relative to home/takeoff point
            # 1 = AMSL        → Absolute altitude above mean sea level
            # 2 = Terrain     → Height above terrain (QGC calculates it, sends as AMSL)
            # 3 = Terrain Frame → ArduPilot native terrain-following (MAV_FRAME_GLOBAL_TERRAIN_ALT)
        # In this mission, altitude is relative to the home/takeoff position and is set by MISSION_ALT
        "globalPlanAltitudeMode": 0, 
        "hoverSpeed": 5, # The default forward speed for multi-rotor vehicles. Even though SITL configures its value, its useful for QGC to estimate flight time
        # The list of mission item objects associated with the mission . The list may contain either/both SimpleItem and ComplexItem objects.
        # Simpleitem link - https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/file_formats/plan.html#mission_simple_item
        # Complexitem link - https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/file_formats/plan.html#mission_complex_item
        "items": [
            {
                "AMSLAltAboveTerrain": None, # Altitude value shown to the user.
                "Altitude": MISSION_ALT,
                # "AltitudeMode": 1, # already set with globalPlanAltitudeMode
                "autoContinue": True, # Tells the autopilot to automatically move to the next mission item after this one completes
                "command": 22, # MAV_CMD_NAV_TAKEOFF
                "doJumpId": 1, 
                "frame": 3,
                "params": [
                    0,
                    0,
                    0,
                    0,
                    HOME_LAT,
                    HOME_LON,
                    MISSION_ALT
                ],
                "type": "SimpleItem"
            },
            {
                "AMSLAltAboveTerrain": None, # Altitude value shown to the user
                "Altitude": MISSION_ALT,
                # "AltitudeMode": 1, # already set with globalPlanAltitudeMode
                "autoContinue": True,
                "command": 16,
                "doJumpId": 2,
                "frame": 3, # 10 - above terrain. 3 - relative to home
                "params": [
                    5,
                    0,
                    0,
                    0,
                    WP2_LAT,
                    WP2_LON,
                    MISSION_ALT
                ],
                "type": "SimpleItem"
            },
            {
                "AMSLAltAboveTerrain": None,
                "Altitude": MISSION_ALT,
                # "AltitudeMode": 1, # already set with globalPlanAltitudeMode
                "autoContinue": True,
                "command": 16,
                "doJumpId": 3,
                "frame": 3,
                "params": [
                    5,
                    0,
                    0,
                    0,
                    WP3_LAT,
                    WP3_LON,
                    MISSION_ALT
                ],
                "type": "SimpleItem"
            },
            {
                "AMSLAltAboveTerrain": None,
                "Altitude": 0,
                # "AltitudeMode": 1, # already set with globalPlanAltitudeMode
                "autoContinue": True,
                "command": 20,
                "doJumpId": 4,
                "frame": 2,
                "params": [
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0
                ],
                "type": "SimpleItem"
            }
        ],
        # The planned home position is shown on the map and used for mission planning when no vehicle is connected. The array values shown above are (from top): latitude, longitude and AMSL altitude.
        "plannedHomePosition": [
            HOME_LAT,
            HOME_LON,
            HOME_ALT_AMSL
        ],
        # Different types here - https://mavlink.io/en/messages/common.html#MAV_TYPE
        "vehicleType": 2, # Quadrotor
        "version": 2 # The version for the mission object
    },
    "rallyPoints": { # (Optional) Rally/Safe point information for this plan
        "points": [],
        "version": 2
    },
    "version": 1
}

with open("/Users/amkrtumyan/Documents/QGroundControl/Missions/AUA_mission.plan", "w") as f:
    json.dump(plan, f, indent=4)

print("Created /Users/amkrtumyan/Documents/QGroundControl/Missions/AUA_mission.plan")
