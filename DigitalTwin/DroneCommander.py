import logging
import time
from typing import Tuple
from pymavlink import mavutil

class DroneCommander():

    MAV_MODE_AUTO = 4

    @staticmethod
    def waypoints_to_string(wps):
        return '\n'.join(map(lambda composite: (lambda i, wp_a: f'\t {i}: {wp_a[0]}, {wp_a[1]} ⤴ {wp_a[2]}')(*composite), enumerate(wps)))

    @staticmethod
    def commands_from_waypoints(waypoints: Tuple[float, float, float]):
        return list(map(lambda wp: (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, wp[1], wp[0], wp[2]), waypoints))

    @staticmethod
    def mission_from_waypoints_vtol(waypoints: Tuple[float, float, float]):
        commands = DroneCommander.commands_from_waypoints(waypoints[1:-1]) # Last waypoint should be landing
        commands.insert(0,(mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, waypoints[0][1], waypoints[0][0], waypoints[0][2]))
        commands.append((mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND, waypoints[-1][1], waypoints[-1][0], waypoints[-1][2]))
        return commands

    @staticmethod
    def mission_from_waypoints_quad(waypoints: Tuple[float, float, float]):
        commands = DroneCommander.commands_from_waypoints(waypoints[1:-1]) # Last waypoint should be landing
        commands.insert(0,(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, waypoints[0][1], waypoints[0][0], waypoints[0][2]))
        commands.append((mavutil.mavlink.MAV_CMD_NAV_LAND, waypoints[-1][1], waypoints[-1][0], waypoints[-1][2]))
        return commands

    def __init__(self, drone_type: int, connection_string: str):
        self.__drone_type = drone_type
        self.__logger = logging.getLogger(__name__)
        
        if connection_string.startswith("mavlink://"):
            connection_string = connection_string[len("mavlink://"):]
        
        self.__master = mavutil.mavlink_connection(connection_string)
        self.connection_string = connection_string
        self.__home_location = None
        self.__mission_waypoints = None
        self.__end_waypoint = None
        self.__commands = []


    def __px4_set_mode(self, mav_mode):
        self.__master.mav.command_long_send(self.__master.target_system, self.__master.target_component,
                                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                            mav_mode,
                                            0, 0, 0, 0, 0, 0)
        
    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

    def set_mission(self, waypoints):
        QUAD = 0
        self.__mission_waypoints = waypoints
        
        self.__logger.info('Constructing new missions from waypoints')
        self.__logger.info('\n'+DroneCommander.waypoints_to_string(waypoints))

        if self.__drone_type == QUAD:
            self.__commands = DroneCommander.mission_from_waypoints_quad(waypoints)
        else:
            self.__commands = DroneCommander.mission_from_waypoints_vtol(waypoints)

        self.__end_waypoint = waypoints[-1]

    def wait_for_home_lock(self):
        while self.__home_location is None:
            msg = self.__master.recv_match(type=['HOME_POSITION'], blocking=True)
            if msg is not None:
                self.__home_location = (msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1e3)
                print(f'Home location: {self.__home_location}')
            else:
                time.sleep(0.5)
    def __wait_for_vehicle_armable(self):
        self.__logger.info('Waiting for vehicle home lock')
        self.wait_for_home_lock()
        self.__logger.info('Waiting for vehicle to be armable (CHECK SKIPPED!)')
        time.sleep(2)

def upload_mission(self):
    self.__logger.info(f'Uploading {len(self.__commands)} commands...')
    self.__master.waypoint_clear_all_send()
    self.__master.waypoint_count_send(len(self.__commands))

    for i, command in enumerate(self.__commands):
        while True:
            msg = self.__master.recv_match(type=['MISSION_REQUEST'], blocking=True)
            if msg.seq == i:
                self.__master.mav.mission_item_send(self.__master.target_system, self.__master.target_component,
                                                    i, mavutil.mavlink.MAV_FRAME_GLOBAL,
                                                    command[0], 0, 0, 0, 0, 0, float('nan'), command[1], command[2], command[3])
                print(f"Uploaded mission item {i}")
                break
            else:
                time.sleep(0.1)

    self.__logger.info(f'Done uploading mission ({len(self.__commands)} items)')

def start_mission(self):
    self.__wait_for_vehicle_armable()
    self.upload_mission()
    self.__logger.info('Starting vehicle mission')
    self.__px4_set_mode(DroneCommander.MAV_MODE_AUTO)

    # Arm the vehicle
    self.__master.mav.command_long_send(self.__master.target_system, self.__master.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                        1, 0, 0, 0, 0, 0, 0)
