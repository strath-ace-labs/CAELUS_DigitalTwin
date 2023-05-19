import json
import logging
from typing import List, Tuple, Dict
from PySmartSkies.Models.Drone import Drone

from DigitalTwin.error_codes import UNKNOWN_VEHICLE
from .ExitHandler import ExitHandler

KEY_NOT_FOUND = lambda k,d: f'Key {k} not found in dict'
KEY_WRONG_TYPE = lambda k,d,t: f'Key {k} is present but with the wrong type (Expected {t}, got {type(d[k])})'

class KeyNotFoundException(Exception):
    pass

def get_config_from_file(fname):
    try:
        with open(f'./available_drones/{fname}', 'r') as f:
            complete_config = json.loads(f.read())
            return complete_config
    except Exception as e:
        logging.getLogger().error('Error in reading drone config file')
        logging.getLogger().error(e)
        ExitHandler.shared().issue_exit_with_code_and_message(UNKNOWN_VEHICLE, f"Failed in reading drone configuration file '{fname}'")

def get(d, k, fail=False, default_if_not_found=None):
    if k not in d:
        if fail:
            raise KeyNotFoundException(KEY_NOT_FOUND(k,d))
        else:
            logging.getLogger().warn(KEY_NOT_FOUND(k,d))
            return default_if_not_found
    else:
        return d[k]

class Unpackable():
    @classmethod
    def from_json_file(cls, file):
        with open(file, 'r') as f:
            s = json.loads(f.read())
            return cls(s)

DRONE_TYPE_QUADROTOR = 0
DRONE_TYPE_FIXED_WING = 1

class ControllerPayload(Unpackable):

    def __init__(self, config_dict):
        self.waypoints: Tuple[float, float, float] = get(config_dict, 'waypoints')
        self.home_elevation: float = get(config_dict, 'initial_lon_lat_alt')[2]
        self.operation_id: str = get(config_dict, 'operation_id')
        self.group_id: str = get(config_dict, 'group_id')
        self.delivery_id: str = get(config_dict, 'delivery_id')
        self.control_area_id: str = get(config_dict, 'control_area_id')
        self.operation_reference_number: str = get(config_dict, 'operation_reference_number')
        self.drone_id: str = get(config_dict, 'drone_id')
        self.drone_registration_number: str = get(config_dict, 'drone_registration_number')
        self.dis_auth_token: str = get(config_dict, 'dis_auth_token')
        self.dis_refresh_token: str = get(config_dict, 'dis_refresh_token')
        self.cvms_auth_token: str = get(config_dict, 'cvms_auth_token')
        self.drone_config_file = get(config_dict, 'drone_config_file')
        self.drone_config_full = get_config_from_file(self.drone_config_file)
        self.max_rpm = self.drone_config_full['drone_config']['max_rpm']
        self.propeller_specs = self.drone_config_full['drone_config']['propeller_specs']
        self.drone_type: str = DRONE_TYPE_QUADROTOR if get(self.drone_config_full, 'type', default_if_not_found=DRONE_TYPE_QUADROTOR) == "QUADCOPTER" else DRONE_TYPE_FIXED_WING
        self.thermal_model_timestep: float = get(config_dict, 'thermal_model_timestep')
        self.weather_data_filepath: str = get(config_dict, 'weather_data_filepath', default_if_not_found=None)
        self.connection_string: str = get(config_dict, 'connection_string', default_if_not_found='127.0.0.1:14550')

class OrchestratorPayload(Unpackable):
    
    def __init__(self, config_dict):
        self.effective_start_time: str = get(config_dict, 'effective_start_time')


class SimulatorPayload(Unpackable):

    def __init__(self, config_dict):
        self.drone_config_file = get(config_dict, 'drone_config_file')
        self.drone_config_full = get_config_from_file(self.drone_config_file)
        self.drone_config = self.drone_config_full['drone_config']
        self.drone_type: str = DRONE_TYPE_QUADROTOR if get(self.drone_config_full, 'type', default_if_not_found=DRONE_TYPE_QUADROTOR) == "QUADCOPTER" else DRONE_TYPE_FIXED_WING
        self.g_acceleration: float = get(config_dict, 'g_acceleration')
        self.initial_lon_lat_alt: Tuple[float, float, float] = get(config_dict, 'initial_lon_lat_alt')
        self.final_lon_lat_alt: Tuple[float, float, float] = get(config_dict, 'final_lon_lat_alt')
        self.aeroacoustic_model_timestep: float = get(config_dict, 'aeroacoustic_model_timestep')
        self.payload_mass = get(config_dict, 'payload_mass')
        self.airframe_id = self.drone_config_full['px4_airframe_reference']