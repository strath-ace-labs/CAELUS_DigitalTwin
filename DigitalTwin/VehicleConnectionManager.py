from threading import Thread, Condition
import logging
from .Interfaces.VehicleManager import VehicleManager
from .Vehicle import Vehicle
from pymavlink import mavutil
import traceback
class VehicleConnectionManager(VehicleManager):
    def __init__(self, vehicle_manager: VehicleManager = None, vehicle_addr='127.0.0.1:14550', connection_timeout=700, baud=57600):
        print(self)
        self.__vehicle_addr = vehicle_addr  # Store the device address as a string
        self.__vehicle_manager = vehicle_manager
        self.__vehicle = None
        self.__logger = logging.getLogger(__name__)
        self.__should_connect = False
        self.__connection_timeout = connection_timeout
        self.__baud = baud


    def stop_connecting(self):
        self.__should_connect = False
        if self.__vehicle is not None:
            self.__vehicle.close()

    def set_vehicle_manager(self, vm):
        self.__vehicle_manager = vm

    def __connect_to_vehicle(self):
        try:
            connection = mavutil.mavlink_connection(device=self.__vehicle_addr, source_system=1, input=True, dialect=None, autoreconnect=True, baud=self.__baud)
            vehicle = Vehicle(self.__vehicle_addr)
            vehicle.start_message_loop()
            self.__vehicle = vehicle
        except Exception as e:
            print("Exception occurred:\n", traceback.format_exc())
            if self.__should_connect:
                self.__logger.warn(e)
                self.__logger.warn(f'Vehicle connection timeout. Retrying...')
        finally:
            if self.__should_connect:
                if self.__vehicle is not None:
                    self.vehicle_available(self.__vehicle)
                else:
                    self.vehicle_timeout(self.__vehicle)



    def vehicle_available(self, vehicle):
        self.__setup_listeners(vehicle)
        if self.__vehicle_manager is not None:
            self.__vehicle_manager.vehicle_available(vehicle)
        else:
            self.__logger.warn('Vehicle lock acquired but no Vehicle Manager to broadcast the vehicle to.')

    def vehicle_timeout(self, vehicle):
        self.__logger.warn('Vehicle timed out')
        if self.__vehicle_manager is not None:
            self.__vehicle_manager.vehicle_timeout(vehicle)
        else:
            self.__logger.warn('Vehicle timed out but no Vehicle Manager to broadcast the event to.')
    def stop_connecting(self):
        self.__should_connect = False
        if self.__vehicle is not None:
            self.__vehicle.close()


    def connect_to_vehicle(self):
        self.__logger.info('Starting vehicle connection')
        self.__should_connect = True
        t = Thread(target=self.__connect_to_vehicle)
        t.name = 'Vehicle Connection'
        t.daemon = True
        t.start()
        
    def __setup_listeners(self, vehicle: Vehicle):
        pass

    def __setup_listeners(self, vehicle: Vehicle):
        pass

    def vehicle_available(self, vehicle):
        self.__setup_listeners(vehicle)
        if self.__vehicle_manager is not None:
            self.__vehicle_manager.vehicle_available(vehicle)
        else:
            self.__logger.warn('Vehicle lock acquired but no Vehicle Manager to broadcast the vehicle to.')

    def vehicle_timeout(self, vehicle):
        self.__logger.warn('Vehicle timed out')
        if self.__vehicle_manager is not None:
            self.__vehicle_manager.vehicle_timeout(vehicle)
        else:
            self.__logger.warn('Vehicle timed out but no Vehicle Manager to broadcast the event to.')