from math import ceil
from ..Logger import Logger
from .JSONDeserialiser import JSONDeserialiser
from .FlightVolume import FlightVolume
from pyproj import Geod

class Operation(JSONDeserialiser):

    def __init__(self, json, logger=Logger()):
        super().__init__(json, logger=logger)
        self.__process_volumes()

    def get_object_properties(self):
        return [
            # Operation ID missing as there seems to be an inconsistency between two endpoints that *should* return
            # operation objects
            # issue: https://github.com/H3xept/CAELUS_SmartSkies/issues/5
            'control_area_id', 'ansp_id', 'user_id', 'organization_id', 'operation_volumes', 'reference_number', 'proposed_flight_speed'
        ]

    def __process_volumes(self):
        self.operation_volumes = [FlightVolume(json) for json in self.operation_volumes]

    def __interpolate_with_max_distance(self, start, end, max_dist):
        geod = Geod(ellps='WGS84')
        az12,az21,dist = geod.inv(start[1], start[0], end[1], end[0])
        result = geod.fwd_intermediate(start[1],start[0],az12,npts=ceil(dist / max_dist),del_s=max_dist)
        alt = start[-1]
        return [(lat, lon, alt) for lat, lon in zip(result.lats, result.lons)]

    def get_waypoints(self, max_distance=900):
        waypoints = [volume.get_centre() for volume in self.operation_volumes]
        start, end = waypoints[0], waypoints[-1]
        return self.__interpolate_with_max_distance(start, end, max_distance)

    def __repr__(self):
        return f'<Operation|reference={self.reference_number}>'