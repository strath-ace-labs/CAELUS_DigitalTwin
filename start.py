from dotenv import load_dotenv
from os import environ
from DigitalTwin.PayloadModels import ControllerPayload, SimulatorPayload
load_dotenv()
import sys
from os import environ, path
import logging 
from os.path import exists
from DigitalTwin.SimulationFactory import new_simulation
from DigitalTwin.error_codes import *
from DigitalTwin.ExitHandler import ExitHandler

logger = logging.getLogger()
logging.basicConfig(level=logging.INFO)

def check_exported_px4():
    local_px4 = 'Dependencies/PX4-Autopilot'
    if not 'PX4_ROOT_FOLDER' in environ:
        if path.exists(local_px4):
            environ['PX4_ROOT_FOLDER'] = local_px4
        else:
            logger.error(f'You must export your local copy of the CAELUS px4 fork folder (export PX4_ROOT_FOLDER=<px4 folder>)')
            exit(-1)

def start_with_payload(payload, headless=False):
    check_exported_px4()

    print('Staring simulation...')
    exit_handler: ExitHandler = ExitHandler.shared()

    try:
        sim_payload = SimulatorPayload(payload)
        controller_payload = ControllerPayload(payload)
        gui, controller, sstack = new_simulation(sim_payload, controller_payload, headless=headless)

        sstack.start()
        if gui is not None:
            gui.start()
    except Exception as e:
        logging.getLogger().error(f'Digital Twin Top Level Error:')
        logging.getLogger().error(e)
        import traceback
        traceback.print_exc()
    finally:
        code, msg = exit_handler.block_until_exit()
        if msg is not None:
            logging.getLogger().error(f'Exiting with error message: {msg}')
        sys.exit(code)

import json
import sys

if __name__ == '__main__':
    if len(sys.argv) == 1 and 'PAYLOAD' not in environ:
        print("Usage `python3 start.py <mission_payload_json>`")
        print("or")
        print("Export 'PAYLOAD={...}' and then issue `python3 start.py`")
        exit(-1)
    if 'PAYLOAD' in environ:
        json_payload = environ['PAYLOAD']
    elif len(sys.argv) == 2:
        try:
            with open(sys.argv[1], 'r') as f:
                json_payload = json.load(f)
        except Exception as e:
            print(f'Failed in reading json payload ({e})')
            exit(JSON_READ_EC)  
    else:
        print("Usage `python3 start.py <mission_payload_json>`")
        exit(-1)
    headless = True if 'IN_DOCKER' in environ else False
    try:
        start_with_payload(json_payload, headless=headless)
    except Exception as e:
        print(f'Failed in reading json payload ({e})')
        exit(JSON_READ_EC)
        
