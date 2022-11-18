import logging
import time

import cv2
import numpy as np
from bosdyn.api import image_pb2
from bosdyn.client import create_standard_sdk, \
    RpcError, \
    lease, \
    estop, \
    robot_command, \
    robot_state, \
    power, \
    image, \
    InvalidRequestError
from bosdyn.client.estop import EstopKeepAlive
from bosdyn.client.lease import LeaseKeepAlive
from bosdyn.client.payload_registration import InvalidPayloadCredentialsError
from bosdyn.client.power import BatteryMissingError
from bosdyn.client.robot_command import NoTimeSyncError, NotPoweredOnError, RobotCommandBuilder

class MovementHelper:
    __VELOCITY_BASE_SPEED = 0.5
    __VELOCITY_BASE_ANGULAR = 0.8
    __VELOCITY_CMD_DURATION = 0.6

    def __init__(self, command_client):
        self.command_client = command_client

    def __execute_command(self, description, command, end_time=None):
        try:
            logging.info(f"Sending command {description}")
            self.command_client.robot_command(command, end_time)
        except RpcError:
            logging.error("Problem communicating with the Spot")
        except InvalidRequestError:
            logging.error("Invalid request")
        except NoTimeSyncError:
            logging.error("It's been too long since last time-sync")
        except NotPoweredOnError:
            logging.error("Engines are not powered")

    def __execute_velocity(self, description, v_x=0.0, v_y=0.0, v_rot=0.0):
        self.__execute_command(
            description,
            RobotCommandBuilder.synchro_velocity_command(
                v_x=v_x,
                v_y=v_y,
                v_rot=v_rot
            ),
            time.time() + self.__VELOCITY_CMD_DURATION

        )

    def sit(self):
        self.__execute_command("sit", RobotCommandBuilder.synchro_sit_command())

    def stand(self):
        self.__execute_command("stand", RobotCommandBuilder.synchro_stand_command())

    def forward(self):
        self.__execute_velocity("forward", v_x=self.__VELOCITY_BASE_SPEED)

    def backward(self):
        self.__execute_velocity("backward", v_x=-self.__VELOCITY_BASE_SPEED)

    def left(self):
        self.__execute_velocity("left", v_y=self.__VELOCITY_BASE_SPEED)

    def right(self):
        self.__execute_velocity("right", v_y=-self.__VELOCITY_BASE_SPEED)

    def rotate_left(self):
        self.__execute_velocity("rotate_left", v_rot=self.__VELOCITY_BASE_ANGULAR)

    def rotate_right(self):
        self.__execute_velocity("rotate_right", v_rot=-self.__VELOCITY_BASE_ANGULAR)