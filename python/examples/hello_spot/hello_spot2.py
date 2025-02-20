# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Tutorial to show how to use the Boston Dynamics API"""
from __future__ import print_function

import argparse
import os
import sys
import time
from datetime import datetime
import cv2
import numpy as np
from bosdyn.api import image_pb2
import bosdyn.client
from bosdyn.client import robot_state
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.client.image import ImageClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit
from movement_helper import MovementHelper


class MyRobot():
    _myrobot = None
    # COMBINATIONS = [
    #     [keyboard.KeyCode(char='a')],
    #     [keyboard.KeyCode(char='w')],
    #     [keyboard.KeyCode(char='s')],
    #     [keyboard.KeyCode(char='d')],
    #     [keyboard.KeyCode(char='z')]
    # ]
    # current = set()
    # threads = []

    def hello_spot(self, config):
        """A simple example of using the Boston Dynamics API to command a Spot robot."""

        # The Boston Dynamics Python library uses Python's logging module to
        # generate output. Applications using the library can specify how
        # the logging information should be output.
        bosdyn.client.util.setup_logging(config.verbose)

        # The SDK object is the primary entry point to the Boston Dynamics API.
        # create_standard_sdk will initialize an SDK object with typical default
        # parameters. The argument passed in is a string identifying the client.
        sdk = bosdyn.client.create_standard_sdk('HelloSpotClient')

        # A Robot object represents a single robot. Clients using the Boston
        # Dynamics API can manage multiple robots, but this tutorial limits
        # access to just one. The network address of the robot needs to be
        # specified to reach it. This can be done with a DNS name
        # (e.g. spot.intranet.example.com) or an IP literal (e.g. 10.0.63.1)
        self.robot = sdk.create_robot(config.hostname)
        self._myrobot = self.robot
        # Clients need to authenticate to a robot before being able to use it.
        bosdyn.client.util.authenticate(self.robot)

        # Establish time sync with the robot. This kicks off a background thread to establish time sync.
        # Time sync is required to issue commands to the robot. After starting time sync thread, block
        # until sync is established.
        self.robot.time_sync.wait_for_sync()

        # Verify the robot is not estopped and that an external application has registered and holds
        # an estop endpoint.
        assert not self.robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                        "such as the estop SDK example, to configure E-Stop."

        # Only one client at a time can operate a robot. Clients acquire a lease to
        # indicate that they want to control a robot. Acquiring may fail if another
        # client is currently controlling the robot. When the client is done
        # controlling the robot, it should return the lease so other clients can
        # control it. The LeaseKeepAlive object takes care of acquiring and returning
        # the lease for us.
        self.lease_client = self.robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        with bosdyn.client.lease.LeaseKeepAlive(self.lease_client, must_acquire=True, return_at_exit=True):
            # Now, we are ready to power on the robot. This call will block until the power
            # is on. Commands would fail if this did not happen. We can also check that the robot is
            # powered at any point.
            self.robot.logger.info("Powering on robot... This may take several seconds.")
            self.robot.power_on(timeout_sec=20)
            assert self.robot.is_powered_on(), "Robot power on failed."
            self.robot.logger.info("Robot powered on.")

            # Tell the robot to stand up. The command service is used to issue commands to a robot.
            # The set of valid commands for a robot depends on hardware configuration. See
            # RobotCommandBuilder for more detailed examples on command building. The robot
            # command service requires timesync between the robot and the client.
            self.robot.logger.info("Commanding robot to stand...")
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            blocking_stand(command_client, timeout_sec=10)
            self.robot.logger.info("Robot standing.")
            time.sleep(3)

            # Tell the robot to stand in a twisted position.
            #
            # The RobotCommandBuilder constructs command messages, which are then
            # issued to the robot using "robot_command" on the command client.
            #
            # In this example, the RobotCommandBuilder generates a stand command
            # message with a non-default rotation in the footprint frame. The footprint
            # frame is a gravity aligned frame with its origin located at the geometric
            # center of the feet. The X axis of the footprint frame points forward along
            # the robot's length, the Z axis points up aligned with gravity, and the Y
            # axis is the cross-product of the two.
            footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.4, roll=0.0, pitch=0.0)
            cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
            command_client.robot_command(cmd)
            self.robot.logger.info("Robot standing twisted.")
            time.sleep(3)

            # Now tell the robot to stand taller, using the same approach of constructing
            # a command message with the RobotCommandBuilder and issuing it with
            # robot_command.
            cmd = RobotCommandBuilder.synchro_stand_command(body_height=0.1)
            command_client.robot_command(cmd)
            self.robot.logger.info("Robot standing tall.")
            time.sleep(3)

            # Capture an image.
            # Spot has five sensors around the body. Each sensor consists of a stereo pair and a
            # fisheye camera. The list_image_sources RPC gives a list of image sources which are
            # available to the API client. Images are captured via calls to the get_image RPC.
            # Images can be requested from multiple image sources in one call.
            image_client = self.robot.ensure_client(ImageClient.default_service_name)
            sources = image_client.list_image_sources()
            image_response = image_client.get_image_from_sources(['frontleft_fisheye_image'])
            self._maybe_display_image(image_response[0].shot.image)
            if config.save or config.save_path is not None:
                self._maybe_save_image(image_response[0].shot.image, config.save_path)

            # custom image data collection
            #todo drive spot and collect data
            try:
                delay=500
                keep_going = True
                _state_client = self.robot.ensure_client(robot_state.RobotStateClient.default_service_name)
                battery = _state_client.get_robot_state().battery_states[0].charge_percentage.value
                count = 0
                while keep_going:
                    # keyboard.wait("1")
                    # with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
                    #     listener.join()
                    command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
                    if battery < .20:
                        keep_going = False
                    image_client = self.robot.ensure_client(ImageClient.default_service_name)
                    sources = image_client.list_image_sources()
                    print(sources)
                    self.collectData(image_client)
                    

                    if count == 100:
                        keep_going = False
                        self.robot.logger.info("Commanding robot to sit...")
                        command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
                        blocking_sit(command_client, timeout_sec=10)
                        self.robot.logger.info("Robot sitting.")
                        time.sleep(3)
                    
            except Exception as e:
                print(e)

            # Log a comment.
            # Comments logged via this API are written to the robots test log. This is the best way
            # to mark a log as "interesting". These comments will be available to Boston Dynamics
            # devs when diagnosing customer issues.
            log_comment = "HelloSpot tutorial user comment."
            self.robot.operator_comment(log_comment)
            self.robot.logger.info('Added comment "%s" to robot log.', log_comment)

            # Power the robot off. By specifying "cut_immediately=False", a safe power off command
            # is issued to the robot. This will attempt to sit the robot before powering off.
            self.robot.power_off(cut_immediately=False, timeout_sec=20)
            assert not self.robot.is_powered_on(), "Robot power off failed."
            self.robot.logger.info("Robot safely powered off.")

    # def execute(self, key_pressed):
    #     if key_pressed == "s":
    #         self._myrobot.logger.info("Commanding robot to stand...")
    #         command_client = self._myrobot.ensure_client(RobotCommandClient.default_service_name)
    #         blocking_stand(command_client, timeout_sec=10)
    #         self._myrobot.logger.info("Robot standing.")
    #         time.sleep(3)
    #     # if key_pressed == "q":
    #     #     self.robot.logger.info("Commanding robot to sit...")
    #     #     command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
    #     #     blocking_sit(command_client, timeout_sec=10)
    #     #     self.robot.logger.info("Robot sitting.")
    #     #     time.sleep(3)

    # def on_press(self, key):
    #     if any([key in COMBO for COMBO in self.COMBINATIONS]):
    #         self.current.add(key)
    #         print('{0} pressed'.format(key))
    #         if any(all(k in self.current for k in COMBO) for COMBO in self.COMBINATIONS):
    #             self.execute(key)


    # def on_release(self, key):
    #     print('{0} release'.format(key))
    #     if key == Key.esc:
    #         # Stop listener
    #         return False
    #     if any([key in COMBO for COMBO in self.COMBINATIONS]):
    #         self.current.remove(key)

    # with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    #     listener.join()


    def _maybe_display_image(self, image, display_time=3.0):
        """Try to display image, if client has correct deps."""
        try:
            import io

            from PIL import Image
        except ImportError:
            logger = bosdyn.client.util.get_logger()
            logger.warning("Missing dependencies. Can't display image.")
            return
        try:
            image = Image.open(io.BytesIO(image.data))
            image.show()
            time.sleep(display_time)
        except Exception as exc:
            logger = bosdyn.client.util.get_logger()
            logger.warning("Exception thrown displaying image. %r", exc)


    def _maybe_save_image(self, image, path, camera = 'fisheye-right'):
        """Try to save image, if client has correct deps."""
        logger = bosdyn.client.util.get_logger()
        try:
            import io

            from PIL import Image
        except ImportError:
            logger.warning("Missing dependencies. Can't save image.")
            return
        working_dir = os.getcwd()
        _path = working_dir + path
        if path is not None and os.path.exists(_path):
            now = datetime.now()
            date_time = datetime.timestamp(now)
            name = str(date_time) + ".jpg"
            name = os.path.join(_path, name)
            logger.info("Saving image to: {}".format(name))
        else:
            logger.info("Saving image to working directory as {}".format(name))
        try:
            image = Image.open(io.BytesIO(image.data))
            image.save(name)
        except Exception as exc:
            logger = bosdyn.client.util.get_logger()
            logger.warning("Exception thrown saving image. %r", exc)

    def collectData(self, image_client):
        logger = bosdyn.client.util.get_logger()
        __sources = ['frontright_fisheye_image', 'frontleft_fisheye_image']

        try:
            image_responses = image_client.get_image_from_sources(
                __sources)
            for image_response in image_responses:
                path = "/python/examples/hello_spot/camera_images"
                path = os.path.join(os.getcwd(), path)

                if image_response.source.name == 'frontright_fisheye_image':
                    path = path + '/frontright_fisheye_image'
                elif image_response.source.name == 'frontleft_fisheye_image':
                    path = path + '/frontleft_fisheye_image'
                else:
                    path = path + '/unknown'
                logger.info("Saving image to: {}".format(path))
                self._maybe_save_image(image_response.shot.image, path, image_response.source.name)
        except Exception as e:
            print(e)

def main(argv):
    """Command line interface."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument(
        '-s', '--save', action='store_true', help=
        'Save the image captured by Spot to the working directory. To chose the save location, use --save_path instead.'
    )
    parser.add_argument(
        '--save-path', default=None, nargs='?', help=
        'Save the image captured by Spot to the provided directory. Invalid path saves to working directory.'
    )
    options = parser.parse_args(argv)
    try:
        myrobot = MyRobot()
        myrobot.hello_spot(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False

if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
