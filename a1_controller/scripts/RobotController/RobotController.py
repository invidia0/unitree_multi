#!/usr/bin/evn python3
#Author: lnotspotl

from locale import currency
import numpy as np
import tf

from . StateCommand import State, Command, BehaviorState
from . RestController import RestController
from . TrotGaitController import TrotGaitController
from . CrawlGaitController import CrawlGaitController
from . StandController import StandController

class Robot(object):
    def __init__(self, body, legs, imu):
        self.THRESHOLD = 0.02
        self.body = body
        self.legs = legs

        self.delta_x = self.body[0] * 0.5
        self.delta_y = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front = 0.02
        self.x_shift_back = -0.0
        self.default_height = 0.25

        self.trotGaitController = TrotGaitController(self.default_stance,
            stance_time = 0.04, swing_time = 0.18, time_step = 0.02,
            use_imu = imu)

        # self.crawlGaitController = CrawlGaitController(self.default_stance,
        #     stance_time = 0.55, swing_time = 0.45, time_step = 0.02)
            
        self.standController = StandController(self.default_stance)

        self.restController = RestController(self.default_stance)

        self.currentController = self.restController
        self.state = State(self.default_height)
        self.state.foot_locations = self.default_stance
        self.command = Command(self.default_height)

    def change_controller(self):
        
        if self.command.trot_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.TROT
                self.currentController = self.trotGaitController
                self.currentController.pid_controller.reset()
                self.state.ticks = 0
            self.command.trot_event = False

        elif self.command.crawl_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.CRAWL
                self.currentController = self.crawlGaitController
                self.currentController.first_cycle = True
                self.state.ticks = 0
            self.command.crawl_event = False

        elif self.command.stand_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.STAND
                self.currentController = self.standController
                self.state.body_local_position[2] = 0.08 * 0.25
            self.command.stand_event = False

        elif self.command.rest_event:
            self.state.behavior_state = BehaviorState.REST
            self.currentController = self.restController
            self.currentController.pid_controller.reset()
            self.command.rest_event = False

    def vel_command(self,msg):
        if (abs(msg.linear.x) > self.THRESHOLD or abs(msg.linear.y) > self.THRESHOLD or abs(msg.angular.z) > self.THRESHOLD):
            print("Moving...")
            self.command.trot_event = True
            self.command.rest_event = False
        else:
            print("Resting...")
            self.command.trot_event = False
            self.command.rest_event = True

        self.currentController.updateStateCommand(msg, self.state, self.command)

    def imu_orientation(self,msg):
        q = msg.orientation
        rpy_angles = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.state.imu_roll = rpy_angles[0]
        self.state.imu_pitch = rpy_angles[1]

    def run(self):
        return self.currentController.run(self.state, self.command)

    @property
    def default_stance(self):
        # FR, FL, RR, RL
        return np.array([[self.delta_x + self.x_shift_front,self.delta_x + self.x_shift_front,-self.delta_x + self.x_shift_back,-self.delta_x + self.x_shift_back],
                         [-self.delta_y                    ,self.delta_y                     ,-self.delta_y                    , self.delta_y                    ],
                         [0                                ,0                                ,0                                ,0                                ]])
