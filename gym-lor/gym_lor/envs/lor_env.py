from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import os
import tensorflow as tf
import numpy as np
import atexit
import gym
from gym import utils
from gym.envs.mujoco import mujoco_env
import time
from gym.spaces import Box
import rospy
from std_msgs.msg import String
from math import *
from IDM import IDMDriver
from PI import PISaturation
from FS import FollowerStopper
import time

class VirtualActuator(object):
    def __init__(self):
        print('Virtual actuator initialized.')

    @staticmethod

    def sub_callback(self,data):
        positions = data.split(',')
        positions = positions[:-1]
        for index, veh in self.vehicles.items():
            t = veh.t + veh.dt
            x = self.circle2line(float(positions[index*3+1]),float(positions[index*3+2]))
            vel = 0.0
            if x<veh.x:
                vel = (x+8.17 - veh.x)/veh.dt
            else:
                vel = (x - veh.x)/veh.dt
            self.vehicles[index].update(t, x, vel)

    def circle2line(self,x,y):
        R = sqrt(x**2+y**2)
        if x==0 and y>0:
            rad = 0.5*np.pi
        if x==0 and y<0:
            rad = 1.5*np.pi
        if y==0 and x>0:
            rad = 0.0
        if y==0 and x<0:
            rad = np.pi
        if x>0 and y>0:
            rad = atan(y/x)   
        if x<0 and y>0:
            rad = 0.5*np.pi+atan(-x/y)
        if x<0 and y<0:
            rad = np.pi+atan(x/y)
        if x>0 and y<0:
            rad = 1.5*np.pi+atan(x/-y)
        displacement = R * rad
        return displacement


    def actuate(vehicles, actions):
        self.vehicles = vehicles
        rospy.Subscriber("axis_position",String,self.sub_callback)
        return self.vehicles

    @staticmethod
    def close():
        print('Virtual actuator closed.')


class VirtualObserver(object):
    def __init__(self):
        print('Virtual observer initialized.')

    @staticmethod
    def observe(vehicles):
        obs = []
        for index in range(len(vehicles)):
            veh = vehicles[index]
            obs.append(veh.vel)
        return obs

    @staticmethod
    def close():
        print('Virtual observer closed.')


class CircleEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 2
    }

    def __init__(self, frame_skp = 5):
        env_config = {
        'dt': 0.05,  # [s]
        'duration': 20,  # [s]
        'num_veh': 20,
        'rlveh_index': 19,
        'safety_threshold': 0.1,  # [m]
        'save_fig': False,
        }
	# 0 for IDM, 1 for FS, 2 for PI
        self.env_config = env_config
        self.dt = env_config['dt']  # [s]
        self.duration = env_config['duration']  # [s]
        self.num_veh = env_config['num_veh']
        self.rlveh_index = env_config['rlveh_index']
        self.elapsed = 0  # [s]
        #self.count = 0
	self.start_time = time.time()
        self._virtual_vehicles = {}
	self.IDMcars = {}
	self.FScars = {}
	self.PIcars = {}
	# initialize three controllers
	for index in range(env_config['num_veh']):
	    self.IDMcars[index] = IDMDriver(
	        t_init=0, x_init=index*0.35, vel_init=0, dt=self.dt)
	'''if self.controller == 0:
		for index in range(env_config['num_veh']):
		    self._virtual_vehicles[index] = IDMDriver(
		        t_init=0, x_init=index*0.35, vel_init=0, dt=self.dt)'''
	self._virtual_vehicles = self.IDMcars

        self.observer = VirtualObserver()
        self.actuator = VirtualActuator()

        self.fig = None
        self.ax = None
        self.line = None
        self.line_xdata = None
        self.line_ydata = None
        self.save_fig = env_config['save_fig']
        self.observation_space = Box(low=0.0000001, high=2.9999, shape=(self.num_veh,))
        self.action_space = Box(low=0.0000001, high=4.0, shape=(1,))
        self.pub = rospy.Publisher('circleenv', String, queue_size=1)
        atexit.register(self.close)

    def _get_obs(self):
        obs = self.observer.observe(self._virtual_vehicles)
        return np.array(obs)

    def reset(self):
        self.elapsed = 0
        self._virtual_vehicles = {}
        for index in range(self.num_veh):
            self._virtual_vehicles[index] = IDMDriver(
                t_init=0, x_init=index*0.35, vel_init=0, dt=self.dt)
        obs = np.array(self.observer.observe(self._virtual_vehicles))
        return obs

    def step(self, action):
        #self.count+=1
        self.elapsed = self.elapsed + self.dt
        self.pub.publish(str(action[0]))
        actions = {}
        for index in range(len(self._virtual_vehicles)):
            veh = self._virtual_vehicles[index]
            leader = self._virtual_vehicles[(index + 1) % self.num_veh]
            max_deccel = (0 - veh.vel) / veh.dt
            accel = max(veh.step(leader), max_deccel)
            actions[index] = accel
        self._virtual_vehicles = self.actuator.actuate(
            self._virtual_vehicles, actions)
        obs = self.observer.observe(self._virtual_vehicles)
        reward = float(0.4 * np.mean(obs) - 0.6 * np.std(obs))
        if np.isnan(obs).any():
            raise ValueError('Observed NaN.')
        if self.elapsed < self.duration:
            done = False
        else:
            done = True
        # if self.count>=400:
        #     done = True
        veh_positions = np.array([
            [veh.t, veh.x]
            for index, veh in self._virtual_vehicles.items()
        ])
        if self.line_xdata is None:
            self.line_xdata = veh_positions[:, 0].tolist()
            self.line_ydata = veh_positions[:, 1].tolist()
        else:
            self.line_xdata.extend(veh_positions[:, 0].tolist())
            self.line_ydata.extend(veh_positions[:, 1].tolist())
        return np.array(obs), reward, done, {}

    def reward(self, obs, acts, next_obs):
        reward = float(0.4 * np.mean(obs) - 0.6 * np.std(obs))
        return reward

    def tf_reward(self, obs, acts, next_obs):
        reward = float(0.4 * np.mean(obs) - 0.6 * np.std(obs))
        reward = tf.convert_to_tensor(reward,np.float32)
        return reward

    def reset_from_obs(self, obs):
        assert len(obs)==len(self._virtual_vehicles)
        for i in range(len(obs)):
            self._virtual_vehicles[i].vel = obs[i]

    def reset_model(self):
        pass

    # def observation_space(self):
    #     return Box(low=-5.0, high=5.0, shape=(self.num_veh,))

    # def action_space(self):
    #     return Box(low=-5.0, high=5.0, shape=(2,))

    def render(self):
        veh_positions = np.array([
            [veh.t, veh.x]
            for index, veh in self._virtual_vehicles.items()
        ])
        if self.fig is None:
            plt.rcParams['font.family'] = ['FreeSans', 'Arial']
            self.fig = plt.figure(figsize=(8, 5))
            self.ax = self.fig.add_subplot(1, 1, 1)
            self.line, = self.ax.plot(self.line_xdata, self.line_ydata, '.b')
            self.ax.set_ylim([0, 8.17])
            self.ax.set_xlim([0, self.duration])
            self.ax.set_xlabel('Time [s]')
            self.ax.set_ylabel('Displacement [m]')
        else:
            self.line.set_data(self.line_xdata, self.line_ydata)
        plt.pause(0.001)

    def close(self):
        # Stop ROS. Replace / recharge batteries.
        timestamp = time.strftime('%Y-%m-%d-%H%M%S')
        if self.save_fig:
            if self.fig is not None:
                plt.savefig('rlexp-{}.png'.format(timestamp))
                plt.close(self.fig)
            else:
                plt.rcParams['font.family'] = ['FreeSans', 'Arial']
                self.fig = plt.figure(figsize=(8, 5))
                self.ax = self.fig.add_subplot(1, 1, 1)
                self.ax.plot(self.line_xdata, self.line_ydata, '.b')
                self.ax.set_ylim([0, 8.17])
                self.ax.set_xlim([0, self.duration])
                self.ax.set_xlabel('Time [s]')
                self.ax.set_ylabel('Displacement [m]')
                plt.savefig('rlexp-{}.png'.format(timestamp))
                plt.close(self.fig)
        self.observer.close()
        self.actuator.close()

