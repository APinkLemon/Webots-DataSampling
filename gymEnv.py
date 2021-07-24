import sys
import time
import numpy as np
from config import cfg
from vehicle import Car
from gym.spaces import Box, Discrete
from controller import Lidar, Keyboard, LidarPoint, GPS, Supervisor
from deepbots.supervisor.controllers.robot_car import RobotSupervisor


class Agent():
    def __init__(self):
        super().__init__()
        self.a = 1

    def work(self, b=1):
        return self.a + b


class AutoDriveEnv(RobotSupervisor):
    def __init__(self):
        super().__init__()
        self.observation_space = Box(low=np.array([-0.4, -np.inf, -1.3, -np.inf]),
                                     high=np.array([0.4, np.inf, 1.3, np.inf]),
                                     dtype=np.float64)
        self.action_space = Discrete(2)

        # self.robot = self.supervisor.getSelf()

        self.episodeScore = 0  # Score accumulated during an episode
        self.episodeScoreList = []  # A list to save all the episode scores, used to check if task is solved

    def get_observations(self):
        # # Position on z axis
        # cartPosition = normalizeToRange(self.robot.getPosition()[2], -0.4, 0.4, -1.0, 1.0)
        # # Linear velocity on z axis
        # cartVelocity = normalizeToRange(self.robot.getVelocity()[2], -0.2, 0.2, -1.0, 1.0, clip=True)
        # # Pole angle off vertical
        # poleAngle = normalizeToRange(self.positionSensor.getValue(), -0.23, 0.23, -1.0, 1.0, clip=True)
        # # Angular velocity x of endpoint
        # endpointVelocity = normalizeToRange(self.poleEndpoint.getVelocity()[3], -1.5, 1.5, -1.0, 1.0, clip=True)

        # return [cartPosition, cartVelocity, poleAngle, endpointVelocity]
        return [0, 0, 0, 0]

    def get_reward(self, action=None):
        return 1

    def is_done(self):
        if self.episodeScore > 195.0:
            return True

        return False

    def solved(self):
        if len(self.episodeScoreList) > 100:  # Over 100 trials thus far
            if np.mean(self.episodeScoreList[-100:]) > 195.0:  # Last 100 episodes' scores average value
                return True
        return False

    def get_default_observation(self):
        return [0.0 for _ in range(self.observation_space.shape[0])]

    def apply_action(self, action):
        print("Hello!")
        self.supervisor.setCruisingSpeed(50)

    def step(self, action):

        if self.supervisor.step() == -1:
            exit()

        self.apply_action(action)
        return (
            self.get_observations(),
            self.get_reward(action),
            self.is_done(),
            self.get_info(),
        )

    def render(self, mode='human'):
        print("render()" + mode + "is not used")

    def get_info(self):
        return None

    def reset(self):
        self.supervisor.simulationReset()
        self.supervisor.simulationResetPhysics()
        while self.supervisor.step() == -1:
            # exit()
            break
        print('#' * 100)
        self.supervisor.setCruisingSpeed(50)
        print('#' * 100)
        return 0


env = AutoDriveEnv()
agent = Agent()
stepPerEpisode = 100
episodeCount = 0
episodeLimit = 10000

while episodeCount < episodeLimit:
    print("*" * 100)
    print(episodeCount)
    for step in range(stepPerEpisode):
        # print(step + 100 * episodeCount)
        print(env.supervisor.getCurrentSpeed() if abs(env.supervisor.getCurrentSpeed()) > 0.1 else 0)
        _, _, _, _ = env.step(10)
        time.sleep(0.02)
    episodeCount += 1
    env.reset()
