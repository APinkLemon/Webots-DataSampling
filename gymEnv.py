import sys
import time
import numpy as np
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
        self.robot = self.supervisor.getSelf()
        self.episodeScore = 0
        self.episodeScoreList = []

    def respawnRobot(self):
        if self.robot is not None:
            print("I'm Removing!")
            self.robot.remove()

        self.supervisor.simulationResetPhysics()

        rootNode = self.supervisor.getRoot()
        childrenField = rootNode.getField('children')
        childrenField.importMFNode(-2, "BmwX5.wbo")

        self.robot = self.supervisor.getSelf()

    def get_observations(self):

        return [0, 0, 0, 0]

    def get_reward(self, action=None):
        return 1

    def is_done(self):
        if self.episodeScore > 195.0:
            return True

        return False

    def solved(self):
        if len(self.episodeScoreList) > 100:
            if np.mean(self.episodeScoreList[-100:]) > 195.0:
                return True
        return False

    def get_default_observation(self):
        return [0.0 for _ in range(self.observation_space.shape[0])]

    def apply_action(self, action):
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
        print("This is respawnRobot!")
        self.respawnRobot()
        self.supervisor.simulationResetPhysics()
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
        # print(env.supervisor.getCurrentSpeed() if abs(env.supervisor.getCurrentSpeed()) > 0.1 else 0)
        _, _, _, _ = env.step(10)
        time.sleep(0.02)
    episodeCount += 1
    env.reset()
