import gym
from mlagents_envs.environment import UnityEnvironment
from gym_unity.envs import UnityToGymWrapper
import numpy as np
# from stable_baselines3.common.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
import os
import time
import pickle

from mlagents_envs.environment import UnityEnvironment
from gym_unity.envs import UnityToGymWrapper

import torch

model = PPO.load('stable_results\\ppo\\1685762489model.zip')

unity_env = UnityEnvironment("Build/ArcadeJetFlightExample")
env = UnityToGymWrapper(unity_env, uint8_visual=False) 

for episode in range(10):
    initial_observation = env.reset()
    done = False
    episode_rewards = 0
    while not done:
        action, _ = model.predict(initial_observation)
        observation, reward, done, info = env.step(action)
        print('observation', observation)
        print('action', action)
        episode_rewards += reward
    print("Total reward this episode: {}".format(episode_rewards))

env.close()