import gym
from mlagents_envs.environment import UnityEnvironment
from gym_unity.envs import UnityToGymWrapper
import numpy as np
# from stable_baselines3.common.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import PPO, DDPG
from stable_baselines3.common.monitor import Monitor
import os
import time
import pickle

from mlagents_envs.environment import UnityEnvironment
from gym_unity.envs import UnityToGymWrapper

import torch

model_name = 'stable_results\ppo\\raghavruns\\final_runs\\a_ddpg'
# model = PPO.load(model_name)
model = DDPG.load(model_name)
# model = PPO.load('just_fly')

unity_env = UnityEnvironment("Eval_Build/ArcadeJetFlightExample", worker_id=3, no_graphics=True)
env = UnityToGymWrapper(unity_env, uint8_visual=False) 

num_success = 0
num_total = 1000
for episode in range(num_total):
    initial_observation = env.reset()
    done = False
    episode_rewards = 0
    while not done:
        action, _ = model.predict(initial_observation)
        observation, reward, done, info = env.step(action)
        # print('observation', observation)
        # print('action', action)
        episode_rewards += reward

    if episode_rewards == 1:
        num_success += 1

    # print("Total reward this episode: {}".format(episode_rewards))

ep_success = num_success/num_total
print('model_name', model_name)
print('ep_success', ep_success)

env.close()