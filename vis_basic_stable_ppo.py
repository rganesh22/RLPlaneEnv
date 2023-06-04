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

if __name__ == "__main__":
    # env_path = <PATH TO RLPlaneEnv>
    # env_path = "/Users/anwesha/Documents/Stanford/cs-stanford/cs224r/RLPlaneEnv"
    # env = UnityEnv(env_path, worker_id=2, use_visual=True)

    unity_env = UnityEnvironment("Build/ArcadeJetFlightExample")
    # unity_env = UnityEnvironment("Build/ArcadeJetFlightExample")
    env = UnityToGymWrapper(unity_env, uint8_visual=False) 

    # Create log dir
    time_int = int(time.time())
    log_dir = "stable_results/ppo/{}".format(time_int)
    os.makedirs(log_dir, exist_ok=True)
    env = Monitor(env, log_dir, allow_early_resets=True)

    env = DummyVecEnv([lambda: env])  # The algorithms require a vectorized environment to run

    # model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)
    # model.learn(total_timesteps=10000)

    with open(r"stable_results/_eval.pkl", "rb") as input_file:
        model = pickle.load(input_file)

    #evaluate agent
    episodes = 100
    ep_r = []
    ep_l = []
    for e in range(episodes):
        obs = env.reset()
        total_r = 0.
        total_l = 0.
        while True:
            action, _states = model.predict(obs)
            obs, reward, done, info = env.step(action)
            # if e < 20:
            #     print(f'Observation: {obs} \n')
            #     print(f'Action: {action} \n\n')
            total_l += 1.
            total_r += reward
            if done:
                break
        ep_r.append(total_r)
        ep_l.append(total_l)
    print("episode mean reward: {:0.3f} mean length: {:0.3f}".format(np.mean(ep_r), np.mean(ep_l)))
    with open('{}_eval.pkl'.format(log_dir), 'wb') as f:
        pickle.dump(ep_r, f)
        pickle.dump(ep_l, f)

    env.close()
    model.save(log_dir+"model")