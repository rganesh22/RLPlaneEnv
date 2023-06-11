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
from pathlib import Path

if __name__ == "__main__":
    eval_unity_env = UnityEnvironment("Eval_Build/ArcadeJetFlightExample", worker_id=2, no_graphics=True)
    eval_env = UnityToGymWrapper(eval_unity_env, uint8_visual=False) 
    model_name = "stable_results/ppo/raghavruns/final_runs/a_3_step_tog_ppo.zip"
    reward_func = "a_3_step_tog_ppo"
    log_dir = f"stable_results/ppo/raghavruns/final_runs/{reward_func}/"
    os.makedirs(log_dir, exist_ok=True)
    model = PPO.load(model_name, env=eval_env, verbose=1, tensorboard_log=log_dir)

    episodes = 100
    success_counter = 0
    ep_r = []
    ep_l = []
    longfile = Path(f"{log_dir}/longlogs.txt")
    longfile.touch(exist_ok=True)
    logfile = Path(f"{log_dir}/readlogs.txt")
    logfile.touch(exist_ok=True)
    print(logfile)
    print(longfile)
    for e in range(episodes):
        obs = eval_env.reset()
        total_r = 0.
        total_l = 0.
        while True:
            action, _states = model.predict(obs)
            obs, reward, done, info = eval_env.step(action)
            total_l += 1.
            if reward == 1:
                total_r = 1
                success_counter += 1
            else:
                total_r += reward
            if done:
                break
            with open(longfile, 'a') as file1:
                file1.write(f"Episode: {e}\n Observation:\n {obs}\n Action:\n {action}\n Action Reward: {reward}, Total Reward: {total_r}, Current Total Length: {total_l} \n\n")
        ep_r.append(total_r)
        ep_l.append(total_l)
        with open(logfile, 'a') as file2:
            file2.write(f"Episode: {e}, Total Reward: {total_r}, Total Length: {total_l} \n")
            if e == episodes - 1:
                file2.write("Final Episode Success Rate: {:0.3f}".format(success_counter/episodes))
    print("Episode Mean Reward: {:0.3f}, Mean Length: {:0.3f}, Success Rate: {:0.3f}".format(np.mean(ep_r), np.mean(ep_l), (success_counter/episodes)))
    with open('{}_eval.pkl'.format(log_dir), 'wb') as f:
        pickle.dump(ep_r, f)
        pickle.dump(ep_l, f)

    eval_env.close()
    # model.save(log_dir+"/model")
    # model.save("latest_model")