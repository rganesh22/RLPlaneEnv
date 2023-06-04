import gym
import imageio
from mlagents_envs.environment import UnityEnvironment
from gym_unity.envs import UnityToGymWrapper
import numpy as np
# from stable_baselines3.common.policies import MlpPolicy
from stable_baselines3.common.vec_env import VecVideoRecorder, DummyVecEnv
from stable_baselines3 import PPO, DDPG, DQN
from stable_baselines3.common.monitor import Monitor
import os
import time
import pickle
from pathlib import Path

if __name__ == "__main__":
    # env_path = <PATH TO RLPlaneEnv>
    # env_path = "/Users/anwesha/Documents/Stanford/cs-stanford/cs224r/RLPlaneEnv"
    # env = UnityEnv(env_path, worker_id=2, use_visual=True)

    

    # Create log dir
    # time_int = int(time.time())
    # Change logdir if you want to make it "no grounding etc." for the kind of reward function we're testing
    # [no_grounding, [point2_target_distance_square, point5_target_distance_linear, etc etc]
    reward_func = "just_fly_plus_highweight_target"
    # log_dir = f"stable_results/ddpg/{reward_func}/{time_int}"
    for lr in [5e-3, 1e-2]:
        if lr == 5e-3: lr_str = "5e-3" 
        else: lr_str = "1e-2"
        for gamma in [0.99, 0.95]:
            gamma_str = f"point{gamma * 100}"
            for batch_size in [64, 32]:
                unity_env = UnityEnvironment("Build/ArcadeJetFlight", no_graphics=True, worker_id=2)
                # unity_env = UnityEnvironment("Build/ArcadeJetFlightExample", worker_id=2)
                env = UnityToGymWrapper(unity_env, uint8_visual=False) 
                log_dir = f"stable_results/ppo/anwesharuns/{reward_func}/lr{lr_str}_gamma{gamma_str}_batch{batch_size}/"
                os.makedirs(log_dir, exist_ok=True)
                env = Monitor(env, log_dir, allow_early_resets=True)
                env = DummyVecEnv([lambda: env])  # The algorithms require a vectorized environment to run
                model = PPO("MlpPolicy", env, n_steps=512, learning_rate=lr, gamma=gamma, batch_size=batch_size, verbose=1, tensorboard_log=log_dir)
                model.learn(total_timesteps=50000)
                model.save(log_dir+"/model")
                env.close()
                time.sleep(60) # give ports time to clear up

                eval_unity_env = UnityEnvironment("Eval_Build/ArcadeJetFlight", no_graphics=False)
                eval_env = UnityToGymWrapper(eval_unity_env, uint8_visual=False) 
                episodes = 100
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
                        total_r += reward
                        if done:
                            break
                        with open(longfile, 'a') as file1:
                            file1.write(f"Episode: {e}, Observation: {obs}, Action: {action}, Action Reward: {reward}, Total Reward: {total_r}, Current Total Length: {total_l} \n")
                    ep_r.append(total_r)
                    ep_l.append(total_l)
                    with open(logfile, 'a') as file2:
                        file2.write(f"Episode: {e}, Total Reward: {total_r}, Total Length: {total_l} \n")
                # print("episode mean reward: {:0.3f} mean length: {:0.3f}".format(np.mean(ep_r), np.mean(ep_l)))
                with open('{}_eval.pkl'.format(log_dir), 'wb') as f:
                    pickle.dump(ep_r, f)
                    pickle.dump(ep_l, f)
                
                images = []
                obs = model.env.reset()
                img = model.env.render(mode='rgb_array')
                for i in range(350):
                    images.append(img)
                    action, _ = model.predict(obs)
                    obs, _, _ ,_ = model.env.step(action)
                    img = model.env.render(mode='rgb_array')

                imageio.mimsave(log_dir+'/sample.gif', [np.array(img) for i, img in enumerate(images) if i%2 == 0], fps=29)
                eval_env.close()
                time.sleep(60)

    # model = PPO.load('just_fly')
    # model.set_env(env)
    # model_list = [
    #     PPO("MlpPolicy", env, n_steps=512, verbose=1, tensorboard_log=log_dir), 
    #     PPO("MlpPolicy", env, n_steps=512, learning_rate=1e-2, verbose=1, tensorboard_log=log_dir),
    #     PPO("MlpPolicy", env, n_steps=512, learning_rate=1e-2, gamma=0.95, verbose=1, tensorboard_log=log_dir),
    #     PPO("MlpPolicy", env, n_steps=512, learning_rate=5e-3, gamma=0.95, clip_range=0.5, verbose=1, tensorboard_log=log_dir),
    #     PPO("MlpPolicy", env, n_steps=512, learning_rate=1e-2, batch_size=32, verbose=1, tensorboard_log=log_dir)
    # ]

    # for model in model_list:
    #     model.learn(total_timesteps=50000)
    #     model.save(log_dir+f"{model.learning_rate()}_{model.gamma()}/model")

    #     eval_unity_env = UnityEnvironment("Eval_Build/ArcadeJetFlight", no_graphics=False)
    #     eval_env = UnityToGymWrapper(eval_unity_env, uint8_visual=False) 

    #     episodes = 100
    #     ep_r = []
    #     ep_l = []
    #     longfile = Path(f"{log_dir}/longlogs.txt")
    #     longfile.touch(exist_ok=True)
    #     logfile = Path(f"{log_dir}/readlogs.txt")
    #     logfile.touch(exist_ok=True)
    #     print(logfile)
    #     print(longfile)
    #     for e in range(episodes):
    #         obs = eval_env.reset()
    #         total_r = 0.
    #         total_l = 0.
    #         while True:
    #             action, _states = model.predict(obs)
    #             obs, reward, done, info = eval_env.step(action)
    #             # if e < 20:
    #             #     print(f'Observation: {obs} \n')
    #             #     print(f'Action: {action} \n\n')
    #             total_l += 1.
    #             total_r += reward
    #             if done:
    #                 break
    #             with open(longfile, 'a') as file1:
    #                 file1.write(f"Episode: {e}, Observation: {obs}, Action: {action}, Action Reward: {reward}, Total Reward: {total_r}, Current Total Length: {total_l} \n")
    #         ep_r.append(total_r)
    #         ep_l.append(total_l)
    #         with open(logfile, 'a') as file2:
    #             file2.write(f"Episode: {e}, Total Reward: {total_r}, Total Length: {total_l} \n")
    #     # print("episode mean reward: {:0.3f} mean length: {:0.3f}".format(np.mean(ep_r), np.mean(ep_l)))
    #     with open('{}_eval.pkl'.format(log_dir), 'wb') as f:
    #         pickle.dump(ep_r, f)
    #         pickle.dump(ep_l, f)
        
    #     images = []
    #     obs = model.env.reset()
    #     img = model.env.render(mode='rgb_array')
    #     for i in range(350):
    #         images.append(img)
    #         action, _ = model.predict(obs)
    #         obs, _, _ ,_ = model.env.step(action)
    #         img = model.env.render(mode='rgb_array')

    #     imageio.mimsave('.gif', [np.array(img) for i, img in enumerate(images) if i%2 == 0], fps=29)

    # model = PPO("MlpPolicy", env, n_steps=512, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, verbose=1, tensorboard_log=log_dir)    
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-2, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-2, gamma=0.95, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-2, gamma=0.95, clip_range=0.5, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-2, batch_size=32, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-2, batch_size=32, clip_range=0.5, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-2, batch_size=16, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-1, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-1, gamma=0.95, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-1, batch_size=32, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-1, batch_size=16, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, batch_size=32, verbose=1, tensorboard_log=log_dir)

    # model = PPO("CnnPolicy", env, n_steps=500, verbose=1, tensorboard_log=log_dir)

    # model = DDPG("MlpPolicy", env, learning_rate=1e-2, verbose=1, tensorboard_log=log_dir)
    # model = DDPG("CnnPolicy", env, verbose=1, tensorboard_log=log_dir)

    # model = DQN("MlpPolicy", env, n_steps=500, verbose=1, tensorboard_log=log_dir)
    # model = DQN("CnnPolicy", env, n_steps=500, verbose=1, tensorboard_log=log_dir)
    
    # # model.learn(total_timesteps=100000)
    # model.learn(total_timesteps=100000)
    
    # model.save(log_dir+"/model")
    # # model.save("latest_model")

    # #evaluate agent
    # eval_unity_env = UnityEnvironment("Eval_Build/ArcadeJetFlightExample", no_graphics=False)
    # eval_env = UnityToGymWrapper(eval_unity_env, uint8_visual=False) 

    # episodes = 100
    # ep_r = []
    # ep_l = []
    # longfile = Path(f"{log_dir}/longlogs.txt")
    # longfile.touch(exist_ok=True)
    # logfile = Path(f"{log_dir}/readlogs.txt")
    # logfile.touch(exist_ok=True)
    # print(logfile)
    # print(longfile)
    # for e in range(episodes):
    #     obs = eval_env.reset()
    #     total_r = 0.
    #     total_l = 0.
    #     while True:
    #         action, _states = model.predict(obs)
    #         obs, reward, done, info = eval_env.step(action)
    #         # if e < 20:
    #         #     print(f'Observation: {obs} \n')
    #         #     print(f'Action: {action} \n\n')
    #         total_l += 1.
    #         total_r += reward
    #         if done:
    #             break
    #         with open(longfile, 'a') as file1:
    #             file1.write(f"Episode: {e}, Observation: {obs}, Action: {action}, Action Reward: {reward}, Total Reward: {total_r}, Current Total Length: {total_l} \n")
    #     ep_r.append(total_r)
    #     ep_l.append(total_l)
    #     with open(logfile, 'a') as file2:
    #         file2.write(f"Episode: {e}, Total Reward: {total_r}, Total Length: {total_l} \n")
    # # print("episode mean reward: {:0.3f} mean length: {:0.3f}".format(np.mean(ep_r), np.mean(ep_l)))
    # with open('{}_eval.pkl'.format(log_dir), 'wb') as f:
    #     pickle.dump(ep_r, f)
    #     pickle.dump(ep_l, f)

    env.close()
    eval_env.close()
    # model.save(log_dir+"/model")
    # model.save("latest_model")