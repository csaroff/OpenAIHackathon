#!/usr/bin/env python3
import gym
import argparse

def train(env_id, num_timesteps, seed):
	from baselines.ppo1 import pposgd_simple, mlp_policy
	import baselines.common.tf_util as U
	U.make_session(num_cpu=1).__enter__()

	def policy_fn(name, ob_space, ac_space):
		return mlp_policy.MlpPolicy(name=name, ob_space=ob_space, ac_space=ac_space,
			hid_size=64, num_hid_layers=2)

	env = gym.make(env_id)
	env.seed(seed)

	pposgd_simple.learn(env, policy_fn,
		max_timesteps=num_timesteps,
		timesteps_per_actorbatch=2048,
		clip_param=0.2, entcoeff=0.0,
		optim_epochs=10, optim_stepsize=3e-4, optim_batchsize=64,
		gamma=0.99, lam=0.95, schedule='linear')

	env.close()

def main():

	parser = arg_parser()
	parser.add_argument('--seed', default=0, help='random seed')
	parser.add_argument('--num-timesteps', default=int(10e6), help='number of timesteps for training')
	args = parser.parse_args()

	env_id = "Aero-v0"

	train(env_id,num_timesteps=args.num_timesteps, seed=args.seed)