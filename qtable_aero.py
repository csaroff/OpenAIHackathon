
import numpy as np
from QuanserAero.aero_env import AeroEnv
import math

env = AeroEnv(serial_port_path="/dev/ttyACM0")
# Q = np.zeros([env.observation_space.n,env.action_space.n])
action_space_size = 3*3
observation_space_size = 5*5

Q = np.zeros([observation_space_size, action_space_size])
lr = .8
num_episodes = 2000
y = .95

def to_discrete_state(state):
	print(state)
	p, y, dp, dy = state
	p = int(max(-2, min(2, p/10))) + 2
	y = int(max(-2, min(2, y/10))) + 2
	return p*5 + y

def to_continous_action(action):
	p = action % 3
	y = action / 3
	motor0_voltage = (p - 1) * 24
	motor1_voltage = (y - 1) * 24
	action = motor0_voltage, motor1_voltage, 0, 0, 0
	return action

rList = []
for i in range(num_episodes):
	state, _, _, _ = env.reset()
	s = to_discrete_state(state)

	total_reward = 0
	d = False
	j = 0
	while j < 100:
		j+=1
		a = np.argmax(Q[s,:] + np.random.randn(1,action_space_size)*(1./(i+1)))
		
		action = to_continous_action(a)

		state, r, d, _ = env.step(action)

		s1 = to_discrete_state(state)

		Q[s,a] = Q[s,a] + lr*(r + y*np.max(Q[s1,:]) - Q[s,a])
		rAll += r
		s = s1
        if d == True:
        	break
	rList.append(total_reward)
print "Score over time: " +  str(sum(rList)/num_episodes)
print "Final Q-Table Values"
print Q