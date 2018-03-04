import gym
import numpy as np


env = AeroEnv()
Q = np.zeros([env.observation_space.n,env.action_space.n])
lr = .8
num_episodes = 2000
y = .95


rList = []
for i in range(num_episodes):
	s = env.reset()
	total_reward = 0
	d = False
	j = 0
	while j < 100:
		j+=1
		a = np.argmax(Q[s,:] + np.random.randn(1,env.action_space.n)*(1./(i+1)))
		s1,r,d,_ = env.step(a)
		Q[s,a] = Q[s,a] + lr*(r + y*np.max(Q[s1,:]) - Q[s,a])
		rAll += r
		s = s1
        if d == True:
        	break
	rList.append(total_reward)
print "Score over time: " +  str(sum(rList)/num_episodes)
print "Final Q-Table Values"
print Q