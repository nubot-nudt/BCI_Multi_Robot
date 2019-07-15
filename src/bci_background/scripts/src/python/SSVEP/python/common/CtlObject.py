# BCI control objects
# contributors: Liu Yang (gloolar@gmail.com)

__all__ = [
	'pendulum', 'ball_1d_pos',
]

from random import random

class pendulum():

	def __init__(self, m_base, m_ball, len_stick):
		self.data = {'xpos': 0.0, 'angle': 0.0, 'angle_degree': 0.0, 'm_base': m_base, 'len_stick': len_stick, 'm_ball': m_ball}
	
	def __getitem__(self, key): return self.data[key]
	
	def __setitem__(self, key, value): self.data[key] = value
		
	def reset(self):
		self['xpos'] = 0.0
		self['angle'] = 0.0
		self['angle_degree'] = 0.0
		self.__dx = 0
		self.__dtheta = 0.2*(random() - 0.5) # initial disturbance

		
	def step(self, timeStep, force=0.0):
		M = self['m_base']
		m = self['m_ball']
		l = self['len_stick']
		
		ddx = m*9.8/M*self['angle'] + force/M
		ddtheta = (M+m)*9.8/(M*l)*self['angle'] + force/(M*l)

		self['xpos'] += self.__dx * timeStep
		self.__dx += ddx * timeStep
		self['angle'] += self.__dtheta * timeStep
		self['angle_degree'] = self['angle']*180.0/pi
		self.__dtheta += ddtheta * timeStep
		

class ball_1d_pos():

	def __init__(self):
		self.state = {'xpos': 0.0}
	
	def __getitem__(self, key): return self.state[key]
	
	def __setitem__(self, key, value): self.state[key] = value
		
	def reset(self):
		self['xpos'] = 0.0
		self.__speed = 10
		
	def set_speed(self, speed):
		self.__speed = speed
		
	def step(self, timeStep, force=0.0):
		self['xpos'] += self.__speed * timeStep * 2*(int(force>0)-0.5)
		print 'dxpos: ', self.__speed * timeStep * 2*(int(force>0)-0.5)
		