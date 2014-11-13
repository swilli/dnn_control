import math
import numpy
from scipy.integrate import odeint

def deriv(y,t):
	print("deriv")
	return math.sqrt(y)*2

time = [1,4]
yinit = 1
y = odeint(deriv,yinit,time)
print(y)

#for i in range(10):
#	print(numpy.random.normal(0,1))