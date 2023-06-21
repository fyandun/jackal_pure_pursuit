import numpy as np
from matplotlib import pyplot as plt
import sys

files = sys.argv[1:]

for i,f in enumerate(files):
	w = np.loadtxt(f,delimiter=',')
	plt.scatter(w[0,0],w[0,1],color='g',marker='o',label='start')
	plt.scatter(w[-1,0],w[-1,1],color='r',marker='x',label='end')
	plt.scatter(w[1:-1,0],w[1:-1,1],label='file_'+str(i))
	plt.legend()


plt.show()
