import numpy as np
import sys

option = sys.argv[1]
f = sys.argv[2]
w = np.loadtxt(f,delimiter=',')

if option == '-r': # reverse
	w = w[::-1]
	np.savetxt(f[:-4]+'_reversed.txt',w,delimiter=',')
elif option == '-rot': # rotate
	w = w[::-1]
	w = np.roll(w,len(w)-2,axis=0)
	np.savetxt(f[:-4]+'_rolled.txt',w,delimiter=',')
else: # shift
	# w[:,0] += 0.4 # shift x
	w[:,1] -= 0.4 # shift y
	np.savetxt(f[:-4]+'_shifted.txt',w,delimiter=',')
