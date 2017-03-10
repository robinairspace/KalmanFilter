import numpy as np
import matplotlib.pyplot as plt

ifilename = "gps.csv"
input = np.genfromtxt(ifilename, delimiter=',')
ofilename = "ekf.csv"
output = np.genfromtxt(ofilename, delimiter=' ')
print(output[:,1])

plt.figure()
plt.subplot(211)
for i in range(16):
    plt.plot(input[:,i]-input[0,i])

plt.subplot(212)
plt.plot(output[:,0])
plt.plot(output[:,1])
plt.plot(output[:,2])

plt.show()
