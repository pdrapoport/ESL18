#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import pylab
from scipy.signal import butter, lfilter, freqz

def butter_lowpass(cutoff, fs, order):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order):
    b, a = butter_lowpass(cutoff, fs, order)
    y = lfilter(b, a, data)
    return y


# Filter requirements.
order = 2
fs = 100       # sample rate, Hz
cutoff = 10  # desired cutoff frequency of the filter, Hz

# # Get the filter coefficients so we can check its frequency response.
# b, a = butter_lowpass(cutoff, fs, order)

# Plot the frequency response.
# w, h = freqz(b, a, worN=8000)
# plt.figure()
# plt.plot(0.5*fs*w/np.pi, np.abs(h), 'b')
# plt.plot(cutoff, 0.5*np.sqrt(2), 'ko')
# plt.axvline(cutoff, color='k')
# plt.xlim(0, 0.5*fs)
# plt.title("Lowpass Filter Frequency Response")
# plt.xlabel('Frequency [Hz]')
# plt.grid()

def mean(numbers):
    return float(sum(numbers)) / max(len(numbers), 1)

fh = open( "pc_terminal/file.txt" );

x = []
t = []
axis_x = []
#read line into array
for line in fh.readlines():
    # add a new sublist
    x.append([])
    # loop over the elemets, split by whitespace
    for i in line.split():
        # convert to integer and append to the last
        # element of the list
        x[-1].append(int(i))

fh.close()

for i in range(0, len(x)):
    t.append(x[i][0])
    axis_x.append(i)


print np.mean(t)
plt.figure()
plt.plot(axis_x, t,linewidth=0.75, label='execution time of processPkt')
plt.legend()

plt.show()
