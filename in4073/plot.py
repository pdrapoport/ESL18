#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import pylab
from scipy.signal import butter, lfilter, freqz
from scipy.fftpack import fft

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
fs = 500       # sample rate, Hz
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

fh = open( "pc_terminal/log_exam2.txt" );

x,sp,sq,phi_kalman,theta_kalman,sr, sr_filtered, sax, sax_filtered, say, say_filtered = [],[],[],[],[],[],[],[],[],[],[]
t = []
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
x_axis = []
diff = []

for i in range(0, len(x)):
    t.append(x[i][0])
    sp.append(x[i][1])
    sq.append(x[i][2])
    sr.append(x[i][3])
    sr_filtered.append(x[i][4])
    phi_kalman.append(x[i][5])
    theta_kalman.append(x[i][6])
    sax.append(x[i][7])
    say.append(x[i][8])
    x_axis.append(i)

for i in range(0, len(t)):
    t[i] = float(t[i])/1000000

for i in range(0, len(x)-1):
    diff.append(t[i+1]-t[i])
diff.append(0)

# plt.figure()
# plt.plot(x_axis, t,linewidth=0.75)
# print("PERIOD_AVG = "+str(mean(t)*1000)+" ms")
# m = max(t)
# print("MAX = "+str(m*1000)+" ms")

sr_ref = butter_lowpass_filter(sr_filtered, cutoff, fs, order)

#Butterworth filters done in the micro
plt.figure()
plt.plot(t,sr,t,sr_filtered,linewidth=0.75)
plt.legend(['sr','sr_filtered'])

plt.figure()
plt.plot(t, sr_filtered,linewidth=0.75, label='sr_filtered')
plt.plot(t, sr_ref,linewidth=0.75, label='sr_ref')
plt.title('sr_filtered micro vs sr_filtered Python')
plt.legend()

p2phi = 0.002
n = len(t)

p_bias = []
phi_kalman_r = []
p_kalman = []
phi_error = []

q_bias = []
theta_kalman_r = []
q_kalman = []
theta_error = []

for i in range(0,n):
    p_bias.append(0)
    phi_kalman_r.append(0)
    p_kalman.append(0)
    phi_error.append(0)
    q_bias.append(0)
    theta_kalman_r.append(0)
    q_kalman.append(0)
    theta_error.append(0)
C1 = 256
C2 = 1000000

for i in range(1,n):
    p_kalman[i] = sp[i-1] - p_bias[i-1];
    phi_kalman_r[i] = phi_kalman_r[i-1] + p_kalman[i] * p2phi;
    phi_error[i] = phi_kalman_r[i] - say[i];
    phi_kalman_r[i] = phi_kalman_r[i] - phi_error[i] / C1;
    p_bias[i] = p_bias[i-1] + (phi_error[i]/p2phi) / C2;

    q_kalman[i] = sq[i-1] - q_bias[i-1];
    theta_kalman_r[i] = theta_kalman_r[i-1] + q_kalman[i] * p2phi;
    theta_error[i] = theta_kalman_r[i] - sax[i];
    theta_kalman_r[i] = theta_kalman_r[i] - theta_error[i] / C1;
    q_bias[i] = q_bias[i-1] + (theta_error[i]/p2phi) / C2;

plt.figure()
plt.subplot(2,1,1)
plt.plot(t, sp,linewidth=0.75, label='sp')
plt.legend()
plt.subplot(2,1,2)
plt.plot(t, sq,linewidth=0.75, label='sq')
plt.legend()

# Kalman Filters
plt.figure()
plt.subplot(2,1,1)
plt.plot(t, phi_kalman,linewidth=0.75, label='phi_kalman')
plt.plot(t, phi_kalman_r, linewidth=0.75, label='phi_kalman_ref')
plt.title('Phi micro vs Phi Python')
plt.legend()

plt.subplot(2,1,2)
plt.plot(t, theta_kalman,linewidth=0.75, label='theta_kalman')
plt.plot(t, theta_kalman_r, linewidth=0.75, label='theta_kalman_ref')
plt.title('Theta micro vs Theta Python')
plt.legend()


freq = 1/(mean(diff))
plt.figure()
plt.plot(t, diff,linewidth=0.75)
plt.title("PERIOD_AVG = "+str(mean(diff)*1000)+" ms\n""FREQ_AVG = "+str(freq)+" Hz")


plt.show()
