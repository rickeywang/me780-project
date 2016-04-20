import scipy.io as sio
from pylab import *
from oc import *

mat = sio.loadmat('matlab.mat')
# print transpose(mat['x'])
imml = mat['imml']
r_max = mat['r_max'][0]
phi_m = mat['phi_m'][0]
r_m_rec = mat['r_m_rec']
og = mat['og']
T = mat['T'][0]
state = transpose(mat['x'])
state_now = state[0]

M_global = 50
N_global = 60
# Occupancy grid in both probability and log odds form
global og_prob_global
og_prob_global = 0.5 * ones((M_global, N_global))
oglog_0 = log(og_prob_global / (1 - og_prob_global))
global ogl  # Log odds
ogl = oglog_0

for t in range(len(T)):
    state_now = state[t]
    r_m = r_m_rec[:, t]

    print(str(t) + " AT " + str(state_now))

    ogout = ogmap(M_global, N_global, ogl, state_now, phi_m, r_m, r_max)

    ogl = ogout['ogl']
    imml = ogout['imml']

    og_prob_global = exp(ogl) / (1 + exp(ogl))
    og_prob_mm = exp(imml) / (1 + exp(imml))

    plt.figure(0)
    aximg0 = plt.imshow(og_prob_global)
    plt.show()
    plt.pause(0.0001)
    plt.figure(1)
    aximg1 = plt.imshow(og_prob_mm)
    for i in range(len(r_m)):
        plot(state_now[1] + r_m[i]*sin(phi_m[i] + state_now[2]), state_now[0] + r_m[i]*cos(phi_m[i] + state_now[2]), 'g.')
    plt.show()
    plt.pause(0.0001)

print("YAYY")

