import numpy as np
import scipy.integrate as integrate

# Radians / sec
omega = 5
# sec
pulse_dt = 0.1
# N
pulse_max = 1

# Until we have characteristics for the thruster, we consider the pulse to be a gaussian.
# Gaussian parameters:
#   a: maximum force
#   s: pulse standard deviation (we use (pulse_dt / 2) / 3)
a = pulse_max
s = pulse_dt / 6


# Constructing the gaussian:
def pulse(t):
    return a * np.exp(-(((t) ** 2) / (2 * (s ** 2))))


# Check our impulse:
J = integrate.quad(pulse, -pulse_dt / 2, pulse_dt / 2)
print(J)

ix, iy, iz = 0, 0, 0
fx, fy, fz = 0, 0, 0

# vec_i = np.array([ix, iy, iz])
# vec_f = np.array([fx, fy, fz])
# i_cross_f = np.cross(vec_i, vec_f)


def calc_dir(qi, vf):
    vi = np.array(qi.vec)
    ideal_dir = np.cross(vi, vf)
    print(ideal_dir)
