from numpy import arctan, asarray, newaxis, reciprocal, sqrt, stack
import numpy as np
import json
import os

from utils.constants import FLIGHT_SOFTWARE_PATH


def stereographic_inv(x, y):
    """Convert stereographic coordinates to spherical coordinates."""
    x = asarray(x)
    y = asarray(y)

    norm = x ** 2 + y ** 2 + 4
    # Integer types not allowed for reciprocal (#10374)
    return (
        stack((-4 * x, -4 * y, (8 - norm)), axis=-1)
        * reciprocal(norm.astype(np.common_type(norm)))[..., newaxis]
    )
    # Division works correctly with integers, unlike reciprocal
    # return stack((-4*x, -4*y, (8 - norm)), axis=-1)/norm[...,newaxis]


def st_circle_inv(rho_c, r_c):
    """
    rho_c: Distance from origin to center of projected body
    r_c: Radius of projected body
    returns: rho, arad
    """
    b = 4 - rho_c ** 2 + r_c ** 2
    x = (-b + sqrt(b ** 2 + 16 * rho_c ** 2)) / (4 * rho_c)
    y = (-4 - rho_c ** 2 + r_c ** 2 + sqrt(b ** 2 + 16 * rho_c ** 2)) / (4 * r_c)
    return 2 * x, 2 * arctan(y)


def get_radius_center_st():
    path = os.path.join(
        FLIGHT_SOFTWARE_PATH,
        "OpticalNavigation/simulations/sim/data/traj-case1c_sim/observations.json",
    )
    st_rad_0 = 0
    center_st_0 = []
    with open(path, "r") as data:
        obs = json.load(data)
        frames = obs["observations"][0]["frames"]
        st_rad_0 = frames[0]["detections"][0]["radius_st"]
        center_st_0 = frames[0]["detections"][0]["center_st"]
    return st_rad_0, center_st_0


# rad, cen = get_radius_center_st()

# print("../simulations/sim/data/traj-case1c_sim/images/cam2_expLow_f0_dt8.37760_st.png")
# rad = 0.00878911 / 2  # from initial implementaiton
# cen = [0.19693145516536165, -0.21108428542264437]  # from Andrew's

print(
    "../simulations/sim/data/traj-case1c_sim/images/cam3_expHigh_f18_dt11.65010_st.png"
)
print("Earth")
rad = 0.06147455 / 2  # from initial implementaiton
cen = [0.22730826157123687, 0.18692091669069819]  # from Andrew's

print(rad)
print(cen)
rho_c = sqrt(cen[0] ** 2 + cen[1] ** 2)
rho, arad = st_circle_inv(rho_c, rad)
print(2 * arad)
