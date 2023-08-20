# from opnav.core.find_algos.tiled_remap import get_angular_size
from numpy import arctan, asarray, cos, newaxis, reciprocal, sin, sqrt, stack, tan
import numpy as np
import json
import os

# from opnav.core.find_algos.tiled_remap import
from fsw.utils.constants import FLIGHT_SOFTWARE_PATH


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


def st_circle(rho, arad):
    """
    rho: Distance from origin to stereographic projection of object center
    arad: Angular size of object
    returns: rho_c, r_c
    """
    x = rho / 2
    y = tan(arad / 2)
    d = 1 - (x ** 2 * y ** 2)
    return 2 * x * (1 + y ** 2) / d, 2 * y * (1 + x ** 2) / d


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
        "opnav/simulations/sim/data/traj-case1c_sim/observations.json",
    )
    st_rad_0 = 0
    center_st_0 = []
    with open(path, "r") as data:
        obs = json.load(data)
        frames = obs["observations"][0]["frames"]
        st_rad_0 = frames[0]["detections"][0]["radius_st"]
        center_st_0 = frames[0]["detections"][0]["center_st"]
    return st_rad_0, center_st_0


# st_center = {"x": 0.19693145516536165, "y": -0.21108428542264437}
# diam = 0.00878911126015293

st_center = {"x": 0.19709011246827635, "y": -0.2109395731658707}
radSt = 0.0046749541685242365
angle = arctan(st_center["y"] / st_center["x"])
dist = np.linalg.norm(np.array([st_center["x"], st_center["y"]]))
# rho, arad = st_circle_inv(dist, radSt)
rho = dist
x_new = cos(angle) * rho
y_new = sin(angle) * rho
angDiam = 2 * arctan((rho + radSt) / 2) - 2 * arctan((rho - radSt) / 2)

print(f"{radSt/2=}")
print(f" {dist=}")
# print(f"  {rho=}, {2*arad=}")
print(f"  {rho=}")
print(f"{x_new=}, {y_new=}")
print(f"{angDiam=}")

# Work on fixing center, see results
