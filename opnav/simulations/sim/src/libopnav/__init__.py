from numpy import arctan, arctan2, asarray, dot, newaxis, reciprocal, sqrt, stack, tan
from numpy.linalg import norm
import numpy as np


def inner1d(u, v):
    return np.sum(u * v, axis=-1)


def vangle(a, b):
    a = asarray(a)
    b = asarray(b)
    # return arctan2(norm(cross(a, b), axis=-1), np.sum(a*b, axis=-1))
    # Assumes |a| == |b|
    return 2 * arctan2(norm(a - b, axis=-1), norm(a + b, axis=-1))


def norm2(v):
    return np.sum(v ** 2, axis=-1)


def sin2_vangle(u, v):
    u = asarray(u)
    v = asarray(v)
    # Assumes |a| == |b| == 1
    return 0.25 * norm2(u - v) * norm2(u + v)


def gnomonic_inv(x, y):
    z = reciprocal(sqrt(x ** 2 + y ** 2 + 1))
    return stack((-z * x, -z * y, z), axis=-1)


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


def stereographic(s):
    s = asarray(s)
    norm = reciprocal((1 + s[..., 2]).astype(np.common_type(s)))
    x = -2 * s[..., 0] * norm
    y = -2 * s[..., 1] * norm
    return x, y


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


def sphere_intersection(center, radius, direction):
    # Assumes center[...,2] > 0, norm(center) > radius
    # Returns (nan,nan,nan) if no intersection
    center = asarray(center)
    radius = asarray(radius)
    direction = asarray(direction)
    alpha = direction[..., 0] / direction[..., 2]
    beta = direction[..., 1] / direction[..., 2]
    a = 1 + alpha ** 2 + beta ** 2
    b = -2 * (alpha * center[..., 0] + beta * center[..., 1] + center[..., 2])
    c = norm2(center) - radius ** 2
    # Choose near (smaller) solution
    z = (-b - sqrt(b ** 2 - 4 * a * c)) / (2 * a)
    # Reject intersections behind the viewer
    z[np.sign(z) != np.sign(direction[..., 2])] = np.nan
    x = alpha * z
    y = beta * z
    return stack((x, y, z), axis=-1)


def is_illuminated(source, target_center, target_radius, direction):
    # Source, target_center in frame with observer at origin
    # Assumes point source
    source = asarray(source)
    target_center = asarray(target_center)
    intersections = sphere_intersection(target_center, target_radius, direction)
    return inner1d(target_center - source, intersections - target_center) < 0


# Batch rotation not needed: transform target rather than pixel rays
# (but in use for vectorizing over bodies when filtering)
def rotate_frame(xyz, q):
    return dot(xyz, q.rotation_matrix.T)
