import unittest

from numpy import array, pi
from numpy.testing import assert_allclose
from pyquaternion import Quaternion

from libopnav import *


class TestLibopnav(unittest.TestCase):
    def test_vangle_scalar(self):
        a = [1, 0, 0]
        b = [0, 1, 0]
        assert_allclose(vangle(a, b), pi / 2)

    def test_vangle_array(self):
        a = array([[1, 0, 0], [1, 0, 0], [1, 0, 0], [0, 0, 1]])
        b = array([[1, 0, 0], [0, 1, 0], [sqrt(0.5), sqrt(0.5), 0], [1, 0, 0]])
        expected = array([0, pi / 2, pi / 4, pi / 2])
        assert_allclose(vangle(a, b), expected)

    def test_gnomonic_inv_scalar(self):
        x = 0
        y = 0
        assert_allclose(gnomonic_inv(x, y), array([0, 0, 1]))

    def test_gnomonic_inv_array(self):
        x = array([0, 1 / sqrt(3), 0])
        y = array([0, 0, 1 / sqrt(3)])
        expected = array([[0, 0, 1], [-0.5, 0, sqrt(3) / 2], [0, -0.5, sqrt(3) / 2]])
        assert_allclose(gnomonic_inv(x, y), expected)

    def test_stereographic_inv_scalar(self):
        x = 0
        y = 0
        assert_allclose(stereographic_inv(x, y), array([0, 0, 1]))

    def test_stereographic_inv_array(self):
        x = array([0, 1 / sqrt(3), 0])
        y = array([0, 0, 1 / sqrt(3)])
        expected = array(
            [
                [0, 0, 1],
                [-12 / (13 * sqrt(3)), 0, 11 / 13],
                [0, -12 / (13 * sqrt(3)), 11 / 13],
            ]
        )
        assert_allclose(stereographic_inv(x, y), expected)

    def test_stereographic_scalar(self):
        s = array([0, 0, 1])
        x, y = stereographic(s)
        assert_allclose(x, 0)
        assert_allclose(y, 0)

    def test_stereographic_array(self):
        s = array(
            [
                [0, 0, 1],
                [-12 / (13 * sqrt(3)), 0, 11 / 13],
                [0, -12 / (13 * sqrt(3)), 11 / 13],
            ]
        )
        x, y = stereographic(s)
        assert_allclose(x, array([0, 1 / sqrt(3), 0]))
        assert_allclose(y, array([0, 0, 1 / sqrt(3)]))

    def test_st_circle(self):
        # Compare with tangent formulations of forward formulae
        rho = 0.06267123160157438
        arad = 0.09288462530483839
        theta = 2 * arctan(rho / 2)
        rho_c_expected = tan((theta + arad) / 2) + tan((theta - arad) / 2)
        r_c_expected = tan((theta + arad) / 2) - tan((theta - arad) / 2)
        rho_c, r_c = st_circle(rho, arad)
        assert_allclose(rho_c, rho_c_expected)
        assert_allclose(r_c, r_c_expected)

    def test_st_circle_roundtrip(self):
        # Check that inverse-forward composition is identity
        rho = 0.06267123160157438
        arad = 0.09288462530483839
        rho_c, r_c = st_circle(rho, arad)
        rho2, arad2 = st_circle_inv(rho_c, r_c)
        assert_allclose(rho2, rho)
        assert_allclose(arad2, arad)

        # Check that forward-inverse composition is identity
        rho_c = 0.06280673427301477
        r_c = 0.09304293187674785
        rho, arad = st_circle_inv(rho_c, r_c)
        rho_c2, r_c2 = st_circle(rho, arad)
        assert_allclose(rho_c2, rho_c)
        assert_allclose(r_c2, r_c)

    def test_sphere_intersection_scalar(self):
        center = [0, 0, 2]
        radius = 1
        direction = [0, 0, 1]
        p = sphere_intersection(center, radius, direction)
        assert_allclose(p, array([0, 0, 1]))

    def test_sphere_intersection_array(self):
        center = array([[0, 0, 2], [-1, 0, 2]])
        radius = [1, 1]
        direction = array([[0, 0, 1], [0, 0, 1]])
        p = sphere_intersection(center, radius, direction)
        expected = array([[0, 0, 1], [0, 0, 2]])
        assert_allclose(p, expected)

        # Only direction is an array
        center = [-1, 0, 2]
        radius = 1
        directions = array([[0, 0, 1], [-1, 0, 1]])
        p = sphere_intersection(center, radius, directions)
        expected = array([[0, 0, 2], [-1, 0, 1]])
        assert_allclose(p, expected)

    def test_is_illuminated(self):
        source = [-1, 0, 0]
        target_center = [-1, 0, 2]
        target_radius = 1
        directions = array([[0, 0, 1], [-1, 0, 1]])
        b = is_illuminated(source, target_center, target_radius, directions)
        expected = array([False, True])
        assert_allclose(b, expected)

    def test_rotate_frame(self):
        xyz = array(
            [
                [0, 0, 1],
                [-0.5, 0, sqrt(3) / 2],
                [0, -0.5, sqrt(3) / 2],
                [-0.40824829, -0.40824829, 0.81649658],
            ]
        )
        q = Quaternion(axis=[0, 0, 1], angle=pi / 2)
        # r = rotate_frame(xyz, q)
        expected = array(
            [
                [0, 0, 1],
                [0, -0.5, sqrt(3) / 2],
                [0.5, 0, sqrt(3) / 2],
                [0.40824829, -0.40824829, 0.81649658],
            ]
        )
        assert_allclose(rotate_frame(xyz, q), expected, atol=1e-15)


if __name__ == "__main__":
    unittest.main()
