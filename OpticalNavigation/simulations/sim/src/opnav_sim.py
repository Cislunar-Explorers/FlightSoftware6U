import json
import math

# import os.path
import os
import numpy as np
from numpy import linspace, radians, zeros

# from OpticalNavigation.core.find_algos import tiled_remap
from pyquaternion import Quaternion
import cv2
from argparse import ArgumentParser
from tqdm import tqdm
from OpticalNavigation.simulations.sim.src.libopnav import (
    sin2_vangle,
    gnomonic_inv,
    stereographic_inv,
    stereographic,
    st_circle,
    is_illuminated,
)
from utils.constants import FLIGHT_SOFTWARE_PATH

# Limitations:
# * No BRDF, PSF
# * Assumes point illumination (no penumbra)
# * No motion blur

SIM_DIR = os.path.join(FLIGHT_SOFTWARE_PATH, "OpticalNavigation/simulations/sim")


def main() -> None:
    argparse = ArgumentParser(
        description="Handles input & image generation args for opnav sim"
    )

    argparse.add_argument(
        "input", help="name of csv file in directory of the sim, w/o extension"
    )

    argparse.add_argument(
        "-g", action="store_true", help="set -g flag to generate images"
    )
    args = argparse.parse_args()
    run_opnav_sim(args.input, args.g)
    return


def run_opnav_sim(input_file: str, gen_img_flag: bool) -> None:
    # Camera properties
    width = 3280
    height = 2464
    fov_h = radians(62.2)
    fov_v = radians(48.8)
    xhat = (1, 0, 0)
    zhat = (0, 0, 1)
    cameras = {
        "A": Camera(
            "A",
            width,
            height,
            fov_h,
            fov_v,
            Quaternion(axis=xhat, degrees=-60) * Quaternion(axis=zhat, degrees=-90),
        ),
        "B": Camera(
            "B",
            width,
            height,
            fov_h,
            fov_v,
            Quaternion(axis=xhat, degrees=60) * Quaternion(axis=zhat, degrees=-90),
        ),
        "C": Camera(
            "C", width, height, fov_h, fov_v, Quaternion(axis=zhat, degrees=180 + 53)
        ),
    }

    # when input_file is outside out this directory, remove all leading folders
    input_file_trimmed = os.path.basename(os.path.normpath(input_file))

    output_dir = os.path.join(SIM_DIR, "data", input_file_trimmed + "_sim")
    os.mkdir(output_dir)
    if gen_img_flag:
        # print("generating images directory")
        os.mkdir(output_dir + "/images")

    # Absolute time corresponding to t0 (from OreKit simulation that produced traj2.csv)
    # Epoch depends on the specific trajectory, but I've left it here for reference -mm2774
    # TODO: every trajectory should have an absolute time, so this shouldn't be hardcoded
    epoch = "2020-06-27T21:08:03.0212 TDB"

    with open(os.path.join(output_dir, "cameras.json"), "w") as f:
        json.dump(
            {"epoch": epoch, "cameras": [c.as_dict() for c in cameras.values()]},
            f,
            indent=4,
        )

    # Spacecraft spin vector in body frame [rad/s]
    omega_body = np.array([0, 5, 0])

    # "spin frame" is an inertially non-spinning frame aligned with body frame at
    # some reference epoch.  This implies that body frame = a rotation of spin frame
    # by omega_body * (t - t_ref).
    q_world2spin = Quaternion(axis=(0, 0, 1), degrees=45)

    # Colors to render Earth, Moon, and Sun
    colors_bgr = [
        np.ubyte([192, 64, 64]),
        np.ubyte([128, 128, 128]),
        np.ubyte([128, 192, 192]),
    ]

    # Hack to serialize after each observation, rather than waiting until the trajectory is
    # exhausted.  Note: this will leave a trailing comma, which is not allowed by JSON.
    # Note: This is also not inside the object enclosing 'cameras' when it should be.
    observations = []
    num_lines = sum(1 for line in open(input_file + ".csv"))
    with open(input_file + ".csv") as f:
        # for line in f:
        for line in tqdm(f, total=num_lines):
            if line[0] == "t":
                continue  # Skip header
            t0, bodies, spacecraft, obs = parse_line(line, q_world2spin, omega_body)
            frames = []
            if gen_img_flag:
                frames = render_acquisition(
                    t0, cameras, spacecraft, obs, colors_bgr, output_dir
                )
            observations.append(
                {
                    "time": t0,
                    "bodies": [b.as_dict() for b in bodies],
                    "spacecraft": spacecraft.as_dict(),
                    "observed_bodies": [b.as_dict() for b in obs],
                    "frames": frames,
                }
            )
    with open(os.path.join(output_dir, "observations.json"), "w") as f:
        json.dump({"observations": observations}, f, indent=4)


class Camera:
    def __init__(self, name, width, height, fov_h, fov_v, q_body2cam):
        """
        width: Number of columns of pixels
        height: Number of rows of pixels
        fov_h: Full horizontal field of view [rad]
        fov_v: Full vertical field of view [rad]
        """
        self.name = name
        self.width = width
        self.height = height
        self.fov_h = fov_h
        self.fov_v = fov_v
        self.q_body2cam = q_body2cam

        # Gnomonic coordinates of pixel columns and rows
        # Average horizontal and vertical scaling to ensure square pixels
        self.gnomonic_scale = (
            ((self.width - 1) / 2) / np.tan(fov_h / 2)
            + ((self.height - 1) / 2) / np.tan(fov_v / 2)
        ) / 2
        xlim_gn = ((self.width - 1) / 2) / self.gnomonic_scale
        ylim_gn = ((self.height - 1) / 2) / self.gnomonic_scale
        self.x_gn = linspace(-xlim_gn, xlim_gn, width, dtype=np.float32)
        self.y_gn = linspace(-ylim_gn, ylim_gn, height, dtype=np.float32)

        # Stereographic coordinates of pixel columns and rows
        # Average horizontal and vertical scaling to ensure square pixels
        self.stereographic_scale = (
            ((self.width - 1) / 2) / (2 * np.tan(fov_h / 4))
            + ((self.height - 1) / 2) / (2 * np.tan(fov_v / 4))
        ) / 2
        xlim_st = ((self.width - 1) / 2) / self.stereographic_scale
        ylim_st = ((self.height - 1) / 2) / self.stereographic_scale
        self.x_st = linspace(-xlim_st, xlim_st, width, dtype=np.float32)
        self.y_st = linspace(-ylim_st, ylim_st, height, dtype=np.float32)

    def gn_to_px(self, x, y):
        return (
            int(round(self.gnomonic_scale * x + (self.width - 1) / 2)),
            int(round(self.gnomonic_scale * y + (self.height - 1) / 2)),
        )

    def st_to_px(self, x, y):
        return (
            int(round(self.stereographic_scale * x + (self.width - 1) / 2)),
            int(round(self.stereographic_scale * y + (self.height - 1) / 2)),
        )

    def st_dist_to_px(self, d):
        return int(round(self.stereographic_scale * d))

    def as_dict(self):
        return {
            "name": self.name,
            "width": self.width,
            "height": self.height,
            "fov_horizontal": self.fov_h,
            "fov_vertical": self.fov_v,
            "gnomonic_scale": self.gnomonic_scale,
            "stereographic_scale": self.stereographic_scale,
            "q_body2cam": self.q_body2cam.elements.tolist(),
        }


class Body:
    def __init__(self, name, radius, position_world):
        """
        radius: [m]
        """
        self.name = name
        self.radius = radius
        self.position_world = position_world

    def as_dict(self):
        return {
            "name": self.name,
            "radius": self.radius,
            "position_world": self.position_world.tolist(),
        }


class Spacecraft:
    def __init__(self, position_world, velocity_world, q_world2body, omega_body):
        self.position_world = position_world
        self.velocity_world = velocity_world
        self.q_world2body = q_world2body
        self.omega_body = omega_body

    def advance(self, dt):
        # Compute world2cam transformation at start of frame
        q_body_t02tf = (
            Quaternion(axis=self.omega_body, angle=dt * np.linalg.norm(self.omega_body))
            if np.linalg.norm(self.omega_body) > 0
            else Quaternion()
        )
        # Only advance attitude, not position
        return Spacecraft(
            self.position_world,
            self.velocity_world,
            self.q_world2body * q_body_t02tf,
            self.omega_body,
        )

    def as_dict(self):
        return {
            "position_world": self.position_world.tolist(),
            "velocity_world": self.velocity_world.tolist(),
            "q_world2body": self.q_world2body.elements.tolist(),
            "omega_body": self.omega_body.tolist(),
        }


class ObservedBody:
    def __init__(self, body, spacecraft):
        """
        radius: [m]
        position: Camera-centered, world-aligned frame [m]
        """
        self.body = body
        # Camera-centered, world-aligned frame
        self.relpos_world = body.position_world - spacecraft.position_world
        self.distance = np.linalg.norm(self.relpos_world)
        self.direction_world = self.relpos_world / self.distance

        self.direction_body = spacecraft.q_world2body.conjugate.rotate(
            self.direction_world
        )
        self.position_body = spacecraft.q_world2body.conjugate.rotate(self.relpos_world)
        # asin, not atan, to account for perspective
        self.angular_radius = math.asin(body.radius / self.distance)

    def as_dict(self):
        return {
            "body": self.body.name,
            "distance": self.distance,
            "direction_body": self.direction_body.tolist(),
            "angular_size": 2 * self.angular_radius,
        }


class Detection:
    """Stereographic projection."""

    def __init__(self, obs_body, camera):
        self.obs_body = obs_body
        self.camera = camera
        self.direction_cam = camera.q_body2cam.conjugate.rotate(obs_body.direction_body)
        self.direction_st = np.array(stereographic(self.direction_cam))
        # Center and radius of image circle in stereographic coordinates
        # (Center does NOT correspond to direction vector)
        rho = np.linalg.norm(self.direction_st)
        self.rho_center, self.radius_st = st_circle(rho, self.obs_body.angular_radius)
        self.center_st = (
            (self.rho_center / rho) * self.direction_st
            if rho > 0
            else self.direction_st
        )

    def is_in_frame(self):
        if self.direction_cam[2] < 0:
            return False
        # FIXME: Choose point in interior closest to center (may not be on edge)
        if self.rho_center == 0:
            return True
        p = ((self.rho_center - self.radius_st) / self.rho_center) * self.center_st
        return (
            p[0] > self.camera.x_st[0]
            and p[0] < self.camera.x_st[-1]
            and p[1] > self.camera.y_st[0]
            and p[1] < self.camera.y_st[-1]
        )

    def as_dict(self):
        return {
            "body": self.obs_body.body.name,
            "direction_cam": self.direction_cam.tolist(),
            "center_st": self.center_st.tolist(),
            "radius_st": self.radius_st,
        }


def parse_line(line, q_world2spin, omega_body):
    radius_e = 6371000  # [m]
    radius_m = 1737400  # [m]
    radius_s = 695700000  # [m]

    tokens = [float(t) for t in line.split(",")]
    t0 = tokens[0]
    camera_pos = np.array([tokens[1], tokens[2], tokens[3]])
    camera_vel = np.array([tokens[4], tokens[5], tokens[6]])
    moon_pos = np.array([tokens[7], tokens[8], tokens[9]])
    sun_pos = np.array([tokens[10], tokens[11], tokens[12]])

    earth = Body("Earth", radius_e, np.array([0, 0, 0]))
    moon = Body("Moon", radius_m, moon_pos)
    sun = Body("Sun", radius_s, sun_pos)
    bodies = [earth, moon, sun]

    # Assumes constant spin rate
    q_spin2body_t0 = Quaternion(axis=omega_body, angle=t0 * np.linalg.norm(omega_body))
    q_world2body_t0 = q_world2spin * q_spin2body_t0
    spacecraft = Spacecraft(camera_pos, camera_vel, q_world2body_t0, omega_body)

    return t0, bodies, spacecraft, [ObservedBody(b, spacecraft) for b in bodies]


def render_acquisition(t0, cameras, spacecraft, obs_bodies, colors_bgr, output_dir):
    """
    t0: Time at start of acquisition [s]
    """

    # Time between first frame of subsequent cameras
    camera_dt = 2.0944

    # Time between frames for a single camera
    frame_dt = 0.06545

    # Number of frames per camera (per exposure, per observation)
    n_frames = 20

    frame_dicts = []
    # Loop over cameras
    for camera in cameras.values():
        # print('  Camera %s' % camera.name)
        cnum = ord(camera.name) - ord("A")
        # Loop over frames captured by the current camera
        for f in range(n_frames):
            # Time at start of frame's exposure
            # Add 3 to cnum to match expHigh times from SurRender case1c
            delta_t = camera_dt * (cnum + 3) + frame_dt * f
            tf = t0 + delta_t

            # Compute transformation at start of frame
            spacecraft_f = spacecraft.advance(delta_t)
            q_world2cam = spacecraft_f.q_world2body * camera.q_body2cam

            obs_f = [ObservedBody(b.body, spacecraft_f) for b in obs_bodies]
            detections = [Detection(b, camera) for b in obs_f]
            detection_dicts = [d.as_dict() for d in detections if d.is_in_frame()]

            illuminator = None
            # illuminator = next((b for b in obs_f if b.body.name == 'Sun'))

            if detection_dicts:
                # print('    Frame %s' % f)
                # Render camera image with rolling shutter
                img = render_bodies(
                    camera, spacecraft_f, obs_f, colors_bgr, illuminator
                )
                if camera.name == "A":
                    name = "1"
                elif camera.name == "B":
                    name = "2"
                else:
                    name = "3"
                exposure = "High"
                for detection_dict in detection_dicts:
                    if detection_dict.get("body") == "Sun":
                        exposure = "Low"

                filename_gn = "cam%s_exp%s_f%d_dt%.5f_gn.png" % (
                    name,
                    exposure,
                    f,
                    delta_t,
                )
                cv2.imwrite(os.path.join(output_dir, "images", filename_gn), img)

                # Render ideal stereographic frame
                img = render_stereographic(camera, obs_f, colors_bgr, illuminator)
                draw_stereographic_detections(detections, img)
                filename_st = "cam%s_exp%s_f%d_dt%.5f_st.png" % (
                    name,
                    exposure,
                    f,
                    delta_t,
                )
                cv2.imwrite(os.path.join(output_dir, "images", filename_st), img)

                frame_dict = {
                    "time": tf,
                    "camera": camera.name,
                    "q_world2cam": q_world2cam.elements.tolist(),
                    "omega_cam": camera.q_body2cam.conjugate.rotate(
                        spacecraft.omega_body
                    ).tolist(),
                    "image_gnomonic": filename_gn,
                    "image_stereographic": filename_st,
                    "detections": detection_dicts,
                }
                frame_dicts.append(frame_dict)
    return frame_dicts


def render_bodies(camera, spacecraft, obs_bodies, colors_bgr, illuminator):
    line_time = 18.904e-6  # [s]
    img = zeros((camera.height, camera.width, 3), dtype=np.ubyte)

    # Pre-compute derived body quantities (direction as quaternion, sin^2 of angular radius)
    bdirs = [Quaternion(vector=b.direction_world) for b in obs_bodies]
    bsins = [
        np.asarray(math.sin(b.angular_radius) ** 2, dtype=camera.x_gn.dtype)
        for b in obs_bodies
    ]

    for row in range(camera.height):
        # 3D coordinates of pixel rays in camera frame
        sc = gnomonic_inv(camera.x_gn, camera.y_gn[row])

        # Transformation from world frame to camera frame at time of row's exposure
        tr = row * line_time
        spacecraft_r = spacecraft.advance(
            tr
        )  # Alternative: increment spacecraft by line_time
        q_world2cam = spacecraft_r.q_world2body * camera.q_body2cam

        for b in range(len(obs_bodies)):
            # Direction to target in camera frame
            # XXX: Secondary hotspot
            # direction_cam = q_world2cam.conjugate.rotate(obs_bodies[b].direction_world)
            direction_cam = (q_world2cam.conjugate * bdirs[b] * q_world2cam).vector

            if illuminator is None or obs_bodies[b] is illuminator:
                # Angular distance from each pixel ray to target center
                # XXX: Hotspot
                # d = vangle(sc, direction_cam)
                # img[row,...] += colors_bgr[b]*(d < obs_bodies[b].angular_radius)[...,newaxis]
                d = sin2_vangle(sc, direction_cam.astype(sc.dtype))
                img[row, ...] += colors_bgr[b] * (d < bsins[b])[..., np.newaxis]
            else:
                sun_pos_cam = q_world2cam.conjugate.rotate(
                    illuminator.relpos_world
                ).astype(sc.dtype)
                body_pos_cam = q_world2cam.conjugate.rotate(
                    obs_bodies[b].relpos_world
                ).astype(sc.dtype)
                radius = np.asarray(obs_bodies[b].body.radius, dtype=sc.dtype)
                img[row, ...] += (
                    colors_bgr[b]
                    * is_illuminated(sun_pos_cam, body_pos_cam, radius, sc)[
                        ..., np.newaxis
                    ]
                )
    return img


def render_stereographic(camera, obs_bodies, colors_bgr, illuminator):
    # 3D coordinates of pixel rays in camera frame
    xx = np.broadcast_to(camera.x_st, (camera.height, camera.width))
    yy = np.broadcast_to(camera.y_st[..., np.newaxis], (camera.height, camera.width))
    sc = stereographic_inv(xx, yy)

    img = zeros((camera.height, camera.width, 3), dtype=np.ubyte)
    for b in range(len(obs_bodies)):
        # Direction to target in camera frame
        direction_cam = camera.q_body2cam.conjugate.rotate(obs_bodies[b].direction_body)
        if illuminator is None or obs_bodies[b] is illuminator:
            # Angular distance from each pixel ray to target center
            d = sin2_vangle(sc, direction_cam.astype(sc.dtype))
            img += (
                colors_bgr[b]
                * (
                    d
                    < np.asarray(
                        math.sin(obs_bodies[b].angular_radius) ** 2, dtype=d.dtype
                    )
                )[..., np.newaxis]
            )
        else:
            sun_pos_cam = camera.q_body2cam.conjugate.rotate(
                illuminator.position_body
            ).astype(sc.dtype)
            body_pos_cam = camera.q_body2cam.conjugate.rotate(
                obs_bodies[b].position_body
            ).astype(sc.dtype)
            radius = np.asarray(obs_bodies[b].body.radius, dtype=sc.dtype)
            img += (
                colors_bgr[b]
                * is_illuminated(sun_pos_cam, body_pos_cam, radius, sc)[..., np.newaxis]
            )
    return img


def draw_stereographic_detections(detections, img):
    for b in detections:
        (px, py) = b.camera.st_to_px(b.center_st[0], b.center_st[1])
        cv2.circle(img, (px, py), b.camera.st_dist_to_px(b.radius_st), (0, 127, 255), 1)


def test_main():
    # Camera properties
    width = 640
    height = 480
    fov_h = radians(62.2)
    fov_v = radians(48.8)
    xhat = (1, 0, 0)
    zhat = (0, 0, 1)
    cameras = {
        "A": Camera(
            "A",
            width,
            height,
            fov_h,
            fov_v,
            Quaternion(axis=xhat, degrees=-60) * Quaternion(axis=zhat, degrees=-90),
        ),
        "B": Camera(
            "B",
            width,
            height,
            fov_h,
            fov_v,
            Quaternion(axis=xhat, degrees=60) * Quaternion(axis=zhat, degrees=-90),
        ),
        "C": Camera(
            "C", width, height, fov_h, fov_v, Quaternion(axis=zhat, degrees=180 + 53)
        ),
    }

    # Spacecraft spin vector in body frame [rad/s]
    omega_body = np.array([0, 0, 0])

    # Colors to render Earth, Moon, and Sun
    colors_bgr = [
        np.ubyte([192, 64, 64]),
        np.ubyte([128, 128, 128]),
        np.ubyte([128, 192, 192]),
    ]

    targets = [
        Body("TC", 1, np.array([0, 0, 4])),
        Body("TA", 1, np.array([0, 4, 4])),
        Body("TB", 1, np.array([0, -4, 4])),
    ]
    spacecraft = Spacecraft(
        np.array([0, 0, 0]), np.array([0, 0, 0]), Quaternion(), omega_body
    )
    obs = [ObservedBody(t, spacecraft) for t in targets]
    illuminator = obs[0]  # None
    t0 = 0
    frames = []
    for camera in cameras.values():
        detections = [Detection(ot, camera) for ot in obs]
        img_gn = render_bodies(camera, spacecraft, obs, colors_bgr, illuminator)
        filename_gn = "gn_%s.png" % camera.name
        cv2.imwrite(os.path.join(SIM_DIR, "src/out", filename_gn), img_gn)
        img_st = render_stereographic(camera, obs, colors_bgr, illuminator)
        draw_stereographic_detections(detections, img_st)
        filename_st = "st_%s.png" % camera.name
        cv2.imwrite(os.path.join(SIM_DIR, "src/out", filename_st), img_st)

        tf = ord(camera.name) - ord("A")
        q_world2cam = spacecraft.q_world2body * camera.q_body2cam
        detection_dicts = [d.as_dict() for d in detections if d.is_in_frame()]
        frames.append(
            {
                "time": tf,
                "camera": camera.name,
                "q_world2cam": q_world2cam.elements.tolist(),
                "image_gnomonic": filename_gn,
                "image_stereographic": filename_st,
                "detections": detection_dicts,
            }
        )

    observations = [
        {
            "time": t0,
            "bodies": [b.as_dict() for b in targets],
            "spacecraft": spacecraft.as_dict(),
            "observed_bodies": [b.as_dict() for b in obs],
            "frames": frames,
        }
    ]
    print(
        json.dumps(
            {
                "epoch": "",
                "cameras": [c.as_dict() for c in cameras.values()],
                "observations": observations,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
