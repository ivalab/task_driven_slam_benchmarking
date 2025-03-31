#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file waypoints_extractor.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 03-05-2024
@version 1.0
@license Copyright (c) 2024
@desc None
"""

import glob
import logging
from pathlib import Path
import os

import apriltag
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from cl_real.utils.utils import (
    extract_image_filenames,
    undistort_image,
)


def yaw_from_quaternion(quat: np.ndarray) -> float:
    """Extract yaw from quaternion (x, y, z, w) in range[-pi, pi]."""
    assert quat.shape == (4,) or quat.shape == (4, 1) or quat.shape == (1, 4)
    rot = Rotation.from_quat(quat).as_matrix()
    return np.arctan2(rot[1, 0], rot[0, 0])


class WaypointsExtractor:
    """Extract waypoints from tag images."""

    def __init__(self, params, named_camera_info):
        self._params = params
        self._named_camera_info = named_camera_info
        self._detector = apriltag.Detector()

    def run(self, method_name: str, method_stamps: list) -> np.ndarray:
        """_summary_

        Args:
            method_name (str): _description_
            method_stamps (list): _description_

        Returns:
            np.ndarray: _description_
        """
        # Detect tags.
        all_timestamps = None
        all_poses = None
        all_points = None
        if self._params["load_detection"]:
            all_timestamps, all_poses, all_points = self.__load_detection(method_name)
        else:
            all_timestamps, all_poses, all_points, no_tag_timestamps = self.__run_detection(method_name, method_stamps)
            self.__save_detection(method_name, all_timestamps, all_poses, all_points, no_tag_timestamps)

        # Extract waypoints.
        waypoints, wpts_in_pixels = self.__extract_waypoints(method_name, all_timestamps, all_poses, all_points)

        # Save waypoints.
        np.save(self._params["data_dir"] / "evaluation" / (method_name + "_waypoints"), waypoints)
        np.save(self._params["data_dir"] / "evaluation" / (method_name + "_waypoints_in_pixels"), wpts_in_pixels)
        return waypoints

    def __run_detection(self, method_name: str, method_stamps: list):
        # Enable vis.
        if self._params["vis_detection"]:
            cv2.namedWindow("vis", cv2.WINDOW_NORMAL)

        # Extract images by the given method start and end timestamps.
        named_images = {}
        for cam_name, _ in self._named_camera_info.items():
            image_filenames = sorted(glob.glob(str(self._params["data_dir"] / "overhead" / cam_name / "*.jpg")))
            named_images[cam_name] = extract_image_filenames(image_filenames, method_stamps)
            logging.info(f"Loaded {len(named_images[cam_name])} images")

        # Detect tag.
        all_timestamps = []
        all_poses = []
        all_points = []
        no_tag_timestamps = []
        all_images_count = 0
        for cam_name, filenames in named_images.items():
            camera_data = self._named_camera_info[cam_name]
            Kmat = self.__make_K(camera_data)

            images_count = 0
            poses_count = 0
            for filename in filenames:
                timestamp = float(Path(filename).stem)
                images_count += 1
                # Read image and detect tag.
                bgr = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
                grayscale = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                undistorted_im = undistort_image(grayscale, camera_data)
                points, pose = self.__detect(undistorted_im, Kmat)

                # Detection can be empty after image undistortion.
                if points is None or pose is None:
                    logging.warn(f"No tag is detected at image {filename}.")
                    no_tag_timestamps.append(timestamp)
                    continue

                # append timestamp
                all_timestamps.append(timestamp)

                # append pose
                pose_array = self.__mat2array(pose)
                all_poses.append(pose_array)
                poses_count += 1

                # append points
                all_points.append(points.reshape(-1))

            # Accumulate image count.
            logging.info(f"Processed {poses_count} poses out of {images_count} images in {cam_name}.")
            all_images_count += images_count

        logging.info(f"Processed {len(all_poses)} poses out of {all_images_count} images in all cams.")

        # Sort the data
        all_timestamps, all_poses, all_points = zip(*sorted(zip(all_timestamps, all_poses, all_points)))

        return all_timestamps, all_poses, all_points, no_tag_timestamps

    def __save_detection(self, method_name, all_timestamps, all_poses, all_points, no_tag_timestamps):
        # Save to file
        prefix = str(self._params["data_dir"] / "evaluation" / method_name)
        np.savetxt(
            prefix + "_tag_poses.txt",
            np.column_stack([all_timestamps, all_poses]),
            fmt="%.06f",
            header="timestamp tx ty tz qx qy qz",
        )
        np.savetxt(
            prefix + "_tag_points.txt",
            np.column_stack([all_timestamps, all_points]),
            fmt="%.06f",
            header="timestamp c0_xy c1_xy c2_xy c3_xy center_xy",
        )
        np.savetxt(
            prefix + "_no_tag_images.txt",
            no_tag_timestamps,
            fmt="%.06f",
            header="timestamp",
        )

    def __load_detection(self, method_name):
        prefix = str(self._params["data_dir"] / "evaluation" / method_name)
        all_points = np.loadtxt(prefix + "_tag_points.txt", ndmin=2)
        all_poses = np.loadtxt(prefix + "_tag_poses.txt", ndmin=2)
        assert all_points.shape[0] == all_poses.shape[0]
        return all_poses[:, 0], all_poses[:, 1:], all_points[:, 1:]

    def __make_K(self, camera_data):
        return np.array(
            [
                camera_data["K"][0, 0],
                camera_data["K"][1, 1],
                camera_data["K"][0, 2],
                camera_data["K"][1, 2],
            ]
        )

    def __detect(self, image, K):
        result = self._detector.detect(image, return_image=False)

        if len(result) < 1:
            return None, None

        # For simplicity, we only estimate the first detected tag.
        pose, e0, e1 = self._detector.detection_pose(result[0], K, tag_size=self._params["tag_size"])

        # Have not taken a look at the meaning of the returned errors.

        points = self.__extract_tag_points(result[0])

        if self._params["vis_detection"]:
            tag_img = self.__plot_tag(image, result)
            cv2.imshow("vis", tag_img)
            cv2.waitKey(1)

        return (points, pose)

    def __extract_tag_points(self, tag):
        return np.array(
            [
                [tag.corners[0][0], tag.corners[0][1]],
                [tag.corners[1][0], tag.corners[1][1]],
                [tag.corners[2][0], tag.corners[2][1]],
                [tag.corners[3][0], tag.corners[3][1]],
                [tag.center[0], tag.center[1]],
            ],
            dtype=np.float64,
        )

    def __mat2array(self, mat):
        quat = Rotation.from_matrix(mat[:3, :3]).as_quat()
        return np.array([mat[0, 3], mat[1, 3], mat[2, 3], quat[0], quat[1], quat[2], quat[3]])

    def __plot_tag(self, img, result):
        # Convert to color.
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        for tag in result:
            # Draw center.
            cx, cy = int(tag.center[0]), int(tag.center[1])
            cv2.circle(img=img, center=(cx, cy), radius=5, thickness=3, color=[0, 0, 255])

            # Draw corners.
            for cn in tag.corners:
                x, y = int(cn[0]), int(cn[1])
                cv2.circle(img=img, center=(x, y), radius=5, thickness=3, color=[0, 255, 0])

        return img

    def __extract_waypoints(self, method_name, timestamps, poses, points):
        assert poses.shape[0] == points.shape[0]
        wpts_count = self._params["wpts_count"]
        rounds = self._params["rounds"]
        loops = self._params["loops"]
        waypoints = np.full((loops, wpts_count, 3, rounds), np.nan)
        wpts_in_pixels = np.full((loops, wpts_count, points.shape[-1], rounds), np.nan)

        # Compute detection errors between any continuous two frames.
        # A waypoint is expected to have less than 1.0 pixel across neighbor frames.
        points_diffs = np.linalg.norm(np.diff(points, axis=0), axis=-1)

        nav_data_dir = self._params["data_dir"] / method_name / self._params["path"] / "*"
        nav_data_files = sorted(glob.glob(str(nav_data_dir)))[:5]
        assert len(nav_data_files) == rounds

        machine_delay = self._params["machine_delay"]
        max_timestamp_diff = self._params["max_timestamp_diff"]

        # Loop over each round of nav data
        for round_index, nav_data_file in enumerate(nav_data_files):
            logging.info(f"Processing round {round_index} ...")
            visited_wpts = np.loadtxt(Path(nav_data_file) / "visited_waypoints_nav_info.txt", ndmin=2)
            logging.info(f"Visited wpts ratio: {len(visited_wpts) / wpts_count}")
            # Loop over each wpt
            for stamped_visited_wpt in visited_wpts:
                stamp = stamped_visited_wpt[0]
                wpt_index = int(stamped_visited_wpt[-2])
                loop_index = int(stamped_visited_wpt[-1])
                tmp_machine_delay = machine_delay
                # if method_name == "dsol":
                #     if wpt_index == 3 or wpt_index == 4:
                #         tmp_machine_delay = 2.0
                #     elif wpt_index >= 6:
                #         tmp_machine_delay = 2.0
                #     if round_index >= 4:
                #         tmp_machine_delay = 2.0
                #     if round_index == 0:
                #         if wpt_index == 3  or wpt_index == 4:
                #             tmp_machine_delay = 2.0
                #         else:
                #             tmp_machine_delay = -6.0
                timestamp_diff = np.abs(stamp - timestamps - tmp_machine_delay)
                min_val = np.min(timestamp_diff)
                if min_val > max_timestamp_diff:  # second
                    logging.warning(
                        f"No wpt was found at stamp {stamp:.06f} within the max timestamp diff, wpt_index: {wpt_index}, loop_index: {loop_index}"
                    )
                else:
                    index = np.argmin(timestamp_diff)

                    # Assure the robot actually stops at this point.
                    assert (
                        points_diffs[index] < 1.0
                    ), f" wpt_stamp: {stamp}, tag_stamp: {timestamps[index]}, loop: {loop_index}, wpt_index_in_loop: {wpt_index}, points_diffs: {points_diffs[index]}"

                    waypoints[loop_index, wpt_index, :, round_index] = [
                        poses[index][0],
                        poses[index][1],
                        yaw_from_quaternion(poses[index][3:]),
                    ]

                    wpts_in_pixels[loop_index, wpt_index, :, round_index] = points[index, :]

        # values = np.min(diffs)
        # indices = np.argmin()
        # print(diffs)
        # print(waypoints)
        return waypoints, wpts_in_pixels

    def run_flip_version(self, method_name, calib):
        wpts_count = self._params["wpts_count"]
        rounds = self._params["rounds"]
        loops = self._params["loops"]
        start_loop_idx = 0 if loops == 1 else 1  # skip mapping phase if in localizatio mode
        waypoints = np.full((loops - start_loop_idx, wpts_count, 3, rounds), np.nan)  # x, y, z
        wpts_in_pixels = np.full((loops - start_loop_idx, wpts_count, 10, rounds), np.nan)  # (x, y) * 5

        Kmat = self.__make_K(calib)
        no_tag_timestamps = []
        # Per each method, load planned wpts.
        if "slam" == self._params["mode"]:
            assert rounds > 1
            assert loops == 1
        elif "localization" == self._params["mode"]:
            assert rounds == 1
            assert loops > 1
        else:
            logging.error(f"Mode {self._params['mode']} is not supported!")
            return None
        method_dir = self._params["data_dir"] / method_name
        print(method_dir)
        trial_files = sorted(glob.glob(str(method_dir / "*")))
        for round_idx, trial_file in enumerate(trial_files):
            for loop_idx in range(start_loop_idx, loops):
                for wpt_idx in range(wpts_count):
                    logging.info(f"Processing round {round_idx}, wpt {wpt_idx}, loop {loop_idx} ...")
                    filenames = glob.glob(str(Path(trial_file) / f"up_g{wpt_idx}_l{loop_idx}_*.jpg"))
                    if len(filenames) == 0:
                        logging.warn("No file exists.")
                        continue
                    filename = filenames[0]
                    # Should only take care of the first one.
                    bgr = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
                    grayscale = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                    # The image is already undistorted.
                    points, pose = self.__detect(grayscale, Kmat)
                    name, ext = os.path.splitext(filename)
                    timestamp = float(name[name.rfind("_") + 1 :] if "_" in name else -1.0)
                    # Detection can be empty after image undistortion.
                    if points is None or pose is None:
                        logging.warn(f"No tag is detected at image {filename}.")
                        no_tag_timestamps.append(timestamp)
                        continue
                    pose_array = self.__mat2array(pose)
                    waypoints[loop_idx - start_loop_idx, wpt_idx, :, round_idx] = [
                        pose_array[0],
                        pose_array[1],
                        yaw_from_quaternion(pose_array[3:]),
                    ]

                    wpts_in_pixels[loop_idx - start_loop_idx, wpt_idx, :, round_idx] = points.reshape(-1)

        np.save(self._params["result_dir"] / "evaluation" / (method_name + "_waypoints"), waypoints)
        np.save(self._params["result_dir"] / "evaluation" / (method_name + "_waypoints_in_pixels"), wpts_in_pixels)
        return waypoints
