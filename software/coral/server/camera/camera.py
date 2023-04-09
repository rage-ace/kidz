from __future__ import annotations

import math
import time
import ctypes
import threading
from enum import Enum
from typing import Tuple

import cv2
import numpy as np

from .util import *
from .kalman import KalmanFilter
from .singleton import Singleton
from .loop_tracker import LoopTracker
from .teensy_serial import TeensySerial


class DebugView(Enum):
    DEFAULT = 0
    ORANGE_MASK = 1
    BLUE_MASK = 2
    YELLOW_MASK = 3
    GREEN_MASK = 4
    RAW_FIELD_MASK = 5
    FIELD_MASK = 6

    def get_name(self) -> str:
        return self.name.replace("_", " ").title()

    @staticmethod
    def from_name(name: str) -> DebugView:
        return DebugView[name.upper().replace(" ", "_")]


class MemoryManager:
    """This object manages the memory shared between the threads."""

    def __init__(self) -> None:
        # Parameters
        self.params = {
            "frame": {
                "shape": (480, 640),
                "center_offset": [-2, 23],
                "crop_radius": 194,
            },
            "mask": {
                "robot_radius": 25,
                "mask_field": 0,  # False
                # # HCI Computer Lab
                # "orange": [(5, 70, 220), (28, 255, 255)],
                # "blue": [(100, 70, 200), (110, 255, 255)],
                # "yellow": [(25, 40, 230), (30, 200, 255)],
                # "green": [(70, 70, 110), (90, 255, 255)],
                # Home (living room, curtains drawn, cool lights on)
                "orange": [(0, 170, 160), (13, 255, 255)],
                "blue": [(98, 160, 60), (120, 255, 255)],
                "yellow": [(15, 110, 110), (40, 255, 255)],
                "green": [(45, 60, 80), (100, 255, 255)],
            },
            "contour_size": {
                "ball": [0, 230],
                "goal": [100, math.inf],
            },
            "filter_endurance": {
                "ball": 50,
                "goal": 200,
            },
            "test": {
                "a": 1,
                "b": 1,
                "c": 1,
                "d": 1,
            },
            "debug_views": [
                DebugView.ORANGE_MASK,
                DebugView.BLUE_MASK,
            ],
            "render": 0,  # False
        }

        # Frame fetched by FetchFrameProcess
        self.fetched_frame = None
        self.new_fetched_frame = threading.Condition()
        # Masks obtained by PreprocessFrameProcess
        self.orange_mask = None
        self.blue_mask = None
        self.yellow_mask = None
        self.new_masks = threading.Condition()
        # Ball and goal positions detected by DetectBallProcess and DetectGoalProcess
        self.ball = None  # (angle, distance)
        self.blue_goal = None  # (angle, distance)
        self.yellow_goal = None  # (angle, distance)
        self.new_detections = threading.Condition()  # either ball or goals are updated
        # Debug values used by AnnotateFrameProcess
        self.debug_values = {
            "cropped_frame": None,
            "green_mask": None,
            "raw_field_mask": None,
            "field_mask": None,
            "raw_ball": None,
            "raw_blue_goal": None,
            "raw_yellow_goal": None,
            "blue_rect": None,
            "yellow_rect": None,
        }
        # Loop Trackers
        self.loop_trackers = {
            "fetch_frame": LoopTracker(),
            "preprocess_frame": LoopTracker(),
            "detect_ball": LoopTracker(),
            "detect_goals": LoopTracker(),
            "send_payload": LoopTracker(),
            "annotate_frame": LoopTracker(),
        }
        # Debug results returned by AnnotateFrameProcess
        self.debug_results = {
            "text": None,
            "mainfeed": None,
            "subfeeds": [None for _ in range(len(self.params["debug_views"]))],
        }
        self.new_debug_results = threading.Condition()


class Camera(metaclass=Singleton):
    """
    This is the main camera manager, which runs on the main process. It manages several
    threads which fetch the camera feed and process it accordingly.
    """

    def __init__(self) -> None:
        self.mem = MemoryManager()

        # Lifecycle flags
        self._did_start = False
        self._stop_event = threading.Event()
        self._should_annotate_event = threading.Event()

        # Child processes
        # This process calls cap.read()
        self._fetch_frame_thread = FetchFrameThread(self.mem, self._stop_event)
        # This process preprocesses the frame for ball and goal detection
        self._preprocess_frame_thread = PreprocessFrameThread(
            self.mem, self._stop_event
        )
        # These processes detect the ball and two goals
        self._detect_ball_thread = DetectBallThread(self.mem, self._stop_event)
        self._detect_goals_thread = DetectGoalsThread(self.mem, self._stop_event)
        # This process sends out the payload to the Teensy
        self._send_payload_thread = SendPayloadThread(self.mem, self._stop_event)
        # This process annotates the frame with the detected objects, and is created
        # everytime a new websocket is created, and ends after the websocket closes
        self._annotate_frame_thread = AnnotateFrameThread(
            self.mem, self._should_annotate_event, self._stop_event
        )

    def start(self) -> None:
        """Start processes in the camera loop."""

        assert not self._did_start  # Ensure it didn't already start
        self._did_start = True

        # Start processes
        self._fetch_frame_thread.start()
        self._preprocess_frame_thread.start()
        self._detect_ball_thread.start()
        self._detect_goals_thread.start()
        self._send_payload_thread.start()
        self._annotate_frame_thread.start()

    def stop(self) -> None:
        """Kill all processes in the camera loop."""

        # Tell server loop to stop
        self._should_annotate_event.clear()
        time.sleep(0.5)

        # Tell threads to exit
        self._stop_event.set()

        # Trigger any blocking conditions that might stop a thread from exiting
        self._should_annotate_event.set()
        with self.mem.new_fetched_frame:
            self.mem.new_fetched_frame.notify_all()
        with self.mem.new_masks:
            self.mem.new_masks.notify_all()
        with self.mem.new_detections:
            self.mem.new_detections.notify_all()
        with self.mem.new_debug_results:
            self.mem.new_debug_results.notify_all()

        # Wait for all threads to exit
        self._fetch_frame_thread.join()
        self._preprocess_frame_thread.join()
        self._detect_ball_thread.join()
        self._detect_goals_thread.join()
        self._send_payload_thread.join()
        self._annotate_frame_thread.join()

    def start_annotating(self):
        """Start the AnnotateFrameProcess."""

        self._should_annotate_event.set()

    def stop_annotating(self):
        """Stop the AnnotateFrameProcess."""

        self._should_annotate_event.clear()

    def is_annotating(self) -> bool:
        """Return whether the AnnotateFrameProcess is running."""

        return self._should_annotate_event.is_set()

    def fetch_new_frame(self) -> np.ndarray:
        """Block until new annotated frames are available for the server, then return."""

        # Wait for new frames
        with self.mem.new_debug_results:
            self.mem.new_debug_results.wait()
        # Copy data from shared memory
        debug_text = self.mem.debug_results["text"]
        mainfeed = self.mem.debug_results["mainfeed"]
        subfeeds = self.mem.debug_results["subfeeds"]

        return debug_text, mainfeed, subfeeds


class FetchFrameThread(threading.Thread):
    def __init__(self, mem: MemoryManager, stop_event: threading.Event) -> None:
        super().__init__()
        self.mem = mem
        self.stop_event = stop_event

    def run(self) -> None:
        # Start the video capture
        cap = cv2.VideoCapture(1)

        # Setup props
        # Available props: ['CAP_PROP_BACKEND', 'CAP_PROP_BUFFERSIZE',
        # 'CAP_PROP_CONVERT_RGB', 'CAP_PROP_FORMAT', 'CAP_PROP_FOURCC',
        # 'CAP_PROP_FPS', 'CAP_PROP_FRAME_HEIGHT', 'CAP_PROP_FRAME_WIDTH',
        # 'CAP_PROP_MODE', 'CAP_PROP_ORIENTATION_AUTO', 'CAP_PROP_POS_MSEC']
        frame_shape = self.mem.params["frame"]["shape"]
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_shape[1])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_shape[0])
        cap.set(cv2.CAP_PROP_FPS, 30)  # the maximum for the coral camera
        cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)  # as small as possible

        # TODO: Figure out how to disable autofocus

        ret, frame = cap.read()
        if not ret:
            raise RuntimeError("Failed to capture initial frame")

        while not self.stop_event.is_set():
            self.mem.loop_trackers["fetch_frame"].start_iteration()

            # Attempt to read a new frame
            ret, frame = cap.read()
            if not ret:
                continue  # try again if it failed

            # Copy frame to shared memory
            self.mem.fetched_frame = frame
            with self.mem.new_fetched_frame:
                self.mem.new_fetched_frame.notify_all()

            self.mem.loop_trackers["fetch_frame"].stop_iteration()

        # Clean up
        cap.release()


class PreprocessFrameThread(threading.Thread):
    def __init__(self, mem: MemoryManager, stop_event: threading.Event) -> None:
        super().__init__()
        self.mem = mem
        self.stop_event = stop_event

    def run(self) -> None:
        while not self.stop_event.is_set():
            # Wait for new frame
            with self.mem.new_fetched_frame:
                self.mem.new_fetched_frame.wait()
            frame = self.mem.fetched_frame

            self.mem.loop_trackers["preprocess_frame"].start_iteration()

            # Preprocess frame
            orange_mask, blue_mask, yellow_mask = self.preprocess(frame)

            # Propagate masks
            self.mem.orange_mask = orange_mask
            self.mem.blue_mask = blue_mask
            self.mem.yellow_mask = yellow_mask
            with self.mem.new_masks:
                self.mem.new_masks.notify_all()

            self.mem.loop_trackers["preprocess_frame"].stop_iteration()

    def preprocess(
        self, frame: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        frame_params = self.mem.params["frame"]
        mask_params = self.mem.params["mask"]

        # Crop the frame
        cropped_frame = crop_circle(
            frame, frame_params["center_offset"], frame_params["crop_radius"]
        )
        cropped_frame = cv2.rotate(cropped_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cropped_frame = cv2.flip(cropped_frame, 1)  # flip horizontally

        # Convert to HSV for easier masking
        hsv = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)
        # Don't bother applying gaussian blur because we can perform morphology
        # operations later

        # Apply masks
        raw_orange_mask = mask(hsv, *mask_params["orange"])
        raw_blue_mask = mask(hsv, *mask_params["blue"])
        raw_yellow_mask = mask(hsv, *mask_params["yellow"])

        field_mask = np.ones_like(raw_orange_mask) * 255
        # needs to be declared for debugging purposes
        raw_green_mask = np.zeros_like(raw_orange_mask)
        raw_field_mask = np.zeros_like(raw_orange_mask)
        # Check if field masking is enabled
        if mask_params["mask_field"]:
            # Determine field mask
            raw_green_mask = mask(hsv, *mask_params["green"])
            raw_field_mask = (
                raw_green_mask | raw_yellow_mask | raw_blue_mask | raw_orange_mask
            )
            # The erosion after dilation is necessary to clean up the boundaries
            raw_field_mask = close_mask(raw_field_mask, 7, 2)
            field_contours, _ = find_contours(raw_field_mask)
            if len(field_contours) > 0:
                # Find largest contour
                cnt = max(field_contours, key=cv2.contourArea)
                # Extract convex hull points
                convex_hull = cv2.convexHull(cnt)
                hull_points = convex_hull.reshape(-1, 2)
                # Redraw mask with convex hull
                field_mask = np.zeros_like(raw_field_mask)
                cv2.fillConvexPoly(field_mask, hull_points, 255)
        # Subtract robot circle from field mask
        center = (int(cropped_frame.shape[1] / 2), int(cropped_frame.shape[0] / 2))
        cv2.circle(field_mask, center, mask_params["robot_radius"], 0, -1)

        # Apply field mask
        orange_mask = raw_orange_mask & field_mask
        blue_mask = raw_blue_mask & field_mask
        yellow_mask = raw_yellow_mask & field_mask
        # Although it would be helpful to close these masks, we don't and rely
        # on the masking to save on computational cost

        # Set debug values
        self.mem.debug_values["cropped_frame"] = cropped_frame
        self.mem.debug_values["green_mask"] = raw_green_mask
        self.mem.debug_values["raw_field_mask"] = raw_field_mask
        self.mem.debug_values["field_mask"] = field_mask

        # Return masks
        return orange_mask, blue_mask, yellow_mask


class DetectBallThread(threading.Thread):
    def __init__(self, mem: MemoryManager, stop_event: threading.Event) -> None:
        super().__init__()
        self.mem = mem
        self.stop_event = stop_event

        # Kalman filter
        F = np.eye(6, dtype=np.float)
        B = 0
        H = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ],
            dtype=np.float,
        )
        # Q represents process noise
        # larger values reduce response time, smaller values reduce noise
        Q = np.array(
            [
                [1e-3, 0, 0, 0, 0, 0],
                [0, 1e-2, 0, 0, 0, 0],
                [0, 0, 1e-2, 0, 0, 0],
                [0, 0, 0, 1e-2, 0, 0],
                [0, 0, 0, 0, 1e-2, 0],
                [0, 0, 0, 0, 0, 1e-3],
            ],
            dtype=np.float,
        )
        # R represents measurement noise
        # smaller values indicate greater measurement precision
        R = np.array(
            [
                [1e-3, 0, 0, 0],
                [0, 1e-3, 0, 0],
                [0, 0, 1e-3, 0],
                [0, 0, 0, 1e-3],
            ],
            dtype=np.float,
        )
        self._ball_filter = KalmanFilter(F=F, B=B, H=H, Q=Q, R=R)
        self._ball_not_found_count = math.inf

    def run(self) -> None:
        while not self.stop_event.is_set():
            # Wait for new masks
            with self.mem.new_masks:
                self.mem.new_masks.wait()
            orange_mask = self.mem.orange_mask

            self.mem.loop_trackers["detect_ball"].start_iteration()

            # Detect ball
            ball = self.detect_ball(orange_mask)

            # Propagate ball
            self.mem.ball = ball
            with self.mem.new_detections:
                self.mem.new_detections.notify_all()

            self.mem.loop_trackers["detect_ball"].stop_iteration()

    def detect_ball(self, mask: np.ndarray) -> np.ndarray:
        # Find contours
        contours, _ = find_contours(mask)

        # Filter contours based on area and shape
        ellipse: Tuple[Tuple[float, float], Tuple[float, float], float] = None
        contour_size_limits = self.mem.params["contour_size"]["ball"]
        for cnt in look_through_contours(contours, *contour_size_limits):
            if len(cnt) >= 5:
                # Fiting an ellipse requires at least 5 points
                ellipse = cv2.fitEllipse(cnt)
                # Check if the ellipse is valid
                (x, y), (a, b), _ = ellipse
                if not (
                    math.isnan(x) or math.isnan(y) or math.isnan(a) or math.isnan(b)
                ):
                    break

            # If we can't fit an ellipse, find the centroid instead
            m = cv2.moments(cnt)
            x, y = m["m10"] / m["m00"], m["m01"] / m["m00"]
            # Define an arbitrary 4x4 ellipse about the centroid
            ellipse = (x, y), (2, 2), 0
            break

        # Find polar position of ball
        ball: Tuple[float, float] = None
        if ellipse:
            # Find ball_dx, ball_dy
            (ball_x, ball_y), _, _ = ellipse
            ball = map_pixels_to_cm(mask.shape, ball_x, ball_y)
            self._ball_not_found_count = 0
        else:
            self._ball_not_found_count += 1

        # Apply kalman filter to cartesian ball position (polar didn't work well)
        # Update the ball filter
        if ball:
            dx, dy = polar_to_cartesian(*ball)
            _, (a, b), _ = ellipse

            # Update state of Kalman filter
            z = np.array([[dx], [dy], [a * 2], [b * 2]], dtype=np.float)
            self._ball_filter.update(z)
        # Predict the ball position if it has been found recently
        filtered_ball: Tuple[float, float] = None
        if self._ball_not_found_count <= self.mem.params["filter_endurance"]["ball"]:
            state = self._ball_filter.predict()
            if state is not None:  # state might be None if it's the initial prediction
                dx, dy = state[0][0], state[1][0]
                # Check that the state values make sense (they are wonky sometimes)
                if (
                    -mask.shape[1] / 2 <= dx <= mask.shape[1] / 2
                    and -mask.shape[0] / 2 <= dy <= mask.shape[0] / 2
                ):
                    filtered_ball = (dx, dy)
        # Convert to polar coordinates
        filtered_ball_polar = (
            cartesian_to_polar(*filtered_ball) if filtered_ball else None
        )

        # Set debug values
        self.mem.debug_values["raw_ball"] = ball

        return filtered_ball_polar


class DetectGoalsThread(threading.Thread):
    def __init__(self, mem: MemoryManager, stop_event: threading.Event) -> None:
        super().__init__()
        self.mem = mem
        self.stop_event = stop_event

        # Kalman filter
        F = np.eye(6, dtype=np.float)
        B = 0
        H = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ],
            dtype=np.float,
        )
        # Q represents process noise
        # larger values reduce response time, smaller values reduce noise
        Q = np.array(
            [
                [1e-3, 0, 0, 0, 0, 0],
                [0, 1e-2, 0, 0, 0, 0],
                [0, 0, 1e-2, 0, 0, 0],
                [0, 0, 0, 1e-2, 0, 0],
                [0, 0, 0, 0, 1e-2, 0],
                [0, 0, 0, 0, 0, 1e-3],
            ],
            dtype=np.float,
        )
        # R represents measurement noise
        # smaller values indicate greater measurement precision
        R = np.array(
            [
                [1e-3, 0, 0, 0],
                [0, 1e-3, 0, 0],
                [0, 0, 1e-3, 0],
                [0, 0, 0, 1e-3],
            ],
            dtype=np.float,
        )
        self._blue_goal_filter = KalmanFilter(F=F, B=B, H=H, Q=Q, R=R)
        self._yellow_goal_filter = KalmanFilter(F=F, B=B, H=H, Q=Q, R=R)
        self._blue_goal_not_found_count = math.inf
        self._yellow_goal_not_found_count = math.inf

    def run(self) -> None:
        while not self.stop_event.is_set():
            # Wait for new masks
            with self.mem.new_masks:
                self.mem.new_masks.wait()
            blue_mask = self.mem.blue_mask
            yellow_mask = self.mem.yellow_mask

            self.mem.loop_trackers["detect_goals"].start_iteration()

            # Detect goals
            blue_goal, yellow_goal = self.detect_goals(blue_mask, yellow_mask)

            # Propagate goals
            self.mem.blue_goal = blue_goal
            self.mem.yellow_goal = yellow_goal
            with self.mem.new_detections:
                self.mem.new_detections.notify_all()

            self.mem.loop_trackers["detect_goals"].stop_iteration()

    def detect_goals(
        self, blue_mask: np.ndarray, yellow_mask: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        # Find contours
        blue_contours, _ = find_contours(blue_mask)
        yellow_contours, _ = find_contours(yellow_mask)

        # Filter contours based on area and shape
        blue_rect: Tuple[Tuple[int, int], Tuple[int, int], float] = None
        contour_size_limits = self.mem.params["contour_size"]["goal"]
        for cnt in look_through_contours(blue_contours, *contour_size_limits):
            poly = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
            if len(poly) <= 6:
                rect = cv2.minAreaRect(cnt)
                blue_rect = rect
                break

        yellow_rect: Tuple[Tuple[int, int], Tuple[int, int], float] = None
        for cnt in look_through_contours(yellow_contours, *contour_size_limits):
            poly = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
            if len(poly) <= 6:
                rect = cv2.minAreaRect(cnt)
                yellow_rect = rect
                break

        # Find polar position of goals
        blue_goal: Tuple[float, float] = None
        if blue_rect:
            (x, y), _, theta = blue_rect
            blue_goal = map_pixels_to_cm(blue_mask.shape, x, y)
            self._blue_goal_not_found_count = 0
        else:
            self._blue_goal_not_found_count += 1

        yellow_goal: Tuple[float, float] = None
        if yellow_rect:
            (x, y), _, theta = yellow_rect
            yellow_goal = map_pixels_to_cm(yellow_mask.shape, x, y)
            self._yellow_goal_not_found_count = 0
        else:
            self._yellow_goal_not_found_count += 1

        # Apply kalman filter to cartesian ball position (polar didn't work well)
        # Update the blue goal filter
        if blue_goal:
            dx, dy = polar_to_cartesian(*blue_goal)
            _, (a, b), _ = blue_rect

            # Update state of Kalman filter
            z = np.array([[dx], [dy], [a], [b]], dtype=np.float)
            self._blue_goal_filter.update(z)
        # Predict the blue goal position if it has been found recently
        filtered_blue_goal: Tuple[float, float] = None
        if (
            self._blue_goal_not_found_count
            <= self.mem.params["filter_endurance"]["goal"]
        ):
            state = self._blue_goal_filter.predict()
            if state is not None:  # state might be None if it's the initial prediction
                dx, dy = state[0][0], state[1][0]
                # Check that the state values make sense (they are wonky sometimes)
                # the acceptable region is twice the mask size on purpose
                if (
                    -blue_mask.shape[1] <= dx <= blue_mask.shape[1]
                    and -blue_mask.shape[0] <= dy <= blue_mask.shape[0]
                ):
                    filtered_blue_goal = (dx, dy)
        # Convert to polar coordinates
        filtered_blue_goal_polar = (
            cartesian_to_polar(*filtered_blue_goal) if filtered_blue_goal else None
        )

        # Apply kalman filter to cartesian ball position (polar didn't work well)
        # Update the yellow goal filter
        if yellow_goal:
            dx, dy = polar_to_cartesian(*yellow_goal)
            _, (a, b), _ = yellow_rect

            # Update state of Kalman filter
            z = np.array([[dx], [dy], [a], [b]], dtype=np.float)
            self._yellow_goal_filter.update(z)
        # Predict the yellow goal position if it has been found recently
        filtered_yellow_goal: Tuple[float, float] = None
        if (
            self._yellow_goal_not_found_count
            <= self.mem.params["filter_endurance"]["goal"]
        ):
            state = self._yellow_goal_filter.predict()
            if state is not None:  # state might be None if it's the initial prediction
                dx, dy = state[0][0], state[1][0]
                # Check that the state values make sense (they are wonky sometimes)
                # the acceptable region is twice the mask size on purpose
                if (
                    -yellow_mask.shape[1] <= dx <= yellow_mask.shape[1]
                    and -yellow_mask.shape[0] <= dy <= yellow_mask.shape[0]
                ):
                    filtered_yellow_goal = (dx, dy)
        # Convert to polar coordinates
        filtered_yellow_goal_polar = (
            cartesian_to_polar(*filtered_yellow_goal) if filtered_yellow_goal else None
        )

        # Set debug values
        self.mem.debug_values["raw_blue_goal"] = blue_goal
        self.mem.debug_values["raw_yellow_goal"] = yellow_goal
        self.mem.debug_values["blue_rect"] = blue_rect
        self.mem.debug_values["yellow_rect"] = yellow_rect

        return filtered_blue_goal_polar, filtered_yellow_goal_polar


class SendPayloadThread(threading.Thread):
    def __init__(self, mem: MemoryManager, stop_event: threading.Event) -> None:
        super().__init__()
        self.mem = mem
        self.stop_event = stop_event

        # Serial manager
        self._serial = TeensySerial()

    def run(self) -> None:
        while not self.stop_event.is_set():
            # Wait for new detections
            with self.mem.new_detections:
                self.mem.new_detections.wait()
            ball = self.mem.ball
            blue_goal = self.mem.blue_goal
            yellow_goal = self.mem.yellow_goal

            self.mem.loop_trackers["send_payload"].start_iteration()

            # Send payload
            self._serial.write_packet(
                ball=(ball[0] if ball else None, ball[1] if ball else None),
                blue_goal=(
                    blue_goal[0] if blue_goal else None,
                    blue_goal[1] if blue_goal else None,
                ),
                yellow_goal=(
                    yellow_goal[0] if yellow_goal else None,
                    yellow_goal[1] if yellow_goal else None,
                ),
            )

            self.mem.loop_trackers["send_payload"].stop_iteration()
        self._serial.close()


class AnnotateFrameThread(threading.Thread):
    def __init__(
        self,
        mem: MemoryManager,
        should_annotate_event: threading.Event,
        stop_event: threading.Event,
    ) -> None:
        super().__init__()
        self.mem = mem
        self.should_annotate_event = should_annotate_event
        self.stop_event = stop_event

        # Trackers
        self._ball_distance_history = []

    def run(self) -> None:
        while not self.stop_event.is_set():
            # Wait for the cue to start annotating
            self.should_annotate_event.wait()
            # Wait for new detections
            with self.mem.new_detections:
                self.mem.new_detections.wait()

            self.mem.loop_trackers["annotate_frame"].start_iteration()

            # Annotate frames
            if self.mem.params["render"]:
                mainfeed_frame = self.get_annotated_frame()
                subfeed_frames = [
                    self.get_annotated_frame(view)
                    for view in self.mem.params["debug_views"]
                ]
            else:
                mainfeed_frame = self.mem.debug_values["cropped_frame"]
                subfeed_frames = [
                    np.zeros_like(self.mem.debug_values["cropped_frame"])
                    for view in self.mem.params["debug_views"]
                ]

            # Find ball position
            raw_ball = self.mem.debug_values["raw_ball"]
            ball = self.mem.ball
            mean_ball_distance: float = -1
            if ball:
                self._ball_distance_history.append(ball[1])
                while len(self._ball_distance_history) > 100:
                    self._ball_distance_history.pop(0)
                mean_ball_distance = sum(self._ball_distance_history) / len(
                    self._ball_distance_history
                )

            # Construct text
            # PROFILING
            text = "PROFILING\n"
            t = self.mem.loop_trackers
            text += f"FPS Ball  : {t['detect_ball'].last_fps():5.1f} FPS\n"
            text += "             Read  Mask  Ball  Goal  Send Render\n"
            text += (
                f"FPS       : {t['fetch_frame'].mean_fps():5.1f} "
                f"{t['preprocess_frame'].mean_fps():5.1f} "
                f"{t['detect_ball'].mean_fps():5.1f} "
                f"{t['detect_goals'].mean_fps():5.1f} "
                f"{t['send_payload'].mean_fps():5.1f} "
                f"{t['annotate_frame'].mean_fps():5.1f} (FPS)\n"
            )
            text += (
                f"Loop Time : {t['fetch_frame'].mean_loop_time():5.1f} "
                f"{t['preprocess_frame'].mean_loop_time():5.1f} "
                f"{t['detect_ball'].mean_loop_time():5.1f} "
                f"{t['detect_goals'].mean_loop_time():5.1f} "
                f"{t['send_payload'].mean_loop_time():5.1f} "
                f"{t['annotate_frame'].mean_loop_time():5.1f} (ms)\n\n"
            )
            # BALL
            text += f"BALL\n"
            if raw_ball:
                text += f"Raw      : {raw_ball[0]:7.2f}º {raw_ball[1]:6.2f} cm away\n"
            else:
                text += f"Raw      :   None\n"
            if ball:
                text += f"Filtered : {ball[0]:7.2f}º {ball[1]:6.2f} cm away\n"
            else:
                text += f"Filtered :   None\n"
            if mean_ball_distance:
                text += f"Mean     :          {mean_ball_distance:6.2f} cm away\n"
            else:
                text += f"Mean     :   None\n\n"
            # GOALS
            text += f"BLUE GOAL\n"
            if self.mem.debug_values["raw_blue_goal"]:
                text += f"Raw      : {self.mem.debug_values['raw_blue_goal'][0]:7.2f}º {self.mem.debug_values['raw_blue_goal'][1]:6.2f} cm away\n"
            else:
                text += f"Raw      :   None\n"
            if self.mem.blue_goal:
                text += f"Filtered : {self.mem.blue_goal[0]:7.2f}º {self.mem.blue_goal[1]:6.2f} cm away\n"
            else:
                text += f"Filtered :   None\n"
            text += f"YELLOW GOAL\n"
            if self.mem.debug_values["raw_yellow_goal"]:
                text += f"Raw      : {self.mem.debug_values['raw_yellow_goal'][0]:7.2f}º {self.mem.debug_values['raw_yellow_goal'][1]:6.2f} cm away\n"
            else:
                text += f"Raw      :   None\n"
            if self.mem.yellow_goal:
                text += f"Filtered : {self.mem.yellow_goal[0]:7.2f}º {self.mem.yellow_goal[1]:6.2f} cm away\n"
            else:
                text += f"Filtered :   None\n"

            # Propagate results
            self.mem.debug_results = {
                "text": text,
                "mainfeed": mainfeed_frame,
                "subfeeds": subfeed_frames,
            }
            with self.mem.new_debug_results:
                self.mem.new_debug_results.notify_all()

            self.mem.loop_trackers["annotate_frame"].stop_iteration()

    def get_annotated_frame(self, view: DebugView = DebugView.DEFAULT) -> np.ndarray:
        # Make a copy of the cropped frame
        frame = self.mem.debug_values["cropped_frame"].copy()

        # Return special views
        if view == DebugView.ORANGE_MASK:
            draw_mask(frame, self.mem.orange_mask)
            return frame
        elif view == DebugView.BLUE_MASK:
            draw_mask(frame, self.mem.blue_mask)
            return frame
        elif view == DebugView.YELLOW_MASK:
            draw_mask(frame, self.mem.yellow_mask)
            return frame
        elif view == DebugView.GREEN_MASK:
            draw_mask(frame, self.mem.debug_values["green_mask"])
            return frame
        elif view == DebugView.RAW_FIELD_MASK:
            draw_mask(frame, self.mem.debug_values["raw_field_mask"])
            return frame
        elif view == DebugView.FIELD_MASK:
            draw_mask(frame, self.mem.debug_values["field_mask"])
            return frame

        # Default view
        # Highlight the field mask
        draw_mask(frame, self.mem.debug_values["field_mask"])

        # Mark the center of the frame
        draw_cross(frame, (frame.shape[1] // 2, frame.shape[0] // 2), (255, 255, 255))

        # Draw crosses at the detections
        purple = (240, 32, 160)
        if self.mem.debug_values["raw_ball"]:
            dx, dy = self.mem.debug_values["raw_ball"]
            x, y = map_cm_to_pixels(frame.shape, dx, dy)
            draw_cross(frame, (int(x), int(y)), (0, 255, 0))
        if self.mem.ball:
            dx, dy = self.mem.ball
            x, y = map_cm_to_pixels(frame.shape, dx, dy)
            draw_cross(frame, (int(x), int(y)), purple)

        if self.mem.debug_values["raw_blue_goal"]:
            dx, dy = self.mem.debug_values["raw_blue_goal"]
            x, y = map_cm_to_pixels(frame.shape, dx, dy)
            draw_cross(frame, (int(x), int(y)), (0, 255, 255))
        if self.mem.blue_goal:
            dx, dy = self.mem.blue_goal
            x, y = map_cm_to_pixels(frame.shape, dx, dy)
            draw_cross(frame, (int(x), int(y)), purple)

            # Draw a rectangle around the blue goal
            box = cv2.boxPoints(self.mem.debug_values["blue_rect"])
            box = np.int0(box)
            cv2.drawContours(frame, [box], 0, (0, 255, 255), 2, cv2.LINE_AA)

        if self.mem.debug_values["raw_yellow_goal"]:
            dx, dy = self.mem.debug_values["raw_yellow_goal"]
            x, y = map_cm_to_pixels(frame.shape, dx, dy)
            draw_cross(frame, (int(x), int(y)), (255, 0, 0))
        if self.mem.yellow_goal:
            dx, dy = self.mem.yellow_goal
            x, y = map_cm_to_pixels(frame.shape, dx, dy)
            draw_cross(frame, (int(x), int(y)), purple)

            # Draw a rectangle around the yellow goal
            box = cv2.boxPoints(self.mem.debug_values["yellow_rect"])
            box = np.int0(box)
            cv2.drawContours(frame, [box], 0, (255, 0, 0), 2, cv2.LINE_AA)

        return frame
