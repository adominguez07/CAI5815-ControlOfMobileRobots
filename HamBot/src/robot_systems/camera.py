from picamera2 import Picamera2
from robot_systems.landmark import Landmark
import numpy as np
import threading
import time
import cv2


class Camera:
    """
    Threaded PiCamera2 wrapper that provides frames in TRUE RGB and simple landmark detection.

    Key points:
    - Frames exposed by this class are always RGB (R,G,B channel order).
    - Optional vertical flip to match GUI orientation or camera mounting.
    - Color-based landmark detection uses RGB values and a fractional tolerance.
    """

    def __init__(self, resolution=(640, 480), fps=5, flip_vertical=True, tolerance =0.05):
        """
        Args:
            resolution (tuple[int, int]): (width, height) capture size.
            fps (int): Target capture FPS.
            flip_vertical (bool): If True, frames are vertically flipped to match GUI orientation.
        """
        self._picam2 = Picamera2()
        self._picam2.configure(
            self._picam2.create_preview_configuration(
                main={"size": resolution, "format": "RGB888"}
            )
        )
        self._picam2.start()

        self._fps = max(1, int(fps))
        self._flip_vertical = bool(flip_vertical)

        self._frame_rgb = None           # latest RGB frame (H, W, 3), uint8
        self._frame_lock = threading.Lock()

        self._target_colors_rgb = []     # list of (R,G,B) tuples
        self._color_tolerance = tolerance

        self._running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

    # --------------------------- Capture Thread ---------------------------

    def _capture_loop(self):
        """Continuously capture frames and normalize to TRUE RGB, honoring flip setting."""
        period = 1.0 / float(self._fps)
        next_t = time.monotonic()
        while self._running:
            arr = self._picam2.capture_array()
            if arr is None:
                # Avoid busy loop if camera hiccups
                time.sleep(period)
                continue

            # Guarantee TRUE RGB order; some stacks may yield BGR even for 'RGB888'
            rgb = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)

            if self._flip_vertical:
                rgb = np.flipud(rgb)

            with self._frame_lock:
                self._frame_rgb = rgb

            # pace to target FPS
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # If we're behind, reset the schedule
                next_t = time.monotonic()

    # --------------------------- Public API ---------------------------

    def get_frame(self, copy=True):
        """
        Returns the latest RGB frame.

        Args:
            copy (bool): If True, returns a copy; otherwise returns the internal reference.
        """
        with self._frame_lock:
            if self._frame_rgb is None:
                return None
            return self._frame_rgb.copy() if copy else self._frame_rgb

    def set_target_colors(self, colors, tolerance=0.05):
        """
        Configure the set of RGB colors to detect as "landmarks".

        Args:
            colors (tuple[int,int,int] | list[tuple[int,int,int]]): RGB tuples (0..255).
            tolerance (float): Fractional tolerance in [0, 1]. Example: 0.10 = ±10% of 255.
        """
        if isinstance(colors, tuple):
            self._target_colors_rgb = [colors]
        else:
            self._target_colors_rgb = list(colors)
        self._color_tolerance = float(np.clip(tolerance, 0.0, 1.0))

    def clear_target_colors(self):
        """Remove all configured target colors."""
        self._target_colors_rgb = []

    def find_landmarks(self, min_area=500):
        """
        Detect regions in the current frame that match any configured target color.

        Args:
            min_area (int): Minimum contour area to consider a detection.

        Returns:
            list[Landmark]: Detected landmarks with center, size, and sampled RGB.
        """
        frame = self.get_frame(copy=False)
        if frame is None or not self._target_colors_rgb:
            return []

        tol = int(round(255 * self._color_tolerance))
        h, w = frame.shape[:2]
        mask_total = np.zeros((h, w), dtype=np.uint8)

        # Build a combined mask across all target colors (in RGB space)
        for (r, g, b) in self._target_colors_rgb:
            lower = np.array([max(0, r - tol), max(0, g - tol), max(0, b - tol)], dtype=np.uint8)
            upper = np.array([min(255, r + tol), min(255, g + tol), min(255, b + tol)], dtype=np.uint8)
            mask = cv2.inRange(frame, lower, upper)
            mask_total = cv2.bitwise_or(mask_total, mask)

        contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            cx, cy = x + w_box // 2, y + h_box // 2
            # sample TRUE RGB at center
            r_s, g_s, b_s = map(int, frame[cy, cx])
            detected.append(Landmark(cx, cy, w_box, h_box, r_s, g_s, b_s))

        return detected

    # --------------------------- Configuration Helpers ---------------------------

    @property
    def fps(self):
        return self._fps

    @fps.setter
    def fps(self, value):
        """Dynamically adjust capture FPS (takes effect on the next cycle)."""
        value = int(value)
        self._fps = max(1, value)

    @property
    def flip_vertical(self):
        return self._flip_vertical

    @flip_vertical.setter
    def flip_vertical(self, flag):
        """Control whether frames are vertically flipped."""
        self._flip_vertical = bool(flag)

    # --------------------------- Teardown ---------------------------

    def stop(self):
        """Stop capture and release the camera."""
        if not self._running:
            return
        self._running = False
        if self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.0)
        try:
            self._picam2.stop()
        finally:
            self._picam2.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()

    def __del__(self):
        # Ensure resources are freed if user forgets to call stop()
        try:
            self.stop()
        except Exception:
            pass
