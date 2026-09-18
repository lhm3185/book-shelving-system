''' 
[EN]
Book Detect
A module that detects books and converts 
the coordinates of objects detected via camera (e.g., RealSense) 
into robot-compatible X, Y, Z spatial coordinates.

[KR]
책을 디텍팅하는 파일
realsense로 검출한객체의 좌표값을 로봇이 사용 가능한 x,y,z 좌표값으로 반환
'''


import numpy as np

class BookDetector:
    def process(self, detections, depth_image, fx, fy, cx, cy, depth_scale=1.0):
        """
        YOLO 바운딩 박스와 Depth 이미지로 책의 3D 좌표를 반환합니다.

        detections는 ``(x1, y1, x2, y2)`` 튜플의 리스트입니다.
        """
        detected_targets = []

        for detection in detections:
            if isinstance(detection, dict):
                box = detection['box']
                confidence = float(detection.get('confidence', 1.0))
                mask = detection.get('mask')
            else:
                box = detection
                confidence = 1.0
                mask = None
            x1, y1, x2, y2 = map(int, box)
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            if not (0 <= v < depth_image.shape[0] and 0 <= u < depth_image.shape[1]):
                continue

            z = self._depth_in_meters(
                self._center_depth(depth_image, u, v),
                depth_scale,
            )
            if z is None:
                continue

            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            detected_targets.append({
                'xyz': (x, y, z),
                'box': (x1, y1, x2, y2),
                'center': (u, v),
                'confidence': confidence,
                'image_angle': self._mask_angle(mask),
            })

        return detected_targets

    def _mask_angle(self, mask):
        """Return the major-axis angle of a segmentation mask in radians."""
        if mask is None:
            return 0.0

        points = np.column_stack(np.nonzero(mask))
        if len(points) < 2:
            return 0.0

        centered = points - points.mean(axis=0)
        covariance = np.cov(centered, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        axis = eigenvectors[:, int(np.argmax(eigenvalues))]
        return float(np.arctan2(axis[0], axis[1]))

    def _sample_depth(self, depth_image, x1, y1, x2, y2):
        """Return the median valid depth inside the detection box."""
        height, width = depth_image.shape[:2]
        x1 = max(0, min(width, x1))
        x2 = max(0, min(width, x2))
        y1 = max(0, min(height, y1))
        y2 = max(0, min(height, y2))
        if x1 >= x2 or y1 >= y2:
            return None

        values = depth_image[y1:y2, x1:x2].reshape(-1)
        valid_values = values[np.isfinite(values) & (values > 0)]
        if valid_values.size == 0:
            return None
        return np.median(valid_values)

    def _center_depth(self, depth_image, u, v, radius=2):
        """Return center-pixel depth, with a small fallback neighborhood."""
        height, width = depth_image.shape[:2]
        center_value = depth_image[v, u]
        if np.isfinite(center_value) and center_value > 0:
            return center_value

        x0 = max(0, u - radius)
        x1 = min(width, u + radius + 1)
        y0 = max(0, v - radius)
        y1 = min(height, v + radius + 1)
        values = depth_image[y0:y1, x0:x1].reshape(-1)
        valid_values = values[np.isfinite(values) & (values > 0)]
        if valid_values.size == 0:
            return None
        return np.median(valid_values)

    def _depth_in_meters(self, depth_value, depth_scale=1.0):
        if depth_value is None:
            return None
        value = float(depth_value)
        if not np.isfinite(value) or value <= 0:
            return None
        return value * float(depth_scale)