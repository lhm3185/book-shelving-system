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
            x1, y1, x2, y2 = map(int, detection)
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            if not (0 <= v < depth_image.shape[0] and 0 <= u < depth_image.shape[1]):
                continue

            z = self._depth_in_meters(depth_image[v, u], depth_scale)
            if z is None:
                continue

            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            detected_targets.append({
                'xyz': (x, y, z),
                'box': (x1, y1, x2, y2),
                'center': (u, v),
            })

        return detected_targets

    def _depth_in_meters(self, depth_value, depth_scale=1.0):
        value = float(depth_value)
        if not np.isfinite(value) or value <= 0:
            return None
        return value * float(depth_scale)