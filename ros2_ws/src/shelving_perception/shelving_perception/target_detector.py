"""Depth 영상에서 책장 빈 공간의 중심과 전면 좌표를 찾는 모듈."""

import math

import cv2
import numpy as np


class TargetDetector:
    """책장 ROI에서 중심과 좌우 Depth를 비교해 빈 공간을 찾습니다."""

    def __init__(
        self,
        sample_radius_px=5,
        side_offset_px=25,
        min_side_depth_difference=0.05,
    ):
        if sample_radius_px < 1:
            raise ValueError('sample_radius_px must be positive')
        if side_offset_px < 1:
            raise ValueError('side_offset_px must be positive')
        if min_side_depth_difference < 0:
            raise ValueError(
                'min_side_depth_difference must be non-negative')

        # 목표점과 좌우 지점 주변에서 깊이를 샘플링할 픽셀 반경입니다.
        self.sample_radius_px = int(sample_radius_px)
        # 목표점 좌우의 깊이를 읽을 때 중심에서 떨어질 픽셀 거리입니다.
        self.side_offset_px = int(side_offset_px)
        # 중심이 좌우보다 이 값 이상 멀어야 빈 공간 경계로 인정합니다.
        self.min_side_depth_difference = float(min_side_depth_difference)

    def find_empty_position(
        self,
        depth_image,
        shelf_box,
        fx,
        fy,
        cx,
        cy,
        depth_scale=1.0,
    ):
        """책장 ROI에서 좌우보다 깊은 빈 공간의 중심을 찾습니다."""
        if depth_image is None or shelf_box is None:
            return None

        height, width = depth_image.shape[:2]
        x1, y1, x2, y2 = map(int, shelf_box)
        x1 = max(0, min(width - 1, x1))
        x2 = max(x1 + 1, min(width, x2))
        y1 = max(0, min(height - 1, y1))
        y2 = max(y1 + 1, min(height, y2))

        radius = max(1, self.sample_radius_px)
        offset = max(radius + 1, self.side_offset_px)
        if x2 - x1 <= 2 * offset:
            return None

        depth_m = depth_image.astype(np.float32) * float(depth_scale)
        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        # Isaac depth는 이 테스트에서 조밀하지만, invalid가 섞여도 blur에
        # 전파되지 않도록 0으로 둔 뒤 최종 mask에서 다시 제외합니다.
        filtered = np.where(valid, depth_m, 0.0).astype(np.float32)
        kernel = 2 * radius + 1
        # OpenCV 5의 medianBlur는 float32 단일 채널을 받지 않으므로,
        # 작은 box 평균으로 센서 노이즈만 완화합니다.
        filtered = cv2.blur(filtered, (kernel, kernel))

        candidate = np.zeros((height, width), dtype=np.uint8)
        xs = slice(x1 + offset, x2 - offset)
        ys = slice(y1, y2)
        center = filtered[ys, xs]
        left = filtered[ys, slice(x1, x2 - 2 * offset)]
        right = filtered[ys, slice(x1 + 2 * offset, x2)]
        local_valid = (
            valid[ys, xs]
            & valid[ys, slice(x1, x2 - 2 * offset)]
            & valid[ys, slice(x1 + 2 * offset, x2)]
        )
        local_candidate = (
            local_valid
            & ((center - left) >= self.min_side_depth_difference)
            & ((center - right) >= self.min_side_depth_difference)
        )
        candidate[ys, xs] = local_candidate.astype(np.uint8)

        count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            candidate, connectivity=8)
        best = None
        for label in range(1, count):
            comp_x, comp_y, comp_w, comp_h, area = map(int, stats[label])
            # 한두 픽셀의 depth 노이즈나 선반 모서리를 빈 칸으로 보지 않습니다.
            if area < 30 or comp_w < 3 or comp_h < 10:
                continue
            rank = (area, comp_h, comp_w)
            if best is None or rank > best[0]:
                best = (rank, label, (comp_x, comp_y, comp_w, comp_h))

        if best is None:
            return None

        _, label, bounds = best
        u_float, v_float = centroids[label]
        u, v = int(round(u_float)), int(round(v_float))
        sample = self._sample_depth_window(
            depth_image, u, v, radius, depth_scale)
        center_depth = sample['median']
        if center_depth is None:
            return None

        background_xyz = (
            (u - float(cx)) * center_depth / float(fx),
            (v - float(cy)) * center_depth / float(fy),
            center_depth,
        )
        inspection = self.inspect_target_position(
            depth_image,
            {'target_xyz': background_xyz, 'camera_xyz': background_xyz},
            fx, fy, cx, cy, depth_scale,
        )
        side_depths = [
            value for value in (
                inspection.get('left_depth'), inspection.get('right_depth'))
            if value is not None and math.isfinite(value) and value > 0.0
        ]
        if len(side_depths) != 2:
            return None
        # 책장 뒤가 열린 구조이므로 중앙 depth는 뒤쪽 배경까지의 거리입니다.
        # 반환점은 양옆 책/기둥으로 추정한 책장 전면 평면에 놓습니다.
        shelf_front_depth = float(np.median(side_depths))
        camera_xyz = (
            (u - float(cx)) * shelf_front_depth / float(fx),
            (v - float(cy)) * shelf_front_depth / float(fy),
            shelf_front_depth,
        )
        inspection['background_xyz'] = background_xyz
        inspection['camera_xyz'] = camera_xyz
        inspection['target_xyz'] = camera_xyz
        inspection['expected_depth'] = shelf_front_depth
        inspection['shelf_front_depth'] = shelf_front_depth
        inspection['type'] = 'detected_empty_shelf_position'
        inspection['component_bounds'] = bounds
        inspection['component_area'] = int(stats[label, cv2.CC_STAT_AREA])
        return inspection

    def inspect_target_position(
        self,
        depth_image,
        target,
        fx,
        fy,
        cx,
        cy,
        depth_scale=1.0,
    ):
        """
        월드 목표점의 투영 위치와 중심/좌/우 Depth 차이를 측정합니다.

        ``target``은 ``target_xyz``와 현재 카메라 기준 ``camera_xyz``를
        가져야 합니다. 결과는 판정 성공 여부와 무관하게 반환하므로, 화면에서
        투영점이 맞는지 먼저 확인할 수 있습니다.
        """
        if depth_image is None or not target:
            return None

        height, width = depth_image.shape[:2]
        camera_x, camera_y, camera_z = map(float, target['camera_xyz'])
        result = {
            'target_xyz': tuple(map(float, target['target_xyz'])),
            'camera_xyz': (camera_x, camera_y, camera_z),
            'expected_depth': camera_z,
            'pixel': None,
            'visible': False,
            'center_depth': None,
            'left_depth': None,
            'right_depth': None,
            'left_difference': None,
            'right_difference': None,
            'left_edge': False,
            'right_edge': False,
            'is_empty': False,
            'sample_windows': {},
            'type': 'empty_shelf_position_diagnostic',
        }
        if not math.isfinite(camera_z) or camera_z <= 0:
            return result

        u = int(round(fx * camera_x / camera_z + cx))
        v = int(round(fy * camera_y / camera_z + cy))
        result['pixel'] = (u, v)
        if not (0 <= u < width and 0 <= v < height):
            return result
        result['visible'] = True

        radius = max(1, self.sample_radius_px)
        offset = max(radius + 1, self.side_offset_px)
        samples = {
            'left': self._sample_depth_window(
                depth_image, u - offset, v, radius, depth_scale),
            'center': self._sample_depth_window(
                depth_image, u, v, radius, depth_scale),
            'right': self._sample_depth_window(
                depth_image, u + offset, v, radius, depth_scale),
        }
        result['sample_windows'] = {
            name: sample['bounds'] for name, sample in samples.items()
        }
        result['left_depth'] = samples['left']['median']
        result['center_depth'] = samples['center']['median']
        result['right_depth'] = samples['right']['median']

        center_depth = result['center_depth']
        if center_depth is None:
            return result
        for side in ('left', 'right'):
            side_depth = result[f'{side}_depth']
            if side_depth is None:
                continue
            difference = center_depth - side_depth
            result[f'{side}_difference'] = difference
            result[f'{side}_edge'] = (
                difference >= self.min_side_depth_difference)

        result['is_empty'] = bool(
            result['left_edge'] and result['right_edge'])
        return result

    @staticmethod
    def _sample_depth_window(
        depth_image,
        center_u,
        center_v,
        radius,
        depth_scale,
    ):
        """한 픽셀 주변의 유효 Depth 중앙값과 실제 샘플 범위를 반환합니다."""
        height, width = depth_image.shape[:2]
        x1 = max(0, int(center_u) - radius)
        x2 = min(width, int(center_u) + radius + 1)
        y1 = max(0, int(center_v) - radius)
        y2 = min(height, int(center_v) + radius + 1)
        bounds = (x1, y1, x2, y2)
        if x1 >= x2 or y1 >= y2:
            return {'median': None, 'valid_count': 0, 'bounds': bounds}

        values = depth_image[y1:y2, x1:x2].reshape(-1)
        valid_values = values[np.isfinite(values) & (values > 0)]
        if valid_values.size == 0:
            return {'median': None, 'valid_count': 0, 'bounds': bounds}

        depths_m = valid_values.astype(float) * float(depth_scale)
        return {
            'median': float(np.median(depths_m)),
            'valid_count': int(depths_m.size),
            'bounds': bounds,
        }
