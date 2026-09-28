"""Depth 영상에서 책장 빈 공간의 실제 중심을 찾는다."""

import math

import cv2
import numpy as np


class TargetDetector:
    """책장 ROI에서 좌우 책보다 깊은 연속 영역을 빈 공간으로 검출한다."""

    def __init__(
        self,
        sample_radius_px=5,
        side_offset_px=25,
        min_side_depth_difference=0.05,
    ):
        if sample_radius_px < 1:
            raise ValueError("sample_radius_px must be positive")
        if side_offset_px < 1:
            raise ValueError("side_offset_px must be positive")
        if min_side_depth_difference < 0:
            raise ValueError(
                "min_side_depth_difference must be non-negative"
            )

        self.sample_radius_px = int(sample_radius_px)
        self.side_offset_px = int(side_offset_px)
        self.min_side_depth_difference = float(
            min_side_depth_difference
        )

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
        """책장 ROI에서 가장 큰 빈 공간의 책장 전면 좌표를 반환한다."""
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
        filtered = np.where(valid, depth_m, 0.0).astype(np.float32)
        kernel = 2 * radius + 1
        filtered = cv2.blur(filtered, (kernel, kernel))

        candidate = np.zeros((height, width), dtype=np.uint8)
        center_x = slice(x1 + offset, x2 - offset)
        rows = slice(y1, y2)
        center = filtered[rows, center_x]
        left = filtered[rows, slice(x1, x2 - 2 * offset)]
        right = filtered[rows, slice(x1 + 2 * offset, x2)]
        local_valid = (
            valid[rows, center_x]
            & valid[rows, slice(x1, x2 - 2 * offset)]
            & valid[rows, slice(x1 + 2 * offset, x2)]
        )
        local_candidate = (
            local_valid
            & (
                (center - left)
                >= self.min_side_depth_difference
            )
            & (
                (center - right)
                >= self.min_side_depth_difference
            )
        )
        candidate[rows, center_x] = local_candidate.astype(np.uint8)

        count, labels, stats, centroids = (
            cv2.connectedComponentsWithStats(
                candidate,
                connectivity=8,
            )
        )

        best = None
        for label in range(1, count):
            comp_x, comp_y, comp_w, comp_h, area = map(
                int,
                stats[label],
            )
            if area < 30 or comp_w < 3 or comp_h < 10:
                continue
            rank = (area, comp_h, comp_w)
            if best is None or rank > best[0]:
                best = (
                    rank,
                    label,
                    (comp_x, comp_y, comp_w, comp_h),
                )

        if best is None:
            return None

        _, label, bounds = best
        u_float, v_float = centroids[label]
        u, v = int(round(u_float)), int(round(v_float))

        center_sample = self._sample_depth_window(
            depth_image,
            u,
            v,
            radius,
            depth_scale,
        )
        center_depth = center_sample["median"]
        if center_depth is None:
            return None

        background_xyz = (
            (u - float(cx)) * center_depth / float(fx),
            (v - float(cy)) * center_depth / float(fy),
            center_depth,
        )
        inspection = self.inspect_target_position(
            depth_image,
            {
                "target_xyz": background_xyz,
                "camera_xyz": background_xyz,
            },
            fx,
            fy,
            cx,
            cy,
            depth_scale,
        )
        if inspection is None:
            return None

        side_depths = [
            value
            for value in (
                inspection.get("left_depth"),
                inspection.get("right_depth"),
            )
            if (
                value is not None
                and math.isfinite(value)
                and value > 0.0
            )
        ]
        if len(side_depths) != 2:
            return None

        shelf_front_depth = float(np.median(side_depths))
        camera_xyz = (
            (u - float(cx)) * shelf_front_depth / float(fx),
            (v - float(cy)) * shelf_front_depth / float(fy),
            shelf_front_depth,
        )

        inspection.update({
            "background_xyz": background_xyz,
            "camera_xyz": camera_xyz,
            "target_xyz": camera_xyz,
            "expected_depth": shelf_front_depth,
            "shelf_front_depth": shelf_front_depth,
            "type": "detected_empty_shelf_position",
            "component_bounds": bounds,
            "component_area": int(
                stats[label, cv2.CC_STAT_AREA]
            ),
        })
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
        """후보 중심과 좌우 표면의 깊이 차이를 검사한다."""
        if depth_image is None or not target:
            return None

        height, width = depth_image.shape[:2]
        camera_x, camera_y, camera_z = map(
            float,
            target["camera_xyz"],
        )
        result = {
            "target_xyz": tuple(map(float, target["target_xyz"])),
            "camera_xyz": (camera_x, camera_y, camera_z),
            "expected_depth": camera_z,
            "pixel": None,
            "visible": False,
            "center_depth": None,
            "left_depth": None,
            "right_depth": None,
            "left_difference": None,
            "right_difference": None,
            "left_edge": False,
            "right_edge": False,
            "is_empty": False,
            "sample_windows": {},
            "type": "empty_shelf_position_diagnostic",
        }
        if not math.isfinite(camera_z) or camera_z <= 0:
            return result

        u = int(round(fx * camera_x / camera_z + cx))
        v = int(round(fy * camera_y / camera_z + cy))
        result["pixel"] = (u, v)
        if not (0 <= u < width and 0 <= v < height):
            return result
        result["visible"] = True

        radius = max(1, self.sample_radius_px)
        offset = max(radius + 1, self.side_offset_px)
        samples = {
            "left": self._sample_depth_window(
                depth_image,
                u - offset,
                v,
                radius,
                depth_scale,
            ),
            "center": self._sample_depth_window(
                depth_image,
                u,
                v,
                radius,
                depth_scale,
            ),
            "right": self._sample_depth_window(
                depth_image,
                u + offset,
                v,
                radius,
                depth_scale,
            ),
        }
        result["sample_windows"] = {
            name: sample["bounds"]
            for name, sample in samples.items()
        }
        for name in ("left", "center", "right"):
            result[f"{name}_depth"] = samples[name]["median"]

        center_depth = result["center_depth"]
        if center_depth is None:
            return result

        for side in ("left", "right"):
            side_depth = result[f"{side}_depth"]
            if side_depth is None:
                continue
            difference = center_depth - side_depth
            result[f"{side}_difference"] = difference
            result[f"{side}_edge"] = (
                difference
                >= self.min_side_depth_difference
            )

        result["is_empty"] = bool(
            result["left_edge"]
            and result["right_edge"]
        )
        return result

    @staticmethod
    def _sample_depth_window(
        depth_image,
        center_u,
        center_v,
        radius,
        depth_scale,
    ):
        height, width = depth_image.shape[:2]
        x1 = max(0, int(center_u) - radius)
        x2 = min(width, int(center_u) + radius + 1)
        y1 = max(0, int(center_v) - radius)
        y2 = min(height, int(center_v) + radius + 1)
        bounds = (x1, y1, x2, y2)
        if x1 >= x2 or y1 >= y2:
            return {
                "median": None,
                "valid_count": 0,
                "bounds": bounds,
            }

        values = depth_image[y1:y2, x1:x2].reshape(-1)
        valid_values = values[
            np.isfinite(values)
            & (values > 0)
        ]
        if valid_values.size == 0:
            return {
                "median": None,
                "valid_count": 0,
                "bounds": bounds,
            }

        depths_m = (
            valid_values.astype(float)
            * float(depth_scale)
        )
        return {
            "median": float(np.median(depths_m)),
            "valid_count": int(depths_m.size),
            "bounds": bounds,
        }
