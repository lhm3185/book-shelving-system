"""YOLO 검출 결과와 깊이 영상을 이용해 책의 3차원 위치를 계산하는 모듈."""

# 배열 계산과 유효한 깊이값 필터링에 사용하는 NumPy를 가져옵니다.
import numpy as np


class BookDetector:
    """책의 2차원 검출 결과를 카메라 기준 3차원 좌표로 변환합니다."""

    def process(
        self,
        detections,
        depth_image,
        fx,
        fy,
        cx,
        cy,
        depth_scale=1.0,
    ):
        """검출 상자와 깊이 영상으로 책별 위치·각도 정보를 계산합니다.

        Args:
            detections: YOLO가 반환한 검출 결과 목록입니다.
            depth_image: RGB 영상과 정렬된 깊이 영상입니다.
            fx, fy: 카메라의 x/y 초점거리입니다.
            cx, cy: 카메라 주점(principal point) 좌표입니다.
            depth_scale: 깊이 원시값을 미터로 바꾸는 배율입니다.

        Returns:
            각 책의 카메라 기준 xyz 좌표, 검출 상자, 중심점,
            신뢰도, 영상상의 회전각을 담은 딕셔너리 목록입니다.
        """
        # 최종적으로 유효한 책만 저장할 빈 목록을 만듭니다.
        detected_targets = []

        # YOLO가 찾은 각각의 객체를 하나씩 처리합니다.
        for detection in detections:
            # 새 형식은 상자·신뢰도·마스크를 담은 딕셔너리입니다.
            if isinstance(detection, dict):
                # 딕셔너리에서 2차원 바운딩 박스를 꺼냅니다.
                box = detection['box']
                # 신뢰도가 없으면 기본값 1.0을 사용합니다.
                confidence = float(detection.get('confidence', 1.0))
                # 세그멘테이션 마스크가 있으면 꺼냅니다.
                mask = detection.get('mask')
            else:
                # 예전 호출 방식처럼 상자만 들어온 경우도 지원합니다.
                box = detection
                # 별도 신뢰도가 없으므로 기본값을 사용합니다.
                confidence = 1.0
                # 마스크가 없으므로 책의 회전각은 0으로 계산됩니다.
                mask = None

            # 상자의 좌표를 정수 픽셀 좌표로 변환합니다.
            x1, y1, x2, y2 = map(int, box)
            # 상자의 가로 중앙 픽셀을 계산합니다.
            u = int((x1 + x2) / 2)
            # 상자의 세로 중앙 픽셀을 계산합니다.
            v = int((y1 + y2) / 2)

            # 중심점이 깊이 영상 범위를 벗어나면 이 검출을 건너뜁니다.
            if not (
                0 <= v < depth_image.shape[0]
                and 0 <= u < depth_image.shape[1]
            ):
                continue

            # 파지할 면의 깊이를 구합니다. **중심 픽셀 하나만 읽으면 안 됩니다** —
            # 그 픽셀이 책과 책 사이 틈에 떨어지면 뒤쪽 바닥 깊이를 읽어
            # 좌표가 0.67 m 까지 튄 적이 있습니다 (2026-09-17 사고 #5).
            raw_depth, depth_source = self._grasp_depth(
                depth_image, mask, x1, y1, x2, y2, u, v)
            z = self._depth_in_meters(raw_depth, depth_scale)
            # 깊이가 없거나 잘못된 값이면 3차원 좌표를 계산할 수 없습니다.
            if z is None:
                continue

            # 핀홀 카메라 모델로 카메라 기준 x 좌표를 계산합니다.
            x = (u - cx) * z / fx
            # 핀홀 카메라 모델로 카메라 기준 y 좌표를 계산합니다.
            y = (v - cy) * z / fy

            # 한 권의 책에 대한 모든 계산 결과를 목록에 추가합니다.
            detected_targets.append({
                # 카메라 기준 3차원 위치입니다.
                'xyz': (x, y, z),
                # 원래 검출된 2차원 상자입니다.
                'box': (x1, y1, x2, y2),
                # 검출 상자의 중심 픽셀입니다.
                'center': (u, v),
                # YOLO가 계산한 검출 신뢰도입니다.
                'confidence': confidence,
                # 마스크의 주축으로 계산한 책의 영상상 회전각입니다.
                'image_angle': self._mask_angle(mask),
                # 깊이를 어디서 얻었는지 (mask / patch / center). 틀린 좌표를 추적할 때 쓴다
                'depth_source': depth_source,
            })

        # 유효한 책 검출 결과를 vision_manager로 반환합니다.
        return detected_targets

    def _mask_angle(self, mask):
        """세그멘테이션 마스크의 긴 축 방향을 라디안으로 계산합니다."""
        # 마스크가 없으면 회전각을 알 수 없으므로 0을 반환합니다.
        if mask is None:
            return 0.0

        # 마스크에서 True인 픽셀의 (행, 열) 좌표를 가져옵니다.
        points = np.column_stack(np.nonzero(mask))
        # 점이 2개 미만이면 방향을 계산할 수 없습니다.
        if len(points) < 2:
            return 0.0

        # 좌표의 평균을 빼서 점 구름의 중심을 원점으로 옮깁니다.
        centered = points - points.mean(axis=0)
        # 중심화된 점들의 공분산 행렬을 계산합니다.
        covariance = np.cov(centered, rowvar=False)
        # 공분산 행렬의 고유값과 고유벡터를 계산합니다.
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        # 가장 큰 고유값에 해당하는 고유벡터를 책의 주축으로 선택합니다.
        axis = eigenvectors[:, int(np.argmax(eigenvalues))]
        # 주축 벡터를 이용해 영상 좌표계 기준 회전각을 계산합니다.
        return float(np.arctan2(axis[0], axis[1]))

    def _grasp_depth(self, depth_image, mask, x1, y1, x2, y2, u, v):
        """**파지할 윗면**의 깊이를 강건하게 구합니다. (깊이, 출처) 를 돌려줍니다.

        손목 카메라는 트레이를 내려다보므로, 책의 **윗면이 카메라에 가장 가깝습니다.**
        그래서 우선순위는 이렇습니다.

        1. **마스크 안 픽셀의 중앙값** — 책 영역만 보므로 배경이 안 섞인다 (가장 좋음)
        2. 상자 **중앙 40% 패치**의 중앙값 — 마스크가 없을 때. 가장자리 배경을 피한다
        3. 중심 픽셀 하나 — 위가 다 실패했을 때만

        상자 **전체**의 중앙값은 쓰지 않습니다 — 세운 책의 상자에는 옆 책과 트레이 바닥이
        많이 들어와 중앙값이 배경으로 끌려갑니다.
        """
        height, width = depth_image.shape[:2]
        # 1) 마스크 안쪽
        if mask is not None and mask.shape[:2] == depth_image.shape[:2]:
            values = depth_image[mask]
            valid = values[np.isfinite(values) & (values > 0)]
            if valid.size >= 10:
                return float(np.median(valid)), 'mask'
        # 2) 상자 중앙 40% 패치
        bw, bh = max(1, x2 - x1), max(1, y2 - y1)
        px1 = max(0, min(width, int(x1 + bw * 0.3)))
        px2 = max(0, min(width, int(x2 - bw * 0.3)))
        py1 = max(0, min(height, int(y1 + bh * 0.3)))
        py2 = max(0, min(height, int(y2 - bh * 0.3)))
        if px2 > px1 and py2 > py1:
            values = depth_image[py1:py2, px1:px2].reshape(-1)
            valid = values[np.isfinite(values) & (values > 0)]
            if valid.size >= 10:
                return float(np.median(valid)), 'patch'
        # 3) 중심 픽셀 (마지막 수단)
        return self._center_depth(depth_image, u, v), 'center'

    def _sample_depth(self, depth_image, x1, y1, x2, y2):
        """검출 상자 내부의 유효 깊이값 중앙값을 반환합니다."""
        # 깊이 영상의 높이와 너비를 가져옵니다.
        height, width = depth_image.shape[:2]
        # 상자 좌표를 영상 범위 안으로 잘라냅니다.
        x1 = max(0, min(width, x1))
        x2 = max(0, min(width, x2))
        y1 = max(0, min(height, y1))
        y2 = max(0, min(height, y2))
        # 상자 크기가 없으면 깊이를 샘플링할 수 없습니다.
        if x1 >= x2 or y1 >= y2:
            return None

        # 상자 영역을 1차원 배열로 펼칩니다.
        values = depth_image[y1:y2, x1:x2].reshape(-1)
        # NaN, 무한대, 0 이하 값을 제거하고 유효값만 남깁니다.
        valid_values = values[np.isfinite(values) & (values > 0)]
        # 유효한 깊이값이 없으면 실패를 나타냅니다.
        if valid_values.size == 0:
            return None
        # 이상치 영향을 줄이기 위해 평균 대신 중앙값을 반환합니다.
        return np.median(valid_values)

    def _center_depth(self, depth_image, u, v, radius=2):
        """중심 픽셀 깊이를 읽고, 잘못되면 주변 영역으로 대체합니다."""
        # 깊이 영상의 높이와 너비를 가져옵니다.
        height, width = depth_image.shape[:2]
        # 우선 검출 중심 픽셀의 깊이를 확인합니다.
        center_value = depth_image[v, u]
        # 중심 픽셀의 값이 유효하면 그대로 사용합니다.
        if np.isfinite(center_value) and center_value > 0:
            return center_value

        # 중심 픽셀이 invalid일 때 사용할 주변 영역의 시작 x를 계산합니다.
        x0 = max(0, u - radius)
        # 주변 영역의 끝 x를 영상 너비 안으로 제한합니다.
        x1 = min(width, u + radius + 1)
        # 주변 영역의 시작 y를 영상 높이 안으로 제한합니다.
        y0 = max(0, v - radius)
        # 주변 영역의 끝 y를 영상 높이 안으로 제한합니다.
        y1 = min(height, v + radius + 1)
        # 주변 영역의 깊이값을 1차원으로 펼칩니다.
        values = depth_image[y0:y1, x0:x1].reshape(-1)
        # 주변 영역에서 유효한 깊이값만 남깁니다.
        valid_values = values[np.isfinite(values) & (values > 0)]
        # 주변에도 유효한 깊이가 없으면 실패를 반환합니다.
        if valid_values.size == 0:
            return None
        # 주변 유효 깊이의 중앙값을 사용합니다.
        return np.median(valid_values)

    def _depth_in_meters(self, depth_value, depth_scale=1.0):
        """깊이 원시값을 검증하고 미터 단위로 변환합니다."""
        # 입력 깊이가 없으면 변환할 수 없습니다.
        if depth_value is None:
            return None
        # NumPy 값 등을 일반 실수형으로 변환합니다.
        value = float(depth_value)
        # NaN, 무한대, 0 이하 값은 잘못된 깊이로 처리합니다.
        if not np.isfinite(value) or value <= 0:
            return None
        # 카메라 깊이 단위의 배율을 곱해 미터 단위로 반환합니다.
        return value * float(depth_scale)
