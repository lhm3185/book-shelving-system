"""Depth 영상으로 책장 각 단의 빈 공간을 찾는 보조 모듈."""

# 깊이값이 유한하고 양수인지 확인할 때 사용하는 수학 모듈입니다.
import math

# 깊이 영상 배열 처리와 중앙값 계산에 사용하는 NumPy입니다.
import numpy as np


class TargetDetector:
    """정면에서 본 5단 책장의 빈 단을 깊이값으로 판별합니다."""

    def __init__(
        self,
        row_count=5,
        depth_margin=0.05,
        inner_width_ratio=0.70,
        inner_height_ratio=0.60,
    ):
        # 책장 단의 개수입니다. 현재 학습 데이터는 5단 책장입니다.
        if row_count <= 0:
            # 0단 이하의 책장은 검사할 수 없으므로 오류를 발생시킵니다.
            raise ValueError('row_count must be positive')
        # 외부 파라미터를 정수형 단 개수로 저장합니다.
        self.row_count = int(row_count)

        # 가까운 책 표면과 먼 배경을 구분할 최소 깊이 차이(m)입니다.
        if depth_margin < 0:
            # 음수 깊이 차이는 판정 기준으로 사용할 수 없으므로 오류를 냅니다.
            raise ValueError('depth_margin must be non-negative')
        # 깊이 차이 기준을 실수형으로 저장합니다.
        self.depth_margin = float(depth_margin)

        # 책장 bbox 안에서 좌우 가장자리와 프레임을 제외할 비율입니다.
        # 책이 들어가는 내부 가로 영역의 비율을 저장합니다.
        self.inner_width_ratio = float(inner_width_ratio)
        # 책장 한 단 안에서 위아래 선반판을 제외할 비율입니다.
        # 깊이를 읽을 내부 세로 영역의 비율을 저장합니다.
        self.inner_height_ratio = float(inner_height_ratio)

    def find_empty_slots(
        self,
        depth_image,
        shelf_box,
        fx,
        fy,
        cx,
        cy,
        depth_scale=1.0,
        shelf_class='unknown',
    ):
        """책장 bbox를 5개 단으로 나누고 빈 단의 카메라 좌표를 반환합니다.

        정면 관찰을 가정합니다. 각 단의 중앙 영역에서 깊이를 읽어,
        책장 안에서 가장 가까운 깊이보다 ``depth_margin`` 이상 먼 단을
        빈 공간으로 판정합니다. ``shelf_open``은 학습 데이터상 전체가 빈
        책장이므로 모든 단을 빈 공간으로 취급합니다.
        """
        # 깊이 영상과 책장 bbox가 없으면 빈 결과를 반환합니다.
        if depth_image is None or shelf_box is None:
            return []

        # bbox를 정수 픽셀 좌표로 변환합니다.
        x1, y1, x2, y2 = map(int, shelf_box)
        # 깊이 영상 크기를 가져옵니다.
        height, width = depth_image.shape[:2]
        # bbox가 영상 밖으로 나가지 않도록 잘라냅니다.
        x1 = max(0, min(width - 1, x1))
        x2 = max(0, min(width, x2))
        y1 = max(0, min(height - 1, y1))
        y2 = max(0, min(height, y2))
        # 잘못된 bbox이면 처리할 수 없습니다.
        if x1 >= x2 or y1 >= y2:
            return []

        # 책장 bbox를 좌우 프레임을 제외한 내부 영역으로 줄입니다.
        box_width = x2 - x1
        # 좌우 프레임을 제외한 샘플 영역의 왼쪽 x를 계산합니다.
        inner_x1 = int(x1 + box_width * (1.0 - self.inner_width_ratio) / 2.0)
        # 좌우 프레임을 제외한 샘플 영역의 오른쪽 x를 계산합니다.
        inner_x2 = int(x2 - box_width * (1.0 - self.inner_width_ratio) / 2.0)

        # 각 단의 깊이와 픽셀 중심을 계산합니다.
        row_measurements = []
        # 책장 전체 높이를 단 개수로 나누어 한 단의 높이를 계산합니다.
        row_height = (y2 - y1) / self.row_count
        # 위쪽 단부터 아래쪽 단까지 순서대로 검사합니다.
        for row_index in range(self.row_count):
            # 위에서부터 현재 단의 세로 범위를 계산합니다.
            row_y1 = int(y1 + row_index * row_height)
            row_y2 = int(y1 + (row_index + 1) * row_height)
            # 선반판을 피하기 위해 단 내부의 중앙 세로 영역만 사용합니다.
            usable_height = max(1, row_y2 - row_y1)
            sample_y1 = int(
                row_y1 + usable_height * (1.0 - self.inner_height_ratio) / 2.0)
            sample_y2 = int(
                row_y2 - usable_height * (1.0 - self.inner_height_ratio) / 2.0)
            sample_y1 = max(0, min(height, sample_y1))
            sample_y2 = max(0, min(height, sample_y2))

            # 현재 단의 중앙 영역에서 유효한 깊이값만 추출합니다.
            values = depth_image[sample_y1:sample_y2, inner_x1:inner_x2]
            # NaN, 무한대, 0 이하인 깊이값을 제거합니다.
            valid_values = values[np.isfinite(values) & (values > 0)]
            # 이 단에서 쓸 수 있는 깊이값이 없으면 다음 단으로 넘어갑니다.
            if valid_values.size == 0:
                continue

            # 일부 책 표면이나 노이즈의 영향을 줄이기 위해 중앙값을 사용합니다.
            depth_m = float(np.median(valid_values)) * float(depth_scale)
            # 단 중심 픽셀을 계산합니다.
            center_u = int((inner_x1 + inner_x2) / 2)
            center_v = int((row_y1 + row_y2) / 2)
            # 깊이와 픽셀 위치를 저장합니다.
            row_measurements.append({
                # 책장 내부에서의 단 번호입니다.
                'row_index': row_index,
                # 해당 단을 대표하는 중심 픽셀의 x 좌표입니다.
                'u': center_u,
                # 해당 단을 대표하는 중심 픽셀의 y 좌표입니다.
                'v': center_v,
                # 해당 단 중앙 영역의 대표 깊이(m)입니다.
                'depth': depth_m,
            })

        # 깊이값을 읽을 수 있는 단이 없으면 빈 결과를 반환합니다.
        if not row_measurements:
            return []

        # 전체가 비어 있다고 학습된 shelf_open은 모든 유효 단을 빈 단으로 처리합니다.
        if shelf_class == 'shelf_open':
            empty_rows = row_measurements
        else:
            # 가장 가까운 깊이는 책 표면일 가능성이 높습니다.
            nearest_depth = min(row['depth'] for row in row_measurements)
            # 가까운 표면보다 충분히 먼 단을 빈 단으로 선택합니다.
            empty_rows = [
                row for row in row_measurements
                if row['depth'] >= nearest_depth + self.depth_margin
            ]

            # 깊이 차이가 전혀 없고 shelf_filled로 분류된 경우 빈 단이 없습니다.
            if shelf_class == 'shelf_filled' and not empty_rows:
                empty_rows = []

        # 픽셀 중심과 깊이로 각 빈 단의 카메라 기준 xyz를 계산합니다.
        empty_slots = []
        for row in empty_rows:
            # 대표 깊이를 카메라 좌표계의 z 거리로 사용합니다.
            z = float(row['depth'])
            # 잘못된 깊이는 좌표 변환에서 제외합니다.
            if not math.isfinite(z) or z <= 0:
                continue
            # 핀홀 카메라 모델로 x 좌표를 계산합니다.
            x = (row['u'] - cx) * z / fx
            # 핀홀 카메라 모델로 y 좌표를 계산합니다.
            y = (row['v'] - cy) * z / fy
            # 빈 단의 픽셀·깊이·3D 좌표를 결과 목록에 추가합니다.
            empty_slots.append({
                # 책장 내 단 번호를 보존합니다.
                'row_index': row['row_index'],
                # 디버그 화면에 표시할 중심 픽셀입니다.
                'pixel': (row['u'], row['v']),
                # 미터 단위의 대표 깊이입니다.
                'depth': z,
                # 카메라 기준 x/y/z 좌표입니다.
                'xyz': (x, y, z),
                # 결과의 의미를 구분하는 타입 문자열입니다.
                'type': 'empty_shelf_position',
            })

        # 위쪽 단부터 아래쪽 단 순서로 반환합니다.
        return empty_slots
