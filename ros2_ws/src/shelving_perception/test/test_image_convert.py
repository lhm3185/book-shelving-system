"""cv_bridge 를 대신하는 영상 변환기 시험.

왜 직접 만들었나: cv_bridge 의 컴파일된 부분이 NumPy 1.x 로 빌드돼 있어,
NumPy 2.x 인 PC(2026-09-21 GPU PC 10.10.0.1, numpy 2.5.3 / OpenCV 5.0.0)에서
`imgmsg_to_cv2` 는 세그폴트로, `cv2_to_imgmsg` 는 KeyError 로 죽었다.
여기 변환기는 순수 파이썬이라 OpenCV·NumPy 판 번호와 무관하다.
"""

import numpy as np
import pytest

from sensor_msgs.msg import Image

from shelving_perception.vision_manager import (
    _bgr8_to_imgmsg,
    _imgmsg_to_array,
    _imgmsg_to_bgr,
)


def _msg(array, encoding, step=None):
    """numpy 배열로 Image 메시지를 만든다 (step 을 넘기면 줄 끝 padding 을 넣는다)."""
    msg = Image()
    msg.height, msg.width = array.shape[0], array.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    row_bytes = array.shape[1] * (array.shape[2] if array.ndim == 3 else 1) * array.itemsize
    msg.step = int(step or row_bytes)
    if msg.step == row_bytes:
        msg.data = array.tobytes()
    else:
        pad = bytes(msg.step - row_bytes)
        msg.data = b''.join(array[i].tobytes() + pad for i in range(msg.height))
    return msg


def test_bgr8_round_trip():
    """BGR 배열 → 메시지 → 배열 이 원본과 **바이트까지** 같아야 한다."""
    rng = np.random.default_rng(0)
    image = rng.integers(0, 256, size=(7, 5, 3), dtype=np.uint8)
    back = _imgmsg_to_array(_bgr8_to_imgmsg(image))
    assert back.shape == image.shape
    assert np.array_equal(back, image)


def test_bgr8_to_imgmsg_header_fields():
    """step 은 가로 × 3 이고, encoding 은 bgr8 이다."""
    msg = _bgr8_to_imgmsg(np.zeros((4, 6, 3), np.uint8))
    assert (msg.height, msg.width, msg.encoding, msg.step) == (4, 6, 'bgr8', 18)
    assert len(msg.data) == 4 * 18


def test_rgb8_is_flipped_to_bgr():
    """rgb8 로 들어온 영상은 채널이 뒤집혀 나와야 한다 — 안 그러면 빨강/파랑이 바뀐다."""
    rgb = np.zeros((2, 2, 3), np.uint8)
    rgb[..., 0] = 200        # R
    bgr = _imgmsg_to_bgr(_msg(rgb, 'rgb8'))
    assert np.array_equal(bgr[..., 2], np.full((2, 2), 200, np.uint8))
    assert bgr[..., 0].max() == 0


def test_bgr8_passes_through_untouched():
    """bgr8 은 변환 없이 그대로 나온다."""
    rng = np.random.default_rng(1)
    image = rng.integers(0, 256, size=(3, 4, 3), dtype=np.uint8)
    assert np.array_equal(_imgmsg_to_bgr(_msg(image, 'bgr8')), image)


def test_depth_32fc1_keeps_values():
    """깊이는 배율을 건드리지 않고 float 값 그대로 나와야 한다."""
    depth = np.array([[0.5, 1.25], [2.0, np.inf]], np.float32)
    back = _imgmsg_to_array(_msg(depth, '32FC1'))
    assert back.dtype == np.float32
    assert np.array_equal(back[:, :1], depth[:, :1])


def test_depth_16uc1_keeps_values():
    """16UC1 깊이(mm) 도 값이 보존된다."""
    depth = np.array([[1000, 65535], [0, 1]], np.uint16)
    assert np.array_equal(_imgmsg_to_array(_msg(depth, '16UC1')), depth)


def test_row_padding_is_stripped():
    """**step 이 가로보다 크면 줄 끝 padding 을 버려야 한다.**

    그냥 reshape 하면 영상이 한 줄씩 밀려 비스듬하게 보인다.
    """
    rng = np.random.default_rng(2)
    image = rng.integers(0, 256, size=(4, 5, 3), dtype=np.uint8)
    back = _imgmsg_to_array(_msg(image, 'bgr8', step=5 * 3 + 7))
    assert np.array_equal(back, image)


def test_unknown_encoding_raises():
    """모르는 encoding 은 조용히 넘어가지 않는다."""
    with pytest.raises(ValueError):
        _imgmsg_to_array(_msg(np.zeros((2, 2), np.uint8), 'yuv422'))
