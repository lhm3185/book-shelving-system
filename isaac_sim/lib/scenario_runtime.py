"""Isaac Timeline 기반 시나리오 실행 상태를 관리한다."""

from __future__ import annotations

import json
import time
from typing import Callable
import uuid

import omni.timeline
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class ScenarioRuntime:
    """Stop/Play 감지와 시나리오 reset 순서를 관리한다."""

    SCHEMA = "scenario_state/v1"

    STOPPED = "STOPPED"
    RESETTING = "RESETTING"
    READY = "READY"
    RUNNING = "RUNNING"
    FAILED = "FAILED"

    def __init__(
        self,
        node: Node,
        timeline,
        state_topic: str = "/simulation/scenario/state",
    ) -> None:
        self._node = node
        self._timeline = timeline

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._state_publisher = node.create_publisher(
            String,
            state_topic,
            state_qos,
        )

        self._timeline_subscription = (
            timeline
            .get_timeline_event_stream()
            .create_subscription_to_pop(
                self._on_timeline_event,
                name="shelving_scenario_runtime",
            )
        )

        self._reset_hooks: list[
            tuple[str, Callable[[], None]]
        ] = []

        self._run_id = ""
        self._state = self.STOPPED
        self._state_message = ""
        self._last_state_publish_at = 0.0

        self._started = False
        self._initial_play_requested = False
        self._ready_when_playing = False

        self._stop_event_pending = False
        self._play_event_pending = False
        self._reset_required = False
        self._reset_in_progress = False

        self._publish_state(
            self.STOPPED,
            "scenario runtime initialized",
        )

    @property
    def run_id(self) -> str:
        """현재 시나리오 실행 식별자를 반환한다."""
        return self._run_id

    @property
    def state(self) -> str:
        """현재 시나리오 상태를 반환한다."""
        return self._state

    def add_reset_hook(
        self,
        name: str,
        callback: Callable[[], None],
    ) -> None:
        """Stop 후 Play할 때 실행할 reset 함수를 등록한다."""
        if not name:
            raise ValueError("reset hook name must not be empty")

        if any(
            existing_name == name
            for existing_name, _ in self._reset_hooks
        ):
            raise ValueError(
                f"duplicate reset hook: {name}"
            )

        self._reset_hooks.append((name, callback))

    def start(self) -> None:
        """최초 시뮬레이션 재생을 시작한다."""
        if self._started:
            raise RuntimeError(
                "scenario runtime has already started"
            )

        self._started = True
        self._initial_play_requested = True

        # 최초 PLAY 이벤트가 누락되더라도 첫 update에서
        # READY를 발행할 수 있도록 예약한다.
        self._ready_when_playing = True

        self._timeline.play()
    def update(self) -> None:
        """메인 루프에서 프레임마다 호출한다."""
        if rclpy.ok():
            rclpy.spin_once(
                self._node,
                timeout_sec=0.0,
            )

        self._process_stop_event()
        self._process_play_event()
        self._process_reset()
        self._publish_ready_if_playing()
        self._publish_heartbeat_if_due()

    def mark_running(
        self,
        message: str = "scenario is running",
    ) -> None:
        """실제 작업이 시작됐음을 표시한다."""
        if self._state != self.READY:
            raise RuntimeError(
                "scenario can enter RUNNING only from READY"
            )

        self._publish_state(
            self.RUNNING,
            message,
        )

    def close(self) -> None:
        """Timeline 이벤트 구독을 정리한다."""
        if self._timeline_subscription is not None:
            self._timeline_subscription.unsubscribe()
            self._timeline_subscription = None

    def _on_timeline_event(self, event) -> None:
        """Timeline callback에서는 플래그만 설정한다."""
        event_type = int(event.type)

        if event_type == int(
            omni.timeline.TimelineEventType.STOP
        ):
            self._stop_event_pending = True

        elif event_type == int(
            omni.timeline.TimelineEventType.PLAY
        ):
            self._play_event_pending = True

    def _process_stop_event(self) -> None:
        if not self._stop_event_pending:
            return

        self._stop_event_pending = False
        self._play_event_pending = False
        self._reset_required = True
        self._reset_in_progress = False

        self._publish_state(
            self.STOPPED,
            "timeline stopped",
        )

    def _process_play_event(self) -> None:
        if not self._play_event_pending:
            return

        self._play_event_pending = False

        if self._initial_play_requested:
            self._initial_play_requested = False
            self._ready_when_playing = True
            return

        if not self._reset_required:
            return

        self._reset_in_progress = True

        self._publish_state(
            self.RESETTING,
            "reset started",
        )

        # Timeline 상태 변경은 다음 프레임부터 반영된다.
        self._timeline.pause()

    def _process_reset(self) -> None:
        if not self._reset_in_progress:
            return

        # pause 요청이 아직 반영되지 않았다.
        if self._timeline.is_playing():
            return

        try:
            for name, callback in self._reset_hooks:
                self._node.get_logger().info(
                    f"Running scenario reset hook: {name}"
                )
                callback()

        except Exception as error:
            self._reset_in_progress = False
            self._reset_required = True

            self._publish_state(
                self.FAILED,
                f"reset failed: {type(error).__name__}: {error}",
            )
            return

        self._reset_in_progress = False
        self._reset_required = False
        self._ready_when_playing = True

        self._timeline.play()

    def _publish_ready_if_playing(self) -> None:
        if not self._ready_when_playing:
            return

        if not self._timeline.is_playing():
            return

        self._ready_when_playing = False
        self._run_id = str(uuid.uuid4())

        self._publish_state(
            self.READY,
            "scenario is ready",
        )

    def _publish_heartbeat_if_due(self) -> None:
        """현재 상태를 1초마다 다시 발행한다."""
        elapsed = (
            time.monotonic()
            - self._last_state_publish_at
        )

        if elapsed < 1.0:
            return

        self._publish_state(
            self._state,
            self._state_message,
        )

    def _publish_state(
        self,
        state: str,
        message: str,
    ) -> None:
        self._state = state
        self._state_message = message
        self._last_state_publish_at = time.monotonic()

        payload = {
            "schema": self.SCHEMA,
            "run_id": self._run_id,
            "state": state,
            "message": message,
        }

        ros_message = String()
        ros_message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )

        self._state_publisher.publish(ros_message)

        self._node.get_logger().info(
            "Scenario state: "
            f"state={state}, "
            f"run_id={self._run_id or '<none>'}, "
            f"message={message}"
        )