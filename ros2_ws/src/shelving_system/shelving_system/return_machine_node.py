"""Virtual return-machine node for publishing tray jobs."""

from collections.abc import Sequence
from typing import Any
from uuid import uuid4

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from shelving_interfaces.msg import TrayJob


def validate_tray_data(
    book_ids: Sequence[str],
    rfid_tags: Sequence[str],
    classification_codes: Sequence[str],
) -> None:
    """Validate book data before creating a TrayJob message."""
    if not book_ids:
        raise ValueError(
            "A tray job must contain at least one book."
        )

    if not (
        len(book_ids)
        == len(rfid_tags)
        == len(classification_codes)
    ):
        raise ValueError(
            "book_ids, rfid_tags, and classification_codes "
            "must have the same length."
        )

    if len(set(book_ids)) != len(book_ids):
        raise ValueError(
            "A tray job must not contain duplicate book IDs."
        )

    fields = {
        "book_ids": book_ids,
        "rfid_tags": rfid_tags,
        "classification_codes": classification_codes,
    }

    for field_name, values in fields.items():
        for index, value in enumerate(values):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(
                    f"{field_name}[{index}] must be "
                    "a non-empty string."
                )


class ReturnMachineNode(Node):
    """Publish virtual returned-book tray jobs."""

    def __init__(self) -> None:
        """Initialize publishers, parameters, service, and timer."""
        super().__init__("return_machine_node")

        self.declare_parameter(
            "tray_job_topic",
            "/return_machine/tray_job",
        )
        self.declare_parameter(
            "publish_service",
            "/return_machine/publish_job",
        )

        self.declare_parameter(
            "tray_id",
            "tray_001",
        )
        self.declare_parameter(
            "book_ids",
            ["book_001"],
        )
        self.declare_parameter(
            "rfid_tags",
            ["rfid_001"],
        )
        self.declare_parameter(
            "classification_codes",
            ["005.7"],
        )

        self.declare_parameter(
            "auto_publish",
            False,
        )
        self.declare_parameter(
            "publish_delay_sec",
            2.0,
        )

        tray_job_topic = str(
            self.get_parameter(
                "tray_job_topic"
            ).value
        )
        publish_service = str(
            self.get_parameter(
                "publish_service"
            ).value
        )

        self._publisher = self.create_publisher(
            TrayJob,
            tray_job_topic,
            10,
        )

        self._publish_service = self.create_service(
            Trigger,
            publish_service,
            self._handle_publish_request,
        )

        self._auto_publish_timer = None

        auto_publish = bool(
            self.get_parameter(
                "auto_publish"
            ).value
        )

        if auto_publish:
            publish_delay_sec = float(
                self.get_parameter(
                    "publish_delay_sec"
                ).value
            )

            if publish_delay_sec <= 0.0:
                raise ValueError(
                    "publish_delay_sec must be positive."
                )

            self._auto_publish_timer = (
                self.create_timer(
                    publish_delay_sec,
                    self._publish_from_timer,
                )
            )

        self.get_logger().info(
            "Return machine node is ready. "
            f"Publishing TrayJob messages on "
            f"'{tray_job_topic}'."
        )
        self.get_logger().info(
            "A tray job can be requested through "
            f"'{publish_service}'."
        )

    def _read_tray_parameters(
        self,
    ) -> tuple[
        str,
        list[str],
        list[str],
        list[str],
    ]:
        """Read and validate tray data from ROS parameters."""
        tray_id = str(
            self.get_parameter("tray_id").value
        ).strip()

        book_ids = list(
            self.get_parameter("book_ids").value
        )
        rfid_tags = list(
            self.get_parameter("rfid_tags").value
        )
        classification_codes = list(
            self.get_parameter(
                "classification_codes"
            ).value
        )

        if not tray_id:
            raise ValueError(
                "tray_id must not be empty."
            )

        validate_tray_data(
            book_ids=book_ids,
            rfid_tags=rfid_tags,
            classification_codes=(
                classification_codes
            ),
        )

        return (
            tray_id,
            book_ids,
            rfid_tags,
            classification_codes,
        )

    def _create_tray_job(self) -> TrayJob:
        """Create one TrayJob message from node parameters."""
        (
            tray_id,
            book_ids,
            rfid_tags,
            classification_codes,
        ) = self._read_tray_parameters()

        message = TrayJob()

        message.job_id = (
            f"job_{uuid4().hex[:12]}"
        )
        message.tray_id = tray_id
        message.created_at = (
            self.get_clock().now().to_msg()
        )

        message.book_ids = book_ids
        message.rfid_tags = rfid_tags
        message.classification_codes = (
            classification_codes
        )

        return message

    def publish_tray_job(self) -> TrayJob:
        """Create and publish one TrayJob message."""
        message = self._create_tray_job()

        self._publisher.publish(message)

        self.get_logger().info(
            "Published TrayJob: "
            f"job_id={message.job_id}, "
            f"tray_id={message.tray_id}, "
            f"books={len(message.book_ids)}"
        )

        return message

    def _handle_publish_request(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Publish a job when the Trigger service is called."""
        del request

        try:
            message = self.publish_tray_job()
        except ValueError as error:
            response.success = False
            response.message = str(error)

            self.get_logger().error(
                f"Failed to publish TrayJob: {error}"
            )

            return response

        response.success = True
        response.message = (
            f"Published job '{message.job_id}' "
            f"for tray '{message.tray_id}'."
        )

        return response

    def _publish_from_timer(self) -> None:
        """Publish one job automatically and stop the timer."""
        try:
            self.publish_tray_job()
        except ValueError as error:
            self.get_logger().error(
                f"Automatic TrayJob failed: {error}"
            )

        if self._auto_publish_timer is not None:
            self.destroy_timer(
                self._auto_publish_timer
            )
            self._auto_publish_timer = None


def main(args: list[str] | None = None) -> None:
    """Run the virtual return-machine node."""
    rclpy.init(args=args)

    node: ReturnMachineNode | None = None

    try:
        node = ReturnMachineNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
