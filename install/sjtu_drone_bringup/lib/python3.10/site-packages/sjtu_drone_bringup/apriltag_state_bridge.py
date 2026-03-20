#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Float32MultiArray


class AprilTagStateBridge(Node):
    def __init__(self):
        super().__init__('apriltag_state_bridge')

        self.declare_parameter('input_topic', '/drone/bottom/tags')
        self.declare_parameter('output_topic', '/landing_tag_state')
        self.declare_parameter('target_id', 0)
        self.declare_parameter('use_target_id', True)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.target_id = int(self.get_parameter('target_id').value)
        self.use_target_id = bool(self.get_parameter('use_target_id').value)

        self.pub = self.create_publisher(Float32MultiArray, output_topic, 10)
        self.sub = self.create_subscription(AprilTagDetectionArray, input_topic, self.cb, 10)

        self.get_logger().info(
            f'AprilTag bridge started: {input_topic} -> {output_topic}, '
            f'use_target_id={self.use_target_id}, target_id={self.target_id}'
        )

    def cb(self, msg: AprilTagDetectionArray):
        det = self._select_detection(msg)

        out = Float32MultiArray()
        # data format:
        # [detected(0/1), tag_id, center_x_px, center_y_px, area_px2, margin, num_tags]
        data = [0.0, -1.0, math.nan, math.nan, math.nan, math.nan, float(len(msg.detections))]

        if det is not None:
            center_x, center_y = self._center(det)
            area = self._area(det)
            margin = float(getattr(det, 'decision_margin', math.nan))
            tag_id = float(getattr(det, 'id', -1))
            data = [1.0, tag_id, center_x, center_y, area, margin, float(len(msg.detections))]

        out.data = data
        self.pub.publish(out)

    def _select_detection(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            return None

        if self.use_target_id:
            for det in msg.detections:
                if int(getattr(det, 'id', -9999)) == self.target_id:
                    return det
            return None

        best = None
        best_margin = -1e9
        for det in msg.detections:
            margin = float(getattr(det, 'decision_margin', 0.0))
            if margin > best_margin:
                best = det
                best_margin = margin
        return best

    @staticmethod
    def _center(det):
        if hasattr(det, 'centre'):
            return float(det.centre.x), float(det.centre.y)
        if hasattr(det, 'center'):
            return float(det.center.x), float(det.center.y)

        corners = getattr(det, 'corners', [])
        if not corners:
            return math.nan, math.nan
        xs = [float(c.x) for c in corners]
        ys = [float(c.y) for c in corners]
        return sum(xs) / len(xs), sum(ys) / len(ys)

    @staticmethod
    def _area(det):
        corners = getattr(det, 'corners', [])
        if len(corners) < 3:
            return math.nan

        xs = [float(c.x) for c in corners]
        ys = [float(c.y) for c in corners]
        xs.append(xs[0])
        ys.append(ys[0])

        shoelace = 0.0
        for i in range(len(corners)):
            shoelace += xs[i] * ys[i + 1] - xs[i + 1] * ys[i]
        return abs(0.5 * shoelace)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
