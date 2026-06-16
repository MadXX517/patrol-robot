#!/usr/bin/env python3
# encoding: utf-8
import json
import os
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import ObjectsInfo
from car_report.security_events import (
    ConfirmFrameTracker,
    class_key,
    parse_target_classes,
    security_event_metadata,
)
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


DEFAULT_MAX_IMAGE_BYTES = 5 * 1024 * 1024


def default_output_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory('car_report')) / 'data')
    except Exception:
        return str(Path(__file__).resolve().parents[1] / 'data')


def expand_path(path):
    return Path(os.path.expanduser(str(path))).resolve()


class EventRecorderNode(Node):
    def __init__(self):
        super().__init__('event_recorder')

        self.declare_parameter('objects_topic', '/car_yolo/object_detect')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('output_dir', default_output_dir())
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('cooldown_sec', 5.0)
        self.declare_parameter('target_classes', '')
        self.declare_parameter('save_image', True)
        self.declare_parameter('max_image_bytes', DEFAULT_MAX_IMAGE_BYTES)
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('confirm_frames', 1)
        self.declare_parameter('area_name', '通信节点外围警戒线')

        self.objects_topic = self.get_parameter('objects_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.output_dir = expand_path(self.get_parameter('output_dir').value)
        self.min_score = max(0.0, float(self.get_parameter('min_score').value))
        self.cooldown_sec = max(0.0, float(self.get_parameter('cooldown_sec').value))
        self.save_image = bool(self.get_parameter('save_image').value)
        self.max_image_bytes = max(1, int(self.get_parameter('max_image_bytes').value))
        self.jpeg_quality = max(30, min(95, int(self.get_parameter('jpeg_quality').value)))
        self.confirm_frames = max(1, int(self.get_parameter('confirm_frames').value))
        self.area_name = str(self.get_parameter('area_name').value or '通信节点外围警戒线')
        self.target_classes = parse_target_classes(
            self.get_parameter('target_classes').value
        )

        self.events_dir = self.output_dir / 'events'
        self.images_dir = self.events_dir / 'images'
        self.events_dir.mkdir(parents=True, exist_ok=True)
        if self.save_image:
            self.images_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_size = (0, 0)
        self.last_event_time = {}
        self.confirm_tracker = ConfirmFrameTracker(self.confirm_frames)

        self.event_pub = self.create_publisher(String, '/car_report/event', 10)
        self.create_subscription(Image, self.image_topic, self.image_callback, 1)
        self.create_subscription(ObjectsInfo, self.objects_topic, self.objects_callback, 10)

        self.get_logger().info(
            'event_recorder started: objects_topic=%s image_topic=%s output_dir=%s '
            'min_score=%.2f cooldown_sec=%.2f target_classes=%s save_image=%s '
            'max_image_bytes=%d jpeg_quality=%d confirm_frames=%d area_name=%s'
            % (
                self.objects_topic,
                self.image_topic,
                str(self.output_dir),
                self.min_score,
                self.cooldown_sec,
                sorted(self.target_classes),
                self.save_image,
                self.max_image_bytes,
                self.jpeg_quality,
                self.confirm_frames,
                self.area_name,
            )
        )

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'failed to convert image: {e}')
            return

        self.latest_image = image
        height, width = image.shape[:2]
        self.latest_image_size = (width, height)

    def objects_callback(self, msg):
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9
        candidates = {}

        for obj in msg.objects:
            class_name = obj.class_name
            score = float(obj.score)
            if not self._passes_score_and_class(class_name, score):
                continue
            key = class_key(class_name)
            if key not in candidates or score > float(candidates[key].score):
                candidates[key] = obj

        confirmed_keys = self.confirm_tracker.update(candidates.keys())

        for key, obj in candidates.items():
            class_name = obj.class_name
            score = float(obj.score)
            if key not in confirmed_keys or not self._passes_cooldown(key, now_sec):
                continue
            self.last_event_time[class_key(class_name)] = now_sec
            event = self._build_event(obj)
            self._write_event(event)
            self.event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))
            self.get_logger().info(
                'recorded event: %s %.2f' % (event['class_name'], event['score'])
            )

    def _passes_score_and_class(self, class_name, score):
        key = class_key(class_name)
        if score < self.min_score:
            return False
        if self.target_classes and key not in self.target_classes:
            return False
        return True

    def _passes_cooldown(self, class_key_value, now_sec):
        last_time = self.last_event_time.get(class_key_value)
        if last_time is not None and now_sec - last_time < self.cooldown_sec:
            return False
        return True

    def _build_event(self, obj):
        event_time = datetime.now().astimezone()
        timestamp = event_time.strftime('%Y%m%d_%H%M%S_%f')
        width, height = self._resolve_image_size(obj)
        image_path = self._save_snapshot(obj.class_name, timestamp)
        image_bytes = os.path.getsize(image_path) if image_path else 0
        metadata = security_event_metadata(obj.class_name)

        event = {
            'time': event_time.isoformat(timespec='seconds'),
            'event_type': metadata['event_type'],
            'event_name': metadata['event_name'],
            'priority': metadata['priority'],
            'risk_level': metadata['risk_level'],
            'area': self.area_name,
            'class_name': obj.class_name,
            'score': round(float(obj.score), 4),
            'bbox': [int(value) for value in obj.box],
            'image_width': int(width),
            'image_height': int(height),
            'image_path': image_path,
            'image_bytes': int(image_bytes),
            'source_topic': self.objects_topic,
        }
        event['action'] = metadata['action']
        event['speech'] = metadata['speech']
        return event

    def _resolve_image_size(self, obj):
        width, height = self.latest_image_size
        if width == 0 and getattr(obj, 'width', 0):
            width = int(obj.width)
        if height == 0 and getattr(obj, 'height', 0):
            height = int(obj.height)
        return width, height

    def _save_snapshot(self, class_name, timestamp):
        if not self.save_image or self.latest_image is None:
            return None

        safe_name = ''.join(c if c.isalnum() or c in ('-', '_') else '_' for c in class_name)
        image_file = self.images_dir / f'{timestamp}_{safe_name}.jpg'
        try:
            image_bytes = self._encode_jpeg_under_limit(self.latest_image)
            image_file.write_bytes(image_bytes)
        except Exception as e:
            self.get_logger().error(f'failed to save snapshot: {e}')
            return None
        return str(image_file)

    def _encode_jpeg_under_limit(self, image):
        quality = max(30, min(95, self.jpeg_quality))
        resized = image

        for _ in range(8):
            ok, encoded = cv2.imencode(
                '.jpg',
                resized,
                [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            )
            if not ok:
                raise RuntimeError('cv2.imencode failed')
            data = encoded.tobytes()
            if len(data) <= self.max_image_bytes:
                return data

            if quality > 45:
                quality -= 15
            else:
                height, width = resized.shape[:2]
                new_size = (max(1, int(width * 0.8)), max(1, int(height * 0.8)))
                resized = cv2.resize(resized, new_size)

        raise RuntimeError(
            f'image is still larger than {self.max_image_bytes} bytes after compression'
        )

    def _write_event(self, event):
        day = datetime.now().strftime('%Y%m%d')
        event_file = self.events_dir / f'events_{day}.jsonl'
        with event_file.open('a', encoding='utf-8') as f:
            f.write(json.dumps(event, ensure_ascii=False) + '\n')


def main():
    rclpy.init()
    node = EventRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
