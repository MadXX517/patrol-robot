#!/usr/bin/env python3
# encoding: utf-8
import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime


DEFAULT_EVENT_TOPIC = "/car_report/event"
DEFAULT_WEBHOOK_URL = (
    "https://oapi.dingtalk.com/robot/send?"
    "access_token=a6d04355c1119b47c1db727a3e3b54e58eb8dd36456313569b3db5f78c2e3c93"
)
DEFAULT_KEYWORD = "巡逻告警"
DEFAULT_TIMEOUT = 10.0


def parse_target_classes(value):
    if value is None:
        return set()
    return {item.strip().lower() for item in str(value).split(",") if item.strip()}


def event_class_key(event):
    class_name = str(event.get("class_name") or "").strip()
    if class_name:
        return class_name.lower()
    return str(event.get("event_type") or "unknown").lower()


def event_score(event):
    raw_score = event.get("score")
    if raw_score in (None, ""):
        return None
    try:
        return float(raw_score)
    except (TypeError, ValueError):
        return None


def format_score(score):
    if score is None:
        return "未记录"
    return f"{score:.2f}"


def should_notify(
    event, min_score, target_classes, last_sent_at, cooldown_sec, now=None
):
    class_key = event_class_key(event)
    score = event_score(event)
    if score is not None and score < min_score:
        return False, "score below threshold"
    if target_classes and class_key not in target_classes:
        return False, "class not in target_classes"

    now = time.time() if now is None else now
    last_time = last_sent_at.get(class_key)
    if last_time is not None and now - last_time < cooldown_sec:
        return False, "cooldown"
    return True, ""


def build_markdown_payload(event, keyword):
    event_type = event.get("event_type") or "unknown"
    class_name = event.get("class_name") or "unknown"
    score = event_score(event)
    bbox = event.get("bbox") or []
    image_path = event.get("image_path") or "无"
    image_width = event.get("image_width") or 0
    image_height = event.get("image_height") or 0
    image_bytes = event.get("image_bytes") or 0
    source_topic = event.get("source_topic") or ""
    event_time = event.get("time") or ""

    title = f"{keyword} - {event_type}"
    text = (
        f"### {keyword} - {event_type}\n\n"
        f"- 时间：{event_time or '未知'}\n"
        f"- 事件类型：{event_type}\n"
        f"- 目标：{class_name}\n"
        f"- 置信度：{format_score(score)}\n"
        f"- 位置框：{bbox}\n"
        f"- 图像尺寸：{image_width}x{image_height}\n"
        f"- 截图大小：{image_bytes} bytes\n"
        f"- 截图路径：`{image_path}`\n"
        f"- 来源话题：`{source_topic}`\n\n"
        f"> 自动识别结果仅作为巡逻辅助，请结合现场画面人工复核。"
    )
    return {
        "msgtype": "markdown",
        "markdown": {
            "title": title,
            "text": text,
        },
    }


def send_dingtalk_markdown(webhook_url, payload, timeout):
    if not webhook_url:
        raise RuntimeError("DingTalk webhook_url is empty")

    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    request = urllib.request.Request(
        webhook_url,
        data=data,
        headers={"Content-Type": "application/json;charset=utf-8"},
        method="POST",
    )

    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            raw = response.read().decode("utf-8", errors="replace")
    except urllib.error.HTTPError as e:
        error_body = e.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"DingTalk HTTP {e.code}: {error_body}") from e
    except urllib.error.URLError as e:
        raise RuntimeError(f"DingTalk request failed: {e}") from e

    try:
        result = json.loads(raw)
    except json.JSONDecodeError as e:
        raise RuntimeError(f"invalid DingTalk response: {raw}") from e

    if result.get("errcode") != 0:
        raise RuntimeError(f"DingTalk error response: {raw}")
    return result


def parse_event_message(raw):
    try:
        event = json.loads(raw)
    except json.JSONDecodeError as e:
        raise ValueError(f"invalid event json: {e}") from e
    if not isinstance(event, dict):
        raise ValueError("event json must be an object")
    return event


def update_cooldown(last_sent_at, event):
    last_sent_at[event_class_key(event)] = time.time()


def run_webhook_test(webhook_url, keyword, timeout):
    event = {
        "time": datetime.now().astimezone().isoformat(timespec="seconds"),
        "event_type": "webhook_test",
        "class_name": "test",
        "score": 1.0,
        "bbox": [],
        "image_width": 0,
        "image_height": 0,
        "image_path": "无",
        "image_bytes": 0,
        "source_topic": "local_test",
    }
    payload = build_markdown_payload(event, keyword)
    print("=== webhook test payload ===")
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    print("\n=== webhook test response ===")
    result = send_dingtalk_markdown(webhook_url, payload, timeout)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


def run_ros_node():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class DingTalkNotifierNode(Node):
        def __init__(self):
            super().__init__("dingtalk_notifier")

            self.declare_parameter("event_topic", DEFAULT_EVENT_TOPIC)
            self.declare_parameter("webhook_url", "")
            self.declare_parameter("keyword", DEFAULT_KEYWORD)
            self.declare_parameter("cooldown_sec", 5.0)
            self.declare_parameter("min_score", 0.5)
            self.declare_parameter("target_classes", "")
            self.declare_parameter("timeout", DEFAULT_TIMEOUT)
            self.declare_parameter("dry_run", False)

            self.event_topic = self.get_parameter("event_topic").value
            webhook_url = self.get_parameter("webhook_url").value
            self.webhook_url = webhook_url or DEFAULT_WEBHOOK_URL
            self.keyword = self.get_parameter("keyword").value
            self.cooldown_sec = max(
                0.0, float(self.get_parameter("cooldown_sec").value)
            )
            self.min_score = max(0.0, float(self.get_parameter("min_score").value))
            self.target_classes = parse_target_classes(
                self.get_parameter("target_classes").value
            )
            self.timeout = max(1.0, float(self.get_parameter("timeout").value))
            self.dry_run = bool(self.get_parameter("dry_run").value)
            self.last_sent_at = {}

            self.create_subscription(String, self.event_topic, self.event_callback, 10)
            self.get_logger().info(
                "dingtalk_notifier started: event_topic=%s keyword=%s min_score=%.2f "
                "cooldown_sec=%.2f target_classes=%s dry_run=%s webhook_configured=%s"
                % (
                    self.event_topic,
                    self.keyword,
                    self.min_score,
                    self.cooldown_sec,
                    sorted(self.target_classes),
                    self.dry_run,
                    bool(self.webhook_url),
                )
            )
            if not self.webhook_url:
                self.get_logger().warning(
                    "webhook_url is empty; events will not be sent"
                )

        def event_callback(self, msg):
            try:
                event = parse_event_message(msg.data)
            except ValueError as e:
                self.get_logger().warning(str(e))
                return

            allowed, reason = should_notify(
                event=event,
                min_score=self.min_score,
                target_classes=self.target_classes,
                last_sent_at=self.last_sent_at,
                cooldown_sec=self.cooldown_sec,
            )
            if not allowed:
                self.get_logger().debug("skip event: %s" % reason)
                return

            payload = build_markdown_payload(event, self.keyword)
            if self.dry_run or not self.webhook_url:
                self.get_logger().info(
                    "dingtalk dry-run payload: %s"
                    % json.dumps(payload, ensure_ascii=False)
                )
                update_cooldown(self.last_sent_at, event)
                return

            try:
                send_dingtalk_markdown(self.webhook_url, payload, self.timeout)
            except RuntimeError as e:
                self.get_logger().error(str(e))
                return

            update_cooldown(self.last_sent_at, event)
            self.get_logger().info(
                "sent dingtalk event: %s %s"
                % (
                    event.get("class_name") or "unknown",
                    format_score(event_score(event)),
                )
            )

    rclpy.init()
    node = DingTalkNotifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="DingTalk notifier for car_report events."
    )
    parser.add_argument(
        "--webhook-test",
        action="store_true",
        help="send a test markdown message to DingTalk",
    )
    parser.add_argument(
        "--webhook-url",
        default=DEFAULT_WEBHOOK_URL,
        help="DingTalk custom robot webhook URL",
    )
    parser.add_argument(
        "--keyword", default=DEFAULT_KEYWORD, help="DingTalk keyword security text"
    )
    parser.add_argument(
        "--timeout", type=float, default=DEFAULT_TIMEOUT, help="HTTP timeout seconds"
    )
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.webhook_test:
        return run_webhook_test(args.webhook_url, args.keyword, args.timeout)
    return run_ros_node()


if __name__ == "__main__":
    sys.exit(main())
