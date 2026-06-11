#!/usr/bin/env python3
# encoding: utf-8
import traceback

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from . import report_generator


class ReportService(Node):
    def __init__(self):
        super().__init__('report_service')

        self.declare_parameter('service_name', '/car_report/generate_report')
        self.declare_parameter('output_dir', report_generator.default_output_dir())
        self.declare_parameter('events_file', '')
        self.declare_parameter('mode', 'text')
        self.declare_parameter('max_images', 3)
        self.declare_parameter('max_image_bytes', report_generator.DEFAULT_MAX_IMAGE_BYTES)
        self.declare_parameter('model', report_generator.DEFAULT_MODEL)
        self.declare_parameter('endpoint', report_generator.DEFAULT_ENDPOINT)
        self.declare_parameter('api_key', '')
        self.declare_parameter('temperature', 0.2)
        self.declare_parameter('timeout', 60.0)

        self.service_name = self.get_parameter('service_name').value
        self.create_service(Trigger, self.service_name, self.handle_generate_report)
        self.get_logger().info(f'report service started: {self.service_name}')

    def handle_generate_report(self, request, response):
        del request
        try:
            report_path = self.generate_report()
        except Exception as exc:
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = str(exc)
            return response

        response.success = True
        response.message = str(report_path)
        return response

    def generate_report(self):
        output_param = str(self.get_parameter('output_dir').value or '')
        output_dir = report_generator.expand_path(
            output_param or report_generator.default_output_dir()
        )
        events_param = str(self.get_parameter('events_file').value or '')
        events_file = (
            report_generator.expand_path(events_param)
            if events_param
            else report_generator.find_latest_events_file(output_dir)
        )
        events = report_generator.load_events(events_file)
        if not events:
            raise RuntimeError(f'no events found in {events_file}')

        mode = self.get_parameter('mode').value
        if mode not in ('text', 'vision'):
            raise RuntimeError('mode must be text or vision')

        api_key = report_generator.get_api_key(
            self.get_parameter('api_key').value
        )
        if not api_key:
            raise RuntimeError(
                'BigModel API key is not configured; set BIGMODEL_API_KEY or ZHIPUAI_API_KEY'
            )

        messages = report_generator.build_messages(
            events=events,
            mode=mode,
            max_images=int(self.get_parameter('max_images').value),
            max_image_bytes=int(self.get_parameter('max_image_bytes').value),
        )
        model = self.get_parameter('model').value
        report = report_generator.call_bigmodel(
            api_key=api_key,
            endpoint=self.get_parameter('endpoint').value,
            model=model,
            messages=messages,
            temperature=float(self.get_parameter('temperature').value),
            timeout=float(self.get_parameter('timeout').value),
        )
        report_path = report_generator.write_report(
            output_dir=output_dir,
            content=report,
            events_file=events_file,
            mode=mode,
            model=model,
        )
        self.get_logger().info(f'generated report: {report_path}')
        return report_path


def main(args=None):
    rclpy.init(args=args)
    node = ReportService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
