import json
import unittest


from car_voice.command_parser import (
    command_to_json,
    duration_for_move,
    parse_fixed_command,
    parse_command_text,
    validate_command,
)


class CommandParserTest(unittest.TestCase):
    def test_fixed_emergency_stop(self):
        self.assertEqual(
            parse_fixed_command('小甲小甲，急停'),
            {'intent': 'emergency_stop'},
        )

    def test_fixed_move_distance_is_limited(self):
        command = parse_fixed_command('向前进 2 米')
        self.assertEqual(command['intent'], 'move')
        self.assertEqual(command['direction'], 'forward')
        self.assertEqual(command['distance_m'], 0.6)

    def test_fixed_lidar_guard(self):
        self.assertEqual(
            parse_fixed_command('开始警卫'),
            {'intent': 'set_lidar_mode', 'mode': 3},
        )

    def test_fixed_report(self):
        self.assertEqual(
            parse_fixed_command('生成巡逻报告'),
            {'intent': 'generate_report'},
        )

    def test_llm_json_validation(self):
        raw = '{"intent":"move","direction":"turn_left","angle_deg":360}'
        self.assertEqual(
            parse_command_text(raw),
            {
                'intent': 'move',
                'direction': 'turn_left',
                'angle_deg': 180.0,
            },
        )

    def test_unknown_intent_rejected(self):
        with self.assertRaises(ValueError):
            validate_command({'intent': 'navigate', 'target': '门口'})

    def test_legacy_step_conversion(self):
        command = validate_command({
            'step': {
                'function': '小车前后平移运动',
                'parameters': {'car_move': [0.2, 0.0]},
            }
        })
        self.assertEqual(
            command,
            {'intent': 'move', 'direction': 'forward', 'distance_m': 0.2},
        )

    def test_command_to_json_roundtrip(self):
        encoded = command_to_json({'intent': 'set_color_follow', 'action': 'on'})
        self.assertEqual(
            json.loads(encoded),
            {'intent': 'set_color_follow', 'action': 'start'},
        )

    def test_duration_for_move(self):
        command = {'intent': 'move', 'direction': 'forward', 'distance_m': 0.3}
        self.assertAlmostEqual(duration_for_move(command, 0.1, 0.1, 0.5), 3.0)


if __name__ == '__main__':
    unittest.main()
