#!/usr/bin/env python3
# encoding: utf-8
import json
import math
import re


DEFAULT_MOVE_DISTANCE_M = 0.30
MAX_MOVE_DISTANCE_M = 0.60
DEFAULT_TURN_DEG = 45.0
MAX_TURN_DEG = 180.0

ALLOWED_INTENTS = {
    'emergency_stop',
    'move',
    'set_lidar_mode',
    'set_color_follow',
    'generate_report',
    'status',
}

MOVE_DIRECTIONS = {
    'forward',
    'backward',
    'left',
    'right',
    'turn_left',
    'turn_right',
}

DIRECTION_ALIASES = {
    'front': 'forward',
    'ahead': 'forward',
    'back': 'backward',
    'left_shift': 'left',
    'right_shift': 'right',
    'left_turn': 'turn_left',
    'right_turn': 'turn_right',
    '左转': 'turn_left',
    '右转': 'turn_right',
    '前进': 'forward',
    '后退': 'backward',
    '左移': 'left',
    '右移': 'right',
}

VOICE_COMMAND_PROMPT = """
你是巡逻机器人的语音指令解析器。只输出一个 JSON 对象，不要输出解释。
可用 intent 只有：
1. {"intent":"emergency_stop"}
2. {"intent":"move","direction":"forward|backward|left|right|turn_left|turn_right","distance_m":0.3,"angle_deg":45}
3. {"intent":"set_lidar_mode","mode":0|1|2|3}
4. {"intent":"set_color_follow","action":"start|stop"}
5. {"intent":"generate_report"}
6. {"intent":"status"}
说明：雷达 mode 0=停止，1=避障，2=跟随，3=警卫。不要生成导航点位任务。
"""


def normalize_text(text):
    return re.sub(r'\s+', '', str(text or '')).lower()


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def extract_json_objects(text):
    objects = []
    start = None
    depth = 0
    in_string = False
    escaped = False

    for index, char in enumerate(str(text or '')):
        if in_string:
            if escaped:
                escaped = False
            elif char == '\\':
                escaped = True
            elif char == '"':
                in_string = False
            continue

        if char == '"':
            in_string = True
        elif char == '{':
            if depth == 0:
                start = index
            depth += 1
        elif char == '}':
            if depth:
                depth -= 1
                if depth == 0 and start is not None:
                    raw = text[start:index + 1]
                    try:
                        objects.append(json.loads(raw))
                    except json.JSONDecodeError:
                        pass
                    start = None
    return objects


def extract_distance_m(text, default=DEFAULT_MOVE_DISTANCE_M):
    match = re.search(r'(\d+(?:\.\d+)?)\s*(厘米|公分|cm|米|m)', str(text), re.I)
    if not match:
        return default
    value = float(match.group(1))
    unit = match.group(2).lower()
    if unit in ('厘米', '公分', 'cm'):
        value = value / 100.0
    return value


def extract_angle_deg(text, default=DEFAULT_TURN_DEG):
    match = re.search(r'(\d+(?:\.\d+)?)\s*(度|°|deg)', str(text), re.I)
    if not match:
        return default
    return float(match.group(1))


def parse_fixed_command(text):
    normalized = normalize_text(text)
    if not normalized:
        return None

    if any(word in normalized for word in ('生成巡逻报告', '生成报告', '巡逻报告', '总结报告')):
        return {'intent': 'generate_report'}

    if any(word in normalized for word in ('状态', '现在情况', '当前情况')):
        return {'intent': 'status'}

    if any(word in normalized for word in ('停止警卫', '关闭警卫', '停止雷达', '关闭雷达', '停止避障')):
        return {'intent': 'set_lidar_mode', 'mode': 0}
    if any(word in normalized for word in ('开始避障', '开启避障', '避障模式')):
        return {'intent': 'set_lidar_mode', 'mode': 1}
    if any(word in normalized for word in ('雷达跟随', '开始跟随', '开启跟随', '跟随我')):
        return {'intent': 'set_lidar_mode', 'mode': 2}
    if any(word in normalized for word in ('开始警卫', '开启警卫', '警卫模式', '进入警卫')):
        return {'intent': 'set_lidar_mode', 'mode': 3}

    if any(word in normalized for word in ('停止视觉跟踪', '停止目标跟踪', '停止颜色跟随', '退出跟踪')):
        return {'intent': 'set_color_follow', 'action': 'stop'}
    if any(word in normalized for word in ('开始视觉跟踪', '开始目标跟踪', '开始颜色跟随', '跟踪目标')):
        return {'intent': 'set_color_follow', 'action': 'start'}

    if any(word in normalized for word in ('急停', '紧急停止', '停止运动', '停车', '停下', '别动')):
        return {'intent': 'emergency_stop'}

    movement_rules = [
        (('向前进', '往前走', '前进'), 'forward'),
        (('向后退', '往后退', '后退'), 'backward'),
        (('向左移', '左平移', '左移'), 'left'),
        (('向右移', '右平移', '右移'), 'right'),
        (('向左转', '左转'), 'turn_left'),
        (('向右转', '右转'), 'turn_right'),
    ]
    for words, direction in movement_rules:
        if any(word in normalized for word in words):
            command = {'intent': 'move', 'direction': direction}
            if direction.startswith('turn_'):
                command['angle_deg'] = extract_angle_deg(text)
            else:
                command['distance_m'] = extract_distance_m(text)
            return validate_command(command)

    return None


def parse_command_text(text):
    fixed = parse_fixed_command(text)
    if fixed is not None:
        return fixed

    for item in extract_json_objects(str(text or '')):
        try:
            return validate_command(item)
        except ValueError:
            continue
    return None


def validate_command(raw):
    if isinstance(raw, str):
        raw = json.loads(raw)
    if not isinstance(raw, dict):
        raise ValueError('command must be a JSON object')

    if 'step' in raw:
        raw = convert_legacy_step(raw)

    intent = str(raw.get('intent') or '').strip()
    if intent not in ALLOWED_INTENTS:
        raise ValueError(f'unsupported intent: {intent}')

    if intent == 'emergency_stop':
        return {'intent': 'emergency_stop'}
    if intent == 'generate_report':
        return {'intent': 'generate_report'}
    if intent == 'status':
        return {'intent': 'status'}
    if intent == 'set_lidar_mode':
        mode = int(raw.get('mode'))
        if mode not in (0, 1, 2, 3):
            raise ValueError('lidar mode must be 0, 1, 2, or 3')
        return {'intent': 'set_lidar_mode', 'mode': mode}
    if intent == 'set_color_follow':
        action = str(raw.get('action') or '').strip().lower()
        if action in ('on', 'true', 'run', 'running', 'start'):
            action = 'start'
        elif action in ('off', 'false', 'stop', 'exit'):
            action = 'stop'
        if action not in ('start', 'stop'):
            raise ValueError('color follow action must be start or stop')
        return {'intent': 'set_color_follow', 'action': action}
    if intent == 'move':
        return normalize_move(raw)

    raise ValueError(f'unsupported intent: {intent}')


def normalize_move(raw):
    direction = str(raw.get('direction') or '').strip()
    direction = DIRECTION_ALIASES.get(direction, direction)
    if direction not in MOVE_DIRECTIONS:
        raise ValueError(f'unsupported move direction: {direction}')

    command = {'intent': 'move', 'direction': direction}
    if direction.startswith('turn_'):
        angle = float(raw.get('angle_deg', DEFAULT_TURN_DEG))
        command['angle_deg'] = clamp(abs(angle), 1.0, MAX_TURN_DEG)
    else:
        distance = float(raw.get('distance_m', DEFAULT_MOVE_DISTANCE_M))
        command['distance_m'] = clamp(abs(distance), 0.01, MAX_MOVE_DISTANCE_M)
    return command


def convert_legacy_step(raw):
    step = raw.get('step') or {}
    function = str(step.get('function') or '')
    params = step.get('parameters') or {}

    if function == '小车前后平移运动' and 'car_move' in params:
        dx, dy = params.get('car_move', [0.0, 0.0])[:2]
        dx = float(dx)
        dy = float(dy)
        if abs(dx) >= abs(dy):
            return {
                'intent': 'move',
                'direction': 'forward' if dx >= 0 else 'backward',
                'distance_m': abs(dx),
            }
        return {
            'intent': 'move',
            'direction': 'left' if dy >= 0 else 'right',
            'distance_m': abs(dy),
        }

    if function == '小车旋转运动' and 'car_turn' in params:
        angle = float(params.get('car_turn', [DEFAULT_TURN_DEG])[0])
        return {
            'intent': 'move',
            'direction': 'turn_left' if angle >= 0 else 'turn_right',
            'angle_deg': abs(angle),
        }

    raise ValueError(f'unsupported legacy step: {function}')


def command_to_json(command):
    return json.dumps(validate_command(command), ensure_ascii=False)


def duration_for_move(command, linear_speed, lateral_speed, angular_speed):
    command = validate_command(command)
    if command['intent'] != 'move':
        return 0.0
    direction = command['direction']
    if direction in ('forward', 'backward'):
        return command['distance_m'] / max(0.01, abs(linear_speed))
    if direction in ('left', 'right'):
        return command['distance_m'] / max(0.01, abs(lateral_speed))
    return math.radians(command['angle_deg']) / max(0.01, abs(angular_speed))
