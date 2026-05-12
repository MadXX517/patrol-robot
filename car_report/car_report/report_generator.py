#!/usr/bin/env python3
# encoding: utf-8
import argparse
import base64
import json
import os
import sys
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path


DEFAULT_ENDPOINT = 'https://open.bigmodel.cn/api/paas/v4/chat/completions'
DEFAULT_MODEL = 'glm-4v-flash'
DEFAULT_MAX_IMAGE_BYTES = 5 * 1024 * 1024
BIGMODEL_API_KEY = '63023e7229c04850a90557fe96ede2fa.8YVCP4HcjEt5mmw4'
API_TEST_EVENT = {
    'time': '2026-05-11T21:30:12+08:00',
    'event_type': 'object_detected',
    'class_name': 'person',
    'score': 0.87,
    'bbox': [120, 80, 300, 420],
    'image_path': '',
}


def default_output_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory('car_report')) / 'data')
    except Exception:
        return str(Path(__file__).resolve().parents[1] / 'data')


def expand_path(path):
    return Path(os.path.expanduser(path)).resolve()


def find_latest_events_file(output_dir):
    events_dir = output_dir / 'events'
    candidates = sorted(events_dir.glob('events_*.jsonl'))
    if not candidates:
        raise FileNotFoundError(f'no events_*.jsonl found in {events_dir}')
    return candidates[-1]


def load_events(events_file):
    events = []
    with events_file.open('r', encoding='utf-8') as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError as e:
                raise ValueError(f'invalid json at {events_file}:{line_no}: {e}') from e
    return events


def build_prompt(events):
    event_lines = []
    for index, event in enumerate(events, start=1):
        event_lines.append(
            '%d. 时间: %s; 类型: %s; 目标: %s; 置信度: %.2f; 位置框: %s; 图片: %s'
            % (
                index,
                event.get('time', ''),
                event.get('event_type', ''),
                event.get('class_name', ''),
                float(event.get('score', 0.0)),
                event.get('bbox', []),
                event.get('image_path') or '无',
            )
        )

    meta = (
        '已知巡逻元信息：\n'
        f'- 事件数量: {len(events)}\n'
        '- 巡逻开始时间: 未记录\n'
        '- 巡逻结束时间: 未记录\n'
        '- 巡逻地点: 未记录\n'
        '- 任务范围: 未记录\n\n'
    )

    return (
        '你是军警巡逻辅助机器人的值班记录员。请根据下面的视觉识别事件生成一份简洁、'
        '可用于比赛演示的 Markdown 巡逻报告。\n\n'
        '报告必须包含：巡逻概况、事件列表、风险判断、处置建议、待人工复核项。\n'
        '只能基于给定事件写报告，未提供的信息写“未记录”，不要编造巡逻时长、结束时间、地点、任务范围或处置结果。\n'
        '目标识别事件不等于异常事件。除非事件类型或上下文明确表示异常，'
        '不要直接写“异常事件”“立即上报”等结论；应说明这是普通目标识别记录，需要人工复核。\n\n'
        + meta +
        '视觉事件如下：\n' + '\n'.join(event_lines)
    )


def image_to_base64(path, max_image_bytes):
    if os.path.getsize(path) <= max_image_bytes:
        with open(path, 'rb') as f:
            return base64.b64encode(f.read()).decode('ascii')

    import cv2

    image = cv2.imread(str(path))
    if image is None:
        raise ValueError(f'failed to read image: {path}')
    data = encode_jpeg_under_limit(image, max_image_bytes)
    return base64.b64encode(data).decode('ascii')


def encode_jpeg_under_limit(image, max_image_bytes):
    import cv2

    quality = 85
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
        if len(data) <= max_image_bytes:
            return data

        if quality > 45:
            quality -= 15
        else:
            height, width = resized.shape[:2]
            new_size = (max(1, int(width * 0.8)), max(1, int(height * 0.8)))
            resized = cv2.resize(resized, new_size)

    raise RuntimeError(
        f'image is still larger than {max_image_bytes} bytes after compression'
    )


def build_messages(events, mode, max_images, max_image_bytes):
    prompt = build_prompt(events)
    messages = [
        {
            'role': 'system',
            'content': '你擅长将机器人巡逻事件整理成简洁、客观、可执行的中文报告。',
        }
    ]

    if mode == 'vision':
        content = [{'type': 'text', 'text': prompt}]
        added = 0
        for event in events:
            image_path = event.get('image_path')
            if not image_path or added >= max_images:
                continue
            image_file = Path(image_path)
            if not image_file.exists():
                continue
            content.append({
                'type': 'image_url',
                'image_url': {'url': image_to_base64(image_file, max_image_bytes)},
            })
            added += 1
        messages.append({'role': 'user', 'content': content})
    else:
        messages.append({'role': 'user', 'content': prompt})

    return messages


def extract_user_text(messages):
    for message in messages:
        if message.get('role') != 'user':
            continue
        content = message.get('content')
        if isinstance(content, str):
            return content
        if isinstance(content, list):
            return '\n'.join(
                item.get('text', '')
                for item in content
                if isinstance(item, dict) and item.get('type') == 'text'
            )
    return ''


def call_bigmodel(api_key, endpoint, model, messages, temperature, timeout):
    payload = {
        'model': model,
        'messages': messages,
        'temperature': temperature,
    }
    data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
    request = urllib.request.Request(
        endpoint,
        data=data,
        headers={
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json',
        },
        method='POST',
    )

    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            raw = response.read().decode('utf-8')
    except urllib.error.HTTPError as e:
        error_body = e.read().decode('utf-8', errors='replace')
        raise RuntimeError(f'BigModel HTTP {e.code}: {error_body}') from e
    except urllib.error.URLError as e:
        raise RuntimeError(f'BigModel request failed: {e}') from e

    result = json.loads(raw)
    try:
        content = result['choices'][0]['message']['content']
    except (KeyError, IndexError, TypeError) as e:
        raise RuntimeError(f'unexpected BigModel response: {raw}') from e
    if isinstance(content, list):
        return ''.join(str(item.get('text', item)) for item in content)
    return str(content)


def write_report(output_dir, content, events_file, mode, model):
    reports_dir = output_dir / 'reports'
    reports_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    report_file = reports_dir / f'report_{timestamp}.md'
    header = (
        f'# 巡逻事件报告\n\n'
        f'- 生成时间: {datetime.now().astimezone().isoformat(timespec="seconds")}\n'
        f'- 事件文件: `{events_file}`\n'
        f'- 模型: `{model}`\n'
        f'- 模式: `{mode}`\n\n'
    )
    report_file.write_text(header + content.strip() + '\n', encoding='utf-8')
    return report_file


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Generate GLM patrol reports from car_report events.')
    parser.add_argument('--output-dir', default=default_output_dir(), help='event/report output directory')
    parser.add_argument('--events-file', default='', help='specific events_YYYYMMDD.jsonl file')
    parser.add_argument('--mode', choices=['text', 'vision'], default='text', help='text summary or vision review')
    parser.add_argument('--max-images', type=int, default=3, help='max event images in vision mode')
    parser.add_argument('--max-image-bytes', type=int, default=DEFAULT_MAX_IMAGE_BYTES, help='max image bytes sent to GLM in vision mode')
    parser.add_argument('--model', default=DEFAULT_MODEL, help='BigModel model id')
    parser.add_argument('--endpoint', default=DEFAULT_ENDPOINT, help='BigModel chat completions endpoint')
    parser.add_argument('--temperature', type=float, default=0.2, help='generation temperature')
    parser.add_argument('--timeout', type=float, default=60.0, help='HTTP timeout seconds')
    parser.add_argument('--api-test', action='store_true', help='call GLM with a built-in event and print test input plus answer')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.api_test:
        messages = build_messages([API_TEST_EVENT], 'text', 0, args.max_image_bytes)
        print('=== API test input ===')
        print(extract_user_text(messages))
        print('\n=== API test answer ===')
        answer = call_bigmodel(
            api_key=BIGMODEL_API_KEY,
            endpoint=args.endpoint,
            model=args.model,
            messages=messages,
            temperature=args.temperature,
            timeout=args.timeout,
        )
        print(answer)
        return 0

    output_dir = expand_path(args.output_dir)
    events_file = expand_path(args.events_file) if args.events_file else find_latest_events_file(output_dir)
    events = load_events(events_file)
    if not events:
        raise SystemExit(f'no events found in {events_file}')

    messages = build_messages(events, args.mode, args.max_images, args.max_image_bytes)
    api_key = BIGMODEL_API_KEY

    report = call_bigmodel(
        api_key=api_key,
        endpoint=args.endpoint,
        model=args.model,
        messages=messages,
        temperature=args.temperature,
        timeout=args.timeout,
    )
    report_file = write_report(output_dir, report, events_file, args.mode, args.model)
    print(str(report_file))
    return 0


if __name__ == '__main__':
    sys.exit(main())
