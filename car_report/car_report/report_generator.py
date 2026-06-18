#!/usr/bin/env python3
# encoding: utf-8
import argparse
import ast
import base64
import json
import os
import sys
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path


DEFAULT_MAX_IMAGE_BYTES = 5 * 1024 * 1024
API_TEST_EVENT = {
    'time': '2026-05-11T21:30:12+08:00',
    'event_type': 'intrusion_detected',
    'event_name': '核心禁区人员闯入',
    'priority': 'highest',
    'risk_level': 'high',
    'area': '通信节点外围警戒线',
    'class_name': 'person',
    'score': 0.87,
    'bbox': [120, 80, 300, 420],
    'action': '停车、语音警告、截图留证、上报值班终端',
    'speech': '警告，您已进入军事通信设施警戒区域，请立即停止前进并配合检查。',
    'image_path': '',
}


def default_output_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory('car_report')) / 'data')
    except Exception:
        return str(Path(__file__).resolve().parents[1] / 'data')


def _call_name(node):
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return ''


def _string_literal(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _car_llm_launch_candidates():
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(
            Path(get_package_share_directory('car_llm')) / 'launch' / 'car_llm.launch.py'
        )
    except Exception:
        pass

    candidates.append(
        Path(__file__).resolve().parents[2] / 'car_llm' / 'launch' / 'car_llm.launch.py'
    )
    return candidates


def _parse_car_llm_launch_defaults(launch_file):
    tree = ast.parse(launch_file.read_text(encoding='utf-8'), filename=str(launch_file))
    defaults = {}
    wanted = {'api_key', 'base_url', 'llm_model', 'vision_model'}

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or _call_name(node.func) != 'DeclareLaunchArgument':
            continue
        if not node.args:
            continue
        name = _string_literal(node.args[0])
        if name not in wanted:
            continue
        for keyword in node.keywords:
            if keyword.arg == 'default_value':
                value = _string_literal(keyword.value)
                if value is not None:
                    defaults[name] = value

    missing = sorted(wanted - set(defaults))
    if missing:
        raise ValueError(f'missing launch defaults: {", ".join(missing)}')
    defaults['source'] = str(launch_file)
    return defaults


def load_car_llm_defaults():
    errors = []
    for launch_file in _car_llm_launch_candidates():
        if not launch_file.exists():
            errors.append(f'{launch_file}: not found')
            continue
        try:
            return _parse_car_llm_launch_defaults(launch_file)
        except Exception as exc:
            errors.append(f'{launch_file}: {exc}')
    raise RuntimeError('failed to load car_llm launch defaults; ' + '; '.join(errors))


def chat_endpoint_from_base_url(base_url):
    base = str(base_url or '').strip().rstrip('/')
    if not base:
        return ''
    if base.endswith('/chat/completions'):
        return base
    return base + '/chat/completions'


def resolve_chat_endpoint(base_url, endpoint):
    endpoint = str(endpoint or '').strip()
    if endpoint:
        return endpoint
    return chat_endpoint_from_base_url(base_url)


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


def format_event_score(event):
    raw_score = event.get('score')
    if raw_score in (None, ''):
        return '未记录'
    try:
        return f'{float(raw_score):.2f}'
    except (TypeError, ValueError):
        return '未记录'


def build_prompt(events):
    event_lines = []
    for index, event in enumerate(events, start=1):
        event_lines.append(
            '%d. 时间: %s; 警情: %s; 类型: %s; 风险: %s; 优先级: %s; 区域: %s; '
            '目标: %s; 置信度: %s; 位置框: %s; 建议动作: %s; 语音提示: %s; 图片: %s'
            % (
                index,
                event.get('time', ''),
                event.get('event_name') or '未记录',
                event.get('event_type', ''),
                event.get('risk_level') or '未记录',
                event.get('priority') or '未记录',
                event.get('area') or '未记录',
                event.get('class_name', ''),
                format_event_score(event),
                event.get('bbox', []),
                event.get('action') or '未记录',
                event.get('speech') or '未记录',
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
        '如果事件包含“人员闯入”或“遗留物”等警情语义，可以按警戒事件分析；'
        '如果只是普通目标识别记录，不要夸大风险，应说明需要人工复核。\n\n'
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
            image_base64 = image_to_base64(image_file, max_image_bytes)
            content.append({
                'type': 'image_url',
                'image_url': {'url': f'data:image/jpeg;base64,{image_base64}'},
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


def call_llm_chat(api_key, endpoint, model, messages, temperature, timeout):
    if not api_key:
        raise RuntimeError('LLM api_key is empty; check car_llm/launch/car_llm.launch.py')
    if not endpoint:
        raise RuntimeError('LLM endpoint is empty; check car_llm base_url or --endpoint')
    if not model:
        raise RuntimeError('LLM model is empty; check car_llm llm_model or --model')

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
        raise RuntimeError(f'LLM HTTP {e.code}: {error_body}') from e
    except urllib.error.URLError as e:
        raise RuntimeError(f'LLM request failed: {e}') from e

    result = json.loads(raw)
    try:
        content = result['choices'][0]['message']['content']
    except (KeyError, IndexError, TypeError) as e:
        raise RuntimeError(f'unexpected LLM response: {raw}') from e
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
    llm_defaults = load_car_llm_defaults()
    parser = argparse.ArgumentParser(description='Generate LLM patrol reports from car_report events.')
    parser.add_argument('--output-dir', default=default_output_dir(), help='event/report output directory')
    parser.add_argument('--events-file', default='', help='specific events_YYYYMMDD.jsonl file')
    parser.add_argument('--mode', choices=['text', 'vision'], default='text', help='text summary or vision review')
    parser.add_argument('--max-images', type=int, default=3, help='max event images in vision mode')
    parser.add_argument('--max-image-bytes', type=int, default=DEFAULT_MAX_IMAGE_BYTES, help='max image bytes sent to LLM in vision mode')
    parser.add_argument('--model', default='', help='LLM model id; default is llm_model for text mode and vision_model for vision mode')
    parser.add_argument('--base-url', default=llm_defaults['base_url'], help='OpenAI-compatible base URL; default is read from car_llm.launch.py')
    parser.add_argument('--endpoint', default='', help='full chat completions endpoint; overrides --base-url when set')
    parser.add_argument('--temperature', type=float, default=0.2, help='generation temperature')
    parser.add_argument('--timeout', type=float, default=60.0, help='HTTP timeout seconds')
    parser.add_argument('--api-test', action='store_true', help='call LLM with a built-in event and print test input plus answer')
    args = parser.parse_args(argv)
    args.llm_api_key = llm_defaults['api_key']
    args.llm_config_source = llm_defaults['source']
    if not args.model:
        args.model = (
            llm_defaults['vision_model']
            if args.mode == 'vision'
            else llm_defaults['llm_model']
        )
    args.resolved_endpoint = resolve_chat_endpoint(args.base_url, args.endpoint)
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.api_test:
        messages = build_messages([API_TEST_EVENT], 'text', 0, args.max_image_bytes)
        print('=== LLM config ===')
        print(f'model: {args.model}')
        print(f'endpoint: {args.resolved_endpoint}')
        print(f'config_source: {args.llm_config_source}')
        print('=== API test input ===')
        print(extract_user_text(messages))
        print('\n=== API test answer ===')
        answer = call_llm_chat(
            api_key=args.llm_api_key,
            endpoint=args.resolved_endpoint,
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

    report = call_llm_chat(
        api_key=args.llm_api_key,
        endpoint=args.resolved_endpoint,
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
