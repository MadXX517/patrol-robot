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
DEFAULT_LLM_CONFIG = {
    'api_key': '',
    'base_url': '',
    'llm_model': '',
    'vision_model': '',
    'config_source': '',
}
# 钉钉自定义机器人(与 car_notify/dingtalk_notifier 同一个)。报告生成后可选发送。
DINGTALK_WEBHOOK_URL = (
    'https://oapi.dingtalk.com/robot/send?'
    'access_token=a6d04355c1119b47c1db727a3e3b54e58eb8dd36456313569b3db5f78c2e3c93'
)
DINGTALK_KEYWORD = '巡逻告警'
DINGTALK_MAX_TEXT = 18000  # 钉钉单条 markdown 上限约 20000 字节,留余量并截断
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


def find_car_llm_launch_file():
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory('car_llm')) / 'launch' / 'car_llm.launch.py')
    except Exception:
        pass

    candidates.append(Path(__file__).resolve().parents[2] / 'car_llm' / 'launch' / 'car_llm.launch.py')
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def parse_launch_argument_defaults(launch_file):
    defaults = {}
    tree = ast.parse(launch_file.read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func_name = getattr(node.func, 'id', '')
        if isinstance(node.func, ast.Attribute):
            func_name = node.func.attr
        if func_name != 'DeclareLaunchArgument' or not node.args:
            continue
        name_node = node.args[0]
        if not isinstance(name_node, ast.Constant) or not isinstance(name_node.value, str):
            continue
        default_value = None
        for keyword in node.keywords:
            if keyword.arg == 'default_value':
                default_value = keyword.value
                break
        if isinstance(default_value, ast.Constant) and isinstance(default_value.value, str):
            defaults[name_node.value] = default_value.value
    return defaults


def load_llm_defaults():
    defaults = dict(DEFAULT_LLM_CONFIG)
    launch_file = find_car_llm_launch_file()
    if not launch_file:
        return defaults
    parsed = parse_launch_argument_defaults(launch_file)
    for key in ('api_key', 'base_url', 'llm_model', 'vision_model'):
        if parsed.get(key):
            defaults[key] = parsed[key]
    defaults['config_source'] = str(launch_file)
    return defaults


def build_chat_endpoint(base_url):
    base = str(base_url or '').rstrip('/')
    if not base:
        return ''
    return base + '/chat/completions'


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


def image_mime_type(path):
    suffix = Path(path).suffix.lower()
    if suffix in ('.jpg', '.jpeg'):
        return 'image/jpeg'
    if suffix == '.png':
        return 'image/png'
    if suffix == '.webp':
        return 'image/webp'
    return 'image/jpeg'


def image_to_data_url(path, max_image_bytes):
    encoded = image_to_base64(path, max_image_bytes)
    mime_type = 'image/jpeg' if os.path.getsize(path) > max_image_bytes else image_mime_type(path)
    return f'data:{mime_type};base64,{encoded}'


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
                'image_url': {'url': image_to_data_url(image_file, max_image_bytes)},
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
        raise RuntimeError('LLM api_key is empty; please update car_llm/launch/car_llm.launch.py')
    if not endpoint:
        raise RuntimeError('LLM endpoint is empty; please update car_llm base_url or pass --endpoint')
    if not model:
        raise RuntimeError('LLM model is empty; please update car_llm llm_model/vision_model or pass --model')
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


def send_report_to_dingtalk(webhook_url, keyword, content, timeout):
    """把生成好的 markdown 报告作为钉钉 markdown 消息发送。
    钉钉自定义机器人有关键词安全校验,故标题与正文都带上 keyword。超长截断。"""
    if not webhook_url:
        raise RuntimeError('DingTalk webhook_url is empty')

    body = content.strip()
    if len(body) > DINGTALK_MAX_TEXT:
        body = body[:DINGTALK_MAX_TEXT] + '\n\n> (报告过长已截断)'
    title = f'{keyword} - 巡逻报告'
    text = f'### {keyword} - 巡逻报告\n\n{body}'
    payload = {'msgtype': 'markdown', 'markdown': {'title': title, 'text': text}}

    data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
    request = urllib.request.Request(
        webhook_url, data=data,
        headers={'Content-Type': 'application/json;charset=utf-8'}, method='POST',
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            raw = response.read().decode('utf-8', errors='replace')
    except urllib.error.HTTPError as e:
        raise RuntimeError(f'DingTalk HTTP {e.code}: {e.read().decode("utf-8", "replace")}') from e
    except urllib.error.URLError as e:
        raise RuntimeError(f'DingTalk request failed: {e}') from e
    result = json.loads(raw)
    if result.get('errcode') != 0:
        raise RuntimeError(f'DingTalk error response: {raw}')
    return result


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
    parser = argparse.ArgumentParser(description='Generate LLM patrol reports from car_report events.')
    parser.add_argument('--output-dir', default=default_output_dir(), help='event/report output directory')
    parser.add_argument('--events-file', default='', help='specific events_YYYYMMDD.jsonl file')
    parser.add_argument('--mode', choices=['text', 'vision'], default='text', help='text summary or vision review')
    parser.add_argument('--max-images', type=int, default=3, help='max event images in vision mode')
    parser.add_argument('--max-image-bytes', type=int, default=DEFAULT_MAX_IMAGE_BYTES, help='max image bytes sent to LLM in vision mode')
    parser.add_argument('--api-key', default='', help='LLM API key override; default reads car_llm launch config')
    parser.add_argument('--base-url', default='', help='LLM OpenAI-compatible base URL override; default reads car_llm launch config')
    parser.add_argument('--model', default='', help='LLM model override; default uses llm_model or vision_model from car_llm')
    parser.add_argument('--endpoint', default='', help='full LLM chat completions endpoint override')
    parser.add_argument('--temperature', type=float, default=0.2, help='generation temperature')
    parser.add_argument('--timeout', type=float, default=60.0, help='HTTP timeout seconds')
    parser.add_argument('--send-dingtalk', action='store_true', help='send the generated report to DingTalk')
    parser.add_argument('--webhook-url', default=DINGTALK_WEBHOOK_URL, help='DingTalk custom robot webhook URL')
    parser.add_argument('--keyword', default=DINGTALK_KEYWORD, help='DingTalk security keyword')
    parser.add_argument('--api-test', action='store_true', help='call LLM with a built-in event and print config, test input and answer')
    return parser.parse_args(argv)


def resolve_llm_settings(args):
    defaults = load_llm_defaults()
    api_key = args.api_key or defaults.get('api_key', '')
    base_url = args.base_url or defaults.get('base_url', '')
    endpoint = args.endpoint or build_chat_endpoint(base_url)
    model = args.model or (
        defaults.get('vision_model') if args.mode == 'vision' else defaults.get('llm_model')
    )
    return {
        'api_key': api_key,
        'base_url': base_url,
        'endpoint': endpoint,
        'model': model,
        'config_source': defaults.get('config_source', ''),
    }


def redact_secret(value):
    if not value:
        return ''
    if len(value) <= 10:
        return value[:2] + '***'
    return value[:6] + '***' + value[-4:]


def main(argv=None):
    args = parse_args(argv)
    llm_settings = resolve_llm_settings(args)
    if args.api_test:
        messages = build_messages([API_TEST_EVENT], args.mode, 0, args.max_image_bytes)
        print('=== LLM config ===')
        print(f"config_source: {llm_settings['config_source'] or 'not found'}")
        print(f"model: {llm_settings['model']}")
        print(f"base_url: {llm_settings['base_url']}")
        print(f"endpoint: {llm_settings['endpoint']}")
        print(f"api_key: {redact_secret(llm_settings['api_key'])}")
        print('\n=== API test input ===')
        print(extract_user_text(messages))
        print('\n=== API test answer ===')
        answer = call_llm_chat(
            api_key=llm_settings['api_key'],
            endpoint=llm_settings['endpoint'],
            model=llm_settings['model'],
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
        api_key=llm_settings['api_key'],
        endpoint=llm_settings['endpoint'],
        model=llm_settings['model'],
        messages=messages,
        temperature=args.temperature,
        timeout=args.timeout,
    )
    report_file = write_report(output_dir, report, events_file, args.mode, llm_settings['model'])
    print(str(report_file))

    if args.send_dingtalk:
        try:
            send_report_to_dingtalk(args.webhook_url, args.keyword, report, args.timeout)
            print('dingtalk: sent')
        except RuntimeError as e:
            print(f'dingtalk: failed: {e}', file=sys.stderr)
            return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
