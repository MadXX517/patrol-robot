SECURITY_CLASS_GROUPS = {
    'person': {
        'event_type': 'intrusion_detected',
        'event_name': '核心禁区人员闯入',
        'priority': 'highest',
        'risk_level': 'high',
        'action': '停车、语音警告、截图留证、上报值班终端',
        'speech': '警告，您已进入军事通信设施警戒区域，请立即停止前进并配合检查。',
    },
    'backpack': {
        'event_type': 'unattended_object',
        'event_name': '设备保障区遗留物',
        'priority': 'high',
        'risk_level': 'medium',
        'action': '停车、截图留证、上报值班终端并请求人工复核',
        'speech': '检测到来源不明遗留物，请值班人员远程复核。',
    },
    'suitcase': {
        'event_type': 'unattended_object',
        'event_name': '设备保障区遗留物',
        'priority': 'high',
        'risk_level': 'medium',
        'action': '停车、截图留证、上报值班终端并请求人工复核',
        'speech': '检测到来源不明遗留物，请值班人员远程复核。',
    },
    'handbag': {
        'event_type': 'unattended_object',
        'event_name': '设备保障区遗留物',
        'priority': 'high',
        'risk_level': 'medium',
        'action': '停车、截图留证、上报值班终端并请求人工复核',
        'speech': '检测到来源不明遗留物，请值班人员远程复核。',
    },
}


def parse_target_classes(value):
    if value is None:
        return set()
    return {item.strip().lower() for item in str(value).split(',') if item.strip()}


def class_key(class_name):
    return str(class_name or '').strip().lower()


def security_event_metadata(class_name):
    return SECURITY_CLASS_GROUPS.get(class_key(class_name), {
        'event_type': 'object_detected',
        'event_name': '目标检测事件',
        'priority': 'normal',
        'risk_level': 'low',
        'action': '记录目标并等待人工复核',
        'speech': '',
    })


class ConfirmFrameTracker:
    def __init__(self, confirm_frames=1):
        self.confirm_frames = max(1, int(confirm_frames))
        self.counts = {}

    def update(self, present_class_keys):
        present_class_keys = set(present_class_keys)
        for key in list(self.counts):
            if key not in present_class_keys:
                self.counts.pop(key, None)

        confirmed = set()
        for key in present_class_keys:
            self.counts[key] = self.counts.get(key, 0) + 1
            if self.counts[key] >= self.confirm_frames:
                confirmed.add(key)
        return confirmed
