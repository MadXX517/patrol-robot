def parse_target_classes(value):
    if value is None:
        return set()
    return {item.strip().lower() for item in str(value).split(',') if item.strip()}


def class_key(class_name):
    return str(class_name or '').strip().lower()


def should_keep_class(class_name, target_classes):
    if not target_classes:
        return True
    return class_key(class_name) in target_classes
