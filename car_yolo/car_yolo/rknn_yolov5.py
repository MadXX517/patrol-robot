import os
from dataclasses import dataclass

import cv2
import numpy as np


COCO_NAMES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
    'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
    'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
    'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
    'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
    'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
    'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
    'scissors', 'teddy bear', 'hair drier', 'toothbrush',
]


DEFAULT_ANCHORS = np.array(
    [
        [[10, 13], [16, 30], [33, 23]],
        [[30, 61], [62, 45], [59, 119]],
        [[116, 90], [156, 198], [373, 326]],
    ],
    dtype=np.float32,
)


@dataclass
class YoloDetectionResult:
    pred: list
    names: list


class RKNNYoloV5:
    def __init__(
        self,
        model_path,
        class_names=None,
        img_size=640,
        conf_thres=0.25,
        iou_thres=0.45,
        npu_core='auto',
    ):
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                'RKNN model not found: %s. Put the converted .rknn model in car_yolo/config first.'
                % model_path
            )

        try:
            from rknnlite.api import RKNNLite
        except ImportError as exc:
            raise RuntimeError(
                'rknnlite.api is not installed. Install rknn-toolkit-lite2 on the RK board.'
            ) from exc

        self.model_path = model_path
        self.img_size = int(img_size)
        self.conf_thres = float(conf_thres)
        self.iou_thres = float(iou_thres)
        self.names = list(class_names or [])
        self.rknn = RKNNLite()
        self.RKNNLite = RKNNLite

        ret = self.rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError('load_rknn failed with code %s: %s' % (ret, model_path))

        core_mask = self._get_core_mask(npu_core)
        if core_mask is None:
            ret = self.rknn.init_runtime()
        else:
            ret = self.rknn.init_runtime(core_mask=core_mask)
        if ret != 0:
            raise RuntimeError('init_runtime failed with code %s: %s' % (ret, model_path))

    def predict(self, image):
        input_image, ratio, pad = self._letterbox(image, self.img_size)
        input_image = np.expand_dims(input_image, axis=0).astype(np.uint8)
        outputs = self.rknn.inference(inputs=[input_image])
        boxes, scores, classes = self._postprocess(outputs, image.shape[:2], ratio, pad)

        if len(boxes) == 0:
            predictions = np.empty((0, 6), dtype=np.float32)
        else:
            predictions = np.concatenate(
                [
                    boxes.astype(np.float32),
                    scores.reshape(-1, 1).astype(np.float32),
                    classes.reshape(-1, 1).astype(np.float32),
                ],
                axis=1,
            )
        return YoloDetectionResult(pred=[predictions], names=self._names_for(classes))

    def release(self):
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None

    def _get_core_mask(self, npu_core):
        value = str(npu_core or 'auto').lower()
        if value in ('', 'auto'):
            return None
        attr = {
            '0': 'NPU_CORE_0',
            '1': 'NPU_CORE_1',
            '2': 'NPU_CORE_2',
            '0_1': 'NPU_CORE_0_1',
            '0_1_2': 'NPU_CORE_0_1_2',
        }.get(value, value.upper())
        return getattr(self.RKNNLite, attr, None)

    def _postprocess(self, outputs, original_shape, ratio, pad):
        if outputs is None:
            return np.empty((0, 4)), np.empty((0,)), np.empty((0,), dtype=np.int32)

        decoded = []
        if len(outputs) == 1:
            decoded.append(self._decode_flat_output(outputs[0]))
        else:
            for index, output in enumerate(outputs[:3]):
                decoded.append(self._decode_grid_output(output, index))

        decoded = [item for item in decoded if item is not None and len(item) > 0]
        if not decoded:
            return np.empty((0, 4)), np.empty((0,)), np.empty((0,), dtype=np.int32)

        detections = np.concatenate(decoded, axis=0)
        boxes = detections[:, :4]
        scores = detections[:, 4]
        classes = detections[:, 5].astype(np.int32)

        keep = self._nms(boxes, scores, classes)
        boxes = boxes[keep]
        scores = scores[keep]
        classes = classes[keep]

        boxes[:, [0, 2]] -= pad[0]
        boxes[:, [1, 3]] -= pad[1]
        boxes /= ratio
        self._clip_boxes(boxes, original_shape)
        return boxes, scores, classes

    def _decode_flat_output(self, output):
        data = np.squeeze(np.asarray(output))
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.ndim != 2 or data.shape[-1] < 6:
            return np.empty((0, 6), dtype=np.float32)

        if data.shape[-1] == 6:
            boxes = data[:, :4].astype(np.float32)
            scores = data[:, 4].astype(np.float32)
            classes = data[:, 5].astype(np.int32)
            boxes = self._maybe_xywh_to_xyxy(boxes)
            boxes = self._scale_normalized_boxes(boxes)
            mask = scores >= self.conf_thres
            return np.column_stack((boxes[mask], scores[mask], classes[mask]))

        boxes = self._xywh_to_xyxy(data[:, :4].astype(np.float32))
        boxes = self._scale_normalized_boxes(boxes)
        objectness = self._sigmoid_if_needed(data[:, 4])
        class_scores = self._sigmoid_if_needed(data[:, 5:])
        classes = np.argmax(class_scores, axis=1).astype(np.int32)
        scores = objectness * class_scores[np.arange(class_scores.shape[0]), classes]
        mask = scores >= self.conf_thres
        return np.column_stack((boxes[mask], scores[mask], classes[mask]))

    def _decode_grid_output(self, output, output_index):
        data = np.asarray(output)
        pred = self._normalize_grid_output(data)
        if pred is None:
            return np.empty((0, 6), dtype=np.float32)

        num_classes = pred.shape[-1] - 5
        if num_classes <= 0:
            return np.empty((0, 6), dtype=np.float32)

        grid_h, grid_w = pred.shape[1], pred.shape[2]
        stride = self.img_size / float(grid_w)
        anchors = DEFAULT_ANCHORS[min(output_index, len(DEFAULT_ANCHORS) - 1)]

        pred = pred.astype(np.float32)
        xy = (self._sigmoid(pred[..., 0:2]) * 2.0 - 0.5)
        wh = (self._sigmoid(pred[..., 2:4]) * 2.0) ** 2
        obj = self._sigmoid(pred[..., 4:5])
        cls = self._sigmoid(pred[..., 5:])

        grid_x, grid_y = np.meshgrid(np.arange(grid_w), np.arange(grid_h))
        grid = np.stack((grid_x, grid_y), axis=-1).astype(np.float32)
        xy = (xy + grid[None, :, :, :]) * stride
        wh = wh * anchors[:, None, None, :]
        boxes = self._xywh_to_xyxy(np.concatenate((xy, wh), axis=-1).reshape(-1, 4))

        scores_all = (obj * cls).reshape(-1, num_classes)
        classes = np.argmax(scores_all, axis=1).astype(np.int32)
        scores = scores_all[np.arange(scores_all.shape[0]), classes]
        mask = scores >= self.conf_thres
        return np.column_stack((boxes[mask], scores[mask], classes[mask]))

    def _normalize_grid_output(self, data):
        data = np.squeeze(data)
        if data.ndim == 4:
            if data.shape[0] == 3:
                return data
            if data.shape[-2] == 3:
                return np.transpose(data, (2, 0, 1, 3))
            if data.shape[1] == 3:
                return np.transpose(data, (1, 2, 3, 0))
            return None

        if data.ndim != 3:
            return None

        anchors = 3
        if data.shape[0] % anchors == 0 and data.shape[0] > data.shape[-1]:
            channels, grid_h, grid_w = data.shape
            return data.reshape(anchors, channels // anchors, grid_h, grid_w).transpose(0, 2, 3, 1)
        if data.shape[-1] % anchors == 0:
            grid_h, grid_w, channels = data.shape
            return data.reshape(grid_h, grid_w, anchors, channels // anchors).transpose(2, 0, 1, 3)
        return None

    def _letterbox(self, image, img_size):
        h, w = image.shape[:2]
        ratio = min(img_size / h, img_size / w)
        new_w, new_h = int(round(w * ratio)), int(round(h * ratio))
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((img_size, img_size, 3), 114, dtype=np.uint8)
        pad_w = (img_size - new_w) // 2
        pad_h = (img_size - new_h) // 2
        canvas[pad_h:pad_h + new_h, pad_w:pad_w + new_w] = resized
        return canvas, ratio, (pad_w, pad_h)

    def _nms(self, boxes, scores, classes):
        keep = []
        for class_id in np.unique(classes):
            indexes = np.where(classes == class_id)[0]
            order = indexes[np.argsort(scores[indexes])[::-1]]
            while order.size > 0:
                current = order[0]
                keep.append(current)
                if order.size == 1:
                    break
                ious = self._box_iou(boxes[current], boxes[order[1:]])
                order = order[1:][ious <= self.iou_thres]
        return np.array(keep, dtype=np.int32)

    @staticmethod
    def _box_iou(box, boxes):
        x1 = np.maximum(box[0], boxes[:, 0])
        y1 = np.maximum(box[1], boxes[:, 1])
        x2 = np.minimum(box[2], boxes[:, 2])
        y2 = np.minimum(box[3], boxes[:, 3])
        inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
        area1 = np.maximum(0, box[2] - box[0]) * np.maximum(0, box[3] - box[1])
        area2 = np.maximum(0, boxes[:, 2] - boxes[:, 0]) * np.maximum(0, boxes[:, 3] - boxes[:, 1])
        return inter / (area1 + area2 - inter + 1e-6)

    @staticmethod
    def _xywh_to_xyxy(boxes):
        result = boxes.copy()
        result[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
        result[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
        result[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
        result[:, 3] = boxes[:, 1] + boxes[:, 3] / 2
        return result

    def _maybe_xywh_to_xyxy(self, boxes):
        if np.any(boxes[:, 2] < boxes[:, 0]) or np.any(boxes[:, 3] < boxes[:, 1]):
            return self._xywh_to_xyxy(boxes)
        return boxes

    def _scale_normalized_boxes(self, boxes):
        if boxes.size and np.nanmax(boxes) <= 2.0:
            return boxes * float(self.img_size)
        return boxes

    @staticmethod
    def _clip_boxes(boxes, shape):
        h, w = shape
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, w)
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, h)

    @staticmethod
    def _sigmoid(value):
        return 1.0 / (1.0 + np.exp(-value))

    def _sigmoid_if_needed(self, value):
        value = value.astype(np.float32)
        if value.size == 0:
            return value
        if value.min() < 0.0 or value.max() > 1.0:
            return self._sigmoid(value)
        return value

    def _names_for(self, classes):
        if self.names:
            return self.names
        max_class = int(classes.max()) if len(classes) else 0
        return ['class_%d' % i for i in range(max_class + 1)]


def load_class_names(package_share_directory, model_name, class_names_value=''):
    if class_names_value:
        return [name.strip() for name in class_names_value.split(',') if name.strip()]

    config_dir = os.path.join(package_share_directory, 'config')
    candidates = [
        os.path.join(config_dir, model_name + '.names'),
        os.path.join(config_dir, model_name + '.txt'),
        os.path.join(config_dir, 'labels.txt'),
        os.path.join(config_dir, 'classes.txt'),
    ]
    for path in candidates:
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as file:
                return [line.strip() for line in file if line.strip()]

    if model_name == 'yolov5s':
        return COCO_NAMES
    return []
