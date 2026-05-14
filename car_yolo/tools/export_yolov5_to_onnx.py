import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description='Export a YOLOv5 .pt model to ONNX.')
    parser.add_argument(
        '--weights',
        default='car_yolo/config/traffic_640n_7.pt',
        help='Path to the YOLOv5 .pt weights file.',
    )
    parser.add_argument(
        '--output',
        default='car_yolo/config/traffic_640n_7.onnx',
        help='Output ONNX path.',
    )
    parser.add_argument('--img-size', type=int, default=640, help='YOLO input image size.')
    parser.add_argument('--opset', type=int, default=12, help='ONNX opset version.')
    parser.add_argument(
        '--yolov5-repo',
        default='',
        help='Optional local ultralytics/yolov5 repo path. If omitted, uses python -m yolov5.export.',
    )
    parser.add_argument(
        '--no-simplify',
        action='store_true',
        help='Do not request ONNX simplification from YOLOv5 export.',
    )
    return parser.parse_args()


def build_export_command(args):
    weights = str(Path(args.weights).resolve())
    command = [sys.executable]

    if args.yolov5_repo:
        export_py = Path(args.yolov5_repo).resolve() / 'export.py'
        if not export_py.exists():
            raise FileNotFoundError('export.py not found in %s' % args.yolov5_repo)
        command.append(str(export_py))
    else:
        command.extend(['-m', 'yolov5.export'])

    command.extend(
        [
            '--weights',
            weights,
            '--img',
            str(args.img_size),
            '--batch',
            '1',
            '--include',
            'onnx',
            '--opset',
            str(args.opset),
        ]
    )

    if not args.no_simplify:
        command.append('--simplify')

    return command


def main():
    args = parse_args()
    weights_path = Path(args.weights).resolve()
    output_path = Path(args.output).resolve()

    if not weights_path.exists():
        raise FileNotFoundError('weights not found: %s' % weights_path)

    subprocess.check_call(build_export_command(args))

    generated_path = weights_path.with_suffix('.onnx')
    if not generated_path.exists():
        raise FileNotFoundError('YOLOv5 export did not create %s' % generated_path)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if generated_path != output_path:
        shutil.move(str(generated_path), str(output_path))

    print('ONNX exported to %s' % output_path)


if __name__ == '__main__':
    main()
