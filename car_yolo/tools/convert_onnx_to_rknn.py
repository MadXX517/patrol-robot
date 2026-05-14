import argparse
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description='Convert a YOLOv5 ONNX model to RKNN.')
    parser.add_argument(
        '--onnx',
        default='car_yolo/config/traffic_640n_7.onnx',
        help='Path to the ONNX model.',
    )
    parser.add_argument(
        '--output',
        default='car_yolo/config/traffic_640n_7.rknn',
        help='Output RKNN model path.',
    )
    parser.add_argument(
        '--target-platform',
        required=True,
        help='Rockchip target platform, for example rk3566, rk3568, rk3588, or rv1126.',
    )
    parser.add_argument(
        '--dataset',
        default='',
        help='Optional quantization dataset txt. If omitted, builds a non-quantized RKNN model.',
    )
    parser.add_argument(
        '--quantized-dtype',
        default='asymmetric_quantized-u8',
        help='RKNN quantized dtype when --dataset is provided.',
    )
    parser.add_argument('--verbose', action='store_true', help='Enable RKNN Toolkit verbose logs.')
    return parser.parse_args()


def check_ret(ret, step):
    if ret != 0:
        raise RuntimeError('%s failed with code %s' % (step, ret))


def main():
    args = parse_args()
    onnx_path = Path(args.onnx).resolve()
    output_path = Path(args.output).resolve()
    dataset_path = Path(args.dataset).resolve() if args.dataset else None

    if not onnx_path.exists():
        raise FileNotFoundError('ONNX model not found: %s' % onnx_path)
    if dataset_path and not dataset_path.exists():
        raise FileNotFoundError('dataset txt not found: %s' % dataset_path)

    try:
        from rknn.api import RKNN
    except ImportError as exc:
        raise RuntimeError(
            'rknn.api is not installed. Install RKNN Toolkit2 on the conversion machine.'
        ) from exc

    rknn = RKNN(verbose=args.verbose)
    try:
        check_ret(
            rknn.config(
                mean_values=[[0, 0, 0]],
                std_values=[[255, 255, 255]],
                target_platform=args.target_platform,
                quantized_dtype=args.quantized_dtype,
                optimization_level=3,
            ),
            'config',
        )
        check_ret(rknn.load_onnx(model=str(onnx_path)), 'load_onnx')

        do_quantization = dataset_path is not None
        check_ret(
            rknn.build(
                do_quantization=do_quantization,
                dataset=str(dataset_path) if dataset_path else None,
            ),
            'build',
        )

        output_path.parent.mkdir(parents=True, exist_ok=True)
        check_ret(rknn.export_rknn(str(output_path)), 'export_rknn')
    finally:
        rknn.release()

    print('RKNN exported to %s' % output_path)


if __name__ == '__main__':
    main()
