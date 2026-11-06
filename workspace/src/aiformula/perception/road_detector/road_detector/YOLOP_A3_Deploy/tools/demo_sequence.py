"""Run a calibrated recorded sequence through the public V76 frame API."""
import argparse
import json
from pathlib import Path
import sys
import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from yolop_a3_runtime import PerceptionModel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--weights', type=Path, default=ROOT / 'weights/shared_best.pth')
    parser.add_argument('--device', choices=('auto', 'cpu', 'cuda'), default='auto')
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Choose a new output directory; existing results are preserved')
    frames = json.loads(args.manifest.read_text(encoding='utf-8'))['frames']
    if not isinstance(frames, list) or not frames:
        raise ValueError('frames must be a nonempty list in capture order')
    model = PerceptionModel(args.weights, device=args.device)
    args.output.mkdir(parents=True)
    metadata = []
    for number, frame in enumerate(frames):
        path = args.manifest.parent / frame['image']
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f'Unreadable frame {number}: {path}')
        result = model.predict_frame(image, sequence=frame['sequence'], timestamp=frame['timestamp'],
                                     plane_to_image=frame['plane_to_image'])
        for name in ('lane_mask', 'obstacle_mask', 'combined_overlay'):
            destination = args.output / f'{number:06d}_{name}.png'
            if not cv2.imwrite(str(destination), getattr(result, name)):
                raise OSError(f'Cannot write {destination}')
        metadata.append(dict(frame=number, timestamp=frame['timestamp'], **result.detections_payload()))
    (args.output / 'results.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    print(f'V76 sequence: {len(frames)} frames -> {args.output.resolve()}')


if __name__ == '__main__':
    main()
