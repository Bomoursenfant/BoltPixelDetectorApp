import argparse
import contextlib
import json
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--imgsz", type=int, default=640)
    args = parser.parse_args()

    try:
        with contextlib.redirect_stdout(sys.stderr):
            from ultralytics import YOLO
            model = YOLO(args.model)
            results = model.predict(
                source=args.image,
                conf=args.conf,
                iou=args.iou,
                imgsz=args.imgsz,
                retina_masks=True,
                verbose=False,
            )

        objects = []
        for result in results:
            if result.boxes is None or result.masks is None:
                continue

            polygons = result.masks.xy
            boxes = result.boxes
            for index, polygon in enumerate(polygons):
                if polygon is None or len(polygon) < 3:
                    continue

                confidence = float(boxes.conf[index].item()) if boxes.conf is not None else 0.0
                xyxy = boxes.xyxy[index].tolist() if boxes.xyxy is not None else []
                objects.append(
                    {
                        "confidence": confidence,
                        "box": [float(value) for value in xyxy],
                        "points": [[float(x), float(y)] for x, y in polygon],
                    }
                )

        print(json.dumps({"objects": objects}, separators=(",", ":")))
        return 0
    except Exception as exc:
        print(json.dumps({"error": str(exc)}, separators=(",", ":")))
        return 1


if __name__ == "__main__":
    sys.exit(main())
