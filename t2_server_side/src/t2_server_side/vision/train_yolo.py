"""Validate labeled images, prepare a YOLO dataset, and optionally fine-tune.

Examples:
  python -m t2_server_side.vision.train_yolo --images /path/images --labels /path/labels --check-only
  python -m t2_server_side.vision.train_yolo --images /path/images --labels /path/labels --prepare-only
Remove --check-only/--prepare-only to start training. Originals are never edited.
Use --all-train to train on every image without held-out validation.
"""

import argparse
from collections import Counter, defaultdict
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import random
import shutil

from PIL import Image
import yaml


def audit(images_dir, labels_dir):
    classes_file = labels_dir / "classes.txt"
    names = classes_file.read_text(encoding="utf-8-sig").splitlines()
    if not names or any(not name.strip() for name in names) or len(set(names)) != len(names):
        raise ValueError("classes.txt must contain unique, nonempty class names")
    names = [name.strip() for name in names]
    images = sorted(
        (path for path in images_dir.iterdir() if path.suffix.lower() in (".png", ".jpg", ".jpeg")),
        key=lambda path: (0, int(path.stem)) if path.stem.isdecimal() else (1, path.name),
    )
    if len(images) < 2 or len({path.stem for path in images}) != len(images):
        raise ValueError("Need at least two images with distinct stems")
    records, counts, sizes, errors = [], Counter(), Counter(), []
    for image in images:
        label = labels_dir / (image.stem + ".txt")
        if not label.is_file():
            errors.append(f"Missing annotation: {label}")
            continue
        try:
            with Image.open(image) as opened:
                size = opened.size
                opened.verify()
            sizes[str(size)] += 1
        except Exception as error:
            errors.append(f"Unreadable image {image}: {error}")
            continue
        classes, rows = set(), []
        for line_number, line in enumerate(label.read_text(encoding="utf-8-sig").splitlines(), 1):
            if not line.strip():
                continue
            try:
                values = line.split()
                if len(values) != 5:
                    raise ValueError("expected class cx cy width height")
                cls = int(values[0])
                cx, cy, width, height = map(float, values[1:])
                if not 0 <= cls < len(names):
                    raise ValueError("class ID outside classes.txt")
                if not all(math.isfinite(v) and 0 <= v <= 1 for v in (cx, cy, width, height)):
                    raise ValueError("coordinates must be finite and normalized to [0,1]")
                if width <= 0 or height <= 0:
                    raise ValueError("box must have positive width and height")
                if min(cx-width/2, cy-height/2) < -2e-6 or max(cx+width/2, cy+height/2) > 1+2e-6:
                    raise ValueError("box extends beyond image boundary")
                counts[cls] += 1
                classes.add(cls)
                rows.append(line.strip())
            except ValueError as error:
                errors.append(f"{label.name}:{line_number}: {error}")
        records.append({"image": image, "label": label, "classes": classes,
                        "hash": hashlib.sha256(image.read_bytes()).hexdigest(),
                        "rows": tuple(sorted(rows))})
    image_stems = {path.stem for path in images}
    for label in labels_dir.glob("*.txt"):
        if label.name != "classes.txt" and label.stem not in image_stems:
            errors.append(f"Orphan annotation: {label.name}")
    if errors:
        raise ValueError("Dataset validation failed:\n" + "\n".join(errors))
    grouped = defaultdict(list)
    for record in records:
        grouped[record["hash"]].append(record)
    for group in grouped.values():
        if len({record["rows"] for record in group}) > 1:
            raise ValueError("Identical images have conflicting labels: " + str([r["image"].name for r in group]))
    report = {
        "images": len(records), "boxes": sum(counts.values()), "image_sizes": dict(sizes),
        "classes": [{"id": cls, "name": name, "boxes": counts[cls],
                     "images": sum(cls in record["classes"] for record in records)}
                    for cls, name in enumerate(names)],
        "duplicate_image_groups": [[r["image"].name for r in group]
                                   for group in grouped.values() if len(group) > 1],
    }
    return names, records, list(grouped.values()), report


def split_groups(groups, val_fraction, seed):
    if len(groups) < 2:
        raise ValueError("Need at least two distinct images for train/validation")
    all_classes = set().union(*(r["classes"] for group in groups for r in group))
    target = max(1, round(sum(map(len, groups)) * val_fraction))
    rng, best = random.Random(seed), None
    # Favor class coverage while keeping exact duplicate images in one split.
    for _ in range(200):
        ordered = list(groups)
        rng.shuffle(ordered)
        val, count, cutoff = [], 0, 0
        while count < target and cutoff < len(ordered)-1:
            val.extend(ordered[cutoff])
            count += len(ordered[cutoff])
            cutoff += 1
        train = [r for group in ordered[cutoff:] for r in group]
        train_classes = set().union(*(r["classes"] for r in train))
        val_classes = set().union(*(r["classes"] for r in val))
        score = (len(all_classes-train_classes), len(all_classes-val_classes), abs(count-target))
        if best is None or score < best[0]:
            best = score, train, val
    return best[1], best[2]


def train_only_trainer():
    """Skip even Ultralytics' mandatory final-epoch and final-checkpoint val."""
    from ultralytics.models.yolo.detect import DetectionTrainer

    class TrainOnlyDetectionTrainer(DetectionTrainer):
        def validate(self):
            # A constant bookkeeping fitness, NOT a measured accuracy score.
            # The resulting checkpoint is the final epoch, not a val-selected best.
            self.best_fitness = 0.0
            return {}, 0.0

        def final_eval(self):
            self.metrics = {}
            print("Held-out validation disabled. Use weights/last.pt for simulator testing.")

    return TrainOnlyDetectionTrainer


def main():
    project = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--weights", type=Path, default=project / "object_detection" / "model" / "yolo26l_kitchen.pt")
    parser.add_argument("--output", type=Path, help="New dataset/output directory; must not exist")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check-only", action="store_true")
    mode.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--val-fraction", type=float, default=.2)
    parser.add_argument("--all-train", action="store_true",
                        help="Use all labeled images; skip validation and early stopping")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--freeze", type=int, default=10)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--device", default="0", help="CUDA device index, or cpu")
    args = parser.parse_args()
    if not 0 < args.val_fraction < 1:
        parser.error("--val-fraction must be between 0 and 1")
    names, records, groups, report = audit(args.images.resolve(), args.labels.resolve())
    train, val = (records, []) if args.all_train else split_groups(groups, args.val_fraction, args.seed)
    report["mode"] = "all-train-no-validation" if args.all_train else "train-validation"
    for name, split in (("train", train), ("val", val)):
        report[name] = {"images": [r["image"].name for r in split],
                        "missing_classes": [names[cls] for cls in range(len(names))
                                            if not any(cls in r["classes"] for r in split)]}
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.all_train:
        print("ALL-TRAIN: every image is used for training. No held-out accuracy or mAP will be measured. "
              "Evaluate with newly captured simulator views; deploy weights/last.pt.")
    else:
        print("NOTE: nearby views from one scene can inflate validation scores. "
              "This random split is for initial experiments; collect separate scenes/sequences for evaluation.")
    if args.check_only:
        return
    if report["train"]["missing_classes"]:
        raise ValueError("Classes absent from training split: " + str(report["train"]["missing_classes"]))
    weights = args.weights.expanduser().resolve()
    if not weights.is_file():
        raise FileNotFoundError(weights)
    if not args.prepare_only:
        import torch
        if args.device != "cpu" and not torch.cuda.is_available():
            raise RuntimeError("CUDA unavailable; explicitly use --device cpu for CPU training")
    output = (args.output or project / "dataset" / ("finetune_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))).resolve()
    output.mkdir(parents=True, exist_ok=False)
    for split_name, split in (("train", train), ("val", val)):
        if not split:
            continue
        image_dir, label_dir = output / "images" / split_name, output / "labels" / split_name
        image_dir.mkdir(parents=True)
        label_dir.mkdir(parents=True)
        for record in split:
            shutil.copy2(record["image"], image_dir / record["image"].name)
            shutil.copy2(record["label"], label_dir / record["label"].name)
    data_yaml = output / "data.yaml"
    data_yaml.write_text(yaml.safe_dump({"path": str(output), "train": "images/train",
                                       # Framework requires a val path even when it is never evaluated.
                                       "val": "images/train" if args.all_train else "images/val",
                                       "names": dict(enumerate(names))},
                                      allow_unicode=True, sort_keys=False), encoding="utf-8")
    (output / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Prepared dataset: {data_yaml}")
    if args.prepare_only:
        print("Preparation only; training was NOT started.")
        return
    from ultralytics import YOLO
    model = YOLO(str(weights))
    extra = {}
    if args.all_train:
        extra["trainer"] = train_only_trainer()
        model.add_callback("on_train_start", lambda trainer: setattr(trainer, "metrics", {}))
    model.train(data=str(data_yaml), epochs=args.epochs, batch=args.batch,
                imgsz=args.imgsz, freeze=args.freeze, workers=args.workers,
                device=args.device, seed=args.seed, optimizer="AdamW", lr0=.0005,
                val=not args.all_train, patience=0 if args.all_train else 15,
                mosaic=.2, close_mosaic=min(10, args.epochs),
                project=str(output / "runs"), name="yolo26l_kitchen", exist_ok=False, **extra)
    print(f"Training output: {model.trainer.save_dir}")
    if args.all_train:
        print(f"Simulator evaluation checkpoint: {model.trainer.last}")
    print("Original pretrained weights and inference configuration were not replaced.")


if __name__ == "__main__":
    main()
