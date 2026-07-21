#!/usr/bin/python3
"""Standalone inference benchmark -- no ROS, no cameras, no rqt.

Run the SAME command on each machine (laptop, CPU robot, GPU robot) to get
comparable numbers for "how fast does the model actually run here":

    source ~/agbot_venv/bin/activate     # wherever lightly_train/torch live
    cd ~/agbot_control_ws/src/agbot_vision_nav
    PYTHONPATH=src python3 scripts/benchmark_inference.py \
        --model config/exported_best.pt --image /path/to/a/640x480/frame.jpg

It prints the torch device the model landed on (a GPU robot printing 'cpu'
means torch has no CUDA there -- that IS the performance problem), then
mean/p50/p95 inference time and the resulting FPS. The first --warmup runs
are excluded: they include CUDA context/kernel compilation and are not
representative.

Interpreting against the live system: the node's end-to-end latency is
roughly one inference time plus a few tens of ms (frame transport/decode),
because the node always processes the latest frame. rqt display lag is NOT
part of control latency.
"""

import argparse
import time


def _percentile(sorted_vals, q):
    idx = int(round(q / 100.0 * (len(sorted_vals) - 1)))
    return sorted_vals[idx]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--model", required=True, help="path to exported_best.pt")
    parser.add_argument("--image", required=True, help="a saved camera frame (jpg/png)")
    parser.add_argument("-n", "--runs", type=int, default=50)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument(
        "--device", default="auto", help="auto (default) | cpu | cuda"
    )
    args = parser.parse_args()

    import cv2  # deferred so --help works without the venv

    from agbot_vision_nav.segmentation_model import SegmentationModel

    frame = cv2.imread(args.image)
    if frame is None:
        raise SystemExit("could not read image: %s" % args.image)

    print("Loading model %s ..." % args.model)
    t0 = time.monotonic()
    model = SegmentationModel(args.model, device=args.device)
    print("Loaded in %.1f s; inference device: %s" % (time.monotonic() - t0, model.device_str))
    if "cpu" in model.device_str:
        print("  NOTE: running on CPU. On the GPU robot this means torch has no")
        print("  usable CUDA (check nvidia-smi / torch.cuda.is_available()).")

    print("Input: %s %dx%d" % (args.image, frame.shape[1], frame.shape[0]))
    for _ in range(args.warmup):
        model.predict(frame)

    times = []
    for i in range(args.runs):
        t = time.monotonic()
        model.predict(frame)  # returns numpy, so any GPU work is synced
        times.append(time.monotonic() - t)

    times.sort()
    mean = sum(times) / len(times)
    print("%d runs (after %d warmup):" % (args.runs, args.warmup))
    print("  mean %.1f ms | p50 %.1f ms | p95 %.1f ms | max %.1f ms" % (
        mean * 1000.0,
        _percentile(times, 50) * 1000.0,
        _percentile(times, 95) * 1000.0,
        times[-1] * 1000.0,
    ))
    print("  sustained inference rate: %.2f FPS" % (1.0 / mean))


if __name__ == "__main__":
    main()
