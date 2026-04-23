#!/usr/bin/env python3
"""
Custom SimpleTrack wrapper for pedestrian 3D tracking from a user-provided JSON file.

This script adapts a detections JSON file with entries of the form:

    BoundingBoxes = [x, y, z, l, w, h, roll, pitch, yaw]

to the internal input format expected by SimpleTrack:

    [x, y, z, yaw, l, w, h, score]

Main design choices:
- Use the official pedestrian YAML from the SimpleTrack repository by default.
- Override `has_velo=False` and `pc=False` because this wrapper does not provide
  detector velocities and runs tracking from detections only.
- Assume a fixed platform / static sensor and therefore use identity ego pose.
- Export only confirmed / valid tracks in a flat AB3DMOT-style JSON output.
"""

import os
import json
import argparse
from pathlib import Path

import numpy as np
import yaml

# Limit thread usage to avoid OMP / MKL issues in shared servers or sandboxed environments.
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("KMP_INIT_AT_FORK", "FALSE")
os.environ.setdefault("MKL_THREADING_LAYER", "GNU")
os.environ.setdefault("NUMBA_DISABLE_JIT", "1")

from mot_3d.mot import MOTModel
from mot_3d.frame_data import FrameData
from mot_3d.data_protos import BBox, Validity


# Default input / output paths.
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_CONFIG_PATH = str(REPO_ROOT / "configs" / "waymo_configs" / "pd_kf_giou.yaml")

def load_config(config_path: str):
    """
    Load an official YAML file from the repository as the base tracker configuration.

    By default, this wrapper uses the pedestrian YAML shipped with the repository:
        configs/waymo_configs/pd_kf_giou.yaml

    Two wrapper-specific overrides are kept intentionally:
      - has_velo = False
      - pc = False

    These overrides are used because the input JSON does not provide detector
    velocities, and this script runs tracking from detections rather than from
    point clouds stored inside the tracker state.
    """
    if not config_path:
        config_path = DEFAULT_CONFIG_PATH

    if not os.path.isfile(config_path):
        raise FileNotFoundError(f"Configuration YAML not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.load(f, Loader=yaml.Loader)

    # Keep the wrapper aligned with a detections-only workflow.
    cfg["running"]["has_velo"] = False
    cfg["data_loader"]["pc"] = False
    return cfg


def load_frames(json_path, default_score=1.0):
    """
    Read the custom detections JSON and convert it into a frame-by-frame structure.

    Expected input format for each detection:
        BoundingBoxes = [x, y, z, l, w, h, roll, pitch, yaw]

    SimpleTrack expects detections in this format:
        [x, y, z, yaw, l, w, h, score]

    Parameters
    ----------
    json_path : str
        Path to the input detections JSON.
    default_score : float
        Artificial confidence score assigned to each detection when the input
        JSON does not include one.

    Returns
    -------
    list[dict]
        One dictionary per frame with:
        - Timestamp
        - File
        - dets
        - source_classes
    """
    with open(json_path, "r", encoding="utf-8") as f:
        raw = json.load(f)

    # Sort by timestamp to preserve temporal order.
    raw = sorted(raw, key=lambda x: x["Timestamp"])
    frames = []

    for entry in raw:
        timestamp = float(entry["Timestamp"])
        file_name = entry["File"]
        labels = entry.get("Labels", [])

        dets = []
        source_classes = []

        for lab in labels:
            bb = lab["BoundingBoxes"]

            # Expected layout:
            # BoundingBoxes = [x, y, z, l, w, h, roll, pitch, yaw]
            if len(bb) < 9:
                continue

            x, y, z = map(float, bb[0:3])
            l, w, h = map(float, bb[3:6])
            yaw = float(bb[8])

            # SimpleTrack detection layout:
            # [x, y, z, yaw, l, w, h, score]
            dets.append(np.asarray([x, y, z, yaw, l, w, h, float(default_score)], dtype=np.float32))
            source_classes.append(lab.get("Class", "unknown"))

        frames.append(
            {
                "Timestamp": timestamp,
                "File": file_name,
                "dets": dets,
                "source_classes": source_classes,
            }
        )

    return frames


def serialize_track_ab3dmot_style(file_name, bbox, track_id):
    """
    Convert a SimpleTrack BBox object into a flat AB3DMOT-style record.

    Output format:
        {
            "File": ...,
            "track_id": ...,
            "x": ...,
            "y": ...,
            "z": ...,
            "l": ...,
            "w": ...,
            "h": ...,
            "theta": ...
        }

    Here, `theta` corresponds to yaw.
    """
    arr = BBox.bbox2array(bbox).astype(float).tolist()
    return {
        "File": file_name,
        "track_id": int(track_id),
        "x": float(arr[0]),
        "y": float(arr[1]),
        "z": float(arr[2]),
        "l": float(arr[4]),
        "w": float(arr[5]),
        "h": float(arr[6]),
        "theta": float(arr[3]),
    }


def run_simpletrack(
    json_path,
    output_json,
    config_path="",
    default_score=1.0,
    det_type=1,
):
    """
    Run SimpleTrack frame by frame on the custom detections JSON.

    Workflow:
    1. Load and convert detections from the custom JSON format.
    2. Load tracker configuration from an official YAML file.
    3. Build one FrameData object per frame.
    4. Run tracker.frame_mot(...) for every frame.
    5. Export only confirmed / valid tracks in flat JSON format.

    Notes
    -----
    - `ego` is set to the 4x4 identity matrix because this wrapper assumes a
      fixed sensor / static platform.
    - `pc=None` because point clouds are not injected into the tracker state here.
    - `det_type=1` is used as a simple placeholder because all humans are treated
      as a single detection category.
    """
    frames = load_frames(json_path=json_path, default_score=default_score)
    configs = load_config(config_path)
    tracker = MOTModel(configs)

    # Identity ego pose: suitable for a fixed sensor setup.
    ego = np.eye(4, dtype=np.float32)

    # Flat output list containing only confirmed tracks.
    valid_tracks_flat = []

    for frame in frames:
        # Convert NumPy arrays to Python lists because FrameData is built from plain values.
        det_arrays = [d.tolist() for d in frame["dets"]]

        # All detections are treated as a single category in this wrapper.
        det_types = [det_type] * len(det_arrays)

        frame_data = FrameData(
            dets=det_arrays,
            ego=ego,
            time_stamp=frame["Timestamp"],
            pc=None,
            det_types=det_types,
            aux_info={
                "is_key_frame": True,  # Every frame in this JSON is treated as a key frame.
                "velos": None,         # No detector velocities are available in the input JSON.
            },
        )

        # Run one tracking step for the current frame.
        results = tracker.frame_mot(frame_data)

        frame_tracks_all = 0
        frame_tracks_valid = 0

        for bbox, track_id, state, _ in results:
            frame_tracks_all += 1

            # Export only tracks that SimpleTrack marks as confirmed / valid.
            if Validity.valid(state):
                frame_tracks_valid += 1
                valid_tracks_flat.append(
                    serialize_track_ab3dmot_style(
                        file_name=frame["File"],
                        bbox=bbox,
                        track_id=track_id,
                    )
                )

        print(
            f"{frame['File']}: dets={len(det_arrays)}, "
            f"tracks_all={frame_tracks_all}, tracks_valid={frame_tracks_valid}"
        )

    # Ensure the output directory exists before writing the JSON file.
    output_path = Path(output_json)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(valid_tracks_flat, f, indent=2)

    print(f"\n✅ Valid SimpleTrack tracks saved to {output_path}")


def build_argparser():
    """
    Build the command-line interface for the wrapper.

    The script assumes:
    - BoundingBoxes = [x, y, z, l, w, h, roll, pitch, yaw]
    - only confirmed / valid tracks are exported
    - the official pedestrian YAML is used by default
    """
    parser = argparse.ArgumentParser(
        description=(
            "Run SimpleTrack on lidar_ann.json assuming "
            "BoundingBoxes=[x,y,z,l,w,h,roll,pitch,yaw], export ONLY tracks_valid "
            "in flat AB3DMOT-style format, and use the official pedestrian YAML "
            "pd_kf_giou.yaml by default."
        )
    )
    parser.add_argument(
        "--json_path",
        type=str,
        required=True,
        help="Path to the input lidar_ann.json file",
    )
    parser.add_argument(
        "--output_json",
        type=str,
        required=True,
        help="Path to the flat AB3DMOT-style output JSON",
    )
    parser.add_argument(
        "--config_path",
        type=str,
        default=DEFAULT_CONFIG_PATH,
        help="Tracker configuration YAML. By default, uses the repository pedestrian YAML.",
    )
    parser.add_argument(
        "--default-score",
        type=float,
        default=1.0,
        help="Artificial detection confidence score used when the input JSON has no score field.",
    )
    parser.add_argument(
        "--det-type",
        type=int,
        default=1,
        help="Detection type written into FrameData. All humans are treated as one category here.",
    )
    return parser


def main():
    """Parse CLI arguments and execute the tracking pipeline."""
    parser = build_argparser()
    args = parser.parse_args()

    run_simpletrack(
        json_path=args.json_path,
        output_json=args.output_json,
        config_path=args.config_path,
        default_score=args.default_score,
        det_type=args.det_type,
    )


if __name__ == "__main__":
    main()
