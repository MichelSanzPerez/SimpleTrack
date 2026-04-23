# SimpleTrack Custom Pedestrian Wrapper

This fork contains a custom wrapper around the original [SimpleTrack](https://github.com/tusen-ai/SimpleTrack) 3D multi-object tracker to run tracking on a user-provided detections JSON file instead of the repository's native dataset loaders.

The custom script is located in:

```text
LINCOLN_PROJECT/my_simpletrack.py
```

## Repository objective

The goal of this fork is to preserve the original SimpleTrack tracking core while enabling a reproducible workflow for custom LiDAR-based pedestrian tracking experiments.

In this workflow:

- detections are provided through a JSON file with per-frame annotations,
- the tracker uses the official pedestrian YAML from the repository by default,
- the sensor is assumed to be static, so the ego pose is set to the identity matrix,
- only confirmed / valid tracks are exported,
- the output is written in a flat AB3DMOT-style JSON format for downstream visualization and evaluation.

## Input format

The script expects a detections JSON where each label contains:

```text
BoundingBoxes = [x, y, z, l, w, h, roll, pitch, yaw]
```

These detections are internally converted into the format expected by SimpleTrack:

```text
[x, y, z, yaw, l, w, h, score]
```

## Default tracker configuration

By default, the script uses the official pedestrian configuration shipped with the repository:

```text
configs/waymo_configs/pd_kf_giou.yaml
```

The path is resolved relative to the repository root, so it does not depend on machine-specific absolute paths.

Two wrapper-specific overrides are intentionally enforced:

- `has_velo = False`
- `pc = False`

These overrides are used because this custom workflow does not provide detector velocities and does not inject point clouds into the tracker state.

## What the script does

`my_simpletrack.py` performs the following steps:

1. reads the custom detections JSON,
2. sorts frames by `Timestamp`,
3. converts detections to the internal SimpleTrack input format,
4. creates one `FrameData` object per frame,
5. runs `tracker.frame_mot(...)`,
6. keeps only confirmed / valid tracks using `Validity.valid(state)`,
7. exports a flat tracking JSON.

## Command-line arguments

```bash
python my_simpletrack.py \
  --json_path "/path/to/lidar_ann.json" \
  --output_json "/path/to/tracks_simpletrack.json"
```

### Required arguments

- `--json_path`: path to the input detections JSON file
- `--output_json`: path to the output flat tracking JSON file

### Optional arguments

- `--config_path`: tracker YAML file. Defaults to `configs/waymo_configs/pd_kf_giou.yaml`
- `--default-score`: artificial confidence score for detections when the JSON does not include one. Default: `1.0`
- `--det-type`: detection type written into `FrameData`. Default: `1`

## Output format

The script writes a flat JSON list where each entry has the form:

```json
{
  "File": "1732099879_188289505.pcd",
  "track_id": 1,
  "x": 0.0,
  "y": 0.0,
  "z": 0.0,
  "l": 0.0,
  "w": 0.0,
  "h": 0.0,
  "theta": 0.0
}
```

Here, `theta` corresponds to yaw.

## Assumptions and scope

This wrapper is appropriate for scenarios where:

- the sensor platform is static,
- tracked objects are pedestrians,
- detections are already available frame by frame,
- point clouds may exist externally for visualization, but tracking itself is performed from detections.

This wrapper preserves the SimpleTrack tracking engine, but it does not reproduce the repository's full native dataset-loading pipeline.

## Environment

A Conda environment file is included as `environment.yml`.

Create and activate the environment with:

```bash
conda env create -f environment.yml
conda activate simple_track
```

For a full Ubuntu setup and a tested reproduction workflow, see [UBUNTU_SETUP.md](UBUNTU_SETUP.md).
