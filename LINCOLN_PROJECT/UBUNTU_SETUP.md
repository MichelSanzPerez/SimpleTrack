# Ubuntu setup and reproducibility guide

This guide explains how to reproduce the custom pedestrian tracking workflow provided in:

```text
LINCOLN_PROJECT/my_simpletrack.py
```

## 1. Clone the fork

```bash
git clone https://github.com/MichelSanzPerez/SimpleTrack.git
cd SimpleTrack
```

## 2. Create the Conda environment

```bash
conda create -n simple_track python=3.10 -y
conda activate simple_track
```

## 3. Install Python dependencies

```bash
python -m pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
conda install pyyaml -y
pip install -e .
```

## 4. Move into the custom project folder

The tested execution workflow was performed from inside `LINCOLN_PROJECT`:

```bash
cd LINCOLN_PROJECT
```

## 5. Run the custom SimpleTrack wrapper

```bash
python my_simpletrack.py \
  --json_path "/path/to/lidar_ann.json" \
  --output_json "/path/to/tracks_simpletrack.json"
```

### Example

```bash
python my_simpletrack.py \
  --json_path "/home/michel/Escritorio/SimpleTrackSalva/lidar_ann.json" \
  --output_json "/home/michel/Escritorio/SimpleTrackSalva/tracks_simpletrack.json"
```

## 6. Expected behavior

During execution, the script prints one line per frame, for example:

```text
1732099879_188289505.pcd: dets=1, tracks_all=1, tracks_valid=1
```

At the end, it writes the final JSON file:

```text
✅ Valid SimpleTrack tracks saved to /path/to/tracks_simpletrack.json
```

## 7. Notes on the tracker configuration

By default, the script uses the official pedestrian YAML:

```text
configs/waymo_configs/pd_kf_giou.yaml
```

You can override it if needed:

```bash
python my_simpletrack.py \
  --json_path "/path/to/lidar_ann.json" \
  --output_json "/path/to/tracks_simpletrack.json" \
  --config_path "/path/to/another_config.yaml"
```

## 8. Reproducibility notes

- The script expects detections in the JSON format:
  `BoundingBoxes = [x, y, z, l, w, h, roll, pitch, yaw]`
- The tracker exports only confirmed / valid tracks.
- The workflow assumes a static sensor and therefore uses an identity ego pose.
- The output JSON is flat and AB3DMOT-style, which makes it convenient for downstream evaluation and visualization.

## 9. Troubleshooting

### `ModuleNotFoundError: No module named 'yaml'`
Install PyYAML explicitly:

```bash
conda install pyyaml -y
```

### `Configuration YAML not found`
Make sure the repository structure is intact and that the file exists at:

```text
configs/waymo_configs/pd_kf_giou.yaml
```

### Running from a different directory
The validated workflow runs from:

```text
SimpleTrack/LINCOLN_PROJECT
```

Activate the environment first, then `cd LINCOLN_PROJECT`, and run the script from there.

### Paths with spaces
If a path contains spaces, wrap it in double quotes:

```bash
python my_simpletrack.py \
  --json_path "/path/with spaces/lidar_ann.json" \
  --output_json "/path/with spaces/tracks_simpletrack.json"
```
