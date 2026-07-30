# Topology V3 Replay

This replay app reads frames directly from the source SQLite DB and streams them
through `topology_v3_core`.

The replay module is independent from the algorithm module:

- `applications/topology_v3_replay`: DB access, frame selection, topic sync,
  output files.
- `algorithms/topology_v3`: V3 frame pipeline and algorithm stages.

## Run

```bash
applications/topology_v3_replay/run_replay.sh
```

or pass a config explicitly:

```bash
applications/topology_v3_replay/run_replay.sh \
  applications/topology_v3_replay/configs/autofused_1970_2090_v3_replay.json
```

## Config

Important fields:

- `replay.source.path`: input DB path.
- `replay.main_axis.topic`: main topic used to select frames.
- `replay.main_axis.range`: frame or timestamp range.
- `replay.topics`: topics synchronized for each selected frame.
- `output.dir`: output dataset directory.
- `output.write_files`: per-frame JSON files to generate.
- `debug.layers`: debug layers to serialize when `topology_v3_debug` is written.

Example:

```json
{
  "output": {
    "write_files": ["frame", "sync", "topology_v3_debug"]
  },
  "debug": {
    "layers": ["visual_reference", "navigation_reference"]
  }
}
```

If an algorithm stage is unchanged or not needed for the current debug pass,
remove its layer from `debug.layers` or remove `topology_v3_debug` from
`output.write_files` to reduce output time.

## Viewer

Serve the `TopologyMapV3` directory and open:

```text
http://127.0.0.1:8093/viewer/topology_v3/
```

The default dataset path is:

```text
/out/autofused_1970_2090_v3_replay
```

## Current Status

The DB frame selection and topic synchronization path is active. The replay
adapter parses these protobuf topics and converts them into V3 input structs:

- `AutoFusedBevRoadGeometry`: visual boundary lines.
- `AutoSensorGnss`: GNSS pose used by navigation projection.
- `AutoSDRoute`: navigation route segments.

The first two algorithm outputs are wired:

- `visual_reference`
- `navigation_reference`

`topology_v3_debug.json` also writes V2-style `viz_layers` for these two
outputs so the viewer can expose them as independent toggles.
