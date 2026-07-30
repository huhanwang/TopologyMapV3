# TopologyMap V3 Architecture

## Goal

TopologyMap V3 rebuilds the topology pipeline around explicit stage outputs and
strict topology ownership. Each stage owns one responsibility, writes a
debuggable artifact, and does not re-detect facts owned by later stages.

The main design goal is to make lane boundary topology explainable frame by
frame:

- where each reference line came from;
- how raw ribbons were built;
- how longitudinal gaps were repaired;
- how key-slice nodes were linked;
- where split/merge junctions were resolved;
- which relations are allowed to propagate.

## Main Pipeline

```text
Replay Adapter
        |
        v
Visual References
Navigation References
        |
        v
Fused Reference
        |
        v
Raw Boundary Evidence
        |
        v
Raw Ribbon Graph
        |
        v
LonLink Repair
        |
        v
KeySlice Graph
        |
        v
Junction Evidence Compiler
        |
        v
Resegmented Boundaries
        |
        v
Single-frame Propagation
        |
        v
Smooth Topology Tracker
        |
        v
Presentation / Debug Compiler
```

## Ownership Layers

### Replay Adapter

Replay owns DB/protobuf parsing, topic synchronization, and source timestamp
selection. Algorithm stages consume `ReplayFrameInput` only.

### Frenet Observation Compiler

The reference, raw evidence, ribbon, lonlink, keyslice, junction evidence,
resegmentation, and single-frame propagation stages compile current-frame facts.
They may create local observation ids and reject reasons, but they do not own
persistent boundary ids, lane relations, or junction lifecycle.

### Smooth Topology Tracker

The smooth tracker is the only stateful topology owner. It owns persistent
boundary ids, lane relation lifecycle, junction lifecycle, commit gates, and
association traces. It consumes current-frame evidence; it does not ask Frenet
stages to rewrite history.

### Presentation / Debug Compiler

Debug serialization and viewer layers expose artifacts. They must not repair,
commit, or reinterpret topology.

## Stage Responsibilities

### 1. Visual References

Build reference candidates from visual evidence.

Inputs:

- raw visual boundary observations;
- current pose;
- optional semantic boundary labels.

Outputs:

- visual reference candidates;
- confidence and source metadata;
- projection diagnostics.

This stage does not create topology.

### 2. Navigation References

Build reference candidates from navigation or map context.

Inputs:

- navigation route;
- map lane or road hints;
- current pose.

Outputs:

- navigation reference candidates;
- confidence and provenance.

This stage does not modify visual boundaries.

### 3. Fused Reference

Choose or synthesize the Frenet reference used by the current frame.

Inputs:

- visual references;
- navigation references;
- previous accepted reference, if available.

Outputs:

- fused reference polyline;
- reference quality;
- chosen source and fallback reason.

All later geometry is expressed in this fused Frenet frame.

### 4. Raw Boundary Evidence

Project visual boundaries into the fused Frenet frame.

Inputs:

- fused reference;
- visual boundaries.

Outputs:

- raw boundary nodes;
- source identity ids;
- semantic type;
- raw/reconstructed/connectivity ratio metadata, if available.

This stage only samples and normalizes observations. It does not repair gaps and
does not decide junctions.

### 5. Raw Ribbon Graph

Build lateral relationships between adjacent boundary observations.

Inputs:

- raw boundary nodes.

Outputs:

- lateral ribbons;
- width samples;
- profile type: stable, opening, closing, wide, narrow, invalid;
- support eligibility and rejection reasons.

This stage answers: "Can these two boundaries form a usable ribbon?" It does not
extend boundary geometry and does not create split/merge topology.

### 6. LonLink Repair

Repair missing longitudinal samples for each boundary.

Inputs:

- raw boundary nodes;
- raw ribbon graph;
- fused reference slices.

Outputs:

- observed lonlinks;
- repaired nodes;
- repaired lonlinks;
- repair stop records.

Repair stop records must be explicit:

- same_ft_missing;
- no_lateral_support;
- rejected_by_support_type;
- rejected_by_width_trend;
- rejected_by_corridor_width;
- stopped_by_near_contact;
- reached_frame_boundary.

This stage may mark a contact candidate when a boundary endpoint reaches another
boundary. It must not resolve split or merge topology.

### 7. KeySlice Graph

Build the sparse frame-local graph used for topology reasoning.

Inputs:

- observed and repaired boundary nodes;
- observed and repaired lonlinks;
- repair contact candidates.

Outputs:

- key slices;
- key-slice nodes;
- keynode links;
- contact candidates attached to key-slice nodes.

This stage is the only bridge between dense boundary geometry and sparse
topology.

### 8. Junction Evidence Compiler

Compile split, merge, and complex junction evidence from the KeySlice graph.

Inputs:

- key slices;
- keynode links;
- contact candidates.

Outputs:

- junction candidates;
- accepted frame-local evidence candidates;
- rejected junction candidates with reasons;
- frame-local resegmentation hints.

Rules:

- current-frame junction classification lives here only;
- a contact candidate is evidence, not an immediate committed junction;
- persistent split/merge commit is owned only by `SmoothTopologyTracker`;
- frame-local candidates must be explainable by keynode degree and local
  geometry, with reject reasons for failed candidates.

### 9. Resegmented Boundaries

Apply junction evidence to frame-local boundary evidence.

Inputs:

- raw/repaired boundary geometry;
- junction evidence;
- frame-local resegmentation hints.

Outputs:

- topology-aware local edges;
- parent/child edge relationships;
- edge provenance.

These outputs are still observations. They do not create persistent topology
identity.

Examples:

- split: one incoming edge is cut into an incoming parent and multiple outgoing
  children;
- merge: multiple incoming edges are attached to one outgoing continuation;
- complex: structure is preserved as explicit unresolved topology instead of
  being collapsed silently.

### 10. Single-frame Propagation

Decide which single-frame relations can propagate geometry or topology.

Inputs:

- resegmented boundaries;
- raw ribbon graph;
- junction evidence.

Outputs:

- propagation candidates;
- propagation direction;
- geometry propagation eligibility;
- association propagation eligibility;
- blocked reasons.

Propagation is a consequence of the current-frame evidence. It must not create
new split/merge topology or persistent track identity.

### 11. Smooth Topology Tracker

Match the current frame to history and update persistent tracks.

Inputs:

- resegmented boundaries;
- single-frame propagation candidates;
- junction evidence;
- previous tracks.

Outputs:

- matched tracks;
- new track candidates;
- confirmed tracks;
- retired tracks;
- persistent lane relations;
- persistent junction tracks.

The smooth tracker is the only stage allowed to commit, keep, or retire
persistent topology. If junction evidence changes owner or type, the tracker
creates new candidates and lets old tracks age out instead of silently mutating
confirmed identity.

## Junction Ownership

Junction handling has two different owners, separated by lifetime:

```text
JunctionEvidenceCompiler owns current-frame junction evidence.
SmoothTopologyTracker owns persistent junction lifecycle and commit.
```

Forbidden patterns:

- LonLink Repair directly committing split/merge topology;
- Frenet stages creating persistent junction ids;
- SmoothTopologyTracker re-detecting junctions from raw geometry instead of
  consuming junction evidence;
- viewer-specific topology fixes;
- multiple thresholds deciding the same "near" fact in different stages.

Allowed pattern:

```text
LonLink Repair records contact candidate
KeySlice Graph attaches candidate to sparse nodes
JunctionEvidenceCompiler classifies split/merge/complex evidence
Resegmented Boundaries apply frame-local evidence
Single-frame Propagation reports eligibility
SmoothTopologyTracker commits or rejects persistent topology over time
```

## Debug Layers

Every stage should produce a viewer layer.

Required layers:

- Visual Ref;
- Navigation Ref;
- Fused Ref;
- Raw Boundaries;
- Raw Ribbons;
- Ribbon Profiles;
- Observed LonLinks;
- Repaired Nodes;
- Repaired LonLinks;
- Repair Stops;
- Key Slices;
- Key Nodes;
- KeyNode Links;
- Contact Candidates;
- Junction Candidates;
- Junction Evidence;
- Resegmented Edges;
- Single-frame Propagation;
- Track Matching;
- Persistent Tracks.

Each debug item should include:

- stable local id;
- source ids;
- frame id;
- stage name;
- confidence or score;
- reason strings for rejection or blocking.

## V2 Reuse Policy

V2 is a reference implementation and data source, not the architecture source of
truth for V3.

Can reuse:

- input and output schema ideas;
- replay tooling;
- viewer interaction patterns;
- stable geometry helpers;
- proven scoring formulas after review.

Must not blindly reuse:

- split/merge detection distributed across multiple files;
- tracking code that reinterprets junctions;
- old Frenet ribbon graph repair logic unless it is isolated and rewritten
  behind the V3 stage interface;
- visualization code that compensates for model defects.

## Implementation Principles

- One stage owns one concept.
- Every stage emits inspectable data.
- Topology decisions are represented explicitly, not inferred from rendering.
- A rejected candidate must carry a reason.
- Geometry repair can create contact candidates, but it cannot commit
  split/merge topology.
- JunctionEvidenceCompiler can classify current-frame evidence, but only
  SmoothTopologyTracker can commit persistent split/merge topology.
- Persistent tracking consumes frame-local evidence and association scores; it
  does not invent topology from raw geometry.
