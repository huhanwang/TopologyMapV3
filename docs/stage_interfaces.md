# TopologyMap V3 Stage Interfaces

V3 keeps the replay adapter, algorithm stages, and debug serialization separate.
Replay owns DB/protobuf parsing. Algorithm stages consume V3 structs only.

## Reference Sources

The stage shape is based on two older code paths:

- V1-style independent modules:
  - `visual_reference/VisualReferenceModule`
  - `navigation_route/NavigationRouteTracker`
  - `fused_reference/FusedReferenceModule`
  - `boundary_sampling/BoundarySamplingModule`
  - `frenet/ribbon_graph/*`
- V2-style core boundary:
  - `TopologyFrameInput`
  - `TopologyFrameOutput`
  - `TopologyPipeline`

V3 combines these choices: each stage is an independent class, but all inputs
and outputs are V3 artifact structs in `frame_types.h`.

## Ownership Layers

V3 has three algorithm ownership layers. Stage order is still useful for debug,
but persistent topology belongs to only one layer.

```text
Replay Adapter
  -> converts DB/protobuf/topic sync into ReplayFrameInput

Reference Layer
  -> builds visual, navigation, and fused references

Frenet Observation Compiler, stateless
  -> compiles current-frame evidence only
  -> owns no persistent boundary, lane, or junction id

Smooth Topology Tracker, stateful
  -> owns persistent boundary ids
  -> owns lane relation lifecycle
  -> owns junction lifecycle and commit gates

Presentation / Debug Compiler
  -> serializes artifacts and viewer layers
  -> never fixes topology
```

## Stage Chain

```text
ReplayFrameInput
  -> VisualReferenceBuilder
  -> NavigationReferenceBuilder
  -> FusedReferenceBuilder
  -> RawVisualBoundaryPreprocessor
  -> RawBoundaryEvidenceBuilder
  -> FrenetSliceIntersectionBuilder
  -> RawFtFilterBuilder
  -> RawFtAssociationBuilder
  -> FrenetSliceGraphBuilder
  -> RibbonProfileCompiler
  -> LonLinkRepairer
  -> KeySliceGraphBuilder
  -> JunctionEvidenceCompiler
  -> BoundaryResegmenter
  -> SingleFramePropagator
  -> StatefulSmoothTopologyTracker
```

## Interface Summary

```cpp
VisualReferenceOutput VisualReferenceBuilder::build(
    const ReplayFrameInput& input) const;

NavigationReferenceOutput NavigationReferenceBuilder::build(
    const ReplayFrameInput& input,
    const VisualReferenceOutput& visual_reference) const;

FusedReferenceOutput FusedReferenceBuilder::build(
    const VisualReferenceOutput& visual_reference,
    const NavigationReferenceOutput& navigation_reference,
    const FusedReferenceOutput* previous_reference) const;

RawBoundaryEvidenceOutput RawBoundaryEvidenceBuilder::build(
    const ReplayFrameInput& input,
    const FusedReferenceOutput& fused_reference) const;

RawVisualBoundaryPreprocessOutput RawVisualBoundaryPreprocessor::build(
    const ReplayFrameInput& input) const;

FrenetSliceIntersectionOutput FrenetSliceIntersectionBuilder::build(
    const RawVisualBoundaryPreprocessOutput& preprocessed,
    const FusedReferenceOutput& fused_reference,
    const SmoothPoseInput* smooth_pose) const;

RawFtFilterOutput RawFtFilterBuilder::build(
    const FrenetSliceIntersectionOutput& intersections) const;

RawFtAssociationOutput RawFtAssociationBuilder::build(
    const FrenetSliceIntersectionOutput& intersections,
    const RawFtFilterOutput& filter) const;

FrenetSliceGraphOutput FrenetSliceGraphBuilder::build(
    const FrenetSliceIntersectionOutput& intersections,
    const RawFtFilterOutput& filter,
    const RawFtAssociationOutput& associations) const;

RawRibbonGraphOutput RibbonProfileCompiler::build(
    const FrenetSliceGraphOutput& slice_graph) const;

LonLinkRepairOutput LonLinkRepairer::repair(
    const FrenetSliceGraphOutput& slice_graph,
    const RawRibbonGraphOutput& ribbon_profiles,
    const FusedReferenceOutput& fused_reference) const;

KeySliceGraphOutput KeySliceGraphBuilder::build(
    const FrenetSliceGraphOutput& slice_graph,
    const LonLinkRepairOutput& lonlink_repair) const;

JunctionEvidenceOutput JunctionEvidenceCompiler::build(
    const KeySliceGraphOutput& key_slice_graph) const;

ResegmentedBoundaryOutput BoundaryResegmenter::apply(
    const LonLinkRepairOutput& lonlink_repair,
    const JunctionEvidenceOutput& junction_evidence) const;

SingleFramePropagationOutput SingleFramePropagator::build(
    const ResegmentedBoundaryOutput& boundaries,
    const RawRibbonGraphOutput& ribbon_profiles) const;

SmoothTopologyOutput StatefulSmoothTopologyTracker::update(
    const FrenetSliceGraphOutput& slice_graph,
    const JunctionEvidenceOutput& junction_evidence);
```

## Ownership Rules

- `FusedReferenceBuilder` is the only stage that chooses the frame reference.
- `RawVisualBoundaryPreprocessor` may only merge obvious same-source sections
  and hard reject unusable input. It must keep rejected records and reasons.
- `RawBoundaryEvidenceBuilder` is the first stage allowed to create Frenet
  samples for visual boundaries. It remains a debug/projection artifact while
  slice intersections become the topology-ready representation.
- `FrenetSliceIntersectionBuilder` creates observed slice nodes. It does not
  filter plausible boundaries except when projection is impossible.
- `RawFtFilterBuilder` owns frame-local raw FT use decisions: kept, pending,
  suppressed, or passive boundary.
- `RawFtAssociationBuilder` owns current-frame raw FT continuation/grouping
  evidence. It does not create persistent identity.
- `FrenetSliceGraphBuilder` owns current-frame nodes, lonlinks, lateral links,
  slice ribbons, and ribbon transition evidence.
- `RibbonProfileCompiler` may summarize slice ribbons for debug display and
  propagation scoring. The older boundary-pair `RawRibbonGraphBuilder` is only
  a temporary compatibility path until the slice graph is live.
- `LonLinkRepairer` repairs longitudinal samples and records contact evidence;
  it does not resolve split/merge topology.
- `JunctionEvidenceCompiler` classifies current-frame split/merge/complex
  evidence, but it does not commit persistent junction topology.
- `BoundaryResegmenter` may create frame-local topology-aware edges from
  current-frame evidence, but those edges are still observations.
- `SingleFramePropagator` reports frame-local propagation eligibility. It does
  not create or retire tracks.
- `StatefulSmoothTopologyTracker` is the only owner of persistent boundary ids,
  lane relation lifecycle, junction lifecycle, and commit gates.
- `Presentation / Debug Compiler` only serializes artifacts; it must not repair
  or reinterpret topology.
