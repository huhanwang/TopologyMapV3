# TopologyMap V3 Open Issues

## Raw FT Association Intermediate Blocker False Positive

Status: deferred

Observed frame: `1999`

Layer: `Raw FT associations`

### Symptom

`R9 -> R10` is displayed as `blocked_by_intermediate_fragment`, even though the
two fragments are visually and geometrically close to one continuous boundary.

Observed values:

- `R9`: slice `30-34`
- `R10`: slice `35-47`
- gap: `2.0m`
- endpoint lateral delta: `0.004m`
- shared anchor `R1`, width delta `0.003m`
- shared anchor `R5`, width delta `0.097m`

This is a strong continuation candidate by endpoint geometry and shared ribbon
anchor evidence.

### Current Rule

`RawFtAssociationBuilder` generates continuation candidates when two raw FT
fragments have:

- valid longitudinal gap;
- small endpoint lateral delta;
- shared endpoint ribbon anchor;
- small width delta to that anchor.

After candidate generation, a `continuation_candidate` is downgraded to
`blocked_by_intermediate_fragment` if another usable lane-line fragment:

- is not the candidate source or target;
- overlaps the candidate gap in `s`;
- has endpoint evidence with the same shared anchor and side.

### Why The Rule Exists

The blocker is intentional. It prevents unsafe one-frame raw FT stitching in
cases such as:

- skipping over a real middle fragment and connecting two farther fragments;
- wrongly stitching across split/merge regions where multiple short fragments
  share the same anchor;
- creating multiple competing continuations from the same local ribbon context.

This rule exists in the base `FrenetRibbonGraphBuilder` as well. It should not
be removed outright.

### Why Frame 1999 Is A False Positive

In frame `1999`, `R5` crosses the `R9 -> R10` gap and shares anchor evidence,
so it blocks the candidate. However, `R5` is the nearby lateral boundary used
as ribbon context; it is not a fragment competing for the same continuation as
`R9 -> R10`.

The current blocker only checks `s` overlap and shared anchor evidence. It does
not check whether the intermediate fragment lies inside the candidate
continuation corridor or merely acts as a side anchor.

### Deferred Fix Direction

Keep the blocker, but make it geometry-aware.

An intermediate fragment should block a candidate only when it is laterally
competitive with the candidate continuation, for example:

- its lateral position is close to the interpolated `from -> to` continuation;
- or it occupies the same continuation corridor between the same anchors;
- or it would produce an equal or better candidate for the same source/target
  side.

Fragments that are only side anchors or stable neighboring boundaries should
not block.

For frame `1999`, this would allow `R9 -> R10` while still blocking long-range
candidates such as `R9 -> R7` or `R9 -> R6` when an actual competing fragment
exists.

### Visualization Note

The `Raw FT associations` layer currently displays all association candidates,
including rejected candidates. Red dashed lines are rejected evidence, not
successful associations.

Possible viewer cleanup:

- keep `Raw FT associations` focused on `ready_continuation`;
- add a separate `Rejected FT associations` layer for
  `blocked_by_intermediate_fragment` and `ambiguous_branch`;
- or hide rejected labels by default.
