add_library(topology_v3_core STATIC
    src/fused_reference_builder.cpp
    src/geometry/frenet_projector.cpp
    src/navigation_reference_builder.cpp
    src/raw_visual_boundary_preprocessor.cpp
    src/raw_boundary_evidence_builder.cpp
    src/raw_ribbon_graph_builder.cpp
    src/topology_pipeline.cpp
    src/visual_reference_builder.cpp
)

target_include_directories(topology_v3_core PUBLIC include)
