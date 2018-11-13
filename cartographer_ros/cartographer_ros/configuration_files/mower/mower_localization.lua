include "mower.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 2.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 4.

return options
