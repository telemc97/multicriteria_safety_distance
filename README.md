# Multicriteria Safety Distance

A submodule for the [UAV Landing Safety Pipeline](https://github.com/telemc97/safety_pipeline.git) that determines the safety radius for each occupied cell of an occupancy grid.

## Subscribing Topics
<ul>
    <li><code>/yolo_pointcloud/point_with_confidence</code></li>
    A custom message that consists of the coordinates of each detection in the coordinates of the camera and the confidence of each detection.
    <li><code>/projected_map</code></li>
    An occupancy grid (In the context of the Safety_Pipeline, it derives by projecting all detections from the world space onto the ground's plane. )
</ul>

## Publishing Topics
<ul>
    <li><code>/projected_map_with_safety_dist</code></li>
    An occupancy grid similar the input yet, in this case each detection is accompanied with its safety radius.
</ul>

## Parameters
<ul>
  <li><code>~res</code></li>
  Resolution of the occupancy grid (ressolution of the whole pipeline)
  <li><code>~min_safety_radius</code></li>
  Minimum safety radius.
  <li><code>~max_safety_radius</code></li>
  Maximum safety radius.
  <li><code>~multisample_resolution</code></li>
  Resolution for calculating the density and gradient of detections (<code>~res</code> x <code>~res</code>).
  <li><code>~detections_weight</code></li>
  Weight for the sum of detections
  <li><code>~average_slope_weight</code></li>
  Weight for the gradient of detections.
  <li><code>~average_density</code></li>
  Weight for the density of the detections.
</ul>