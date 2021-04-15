#!/bin/bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{
    header: {
      frame_id: map
    },
    pose: {
      pose: {
        position: {
          x: 0,
          y: 0,
          z: 0
        },
        orientation: {
          x: 0,
          y: 0,
          z: 0,
          w: 1
        }
      }
    }
  }"
