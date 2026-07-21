# Perception backend contract

The official pick-and-place tree is deliberately kept unchanged. A perception
backend is replaceable if it provides:

| Interface | Purpose |
|---|---|
| `GetObjects` action at `/get_objects` | Stable object IDs, 2D boxes, class IDs and confidence |
| `GetObjectPose` action at `/get_object_pose` | Object pose in the configured camera frame |
| `AddMeshToObject` service | Associate an object ID with its mesh |
| `AssignNameToObject` service | Associate an object ID with its TF frame name |
| `ClearObjects` service | Clear backend state when requested |

The upstream ObjectSelection server continues to provide
`/get_selected_object`.

The AprilTag backend is the initial implementation. A later FoundationPose +
RT-DETR or SAM/SAM2 backend can be launched beside this repository and selected
with `start_apriltag:=False`. The AprilTag Object Server follows that flag by
default, preventing duplicate `/get_objects` and `/get_object_pose` action
names. Set `start_apriltag_object_server:=True` only when an external detector
still publishes `AprilTagDetectionArray` and should use this repository's
adapter. Set `start_object_selection_server:=False` if the replacement backend
also owns `/get_selected_object`. Its object frame must still agree with the
mesh and grasp reference frame.

AprilTag pose estimation is monocular and does not require depth. Aligned depth
is nevertheless retained for Robot Segmenter and nvblox, so dynamic collision
avoidance remains identical across perception backends.
