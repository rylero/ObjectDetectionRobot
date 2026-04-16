# FRC 6238 Object Detection Robot

FRC robot project featuring a full object detection and autonomous acquisition pipeline. A Jetson coprocessor runs a TensorRT-accelerated neural network to detect game pieces, publishing detections to the roboRIO over NetworkTables. The robot then clusters, filters, tracks, and autonomously drives to pick them up.

## Object Detection Pipeline

<img width="684" height="718" alt="image" src="https://github.com/user-attachments/assets/3e98542c-b3cd-4438-92c9-e263e1111bc7" />

---

## Detection (`ObjectDetectionIOJetson`)

Raw detections arrive from the Jetson coprocessor via NetworkTables as an array of `TargetDetection` structs. Each detection carries:

| Field | Description |
|---|---|
| `dx` / `dy` | Camera-relative displacement to object (meters) |
| `area` | Bounding box area |
| `confidence` | Model confidence score |
| `timestamp` | FPGA timestamp of the frame |

A heartbeat topic is monitored to detect coprocessor connectivity. If the heartbeat stops updating within 0.5 seconds, the subsystem reports disconnected and stops accepting detections.

Detections are deduplicated by NT timestamp — the subsystem skips frames it has already processed.

---

## Clustering & World-Space Projection (`ObjectDetection`)

Each incoming detection is transformed from camera-relative coordinates into field-relative world coordinates:

1. The robot's pose is looked up at the **exact detection timestamp** using `drive.getTimestampPose(timestamp)` — compensating for robot motion between when the frame was captured and when it is processed.
2. The camera's offset from the robot center (`ROBOT_TO_CAMERA`, 15 inches forward) is applied.
3. The `dx`/`dy` displacement is applied to get a field-space `Pose2d` for the object.

Detections outside valid bounds (`dx <= 0` or `dx > 10m && |dy| > 10m`) are dropped before projection.

---

## Tracking & Filtering (`TrackedObject` / `MovingAveragePoseFilter`)

Projected poses are associated with existing tracked objects using nearest-neighbor clustering:

- If a new detection falls within **1.2 meters** of an existing tracked object, it is fused into that object.
- Otherwise, a new `TrackedObject` is spawned.
- Tracked objects that have not been seen for more than **1 second** are evicted.

Each `TrackedObject` smooths its pose through a **5-tap moving average filter** (`MovingAveragePoseFilter`) that averages X, Y, and heading independently (heading uses circular mean via `atan2(sin, cos)` to avoid wrap-around artifacts).

All tracked object poses are logged to AdvantageKit each loop for visualization in AdvantageScope.

---

## Pathfinding & Autonomous Acquisition (`AutoPilotUtils`)

When the driver holds **B** (or the `DynamicPickup` PathPlanner named command fires in auto), the robot autonomously drives to and acquires the closest tracked object.

### Standoff Pose Computation

Rather than driving directly to the object, the robot computes a **standoff pose** — a point `0.71 meters` behind the object along the approach vector. The approach angle is calculated as the vector from the robot's current position to the object's field-space position.

### Iterative Replanning

The pickup command runs in a tight loop (`oneSegment.repeatedly()`):

1. Compute standoff pose for closest tracked object.
2. Drive to standoff using the **Autopilot** holonomic path follower (`therekrab/autopilot`) with a `ProfiledPIDController` for heading.
3. While driving, monitor if the tracked object's position drifts more than **0.1 meters** from the original standoff. If so, replan immediately with an updated standoff.
4. Terminate successfully when the robot reaches within **0.06 meters** of the standoff.
5. Terminate with failure (and double-rumble the controller) if the target is lost for more than **0.5 seconds**.

---

## Setup & Deployment

See [SETUP_JETSON.md](SETUP_JETSON.md) for first-time laptop setup (SSH keys, Docker context, Git LFS).

See [JETSON_BUILD.md](JETSON_BUILD.md) for architecture details on the Docker/TensorRT build system.

```bash
# Clone with vision submodule
git clone --recursive https://github.com/6238/ObjectDetectionRobot.git

# Build and deploy vision code to Jetson, then restart
./gradlew deploy-jetson start-jetson

# Build and deploy robot code to roboRIO
./gradlew deploy
```
