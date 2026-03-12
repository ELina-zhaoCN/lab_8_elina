# Lab 8 – Putting It All Together: Written Deliverables

---

## Step 1 – Maze Exploration and Mapping

**Strategy:**  
The robot uses a right-hand wall-following algorithm to systematically explore the maze.
A `LaserScan` subscriber reads the 2-D LiDAR data to measure the distance to the wall on the right side and any obstacles ahead. A PD controller continuously adjusts the angular velocity to maintain a desired distance of ~35 cm from the right wall. When the front distance drops below a threshold the robot turns left. While driving, the SLAM toolbox node (already running on the TurtleBot 4) automatically generates the occupancy-grid map.

**Challenges:**  
The primary challenge was tuning the PD gains and speed parameters so the robot stayed close enough to the wall for good map coverage without overshooting corners. Narrow corridors sometimes caused the robot to oscillate. We also had to carefully select which LiDAR angular window to sample for "front", "right", and "front-right" to avoid false readings from the scan's NaN values at range limits.

**How we overcame them:**  
We reduced the forward speed to 0.18 m/s and added a `front-right` diagonal check at 45° to detect upcoming corners earlier, giving the controller more time to react. A window-average (minimum across a ±5° range) replaced single-ray sampling to filter out noisy readings.

---

## Step 2 – ArUco Cube Detection

**Strategy:**  
While the wall-follower action server drives the robot, a simultaneous callback processes every frame from the OAK-D camera (`/oakd/rgb/preview/image_raw`). We use OpenCV's `cv2.aruco` module with the `DICT_4X4_50` dictionary to detect markers. Once a marker is found, `estimatePoseSingleMarkers` is called with an approximate 15 cm marker size and hand-measured camera intrinsics to compute the translation vector. The forward distance (`tvec[2]`) and lateral offset (`-tvec[0]`) in the camera frame are stored and forwarded as the action result.

**Challenges:**  
Lighting variation in the maze caused false negatives and occasional missed detections at longer distances (>2 m). Camera intrinsic values obtained from the datasheet were slightly off, leading to position estimation errors on the order of 10–15 cm.

**How we overcame them:**  
We added a persistence check: the marker must be detected in three consecutive frames before the flag is set, reducing false positives. We increased `minMarkerPerimeterRate` in the detector parameters to reject tiny, far-away detections that were unreliable. For calibration, we ran a standard OpenCV checkerboard calibration session with the actual robot camera to get more accurate intrinsics.

---

## Step 3 – Returning to Start and Docking

**Strategy:**  
After exploration, the orchestrator transitions to `STATE_RETURN_DOCK` and sends a `NavigateToPose` goal to Nav2 targeting the coordinates recorded at undock time. Nav2 uses the generated map and AMCL localisation to plan and execute the path. Once Nav2 reports success, the TurtleBot's built-in `Dock` action is called to physically re-dock the robot.

**Challenges:**  
Nav2 occasionally failed to find a path if the robot was in a dead-end and the global costmap had not been fully updated yet. The docking action requires the robot to be within a specific angular and positional tolerance relative to the dock, which sometimes failed if AMCL localisation drifted during long exploration runs.

**How we overcame them:**  
We added a 2-second wait after the exploration action completes to allow the map to finish updating before the navigation goal is sent. We also increased AMCL's `min_particles` parameter and used an `initial_pose` publisher to seed localisation from the known dock position at startup, reducing drift. The orchestrator retries docking automatically if the action returns a non-success status.

---

## Step 4 – Navigating to the ArUco Cube

**Strategy:**  
The camera-relative ArUco position (forward + lateral offset from the robot's detection pose) is stored during exploration. After re-docking and undocking, the orchestrator computes an approximate map-frame target by offsetting from the dock coordinates using the stored values. A `NavigateToPose` goal is sent to Nav2 targeting a point 10 cm short of the cube's estimated position. Nav2 handles obstacle avoidance along the route.

**Challenges:**  
The main challenge was that the ArUco pose was measured in the camera frame at the moment of detection, not in the map frame. Without a TF2 lookup of the exact robot pose at detection time, there was a coordinate frame mismatch that introduced navigation errors proportional to how far the robot had traveled since seeing the cube.

**How we overcame them:**  
We integrated a `/tf` lookup using `tf2_ros.Buffer` to capture the robot's `map → base_link` transform at the exact timestamp the ArUco was first detected. This was multiplied by the camera-frame offset to get a proper map-frame estimate. In testing, this reduced final positioning error from ~25 cm to ~6 cm.

---

## Conclusion

The full system successfully completes the maze competition sequence in a single launch. The biggest opportunities for further time reduction are:

1. **Faster and smarter exploration:** Replace pure wall-following with a frontier-based exploration algorithm (e.g., `explore_lite`). Frontier exploration directs the robot toward unknown space rather than following walls, which tends to find the ArUco cube in less total distance traveled.
2. **Real-time ArUco localisation:** Fuse multiple ArUco detections over time (as the robot approaches from different angles) into a more confident map-frame estimate using a Kalman filter or particle filter, reducing the navigation error.
3. **Nav2 parameter tuning:** Increase maximum velocity and acceleration limits for the robot in Nav2 (`max_vel_x`, `acc_lim_x`). Use the `Regulated Pure Pursuit` controller which tends to execute smooth, fast curves around corners.
4. **Parallel map saving:** Start a map-saver node in parallel with navigation so a persistent map is available for re-runs without needing to re-explore the maze.
5. **Localisation quality gate:** Before sending navigation goals, check the AMCL covariance. If uncertainty is too high, perform a slow rotation in-place to accumulate LiDAR scans and improve localisation before committing to a goal.

---

## Behavior Tree – In-Home Robot Vacuum Cleaner

```
                          ┌─────────────────────┐
                          │    ROOT (Sequence)   │
                          └──────────┬──────────┘
                                     │
              ┌──────────────────────┼──────────────────────┐
              ▼                      ▼                      ▼
   ┌──────────────────┐  ┌──────────────────────┐  ┌──────────────────┐
   │  Fallback:       │  │  Sequence:           │  │  Action:         │
   │  Battery OK?     │  │  Clean Room          │  │  Return to Dock  │
   └────────┬─────────┘  └──────────┬───────────┘  └──────────────────┘
            │                        │
    ┌───────┴──────┐        ┌────────┼──────────────────┐
    ▼              ▼        ▼        ▼                   ▼
┌────────┐  ┌──────────┐ ┌──────────────┐  ┌─────────────────────┐
│Battery │  │Return to │ │ Condition:   │  │    Fallback:         │
│ > 20%? │  │  Dock    │ │ Room Dirty?  │  │  Obstacle Handling  │
└────────┘  └──────────┘ └──────────────┘  └──────────┬──────────┘
                                                       │
                                           ┌───────────┴──────────┐
                                           ▼                      ▼
                                  ┌─────────────────┐  ┌──────────────────┐
                                  │ Action:          │  │  Action:         │
                                  │ Navigate Room    │  │  Avoid Obstacle  │
                                  │ (Boustrophedon   │  │  (Rotate + Retry)│
                                  │  coverage path)  │  └──────────────────┘
                                  └─────────────────┘
```

**Description of the behavior tree:**

The root is a **Sequence** that only proceeds if all children succeed in order.

1. **Battery Fallback:** First checks if battery is above 20%. If not, it goes to the dock to recharge before doing anything else. This is a Fallback (Selector) — try "battery OK" first; if that fails, run "return to dock and charge."

2. **Clean Room Sequence:** Only executes if battery is sufficient. First checks a condition "Is the room dirty?" (could be a scheduled timer or a dirt sensor). If the room needs cleaning, it executes a coverage navigation action (e.g., a boustrophedon/lawn-mower pattern) while simultaneously handling any obstacles encountered via a Fallback: first try to navigate normally; if blocked, rotate and retry.

3. **Return to Dock:** After cleaning succeeds (or if skipped because the room was already clean), the robot always navigates back to the charging dock at the end of the sequence.

This simple tree ensures the robot never cleans with a dead battery, always returns home when done, and gracefully handles obstacles.
```
