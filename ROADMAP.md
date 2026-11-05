# SesAnt roadmap: v2 (home puck), v3 (neck + mandibles), SesAnt Jumper (Jumper remix)

v1 is in this folder. It puts Sesame's electronics, MG90S servos and face on RealAnt's X body plan, with RealAnt blade shins and the RealAnt serial protocol. v2 and v3 build on it, and both stay at v1 size and on v1's electronics.

> **Status:** both are designed, generated and checked in CAD, but not printed yet: **[v2 → `v2/`](v2/README.md)** and **[v3 → `v3/`](v3/README.md)**.
>
> An earlier version of this roadmap had a 246 mm walk-on dock with one groove per foot, and an SO-ARM101 on a RealAnt-sized STS3215 robot. Both were replaced because they were impractical:
> * The dock needed four open-loop feet to land within ±6 mm.
> * The arm was three times the robot's size.

---

## v2: home puck (autonomous charging and RL reset pose)

### Goal
Let SesAnt run long RL or autonomy sessions without a human: walk home, charge, and start each episode from a known pose.

### Design
* **One alignment feature, not four.** A cone under the belly (bolted on with the bottom cover's own two screws) drops into a 30° funnel in a small round puck, 104 mm across and 22 mm high. The feet just stand on the floor.
* **Docking:** the robot walks over the puck, opens its knees (51° outward), then relaxes its servos so the cone slides into the centre.
* **Contacts:** a disc and a ring on the cone tip, with two spring pins in the funnel floor. They work at any heading and polarity can't reverse.
* **Homing:** the CoreS3 camera homes on an AprilTag flag that plugs into the puck.
* **Charging:** a USB-C PD trigger and a 2S charger sit inside the puck.

### Verified in CAD
* Capture up to ±11 mm off centre at headings of 0 and ±15°. The belly cone is always the first part to touch.
* Seated, only the cone touches the puck.
* 16 mm clearance when walking over the puck; knee load about 0.4 kg·cm.

### Next
1. Bench: 100 dock/undock cycles with `v2/tools/dock_bench.py`, logging success rate and charge voltage.
2. Check that limp MG90S legs let the body slide on a real floor. If not, add a small sway in the firmware before relaxing.
3. Soak test: 8 h unattended walk → dock → charge loop.

---

## v3: StackChan neck + ant mandibles (look and grab)

### Goal
Give SesAnt an actively aimed camera and a way to pick things up, without growing the robot or changing its electronics.

### Design
* **Neck:** two Feetech **SCS0009** servos, the ones in StackChan's own body, form a pan/tilt neck under the CoreS3 head. That gives the camera ±120° of pan and 20° up to 60° down, independent of the body.
* **Mandibles:** one MG90S turns the left mandible directly, and a printed involute gear pair (m 1.25, z 14) mirrors it onto the right one.
  * Fully open, the tips are 56 mm apart and reach 52 mm in front of the body.
* **Grabbing:** to pick something up, the robot crouches with its knees 65° out. The open mandibles end up 2 mm above the floor, and the head tilts 60° down to watch them.
* **Paw:** lifts the front-left leg on three feet to tap or push things.
* **Docking:** v3 keeps the v2 belly cone, so it still docks on the home puck.

### Verified in CAD
* Neck collision-free over the full pan range, from 25° up to 70° down.
* Gears mesh with zero overlap.
* Mandibles clear the housing, servo, chassis and belly cone.
* Grab pose: only the feet touch the floor, and the mandible tips sit on the camera axis.
* Paw pose: CoM 14 mm inside the three-foot support triangle.
* Front hips: −42…+90° (forward swing limited by the jaw).

### Next
1. Print the mandible pair and check gear backlash and grip on a cap, a ball and a cable.
2. Grasping on the CoreS3: detect the object with the camera, centre it between the mandibles, crouch, close the jaw.
3. Add the measured neck angles and the CoreS3 IMU to the RL observation (the head is now the robot's main sensor).

---

## SesAnt Jumper: the Jumper remix (arms with claws + simulator)

### Goal
Borrow what Jumper (KingKong Robotics' 22-DoF crab robot) does best, without copying its 400 mm, 24 V body: front arms with claws, and its simulation and RL training stack.

### Design
* **Arms:** two Jumper-style front arms, 3 MG90S each: shoulder yaw, shoulder pitch, gripper. The gripper servo sits behind the shoulder and drives the jaws with a wire pushrod, so there is no servo at the claw. The claws use Jumper's palm + finger outline (projected from its URDF meshes, ×0.34), with both jaws geared to swing open sideways together (horizontal grab). They replace the mandibles (`sesant_jumper/firmware/sesant_jumper_body`).
* **Simulator:** a MuJoCo model generated from the SesAnt CAD (meshes, joint axes and limits, estimated masses, servo saturation), plus a `sesant.flat` velocity task for Jumper's mjlab / rsl_rl framework, with an install script.

### Verified
* **Arms:** jaw gears mesh with zero overlap, and the pushrod linkage is smooth and clear. Every pose within yaw −5…+15° and pitch −15…+70° is collision-free. Standing, the claws reach the floor at 19° of pitch, the shoulder load is 3 % of stall, and the robot still docks (arms lifted). Walk with the arms level (front hips ±21°).
* **MuJoCo:** the model's mass matches the CAD. It stands on its 4 feet, reaches the floor with both claws to within 1.3 mm of the CAD pose, and stays upright through neck and arm sweeps and a drop.
* **Training:** the task registers, and PPO trains with Jumper's CPU backend (see `sesant_jumper/README.md`).

### Next
1. Train `sesant.flat` on a GPU, play the policy back in Jumper's viewer, then export it to ONNX.
2. Run the policy on the S2 Mini through the RealAnt `sN` / `meas` protocol (host-side inference first).
3. Add `sesant.grab`: reach, centre an object in a claw from the head camera, close, lift.

## Suggested order
1. **v1 print + walk test:** validate fit, tune `TRIM`/`DIR`, and confirm the 50 mm clearance holds under load.
2. **v2 home puck:** the cheapest way to unattended sessions.
3. **v3 neck + mandibles**, once v1 walks reliably.
4. **SesAnt Jumper:** the simulator can start right away (no hardware needed); the claw comes with v3.
