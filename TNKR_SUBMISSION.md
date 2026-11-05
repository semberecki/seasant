# Tnkr Remix Challenge: submission kit

## Mod title
**SesAnt: Sesame goes RealAnt, with a StackChan face** (radial X-chassis, blade legs, CoreS3 camera head, RealAnt protocol)

## One-line summary
A remix of Sesame onto RealAnt's body plan. The hips move out onto a RealAnt X-chassis for a collision-free ±90° hip range, the shins become RealAnt blades (+40 %), and new firmware speaks RealAnt's serial protocol, so RealAnt's RL stack can drive a low-cost robot.

## What changed functionally (the "change how it works" part)
1. **Kinematics:** hips moved from a 49 × 51 mm box onto RealAnt-style radial pods at 61 mm on the diagonals, with servos along the arms. Collision-free hip sweep goes from ±56° to **±90°**.
2. **Legs:** RealAnt tapered blade shins on Sesame's foot clamps. Shin is +40 % longer and ground clearance goes from ~31 to ~50 mm.
3. **StackChan head:** Sesame's OLED cover is replaced by a head with an M5Stack CoreS3 built in. Its screen is the face, and it adds a forward camera, mics, speaker and IMU.
4. **Control:** ESP32 firmware that implements RealAnt's `sN <pos>` / `meas` protocol, so `ant_server.py` and realant-rl run unmodified.

## Follow-ups in the same package (optional extra Mods)
* **v2 home puck** (`v2/`): the robot walks over a 104 mm puck and sits down. A belly cone drops into a funnel and centres itself, and it charges at any heading. A good "how it works" story.
* **v3 neck + mandibles** (`v3/`): StackChan's own pan/tilt servos become SesAnt's neck, and a geared pair of ant mandibles picks things up while the head watches. All three source robots do real work here. A Leo prompt to try: "add a pan/tilt neck under the CoreS3 head using StackChan's servos", then compare the result with `v3/`.
* **SesAnt Jumper** (`sesant_jumper/`), the Jumper remix: Two Jumper-style front arms (shoulder yaw + pitch) with two-jaw claws that grab horizontally: the jaw outline is taken from Jumper's URDF, and both jaws are geared to swing open together, driven by a pushrod from a servo behind the shoulder. Also a MuJoCo model of SesAnt with a walking task for Jumper's RL training framework. A fourth robot remixed in, and the simulator lets anyone try SesAnt before printing it.

## Files to upload to the Mod
* `stl/*.stl`: 7 new printed parts (incl. the StackChan head), plus Sesame's original femurs (listed as reused)
* `assembly/sesant_assembly.glb`: full assembly for the 3D viewer
* `firmware/sesant_realant_bridge/`
* `README.md`: BOM, print, assembly, verification
* `images/`: hero, comparison and print-orientation renders

## Where Leo fits (for the "remix it with Leo" rule)
Starting points to try in Leo on Tnkr:
* Fork **Sesame** on Tnkr and ask Leo to "move the hip servos onto a RealAnt-style X plate with radial pods and keep the Sesame top cover interface". Compare its result with `stl/sesant_chassis.stl`.
* Ask Leo to "replace the foot truss with a longer tapered blade while keeping the servo clamp and screw holes".
* Ask Leo to "add RealAnt's serial protocol to the Sesame firmware".

Record where it succeeds and where it breaks: the challenge counts failures too. This package is the reference result to judge Leo against. It is fully verified in CAD, but not yet printed.

## Draft post
> Remixed two open-source quadrupeds for the @tnkr remix challenge: **Sesame** (cheap ESP32 + MG90S) × **RealAnt** (RL research ant) × **StackChan** (CoreS3 face + camera).
> SesAnt moves Sesame's hips onto a RealAnt X-chassis: ±90° collision-free hip range (was ±56°), RealAnt blade shins (+40 %), a StackChan CoreS3 head as face and camera, and firmware that speaks RealAnt's serial protocol, so the RealAnt RL stack drives a low-cost robot.
> Every new part is generated from the original STLs/STEP with booleans, then checked for collisions over the full joint range.
> [link to Mod] #robotics #opensource #3dprinting

## Feedback notes (fill in after trying Leo)
* What Leo did well:
* Where it broke:
* What would have helped:
