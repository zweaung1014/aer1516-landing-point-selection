# Diagnosis: controller fails after the first waypoint (2026-10-27)

The controller switches to the next waypoint one hop too late. Because of that it plans a full braking stop on the first waypoint (WP1), and every run fails at that landing. Two more problems sit behind it: the wall hops ask for about 3× more energy than the push can deliver, and the controller flies its own arc, not the planner's. That's why raising the clearance gate didn't help. The target_apex wiring does work: the push controller (JHC) gets the higher targets. They just arrive too late.

## 1. Waypoints switch one hop late (all 8 runs with logs)

The next hop's direction (landing attitude, set by the jumping controller LJC) and its height (JHC push time) are set only at the apex ([hopcopter.py:798-830](../ros2_ws/src/hopping_robot/hopping_robot/hopcopter.py#L798-L830)). The waypoint is popped when the live position comes within 0.4 m ([hopcopter.py:680-691](../ros2_ws/src/hopping_robot/hopping_robot/hopcopter.py#L680-L691)). On the hop to WP1 that happens 0.13–0.28 s *after* the apex, and 0.02–0.17 s before touchdown, in every run.

So at the apex that decides the WP1 landing, the target is still WP1. LJC plans "land on WP1, take off straight up." JHC sizes the push for WP1's 0.70 m instead of WP2's 1.15 m. In the 10:52 run, the apex at t=2.58 s commands (24°, −15°). That is exactly the attitude that cancels the ~3 m/s incoming horizontal speed. WP2 only loads at 2.83 s, 0.06 s before touchdown.

## 2. Every run fails at the WP1 braking landing

| Run | WP1 landing (tilt, speed) | Prop work in next climb | Result |
|---|---|---|---|
| 09:13 | 30°, 4.2 m/s | 1.68 J | bounced 1.4 m back onto the wall, reached goal 3 hops later |
| 09:44 | 33°, 4.1 m/s | 0.72 J | bounced 1.1 m back, tipped over |
| 10:30 | 30°, 3.7 m/s | 0.63 J | bounced 0.7 m back, tipped over |
| 09:42, 09:47, 10:00, 10:25, 10:41, 10:52 | 28–34°, 3.7–4.2 m/s | **0.000 J** | tipped over (78–129°) |

- **The spin:** a stance at ~30° tilt spins the body at ~130°/s.
- **The six zero-work runs:** the spin keeps going at a constant rate, through level and onto the robot's side. In 10:52, roll goes +20° → −26° at the apex → −83° at the next touchdown.
- **What that means:** a constant rate means zero control torque. With the motors running, the Crazyflie (CF) would stop that spin in well under 0.1 s. Together with exactly 0 J of prop work, this means the CF stopped producing thrust after that landing.
- **Not the tumble cut-off:** `supervisorUpdate` and `supervisorOverrideSetpoint` are commented out in this SITL build ([stabilizer.c:330](../crazyflie-firmware/src/modules/src/stabilizer.c#L330)).
- **Suspected cause (not confirmed):** the Gazebo plugin packs acceleration into int16 at ±16 g with an unchecked cast ([crazysim_plugin.cpp:146-148](../crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp#L146-L148)). The leg spring is 1355 N/m with ~40 N (≈11 g) preload, so landings are estimated to peak around 25–30 g. That would feed the firmware's state estimator wrapped accelerometer data on every landing, worst on this one.

## 3. The wall hop needs more energy than the push gives

Compared per kg, because the planner uses `physics.mass` 0.8 kg and the real robot is 0.361 kg.

- **Planner limit:** up to g·1 m = 9.8 J/kg of injection per hop ([ballistic_planner_node.cpp:59](../ros2_ws/src/ballistic_motion_planner/src/ballistic_planner_node.cpp#L59)).
- **What it asked for:** every wall-crossing hop planned on 2026-10-27 uses 7.7–9.8 J/kg.
- **What the robot delivers:** logged climbs give 2.3–3.6 J/kg (best 4.6). Thrust 21000 is only ≈0.48 g, so the estimated ceiling is ~5–6 J/kg even if the push lasted the whole climb.
- The first hop needs 3.2 J/kg, which is within budget. That's why WP1 is reached so accurately.

## 4. The robot doesn't fly the planned arc, so the clearance check doesn't apply

- **Angle:** LJC picks its own takeoff angle. For these distances it usually caps at 45° ([jumping_model.py:149-151](../ros2_ws/src/hopping_robot/hopping_robot/JumpLib/jumping_model.py#L149-L151)), not the planner's 52–58°. JHC also picks its own push.
- **Result:** the hop onto WP1 peaks at 0.29–0.43 m in every run, against 0.70 m planned.
- **The 09:13 success:** its wall crossing was hop 1, the only hop whose target_apex arrives in time. It still peaked at 0.49 m against 1.28 m planned, about 0.1 m over the 0.4 m wall instead of 0.5 m.

## What to change, in order

1. **Switch waypoints at the apex:** use the predicted landing point (`LSE.landing_x/y`), so the apex before WP_k already aims at WP_{k+1}'s direction and target_apex. The 10:53 plan still turns ~93° at WP1, so that landing will be hard even with correct timing.
2. **Make plans executable:** set `e_inject_max` from measured climbs (~3 J/kg), cap hop length, and consider limiting the turn angle at waypoints.
3. **Fly the planned arc:** pass the planner's per-hop takeoff angle to LJC, so the planned arc and its clearance actually apply.
4. **Confirm the motor cut-out:** add the four motor PWMs (already subscribed) and the CF's own attitude (`/cf_1/pose`) to the per-tick attitude CSV. Then check whether the PWMs drop to zero at the WP1 touchdown.
