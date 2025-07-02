README_implementation.txt

# Distance-Based Control with PID Velocity Commands for Crazyflie Swarm

## Controller Configuration

In this implementation, the Crazyflie firmware controller was switched from the default **Mellinger controller** to the **PID controller**. This change was necessary because `cmdVelocityWorld()` commands are not supported by the Mellinger controller, which is tuned for high-level trajectory commands like `takeoff()`, `goTo()`, and `land()`.

To enable velocity control, the controller type must be changed in the launch file (`hover_swarm.launch`) by adding or modifying the following parameter: firmwareParams.stabilizer.controller = 1 (1: PID, 2: Mellinger)

## Control Task Description:
The task performed in this script involves two Crazyflie drones taking off to a height of approximately 1 meter using a velocity based proportional controller. Once airborne, they adjust their relative positions to achieve a target distance of 0.5 meters between each other, using a distance-based formation controller. All control actions are sent as velocity commands in the world frame, fully compatible with the PID controller. After the formation is stabilized, the drones descend safely back to the ground.

