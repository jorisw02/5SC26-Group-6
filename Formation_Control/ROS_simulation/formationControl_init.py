# #!/usr/bin/env python
import numpy as np
from pycrazyswarm import Crazyswarm

# Distance based controller
class GradientFormationController:
    def __init__(self, desired_distance=0.5, kP=0.005):
        self.kP = kP
        self.edges = [(0, 1), (1, 2), (2, 0)]  
        self.d_star = np.zeros((3, 3))
        for i, j in self.edges:
            self.d_star[i, j] = desired_distance
            self.d_star[j, i] = desired_distance

    def compute_control_inputs(self, positions):
        controls = np.zeros_like(positions)
        for i in range(3):
            ui = np.zeros(3)
            pi = positions[:, i]
            for (a, b) in self.edges:
                if a == i:
                    j = b
                elif b == i:
                    j = a
                else:
                    continue
                pj = positions[:, j]
                dij_star = self.d_star[i, j]
                diff = pj - pi
                error = np.linalg.norm(diff)*2 - dij_star*2
                ui += self.kP * error * diff
            controls[:, i] = ui
        return controls 

# Main 
if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    controller = GradientFormationController(desired_distance=0.5, kP=0.005)

    TRIALS = 1
    TIMESCALE = 1.0
    dt = 0.05
    sim_duration = 10.0

    for _ in range(TRIALS):
        # Takeoff
        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)

        # Vertical positioning above initial
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.goTo(pos, yaw=0, duration=2.0)
        timeHelper.sleep(2.5)

        # Formation control phase
        start_time = timeHelper.time()
        while timeHelper.time() - start_time < sim_duration:
            positions = np.array([cf.position() for cf in allcfs.crazyflies]).T
            center_of_mass = np.mean(positions, axis=1, keepdims=True)
            rel_positions = positions - center_of_mass

            controls = controller.compute_control_inputs(rel_positions)
            mean_control = np.mean(controls, axis=1, keepdims=True)
            controls -= mean_control

            for i, cf in enumerate(allcfs.crazyflies):
                vx, vy, vz = controls[:, i]
                cf.cmdVelocityWorld(np.array([vx, vy, vz]), yawRate=0.0)

            timeHelper.sleep(dt)

        # Stop motion and land
        for cf in allcfs.crazyflies:
            cf.cmdVelocityWorld(np.zeros(3), yawRate=0.0)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)
