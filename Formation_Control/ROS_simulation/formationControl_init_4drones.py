# #!/usr/bin/env python

# ---------------------------------- 4 drones formation eqyal z direction initially ------------------------------------
import numpy as np
from pycrazyswarm import Crazyswarm

# Distance based controller for a 4-drone square formation
class GradientFormationController:
    def __init__(self, desired_distance=0.5, kP=0.02):
        self.kP = kP
        # Fully connected graph: each drone is connected to all others
        self.edges = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
        self.d_star = np.zeros((4, 4))
        for i, j in self.edges:
            if i-j == 2 or j-i == 2:
                self.d_star[i, j] = desired_distance * np.sqrt(2)
                self.d_star[j, i] = desired_distance * np.sqrt(2)
            else:
                self.d_star[i, j] = desired_distance
                self.d_star[j, i] = desired_distance

    def compute_control_inputs(self, positions):
        controls = np.zeros_like(positions)  
        for i in range(4):
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
                error = np.linalg.norm(diff)**2 - dij_star**2
                ui += self.kP * error * diff
            controls[:, i] = ui
        return controls 

# Main 
if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    controller = GradientFormationController(desired_distance=1.0, kP=0.02)

    TRIALS = 1
    TIMESCALE = 1.0
    dt = 0.05
    sim_duration = 30.0

    for i in range(TRIALS):
        # Takeoff
        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)

        # Vertical positioning above initial
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) +  np.array([0, 0, 1.0])
            cf.goTo(pos, yaw=0, duration=2.0)
        timeHelper.sleep(2.5)

        # Formation control phase
        start_time = timeHelper.time()
        while timeHelper.time() - start_time < sim_duration:
            positions = np.array([cf.position() for cf in allcfs.crazyflies]).T

            controls = controller.compute_control_inputs(positions)

            for i, cf in enumerate(allcfs.crazyflies):
                vx, vy, vz = controls[:, i]
                cf.cmdVelocityWorld(np.array([vx, vy, vz]), yawRate=0.0)

            timeHelper.sleep(dt)

        # Stop motion and land
        for cf in allcfs.crazyflies:
            cf.cmdVelocityWorld(np.zeros(3), yawRate=0.0)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)


    