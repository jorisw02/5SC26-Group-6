import numpy as np
from pycrazyswarm import Crazyswarm

# ----------------------------------------------------------

# Distance-based controller for a 2-drone line formation
class GradientFormationController:
    def __init__(self, desired_distance=0.5, kP=0.02):
        self.kP = kP
        # Single edge for two drones
        self.edges = [(0, 1)]
        self.d_star = np.zeros((2, 2))
        for i, j in self.edges:
            self.d_star[i, j] = desired_distance
            self.d_star[j, i] = desired_distance

    def compute_control_inputs(self, positions):
        controls = np.zeros_like(positions)  
        for i in range(2):
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
                # ui = np.clip(ui,-0.2,0.2)
                # ui[1] = np.clip(ui[1],-0.2,0.2)
                # ui[2] = np.clip(ui[2],-0.2,0.2)
            controls[:, i] = ui
        return controls 

# Main 
if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    controller = GradientFormationController(desired_distance=0.5, kP=0.1)

    TRIALS = 1
    TIMESCALE = 1.0
    dt = 0.05
    sim_duration = 15.0
    hover_time = 5
    eps = 0.1
    hover_steps = 200

    # Define initial positions for two drones
    initial_positions = np.array([cf.position() for cf in allcfs.crazyflies])

    print(initial_positions)
    # initial_positions = [[-1.0, 0.0, 0.0],[1.0, 0.0, 0.0]]


    try: 
        for _ in range(TRIALS):
            print("takeoff")
            dif1 = 1
            dif2 = 2
            start_time = timeHelper.time()
            while abs(dif1) > eps and abs(dif2) > eps:
                for i, cf in enumerate(allcfs.crazyflies):
                    pos = cf.position()
                    print("pos:")
                    print(f"{pos}")
                    print(f"{pos[2]}")
                    cf.cmdVelocityWorld(np.array([0.0, 0.0, (1.0-pos[2])*0.3]), yawRate=0.0)
                    print("Time:")
                    currenttime = timeHelper.time() - start_time
                    print(f"{currenttime}")
                    print("positions:")
                    positions = np.array([cf.position() for cf in allcfs.crazyflies]).T
                    print(f"{positions}")
                    timeHelper.sleepForRate(30)
                    print(f'{positions[2,1]}')
                    dif1 = positions[2,0] - 1.0
                    dif2 = positions[2,1] - 1.0
            print("takeoff done")

            # Formation control phase to achieve 0.5-meter separation
            print("control start")
            start_time = timeHelper.time()
            diff = 1
            while abs(diff-0.5) < eps:
                positions = np.array([cf.position() for cf in allcfs.crazyflies]).T
                print("positions:")
                print(f"{positions}")
                controls = controller.compute_control_inputs(positions)
                # dynamically changing control action
                # if diff < 0.8:
                #     controller = GradientFormationController(desired_distance=0.5, kP=0.3)
                print("control command:")
                print(f"{controls}")
                for i, cf in enumerate(allcfs.crazyflies):
                    vx, vy, vz = controls[:, i]
                    cf.cmdVelocityWorld(np.array([vx, vy, vz]), yawRate=0.0)
                    timeHelper.sleepForRate(30)
                positions = np.array([cf.position() for cf in allcfs.crazyflies]).T
                diff = np.linalg.norm(positions[:,0]-positions[:,1])
                print("diff:")
                print(f"{diff}")
            print("control done")
            
            for _ in range(hover_steps):
                for cf in allcfs.crazyflies:
                    cf.cmdVelocityWorld(np.array([0.0, 0.0, 0.0]), yawRate=0.0)
                    timeHelper.sleepForRate(30)

            # Stop motion and land
            dif1 = 1
            dif2 = 2
            print
            while abs(dif1) > eps and abs(dif2) > eps:
                for i, cf in enumerate(allcfs.crazyflies):
                    pos = cf.position()
                    print("pos:")
                    print(f"{pos}")
                    print(f"{pos[2]}")
                    cf.cmdVelocityWorld(np.array([0.0, 0.0, -(pos[2])*0.3]), yawRate=0.0)
                    print("Time:")
                    currenttime = timeHelper.time() - start_time
                    print(f"{currenttime}")
                    print("positions:")
                    positions = np.array([cf.position() for cf in allcfs.crazyflies]).T
                    print(f"{positions}")
                    timeHelper.sleepForRate(30)
                    print(f'{positions[2,1]}')
                    dif1 = positions[2,0] 
                    dif2 = positions[2,1]
            timeHelper.sleep(3.0)

    except Exception as error:
        # handle the exception
        print("An exception occurred:", error) # An exception occurred: division by zero
        # Stop motion and land
        for cf in allcfs.crazyflies:
            cf.cmdVelocityWorld(np.zeros(3), yawRate=0.0)
        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)