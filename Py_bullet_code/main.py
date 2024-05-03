import pybullet as p
import time
import math
import numpy as np
from math import *
import keyboard
import simulation_pybullet as simulation

si = simulation.SimulationPyBullet()

def main() :
    while True :
        si.run_simulation()
        # si.verification_init = False
        dk_joint_angle = si.get_dk_angle()
        if si.verification_apply  :
            print("Joint Angles:", dk_joint_angle)
        si.close_simulation()


# Utilisation de la classe SimulationPyBullet
if __name__ == "__main__":  
    main()


# (-1.7453292519943295, 2.6179938779914944, -2.6179938779914944, -3.141592653589793, 3.490658503988659)