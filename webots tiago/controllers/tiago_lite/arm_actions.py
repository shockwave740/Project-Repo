# arm_actions.py
from controller import Supervisor
import numpy as np
from datetime import datetime # Must be imported for print_debug
import py_trees
# --- NEW: Lowered Arm Position for Initialization (The "Bottom" Pose) ---
LOWERED_ARM_JOINTS = {
    "torso_lift_joint": 0.15,
    "arm_1_joint": 1.571,
    "arm_2_joint": 0.5,     # Lowered elbow/shoulder joint
    "arm_3_joint": 0.0,
    "arm_4_joint": -0.262,
    "arm_5_joint": 1.571,
    "arm_6_joint": 0.0,
    "arm_7_joint": 0.0,
    "gripper_left_finger_joint": 0.045,
    "gripper_right_finger_joint": 0.045,
}

# Arm joint configuration needed for the arm actions
ARM_JOINTS = {
    "torso_lift_joint": 0.28,
    "arm_1_joint": 1.571,
    "arm_2_joint": 1.000,
    "arm_3_joint": 0.0,
    "arm_4_joint": -0.262,
    "arm_5_joint": 1.571,
    "arm_6_joint": 0.0,
    "arm_7_joint": 0.0,
    "gripper_left_finger_joint": 0.045,
    "gripper_right_finger_joint": 0.045,
}


class ArmController:
    def __init__(self, robot: Supervisor, timestep: int):
        self.robot = robot
        self.timestep = timestep
        self.motor_handles = {}
        
        print("ARM CONTROLLER DEBUG: Initializing Motors and Velocities...")
        
        # Initialize arm motors and set velocities
        for joint_name in ARM_JOINTS.keys():
            handle = self.robot.getDevice(joint_name)
            if handle:
                self.motor_handles[joint_name] = handle
                
                # --- CRITICAL FIX: Set Velocities Safely ---
                if 'torso_lift' in joint_name:
                    # Max velocity = 0.07. Set below limit.
                    handle.setVelocity(0.06) 
                elif 'gripper' in joint_name:
                    # Max velocity = 0.05. Set below limit.
                    handle.setVelocity(0.04) 
                else:
                    # Standard arm joints 
                    handle.setVelocity(0.5) 

    def control_arm(self, positions):
        """Sets arm joints to specified positions."""
        # Using a check to ensure motor handles exist before setting position
        for name, pos in positions.items():
            handle = self.motor_handles.get(name)
            if handle:
                handle.setPosition(pos)
        return True

    def wait_blocking(self, duration_steps):
        """Blocking wait for a specified number of time steps."""
        for _ in range(duration_steps):
            self.robot.step(self.timestep)

    def execute_coded_mission(self):
        """Executes the entire blocking grab/drop sequence."""
        self.print_debug("Executing  Arm Mission (Grab and Drop)")

        # 0. Initial Arm Pose -> GOTO BOTTOM
        self.control_arm(LOWERED_ARM_JOINTS) 
        self.wait_blocking(50) 
        
        # --- GRAB SEQUENCE ---
        
        # A. Lower Arm for Grab (Step 1)
        self.control_arm(
            {"arm_2_joint": 0, "arm_4_joint": 0, "arm_5_joint": 1.571}
        )
        self.wait_blocking(50)
        
        # B. Close Gripper (Step 2)
        self.control_arm(
            {"gripper_left_finger_joint": 0.015, "gripper_right_finger_joint": 0.015}
        )
        self.wait_blocking(50)
        
        # C. Lift Arm (Step 3)
        self.control_arm(
            {"arm_2_joint": 1.0, "arm_4_joint": -0.262, "arm_5_joint": 1.571}
        )
        self.wait_blocking(50)

        # --- DROP SEQUENCE ---
        
        # D. Position for drop (Step 4)
        self.control_arm(
            {"torso_lift_joint": 0.10, "arm_2_joint": 0, "arm_6_joint": -0.524}
        )
        self.wait_blocking(50)
        
        # E. Open Gripper (Step 5)
        self.control_arm(
            {"gripper_left_finger_joint": 0.045, "gripper_right_finger_joint": 0.045}
        )
        self.wait_blocking(50)
        
        # F. Reset Arm (Step 6)
        self.control_arm(
            {"torso_lift_joint": 0.28, "arm_2_joint": 1.0, "arm_6_joint": 0}
        )
        self.wait_blocking(50)

        self.print_debug(" Arm Mission Complete.")
        return True

    def print_debug(self, msg):
        # Note: Added datetime import at the top
        print(f"[{datetime.now().strftime('%H:%M:%S')}] ARM: {msg}")
        
        
        
        
    def grab_object(self):
        """Executes the arm sequence to grab and lift the object."""
        self.print_debug("Executing GRAB sequence...")
        
        # 0. Ensure arm is lowered (from the old execute_coded_mission logic)
        self.control_arm(LOWERED_ARM_JOINTS) 
        self.wait_blocking(50) 
        
        # A. Lower Arm for Grab (Step 1)
        self.control_arm(
            {"arm_2_joint": 0, "arm_4_joint": 0, "arm_5_joint": 1.571}
        )
        self.wait_blocking(50)
        
        # B. Close Gripper (Step 2)
        self.control_arm(
            {"gripper_left_finger_joint": 0.015, "gripper_right_finger_joint": 0.015}
        )
        self.wait_blocking(50)
        
        # C. Lift Arm (Step 3)
        self.control_arm(
            {"arm_2_joint": 1.0, "arm_4_joint": -0.262, "arm_5_joint": 1.571}
        )
        self.wait_blocking(50)
        
        return True # Successful grab and lift

    def drop_object(self):
        """Executes the arm sequence to position, drop, and reset."""
        self.print_debug("Executing DROP sequence...")
        
        # D. Position for drop (Step 4)
        self.control_arm(
            {"torso_lift_joint": 0.10, "arm_2_joint": 0, "arm_6_joint": -0.524}
        )
        self.wait_blocking(50)
        
        # E. Open Gripper (Step 5)
        self.control_arm(
            {"gripper_left_finger_joint": 0.045, "gripper_right_finger_joint": 0.045}
        )
        self.wait_blocking(50)
        
        # F. Reset Arm (Step 6)
        self.control_arm(
            {"torso_lift_joint": 0.28, "arm_2_joint": 1.0, "arm_6_joint": 0}
        )
        self.wait_blocking(50)

        return True # Successful drop and reset
    