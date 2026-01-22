import matplotlib.pyplot as plt
import numpy as np

class RemoteCommsUtils:
    def __init__(self):
        # Constants from the original file
        self.ThumbCenter = 127.0
        self.ThumbDeadZone = 10.0
        self.ThumbDeadZone_arm = 10.0
        self.arm_mode = False

    def thumb_curve(self, x):
        # Exact copy of the function from remote_messages_node.py
        # Intentionally preserving the logic to visualize the current behavior
        if self.arm_mode:
            if x > self.ThumbCenter + self.ThumbDeadZone_arm:
                # Note: Fixed the typo (was x - self.ThumbDeadZone_arm - self.ThumbDeadZone_arm)
                return (1.3 ** ((x - self.ThumbCenter - self.ThumbDeadZone_arm) / 10) - 1) / (1.3 ** ((self.ThumbCenter - self.ThumbDeadZone_arm) / 10) - 1)
            elif x < self.ThumbCenter - self.ThumbDeadZone_arm:
                return -(1.3 ** ((self.ThumbCenter - x - self.ThumbDeadZone_arm) / 10) - 1) / (1.3 ** ((self.ThumbCenter - self.ThumbDeadZone_arm) / 10) - 1)
            else:
                return 0.0
        else:
            if x > self.ThumbCenter + self.ThumbDeadZone:
                return (1.3 ** ((x - self.ThumbCenter - self.ThumbDeadZone) / 10) - 1) / (1.3 ** ((self.ThumbCenter - self.ThumbDeadZone) / 10) - 1)
            elif x < self.ThumbCenter - self.ThumbDeadZone:
                return -(1.3 ** ((self.ThumbCenter - x - self.ThumbDeadZone) / 10) - 1) / (1.3 ** ((self.ThumbCenter - self.ThumbDeadZone) / 10) - 1)
            else:
                return 0.0

def main():
    utils = RemoteCommsUtils()
    
    # Generate X values from 0 to 255
    x = np.linspace(0, 255, 1000)
    
    # Calculate Y values for Rover Mode (arm_mode = False)
    utils.arm_mode = False
    y_rover = [utils.thumb_curve(val) for val in x]
    
    # Calculate Y values for Arm Mode (arm_mode = True)
    utils.arm_mode = True
    y_arm = [utils.thumb_curve(val) for val in x]
    
    # Plotting
    plt.figure(figsize=(12, 8))
    
    plt.plot(x, y_rover, label='Rover Mode (arm_mode=False)', color='blue', linewidth=2)
    plt.plot(x, y_arm, label='Arm Mode (arm_mode=True)', color='red', linestyle='--', linewidth=2)
    
    # Add reference lines and zones
    plt.axhline(0, color='black', linewidth=1)
    plt.axvline(utils.ThumbCenter, color='green', linestyle=':', label='Center (127)')
    
    # Visualize Deadzones
    plt.axvspan(utils.ThumbCenter - utils.ThumbDeadZone, utils.ThumbCenter + utils.ThumbDeadZone, 
                color='blue', alpha=0.1, label='Rover Deadzone (+/- 10)')
    plt.axvspan(utils.ThumbCenter - utils.ThumbDeadZone_arm, utils.ThumbCenter + utils.ThumbDeadZone_arm, 
                color='red', alpha=0.1, ymin=0.0, ymax=0.05, label='Arm Deadzone (+/- 20)')

    plt.title('Analysis of thumb_curve Function Behavior')
    plt.xlabel('Input X (0-255)')
    plt.ylabel('Output Y')
    plt.legend()
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    
    output_file = 'thumb_curve_analysis.png'
    try:
        plt.savefig(output_file)
        print(f"Plot saved to {output_file}")
        # Attempt to show if environment supports it, though often it won't in headless
        # plt.show() 
    except Exception as e:
        print(f"Could not save or show plot: {e}")

if __name__ == "__main__":
    main()
