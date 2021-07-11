import time
import random
import sys
sys.path.append('../')

from Common_Libraries.p3b_lib import *

import os
from Common_Libraries.repeating_timer_lib import repeating_timer

def update_sim():
    try:
        my_table.ping()
    except Exception as error_update_sim:
        print (error_update_sim)

### Constants
speed = 0.1 #Qbot's speed

### Initialize the QuanserSim Environment
my_table = servo_table()
arm = qarm()
arm.home()
bot = qbot(speed)

##---------------------------------------------------------------------------------------
## STUDENT CODE BEGINS
##---------------------------------------------------------------------------------------

from typing import Tuple

def grab_container() -> None:
    """
    Function: grab_container()

    Purpose: Commands the Q-Arm to pick up the container from the table
    and prepare its position to be loaded onto the Q-Bot.

    Inputs: None
    
    Outputs: None

    Made By: Linan Yu
    """
    arm.move_arm(0.6791, 0.0, 0.2684)
    time.sleep(0.25)
    arm.control_gripper(45)
    arm.move_arm(0.372, 0.0, 0.241)
    arm.move_arm(0.3819, -0.0, 0.6216)

def position_container(position_id: int) -> None:
    """
    Function: position_container()

    Purpose: Positions the container above the desired location
    of the container, depending on which number
    container it is, out of 3 containers

    Inputs: position_id - position between 0 and 2 inclusive of
    which location number the container should be loaded to
    
    Outputs: None

    Made By: Willie Pai
    """
    LOAD_POSITIONS = [(-0.1433, -0.3938, 0.6197), (0.0, -0.3819, 0.6216), (0.1053, -0.3928, 0.6207)]

    # Move the Q-Arm to specific drop-off location based on which count of container is being transported
    arm.move_arm(LOAD_POSITIONS[position_id][0], LOAD_POSITIONS[position_id][1], LOAD_POSITIONS[position_id][2])

def drop_container() -> None:
    """
    Function: drop_container()

    Purpose: Lowers the Q-Arm's eblow, aligning the container
    upright and dropping the container onto the
    Q-Bot's hopper box, avoiding interfering with other loaded containers

    Inputs: None
    
    Outputs: None

    Made By: Linan Yu
    """
    arm.rotate_elbow(27)
    arm.control_gripper(-27)
    arm.rotate_elbow(-27)

def load_container(properties: Tuple[str, float, str]) -> Tuple[str, Tuple[str, float, str]]:
    """
    Function: load_container()
    
    Purpose: Loads the container that is already dispensed onto the table
    onto the Q-Bot, then dispenses and loads up to two more
    identical containers, while maintaining below a maximum mass
    of 90 grams. Returns the properties of the
    last dispensed container on the table

    Inputs: properties: a tuple holding the properties of the
    last dropped container
    
    Outputs: A string of the bin ID that the Q-Bot needs to deposit the
    containers into, and the properties of the last dropped container

    Made By: Willie Pai
    """
    MAX_MASS = 90
    BIN_ID_INDEX = 2
    MASS_INDEX = 1
    
    container_count = 0
    total_mass = 0

    reference_id = properties[BIN_ID_INDEX]

    print("\n---")
    print("Reference Container:")
    print("bin_id:", reference_id)
    print("mass:", properties[MASS_INDEX], "\n")

    # Continuously loads a new container until the 3 containers have been loaded, the next container's ID does not match, or the total mass exceeds the Q-Bot's limit
    while total_mass + properties[MASS_INDEX] <= MAX_MASS and container_count < 3 and reference_id == properties[BIN_ID_INDEX]:

        grab_container()

        position_container(container_count)
        drop_container()
        
        arm.home()

        container_count += 1
        total_mass += properties[MASS_INDEX]

        print("Current total mass on Q-Bot:", total_mass, "\n")

        properties = my_table.container_properties(random.randint(1, 5))

        print("Next Container:")
        print("bin_id:", properties[BIN_ID_INDEX])
        print("mass:", properties[MASS_INDEX])
        
        my_table.dispense_container()

    print("---\n")

    return(reference_id, properties)
    

def transfer_container(bin_id: str) -> None:
    """
    Function: transfer_container()

    Purpose: Transfers the containers from the Q-Bot into its target bin by
    using an ultrasonic sensor to detect the bin's location.

    Inputs: bin_id - a string of the bin ID that the Q-bot needs to
    deposit the containers into

    Outputs: None

    Made By: Willie Pai
    """
    print("Transferring the containers to", bin_id, "\n")
    
    bot.bot.move_time([0.05, -0.05], 8)
    time.sleep(0.5)
    
    bot.activate_ultrasonic_sensor()

    # Checks if distance of target bin is greater than 0.13 m away from the target bin, moving the Q-Bot at 0.2 m/s for faster travel
    while bot.read_ultrasonic_sensor(bin_id) > 0.13:
        velocity = bot.follow_line(0.2)[1]
        bot.forward_velocity(velocity)

    # Checks if distance of target bin is greater than 0.11 m away from the target bin, moving the Q-Bot at 0.1 m/s for careful travel
    while bot.read_ultrasonic_sensor(bin_id) > 0.11:
        velocity = bot.follow_line(0.1)[1]
        bot.forward_velocity(velocity)

    bot.forward_time(2.75)
    time.sleep(0.5)

    bot.activate_actuator()
    dump_containers()
    bot.deactivate_actuator()

    # Traverses the Q-Bot to Bin04 (bin closest to loop) to traverse over the intersection without alignment error
    while bot.read_ultrasonic_sensor("Bin04") > 0.15:
        velocity = bot.follow_line(0.25)[1]
        bot.forward_velocity(velocity)

    bot.deactivate_ultrasonic_sensor()

    bot.speed = 0.25
    bot.forward_time(5)
    bot.speed = 0.1


def return_home() -> None:
    """
    Function: return_home()

    Purpose: Sends the Q-bot back to its home position in front
    of the Q-Arm for loading by traversing Q-Bot until it locates a wall

    Inputs: None

    Outputs: None

    Made By: Willie Pai
    """
    lost_line = 0
    while lost_line < 1:
        lost_line, velocity = bot.follow_line(0.25)
        bot.forward_velocity(velocity)
        
    bot.depth()
    bot.speed = 0.1
    bot.travel_forward(0.179)
    time.sleep(0.5)

def dump_containers() -> None:
    """
    Function: dump_containers()
    
    Purpose: Commands the Q-Bot to dump the containers into the bin
    based on values processed from an angle vs time text file

    Input: None
    Output: None

    Made By: Willie Pai
    """
    times, angles = bot.process_file("dump.txt")

    # Iterates through each step in the dumping process, rotating the hopper based on angle vs. time data
    for i in range(1, len(times)):
        bot.rotate_actuator(angles[i])
        time.sleep(times[i] - times[i-1])

if __name__ == "__main__":
    return_home()
    
    last_container_properties = my_table.container_properties(random.randint(1, 5))
    my_table.dispense_container()
    
    while(True):
        bin_id, last_container_properties = load_container(last_container_properties)
        transfer_container(bin_id)
        return_home()
    

##---------------------------------------------------------------------------------------
## STUDENT CODE ENDS
##---------------------------------------------------------------------------------------
update_thread = repeating_timer(2, update_sim)
