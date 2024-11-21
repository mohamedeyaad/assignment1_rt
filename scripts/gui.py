#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from assignment1_rt.msg import Vel
import tkinter as tk
from tkinter import messagebox

# Initialize publishers
pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
pub3 = rospy.Publisher('/turtle1/vel', Vel, queue_size=10)
pub4 = rospy.Publisher('/turtle2/vel', Vel, queue_size=10)

# Function to spawn the second turtle
def spawn_turtle():
    """Spawns turtle2 at a specified location."""
    x = rospy.get_param("/turtle2_pose/x")
    y = rospy.get_param("/turtle2_pose/y")
    theta = rospy.get_param("/turtle2_pose/theta")

    rospy.wait_for_service('/spawn')
    try:
        spawn_turtle_service = rospy.ServiceProxy('/spawn', Spawn)
        spawn_turtle_service(x, y, theta, "turtle2")
        rospy.loginfo("Successfully spawned turtle2.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

# Function to control the turtle
def control_turtle(event=None):  # `event` is optional and will be used when the Enter key is pressed
    """Controls the selected turtle by setting linear and angular velocities."""
    turtle = turtle_var.get()  # Get selected turtle name from dropdown

    try:
        # Get velocities from the input fields
        linear_vel = float(linear_vel_entry.get())
        angular_vel = float(angular_vel_entry.get())
    except ValueError:
        # Handle invalid input
        result_label.config(text="Invalid input. Please enter valid numbers for velocity.", fg="red")
        return

    # Select the appropriate publisher based on the selected turtle
    pub = pub1 if turtle == "turtle1" else pub2
    pub_vel = pub3 if turtle == "turtle1" else pub4

    # Create and configure the Twist message
    twist = Twist()
    twist.linear.x = linear_vel
    twist.angular.z = angular_vel
    
    # Create and configure the Vel message
    vel = Vel()
    vel.linear_velocity = linear_vel
    vel.angular_velocity = angular_vel

    # Publish the velocity command to the selected turtle
    pub.publish(twist)
    pub_vel.publish(vel)

    # Display the control result
    result_label.config(text=f"Controlling {turtle} with linear velocity {linear_vel} and angular velocity {angular_vel}", fg="green")

    # Introduce a delay of 1 second before stopping the turtle
    rospy.sleep(rospy.Duration(1)) 

    # Publish zero velocity after the delay
    twist.linear.x = 0
    twist.angular.z = 0
    vel.linear_velocity = 0
    vel.angular_velocity = 0
    pub.publish(twist)
    pub_vel.publish(vel)
    rospy.loginfo(f"Stopped {turtle} after 1 second.")

# Initialize the ROS node
def init_ros_node():
    """Initializes the ROS node and spawns turtle2."""
    rospy.init_node('ui_node', anonymous=True)
    spawn_turtle()  # Spawn turtle2 at startup

# Create the GUI window
def create_gui():
    """Creates and displays the GUI for controlling the turtles."""
    global turtle_var, linear_vel_entry, angular_vel_entry, result_label

    # Initialize the main window
    window = tk.Tk()
    window.title("Turtle Control")
    window.geometry("500x300+{}+{}".format((window.winfo_screenwidth() - 500) // 2, (window.winfo_screenheight() - 300) // 2))
    window.config(bg="#f4f4f4")

    # Set up the StringVar for the dropdown menu
    turtle_var = tk.StringVar()
    
    # Frame to hold input controls
    input_frame = tk.Frame(window, bg="#f4f4f4")
    input_frame.pack(pady=20)

    # Dropdown menu for selecting the turtle
    turtle_label = tk.Label(input_frame, text="Select Turtle:", font=("Arial", 12), bg="#f4f4f4")
    turtle_label.grid(row=0, column=0, padx=10, pady=5)

    turtle_var.set("turtle1")  # Default selection is turtle1
    turtle_dropdown = tk.OptionMenu(input_frame, turtle_var, "turtle1", "turtle2")
    turtle_dropdown.grid(row=0, column=1, padx=10, pady=5)

    # Entry fields for linear and angular velocities
    linear_vel_label = tk.Label(input_frame, text="Enter Linear Velocity:", font=("Arial", 12), bg="#f4f4f4")
    linear_vel_label.grid(row=1, column=0, padx=10, pady=5)

    linear_vel_entry = tk.Entry(input_frame, font=("Arial", 12), width=15, bd=2, relief="solid")
    linear_vel_entry.grid(row=1, column=1, padx=10, pady=5)

    angular_vel_label = tk.Label(input_frame, text="Enter Angular Velocity:", font=("Arial", 12), bg="#f4f4f4")
    angular_vel_label.grid(row=2, column=0, padx=10, pady=5)

    angular_vel_entry = tk.Entry(input_frame, font=("Arial", 12), width=15, bd=2, relief="solid")
    angular_vel_entry.grid(row=2, column=1, padx=10, pady=5)

    # Button to trigger turtle control
    control_button = tk.Button(window, text="Control Turtle", font=("Arial", 14), bg="#4CAF50", fg="white", bd=0, padx=20, pady=10, command=control_turtle)
    control_button.pack(pady=20)

    # Label to show control results
    result_label = tk.Label(window, text="", font=("Arial", 12), bg="#f4f4f4")
    result_label.pack(pady=10)

    # Bind the Enter key to trigger the control_turtle function
    window.bind("<Return>", control_turtle)

    # Start the Tkinter main loop
    window.mainloop()

# Initialize ROS node and create GUI
if __name__ == "__main__":
    init_ros_node()
    create_gui()
