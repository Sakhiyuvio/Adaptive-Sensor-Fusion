import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

# Open serial port (replace with your actual COM port)
ser = serial.Serial('COM11', 115200)  # Change COM11 to your actual port

# Create figure and axes for the graphs
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

# Lists to store the data
x_vals = []  # Time values
pitch_vals = []  # Pitch values
roll_vals = []  # Roll values

# Define the update function for animation
def animate(i):
    data = ser.readline().decode('utf-8').strip()  # Read line, decode, and strip newlines
    print(f"Raw data: {data}")  # Print the raw data to debug

    try:
        # Split the data into pitch and roll values
        pitch, roll = data.split(",")  # Expecting format "pitch,roll"
        pitch_value = float(pitch.strip())  # Convert pitch to float
        roll_value = float(roll.strip())    # Convert roll to float
    except ValueError:
        print("Error: Invalid data received")
        return

    # Append the time and pitch/roll values to the lists
    current_time = time.time()  # Get current time in seconds since epoch
    x_vals.append(current_time)
    pitch_vals.append(pitch_value)
    roll_vals.append(roll_value)

    # Keep only the last 50 values for smoother plot
    if len(x_vals) > 50:
        x_vals.pop(0)
        pitch_vals.pop(0)
        roll_vals.pop(0)

    # Clear the axes and plot the data
    ax1.clear()
    ax2.clear()

    # Plot pitch vs time
    ax1.plot(x_vals, pitch_vals, label="Pitch", color="b")
    ax1.set_title("Pitch vs Time")
    ax1.set_ylabel("Pitch (degrees)")
    ax1.legend(loc="upper right")

    # Plot roll vs time
    ax2.plot(x_vals, roll_vals, label="Roll", color="r")
    ax2.set_title("Roll vs Time")
    ax2.set_xlabel("Time (seconds)")
    ax2.set_ylabel("Roll (degrees)")
    ax2.legend(loc="upper right")

# Create the animation
ani = FuncAnimation(fig, animate, interval=1000)  # Update every 1000 ms

# Display the plot
plt.tight_layout()
plt.show()