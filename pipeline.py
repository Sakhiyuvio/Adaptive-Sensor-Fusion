import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import numpy as np

# Open serial port (replace with your actual COM port)
ser = serial.Serial('COM7', 115200)  

# Create figure and axes for the graphs
fig, ((ax1, ax2, ax5), (ax3, ax4, ax6)) = plt.subplots(2, 3, sharex=True, figsize=(12, 8))

# Lists to store the data
x_vals = []  # Time values
pitch_vals = []  # Pitch values
noisy_pitch_vals = []
filtered_pitch_vals = []
roll_vals = []  # Roll values
noisy_roll_vals = []
filtered_roll_vals = []

# LMS filter hyper-parameters
learning_rate = 0.001
filter_order = 10
lms_weight_pitch = np.zeros(filter_order)
lms_weight_roll = np.zeros(filter_order)

start_time = time.time()

def per_sample_lms_filter(noisy_sig, true_sig, weights, learning_rate, order):
    # based on the order of the filter U, we get the Mth order of the noisy signal!
    if len(noisy_sig) < order:
        # if true_sig:
        #     return true_sig[-1]
        if noisy_sig:
            return noisy_sig[-1]
        return 0
    
    # M samples retrieved successfully
    m_samples_noisy_sig = np.array(noisy_sig[-order:])[::-1] # reverse the samples for dot product (y output)

    # filter predicted output
    predict_output = np.dot(weights, m_samples_noisy_sig)

    # now we have both prediction and true signal
    # we get the error to perform gradient descent on weight updates
    error = true_sig[-1] - predict_output

    # update the weights, continue in real-time for adaptive lms

    # NOTE: may consider weight regularization to avoid gradient overflow issues 
    weights += learning_rate*error*m_samples_noisy_sig*2

    # weights = np.clip(weights, -2.0, 2.0)  # Clip weight updates to avoid extreme values

    return predict_output

# Define the update function for animation
def animate(i):
    data = ser.readline().decode('utf-8').strip()  # Read line, decode, and strip newlines
    print(f"Raw data: {data}")  # Print the raw data to debug

    try:
        # Split the data into pitch and roll values
        pitch, roll, noisy_pitch, noisy_roll = data.split(",")  # Expecting format "pitch,roll"
        pitch_value = float(pitch.strip())  # Convert pitch to float
        roll_value = float(roll.strip())    # Convert roll to float
        noisy_pitch_value = float(noisy_pitch.strip())
        noisy_roll_value = float(noisy_roll.strip())
    except ValueError:
        print("Error: Invalid data received")
        return

    # Append the time and pitch/roll values to the lists
    current_time = time.time()  # Get current time in seconds since epoch
    delta_t = current_time - start_time 
    x_vals.append(delta_t)
    pitch_vals.append(pitch_value)
    noisy_pitch_vals.append(noisy_pitch_value)
    roll_vals.append(roll_value)
    noisy_roll_vals.append(noisy_roll_value)

    # Perform adaptive filtering on noisy pitch and roll values
    filtered_pitch_value = per_sample_lms_filter(noisy_pitch_vals, pitch_vals, lms_weight_pitch, learning_rate, filter_order)
    filtered_roll_value = per_sample_lms_filter(noisy_roll_vals,roll_vals, lms_weight_roll, learning_rate, filter_order)
    filtered_pitch_vals.append(filtered_pitch_value)
    filtered_roll_vals.append(filtered_roll_value)

    # Keep only the last 50 values for smoother plot
    if len(x_vals) > 50:
        x_vals.pop(0)
        pitch_vals.pop(0)
        noisy_pitch_vals.pop(0)
        filtered_pitch_vals.pop(0)
        roll_vals.pop(0)
        noisy_roll_vals.pop(0)
        filtered_roll_vals.pop(0)

    # Clear the axes and plot the data
    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax4.clear()
    ax5.clear()
    ax6.clear()

    # get comparable resolution between desired and filtered sigs
    pitch_min = min(pitch_vals) - 0.1
    pitch_max = max(pitch_vals) + 0.1
    roll_min = min(roll_vals) - 0.01
    roll_max = max(roll_vals) + 0.01

    # Plot pitch vs time
    ax1.plot(x_vals, pitch_vals, label="Pitch", color="b")
    ax1.set_title("Pitch vs Time")
    ax1.set_ylabel("Pitch (degrees)")
    ax1.legend(loc="upper right")

    # Plot noisy pitch vs time
    ax2.plot(x_vals, noisy_pitch_vals, label="Noisy Pitch", color="g")
    ax2.set_title("Noisy Pitch vs Time")
    ax2.set_ylabel("Noisy Pitch (degrees)")
    ax2.legend(loc="upper right")

    # Plot roll vs time
    ax3.plot(x_vals, roll_vals, label="Roll", color="r")
    ax3.set_title("Roll vs Time")
    ax3.set_xlabel("Time (seconds)")
    ax3.set_ylabel("Roll (degrees)")
    ax3.legend(loc="upper right")

    # Plot noisy roll vs time
    ax4.plot(x_vals, noisy_roll_vals, label="Noisy Roll", color="orange")
    ax4.set_title("Noisy Roll vs Time")
    ax4.set_xlabel("Time (seconds)")
    ax4.set_ylabel("Noisy Roll (degrees)")
    ax4.legend(loc="upper right")

    # Plot filtered pitch vs time
    ax5.plot(x_vals, filtered_pitch_vals, label="Filtered Pitch", color="purple")
    ax5.set_title("Filtered Pitch vs Time")
    ax5.set_ylabel("Filtered Pitch (degrees)")
    ax5.legend(loc="upper right")
    # ax5.set_ylim(pitch_min, pitch_max)

    # Plot filtered roll vs time
    ax6.plot(x_vals, filtered_roll_vals, label="Filtered Roll", color="cyan")
    ax6.set_title("Filtered Roll vs Time")
    ax6.set_xlabel("Time (seconds)")
    ax6.set_ylabel("Filtered Roll (degrees)")
    ax6.legend(loc="upper right")
    ax6.set_ylim(roll_min, roll_max)

def main():
    # Create the animation
    ani = FuncAnimation(fig, animate, interval=1000)  # Update every 1000 ms

    # Display the plot
    plt.tight_layout()
    # plt.subplots_adjust(hspace=0.5, wspace=0.5)  # Adjust space between subplots
    plt.show()

if __name__ == "__main__":
    main()
