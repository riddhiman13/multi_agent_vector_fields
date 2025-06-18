import tkinter as tk
from tkinter import ttk
import yaml

# Initial parameters (defaults matching your provided values)
params = {
    "detect_shell_rad": 0.2,
    "agent_mass": 1.0,
    "agent_radius": 0.2,
    "velocity_max": 1.0,
    "approach_dist": 0.1,
    "k_a_ee": [4.0] * 7,
    "k_c_ee": [2.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0],
    "k_r_ee": [2.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0],
    "k_r_force": [0.0] * 7,
    "k_d_ee": [1.0] * 7,
    "k_manip": [0.0] * 7
}

# Define labels for inline comments
comments = {
    "detect_shell_rad": "sensing range [m]",
    "agent_mass": "agent mass [kg]",
    "agent_radius": "agent size [m]",
    "velocity_max": "max speed [m/s]",
    "approach_dist": "stop threshold [m]",
    "k_a_ee": "attraction",
    "k_c_ee": "circumvention",
    "k_r_ee": "repulsion",
    "k_r_force": "body repulsion",
    "k_d_ee": "damping",
    "k_manip": "manipulability"
}

float_entries = {}
list_entries = {}

def save_to_yaml():
    # Read single-value parameters
    for key in float_entries:
        params[key] = float(float_entries[key].get())

    # Read list parameters
    for key in list_entries:
        params[key] = [float(val.get()) for val in list_entries[key]]

    # Write to YAML with desired formatting
    with open("agent_parameters.yaml", "w") as file:
        for key, value in params.items():
            if isinstance(value, list):
                file.write(f"{key}: {value}   # {comments[key]}\n")
            else:
                file.write(f"{key}: {value:<8} # {comments[key]}\n")
    status_label.config(text="Saved to parameters.yaml!")

# GUI setup
root = tk.Tk()
root.title("Agent Parameter Tuner")
root.geometry("450x600")

# Float parameters (single values)
float_params = ["detect_shell_rad", "agent_mass", "agent_radius", "velocity_max", "approach_dist"]

for param in float_params:
    frame = ttk.Frame(root)
    frame.pack(pady=5, fill='x')
    ttk.Label(frame, text=param).pack(side='left')
    entry = ttk.Entry(frame, width=10)
    entry.insert(0, str(params[param]))
    entry.pack(side='right')
    float_entries[param] = entry

# List parameters (arrays for agents)
list_params = ["k_a_ee", "k_c_ee", "k_r_ee", "k_r_force", "k_d_ee", "k_manip"]

for param in list_params:
    ttk.Label(root, text=param).pack()
    frame = ttk.Frame(root)
    frame.pack(pady=2)
    entries = []
    for i in range(7):
        entry = ttk.Entry(frame, width=4)
        entry.insert(0, str(params[param][i]))
        entry.pack(side='left')
        entries.append(entry)
    list_entries[param] = entries

# Save button
ttk.Button(root, text="Save to YAML", command=save_to_yaml).pack(pady=20)

# Status label
status_label = ttk.Label(root, text="")
status_label.pack()

root.mainloop()
