import subprocess
import tkinter as tk

CAN_INTERFACE = "can0"
CAN_POSITION_IDS = (0x210, 0x211, 0x212)
METACARPAL_MOTORS = (0, 2, 4)
CARRIAGE_MOTORS = (1, 3, 5)
SLIDER_UPDATE_MS = 50
SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 270.0

# ---- Fake data (for now) ----
motor_positions = [0, 0, 0, 0, 0, 0]
motor_position_targets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
motor_currents = [0.10, 0.20, 0.15, 0.30, 0.25, 0.18]

servo_positions = [90, 90]
servo_position_targets = [90.0, 90.0]
current_config = 1

CONFIG_PRESETS = {
    1: {"name": "One-sided Grip", "servo_angles": [5.0, 210.0]},
    2: {"name": "Pinch Grip", "servo_angles": [102.0, 98.0]},
    3: {"name": "Claw Grip", "servo_angles": [140.0, 45.0]},
    4: {"name": "Coffee Cup Grip", "servo_angles": [210.0, 5.0]},
}

# ---- Functions ----
def pack_i32_le(value):
    return int(value).to_bytes(4, byteorder="little", signed=True)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def send_position_targets():
    for frame_index, can_id in enumerate(CAN_POSITION_IDS):
        first_motor = frame_index * 2
        payload = (
            pack_i32_le(motor_positions[first_motor])
            + pack_i32_le(motor_positions[first_motor + 1])
        )
        frame = f"{can_id:03X}#{payload.hex().upper()}"

        try:
            subprocess.run(
                ["cansend", CAN_INTERFACE, frame],
                check=True,
                capture_output=True,
                text=True,
            )
        except FileNotFoundError:
            print("cansend is not installed or not on PATH")
            return False
        except subprocess.CalledProcessError as exc:
            error_text = exc.stderr.strip() or exc.stdout.strip() or str(exc)
            print(f"Failed to send position CAN frame {frame}: {error_text}")
            return False

    print(f"Position targets sent: {motor_positions}")
    return True


def send_target(motor_index):
    try:
        target = int(entry_vars[motor_index].get())
    except ValueError:
        print(f"Motor {motor_index}: invalid input")
        return

    print(f"Motor {motor_index} target set to {target}")
    motor_position_targets[motor_index] = float(target)
    motor_positions[motor_index] = target
    send_position_targets()
    update_display()


def update_targets_from_sliders():
    now_ms = int(root.tk.call("clock", "milliseconds"))
    dt_s = (now_ms - update_targets_from_sliders.last_ms) / 1000.0
    update_targets_from_sliders.last_ms = now_ms

    motor_targets_changed = False
    display_changed = False
    slider_motor_groups = (
        (0, METACARPAL_MOTORS),
        (1, CARRIAGE_MOTORS),
    )

    for slider_index, motor_indices in slider_motor_groups:
        rate_counts_per_s = slider_vars[slider_index].get()
        if abs(rate_counts_per_s) < 0.001:
            continue

        delta_counts = rate_counts_per_s * dt_s
        for motor_index in motor_indices:
            motor_position_targets[motor_index] += delta_counts
            motor_positions[motor_index] = int(round(motor_position_targets[motor_index]))
            entry_vars[motor_index].set(str(motor_positions[motor_index]))
        motor_targets_changed = True
        display_changed = True

    servo_rate_deg_per_s = slider_vars[2].get()
    if abs(servo_rate_deg_per_s) >= 0.001:
        delta_deg = servo_rate_deg_per_s * dt_s
        servo_position_targets[0] = clamp(
            servo_position_targets[0] + delta_deg, SERVO_MIN_DEG, SERVO_MAX_DEG
        )
        servo_position_targets[1] = clamp(
            servo_position_targets[1] - delta_deg, SERVO_MIN_DEG, SERVO_MAX_DEG
        )

        for servo_index in range(2):
            servo_positions[servo_index] = servo_position_targets[servo_index]
            servo_vars[servo_index].set(f"{servo_positions[servo_index]:.1f}")
        display_changed = True

    if motor_targets_changed:
        send_position_targets()

    if display_changed:
        update_display()

    root.after(SLIDER_UPDATE_MS, update_targets_from_sliders)


def send_servo(servo_index):
    try:
        target = float(servo_vars[servo_index].get())
    except ValueError:
        print(f"Servo {servo_index}: invalid input")
        return

    target = clamp(target, SERVO_MIN_DEG, SERVO_MAX_DEG)
    print(f"Servo {servo_index} set to {target}")
    servo_position_targets[servo_index] = target
    servo_positions[servo_index] = target
    servo_vars[servo_index].set(f"{target:.1f}")
    update_display()


def home_all():
    try:
        subprocess.run(
            ["cansend", CAN_INTERFACE, "120#01"],
            check=True,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        print("cansend is not installed or not on PATH")
        homing_label.config(text="cansend not found")
        return
    except subprocess.CalledProcessError as exc:
        error_text = exc.stderr.strip() or exc.stdout.strip() or str(exc)
        print(f"Failed to send homing CAN frame: {error_text}")
        homing_label.config(text="Homing send failed")
        return

    print(f"Homing CAN frame sent: {CAN_INTERFACE} 120#01")
    for i in range(6):
        motor_position_targets[i] = 0.0
        motor_positions[i] = 0
        entry_vars[i].set("0")
    homing_label.config(text="Homing command sent")
    update_display()


def next_config():
    global current_config
    current_config += 1
    if current_config > 4:
        current_config = 1
    apply_config(current_config)


def apply_config(config_number):
    preset = CONFIG_PRESETS[config_number]

    print(f"Changed to Config {config_number}: {preset['name']}")

    for i, angle in enumerate(preset["servo_angles"]):
        servo_position_targets[i] = angle
        servo_positions[i] = angle
        if i < len(servo_vars):
            servo_vars[i].set(f"{angle:.1f}")

    config_label.config(text=f"{config_number}: {preset['name']}")
    update_display()


def update_display():
    for i in range(6):
        pos_labels[i].config(text=str(motor_positions[i]))
        current_labels[i].config(text=f"{motor_currents[i]:.2f}")

    for i in range(2):
        servo_pos_labels[i].config(text=f"{servo_positions[i]:.1f}")


# ---- Main window ----
root = tk.Tk()
root.title("Rover Hand Control")
root.geometry("1120x620")

BG = root.cget("bg")
FG = "black"
VALUE_FG = "black"
PANEL_BG = BG
FONT = ("Arial", 10)
MONO = ("Courier New", 10)

# ---- Storage ----
entry_vars = []
pos_labels = []
current_labels = []

servo_vars = []
servo_pos_labels = []

left_panel = tk.Frame(root)
left_panel.grid(row=0, column=0, padx=(12, 24), pady=10, sticky="nw")

right_panel = tk.Frame(root)
right_panel.grid(row=0, column=1, padx=(0, 12), pady=10, sticky="nw")

# =========================
# Motors Section
# =========================
tk.Label(left_panel, text="Motors", font=("Arial", 12, "bold")).grid(
    row=0, column=0, columnspan=5, padx=8, pady=(10, 5), sticky="w"
)

motor_headers = ["Motor", "Target", "Send", "Pos (counts)", "Current (A)"]
for col, text in enumerate(motor_headers):
    tk.Label(left_panel, text=text, font=("Arial", 10, "bold"), justify="center").grid(
        row=1, column=col, padx=8, pady=5
    )

for i in range(6):
    row_index = i + 2

    tk.Label(left_panel, text=f"Motor {i}").grid(row=row_index, column=0, padx=8, pady=5)

    var = tk.StringVar(value="0")
    entry_vars.append(var)
    tk.Entry(left_panel, textvariable=var, width=10).grid(row=row_index, column=1, padx=8, pady=5)

    tk.Button(
        left_panel,
        text="Send",
        command=lambda idx=i: send_target(idx)
    ).grid(row=row_index, column=2, padx=8, pady=5)

    pos = tk.Label(left_panel, text="0")
    pos.grid(row=row_index, column=3, padx=8, pady=5)
    pos_labels.append(pos)

    current = tk.Label(left_panel, text="0.00")
    current.grid(row=row_index, column=4, padx=8, pady=5)
    current_labels.append(current)

# =========================
# Right-side Controls
# =========================
tk.Label(right_panel, text="System", font=("Arial", 12, "bold")).grid(
    row=0, column=0, columnspan=2, padx=20, pady=(10, 5), sticky="w"
)

system_frame = tk.Frame(right_panel)
system_frame.grid(row=1, column=0, padx=20, pady=10, sticky="w")

home_frame = tk.Frame(system_frame)
home_frame.grid(row=0, column=0, padx=(0, 20), sticky="nw")

config_frame = tk.Frame(system_frame)
config_frame.grid(row=0, column=1, sticky="nw")

tk.Button(home_frame, text="Home All", width=12, command=home_all).grid(
    row=0, column=0, sticky="w"
)

tk.Button(config_frame, text="Change Config", width=12, command=next_config).grid(
    row=0, column=0, sticky="w"
)

homing_label = tk.Label(home_frame, text="", font=("Arial", 10))
homing_label.grid(row=1, column=0, pady=5, sticky="w")

config_label = tk.Label(config_frame, text="", font=("Arial", 10))
config_label.grid(row=1, column=0, pady=5, sticky="w")

snap_var = tk.BooleanVar(value=True)
slider_vars = []
disp_vars = []

snap_frame = tk.Frame(right_panel, bg=BG)
snap_frame.grid(row=2, column=0, padx=20, pady=(10, 0), sticky="w")
tk.Checkbutton(
    snap_frame,
    text="Snap rate sliders to zero on release",
    variable=snap_var,
    bg=BG,
    fg=FG,
    selectcolor=PANEL_BG,
    activebackground=BG,
    activeforeground=FG,
    font=("Arial", 10),
).pack(side=tk.LEFT)

ctrl = tk.LabelFrame(right_panel, text="Drive Command", font=("Arial", 10, "bold"))
ctrl.grid(row=3, column=0, padx=20, pady=(6, 4), sticky="w")

SLIDER_DEFS = [
    ("Metacarpal (counts/s)", -2500, 2500),
    ("Carriage (counts/s)", -10000, 10000),
    ("Servos (deg/s)", -15, 15),
]

for row_idx, (label, lo, hi) in enumerate(SLIDER_DEFS):
    tk.Label(
        ctrl, text=label, bg=BG, fg=FG, font=FONT, width=18, anchor="e"
    ).grid(row=row_idx, column=0, padx=(4, 4), pady=5)

    var = tk.DoubleVar(value=0.0)
    slider_vars.append(var)

    sl = tk.Scale(
        ctrl,
        variable=var,
        from_=lo,
        to=hi,
        resolution=0.01,
        orient=tk.HORIZONTAL,
        length=260,
        bg=PANEL_BG,
        fg=FG,
        troughcolor="#b0b0b0",
        activebackground="#5599ff",
        highlightthickness=0,
        showvalue=False,
    )
    sl.grid(row=row_idx, column=1, padx=(2, 2), pady=5)

    def _on_release(event, sv=var):
        if snap_var.get():
            sv.set(0.0)

    sl.bind("<ButtonRelease-1>", _on_release)

    dv = tk.StringVar(value=" 0.00")
    disp_vars.append(dv)
    tk.Label(
        ctrl, textvariable=dv, bg=BG, fg=VALUE_FG, font=MONO, width=7
    ).grid(row=row_idx, column=2, padx=(4, 8))

    def _update_slider_display(*_, slider_var=var, display_var=dv):
        display_var.set(f"{slider_var.get(): .2f}")

    var.trace_add("write", _update_slider_display)
    _update_slider_display()

# =========================
# Servos Section
# =========================
servo_section_row = 10

tk.Label(left_panel, text="Servos", font=("Arial", 12, "bold")).grid(
    row=servo_section_row, column=0, columnspan=4, padx=8, pady=(20, 5), sticky="w"
)

servo_headers = ["Servo", "Target", "Send", "Pos (deg)"]
for col, text in enumerate(servo_headers):
    tk.Label(left_panel, text=text, font=("Arial", 10, "bold"), justify="center").grid(
        row=servo_section_row + 1, column=col, padx=8, pady=5
    )

for i in range(2):
    row_index = servo_section_row + 2 + i

    tk.Label(left_panel, text=f"Servo {i}").grid(row=row_index, column=0, padx=8, pady=5)

    var = tk.StringVar(value=f"{servo_positions[i]:.1f}")
    servo_vars.append(var)
    tk.Entry(left_panel, textvariable=var, width=10).grid(row=row_index, column=1, padx=8, pady=5)

    tk.Button(
        left_panel,
        text="Send",
        command=lambda idx=i: send_servo(idx)
    ).grid(row=row_index, column=2, padx=8, pady=5)

    pos = tk.Label(left_panel, text="90")
    pos.grid(row=row_index, column=3, padx=8, pady=5)
    servo_pos_labels.append(pos)

# ---- Initialize display ----
apply_config(current_config)
update_display()
update_targets_from_sliders.last_ms = int(root.tk.call("clock", "milliseconds"))
root.after(SLIDER_UPDATE_MS, update_targets_from_sliders)

# ---- Run ----
root.mainloop()
