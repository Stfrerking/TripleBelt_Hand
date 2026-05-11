import queue
import re
import subprocess
import threading
import tkinter as tk


CAN_INTERFACE = "can0"

# Must match src/CAN_testing.cpp and src/main.cpp.
CAN_ID_HOMING_CMD = 0x120
CAN_ID_SET_POS_01 = 0x210
CAN_ID_SET_POS_23 = 0x211
CAN_ID_SET_POS_45 = 0x212
CAN_ID_SET_SERVO_POS = 0x220
CAN_ID_ENCODER_BASE = 0x110
CAN_ID_ROM_BASE = 0x130
CAN_ID_CURRENT_BASE = 0x140

CAN_POSITION_IDS = (CAN_ID_SET_POS_01, CAN_ID_SET_POS_23, CAN_ID_SET_POS_45)
METACARPAL_MOTORS = (0, 2, 4)
CARRIAGE_MOTORS = (1, 3, 5)
SLIDER_UPDATE_MS = 50
CAN_POLL_MS = 50
CONTROLLER_HEARTBEAT_MS = 100
SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 270.0
UNKNOWN_ROM_TEXT = "--"


target_positions = [0, 0, 0, 0, 0, 0]
encoder_positions = [0, 0, 0, 0, 0, 0]
rom_low = [None, None, None, None, None, None]
rom_high = [None, None, None, None, None, None]
motor_currents_mA = [None, None, None, None, None, None]

servo_positions = [90.0, 90.0]
servo_targets = [90.0, 90.0]
current_config = 1

rx_queue = queue.Queue()
candump_process = None
stop_rx_thread = threading.Event()

CONFIG_PRESETS = {
    1: {"name": "One-sided Grip", "servo_angles": [5.0, 210.0]},
    2: {"name": "Pinch Grip", "servo_angles": [102.0, 98.0]},
    3: {"name": "Claw Grip", "servo_angles": [140.0, 45.0]},
    4: {"name": "Coffee Cup Grip", "servo_angles": [210.0, 5.0]},
}


def pack_i32_le(value):
    return int(round(value)).to_bytes(4, byteorder="little", signed=True)


def unpack_i32_le(payload):
    return int.from_bytes(payload, byteorder="little", signed=True)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def set_status(text):
    status_var.set(text)
    print(text)


def send_can_frame(can_id, payload, description, report=True):
    frame = f"{can_id:03X}#{payload.hex().upper()}"

    try:
        subprocess.run(
            ["cansend", CAN_INTERFACE, frame],
            check=True,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        set_status("cansend is not installed or not on PATH")
        return False
    except subprocess.CalledProcessError as exc:
        error_text = exc.stderr.strip() or exc.stdout.strip() or str(exc)
        set_status(f"{description} failed: {error_text}")
        return False

    if report:
        set_status(f"{description}: {CAN_INTERFACE} {frame}")
    return True


def send_position_pair(frame_index, report=True):
    first_motor = frame_index * 2
    payload = (
        pack_i32_le(target_positions[first_motor])
        + pack_i32_le(target_positions[first_motor + 1])
    )
    return send_can_frame(
        CAN_POSITION_IDS[frame_index],
        payload,
        f"Sent M{first_motor}/M{first_motor + 1} targets",
        report=report,
    )


def send_position_targets(report=True):
    ok = True
    for frame_index in range(len(CAN_POSITION_IDS)):
        ok = send_position_pair(frame_index, report=False) and ok
    if ok and report:
        set_status(f"All motor targets sent: {target_positions}")
    return ok


def send_servo_positions(report=True):
    payload = pack_i32_le(servo_targets[0]) + pack_i32_le(servo_targets[1])
    return send_can_frame(CAN_ID_SET_SERVO_POS, payload, "Sent servo targets", report=report)


def send_target(motor_index):
    try:
        target = int(entry_vars[motor_index].get())
    except ValueError:
        set_status(f"Motor {motor_index}: invalid target")
        return

    target_positions[motor_index] = target
    send_position_pair(motor_index // 2)
    update_display()


def send_all_targets():
    for motor_index, var in enumerate(entry_vars):
        try:
            target_positions[motor_index] = int(var.get())
        except ValueError:
            set_status(f"Motor {motor_index}: invalid target")
            return

    for servo_index, var in enumerate(servo_vars):
        try:
            target = float(var.get())
        except ValueError:
            set_status(f"Servo {servo_index}: invalid target")
            return
        servo_targets[servo_index] = clamp(target, SERVO_MIN_DEG, SERVO_MAX_DEG)
        servo_positions[servo_index] = servo_targets[servo_index]
        var.set(f"{servo_targets[servo_index]:.1f}")

    send_position_targets()
    send_servo_positions()
    update_display()


def sync_targets_to_encoders(send=False):
    for motor_index, encoder_position in enumerate(encoder_positions):
        target_positions[motor_index] = encoder_position
        entry_vars[motor_index].set(str(encoder_position))

    update_display()
    if send:
        send_position_targets()
    else:
        set_status(f"Motor targets synced to encoders: {target_positions}")


def sync_target_to_rom_midpoint(motor_index):
    if rom_low[motor_index] is None or rom_high[motor_index] is None:
        return

    midpoint_sum = rom_low[motor_index] + rom_high[motor_index]
    midpoint = midpoint_sum // 2 if midpoint_sum >= 0 else -((-midpoint_sum) // 2)
    target_positions[motor_index] = midpoint
    entry_vars[motor_index].set(str(midpoint))
    send_position_pair(motor_index // 2)
    homing_label.config(text=f"Motor {motor_index} midpoint target: {midpoint}")
    update_display()


def update_targets_from_sliders():
    now_ms = int(root.tk.call("clock", "milliseconds"))
    dt_s = (now_ms - update_targets_from_sliders.last_ms) / 1000.0
    update_targets_from_sliders.last_ms = now_ms

    motor_targets_changed = False
    servo_targets_changed = False
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
            target_positions[motor_index] = int(round(target_positions[motor_index] + delta_counts))
            entry_vars[motor_index].set(str(target_positions[motor_index]))
        motor_targets_changed = True

    servo_rate_deg_per_s = slider_vars[2].get()
    if abs(servo_rate_deg_per_s) >= 0.001:
        delta_deg = servo_rate_deg_per_s * dt_s
        servo_targets[0] = clamp(servo_targets[0] + delta_deg, SERVO_MIN_DEG, SERVO_MAX_DEG)
        servo_targets[1] = clamp(servo_targets[1] - delta_deg, SERVO_MIN_DEG, SERVO_MAX_DEG)

        for servo_index in range(2):
            servo_positions[servo_index] = servo_targets[servo_index]
            servo_vars[servo_index].set(f"{servo_targets[servo_index]:.1f}")
        servo_targets_changed = True

    if motor_targets_changed:
        send_position_targets(report=False)

    if servo_targets_changed:
        send_servo_positions(report=False)

    if motor_targets_changed or servo_targets_changed:
        update_display()

    root.after(SLIDER_UPDATE_MS, update_targets_from_sliders)


def update_controller_heartbeat():
    if controller_heartbeat_var.get():
        send_position_targets(report=False)
    root.after(CONTROLLER_HEARTBEAT_MS, update_controller_heartbeat)


def send_servo(servo_index):
    try:
        target = float(servo_vars[servo_index].get())
    except ValueError:
        set_status(f"Servo {servo_index}: invalid target")
        return

    target = clamp(target, SERVO_MIN_DEG, SERVO_MAX_DEG)
    servo_targets[servo_index] = target
    servo_positions[servo_index] = target
    servo_vars[servo_index].set(f"{target:.1f}")
    send_servo_positions()
    update_display()


def home_all():
    if send_can_frame(CAN_ID_HOMING_CMD, bytes([1]), "Homing command sent"):
        homing_label.config(text="Homing command sent")


def next_config():
    global current_config
    current_config += 1
    if current_config > len(CONFIG_PRESETS):
        current_config = 1
    apply_config(current_config, send=True)


def apply_config(config_number, send=False):
    preset = CONFIG_PRESETS[config_number]

    for i, angle in enumerate(preset["servo_angles"]):
        servo_targets[i] = angle
        servo_positions[i] = angle
        if i < len(servo_vars):
            servo_vars[i].set(f"{angle:.1f}")

    config_label.config(text=f"{config_number}: {preset['name']}")
    if send:
        send_servo_positions()
    update_display()


def parse_candump_line(line):
    log_match = re.search(r"\s([0-9A-Fa-f]{1,8})#([0-9A-Fa-f]*)", line)
    if log_match:
        can_id = int(log_match.group(1), 16)
        data_hex = log_match.group(2)
        return can_id, bytes.fromhex(data_hex)

    spaced_match = re.search(
        r"\s([0-9A-Fa-f]{1,8})\s+\[(\d+)\]\s+((?:[0-9A-Fa-f]{2}\s*)*)",
        line,
    )
    if spaced_match:
        can_id = int(spaced_match.group(1), 16)
        length = int(spaced_match.group(2))
        data_hex = "".join(spaced_match.group(3).split())
        payload = bytes.fromhex(data_hex[: length * 2])
        return can_id, payload

    return None, None


def candump_reader():
    global candump_process

    try:
        candump_process = subprocess.Popen(
            ["candump", "-L", CAN_INTERFACE],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except FileNotFoundError:
        rx_queue.put(("status", "candump is not installed or not on PATH"))
        return

    rx_queue.put(("status", f"Listening on {CAN_INTERFACE}"))

    while not stop_rx_thread.is_set() and candump_process.poll() is None:
        line = candump_process.stdout.readline()
        if not line:
            break
        can_id, payload = parse_candump_line(line)
        if can_id is not None:
            rx_queue.put(("frame", can_id, payload))

    if candump_process and candump_process.poll() not in (None, 0):
        err = candump_process.stderr.read().strip()
        rx_queue.put(("status", err or "candump stopped"))


def start_can_listener():
    thread = threading.Thread(target=candump_reader, daemon=True)
    thread.start()


def handle_received_frame(can_id, payload):
    if CAN_ID_ENCODER_BASE <= can_id < CAN_ID_ENCODER_BASE + 6 and len(payload) == 4:
        motor_index = can_id - CAN_ID_ENCODER_BASE
        encoder_positions[motor_index] = unpack_i32_le(payload)
        update_display()
        return

    if CAN_ID_ROM_BASE <= can_id < CAN_ID_ROM_BASE + 6 and len(payload) == 8:
        motor_index = can_id - CAN_ID_ROM_BASE
        rom_low[motor_index] = unpack_i32_le(payload[:4])
        rom_high[motor_index] = unpack_i32_le(payload[4:])
        sync_target_to_rom_midpoint(motor_index)
        return

    if CAN_ID_CURRENT_BASE <= can_id < CAN_ID_CURRENT_BASE + 6 and len(payload) == 4:
        motor_index = can_id - CAN_ID_CURRENT_BASE
        motor_currents_mA[motor_index] = unpack_i32_le(payload)
        update_display()


def poll_can_queue():
    while True:
        try:
            item = rx_queue.get_nowait()
        except queue.Empty:
            break

        if item[0] == "status":
            set_status(item[1])
        elif item[0] == "frame":
            handle_received_frame(item[1], item[2])

    root.after(CAN_POLL_MS, poll_can_queue)


def update_display():
    for i in range(6):
        target_labels[i].config(text=str(target_positions[i]))
        pos_labels[i].config(text=str(encoder_positions[i]))
        current_text = "--" if motor_currents_mA[i] is None else f"{motor_currents_mA[i] / 1000.0:.3f} A"
        current_labels[i].config(text=current_text)
        rom_low_labels[i].config(text=UNKNOWN_ROM_TEXT if rom_low[i] is None else str(rom_low[i]))
        rom_high_labels[i].config(text=UNKNOWN_ROM_TEXT if rom_high[i] is None else str(rom_high[i]))

    for i in range(2):
        servo_pos_labels[i].config(text=f"{servo_positions[i]:.1f}")


def on_close():
    stop_rx_thread.set()
    if candump_process and candump_process.poll() is None:
        candump_process.terminate()
    root.destroy()


root = tk.Tk()
root.title("Rover Hand Control")
root.geometry("1240x650")
root.protocol("WM_DELETE_WINDOW", on_close)

BG = root.cget("bg")
FG = "black"
VALUE_FG = "black"
PANEL_BG = BG
FONT = ("Arial", 10)
MONO = ("Courier New", 10)

entry_vars = []
target_labels = []
pos_labels = []
rom_low_labels = []
rom_high_labels = []
current_labels = []

servo_vars = []
servo_pos_labels = []

status_var = tk.StringVar(value=f"Ready on {CAN_INTERFACE}")

left_panel = tk.Frame(root)
left_panel.grid(row=0, column=0, padx=(12, 24), pady=10, sticky="nw")

right_panel = tk.Frame(root)
right_panel.grid(row=0, column=1, padx=(0, 12), pady=10, sticky="nw")

tk.Label(left_panel, text="Motors", font=("Arial", 12, "bold")).grid(
    row=0, column=0, columnspan=7, padx=8, pady=(10, 5), sticky="w"
)

motor_headers = ["Motor", "Target", "Send Pair", "Last Target", "Encoder", "Current", "ROM Low", "ROM High"]
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
        command=lambda idx=i: send_target(idx),
    ).grid(row=row_index, column=2, padx=8, pady=5)

    target = tk.Label(left_panel, text="0", width=10, anchor="e", font=MONO)
    target.grid(row=row_index, column=3, padx=8, pady=5)
    target_labels.append(target)

    pos = tk.Label(left_panel, text="0", width=10, anchor="e", font=MONO)
    pos.grid(row=row_index, column=4, padx=8, pady=5)
    pos_labels.append(pos)

    current = tk.Label(left_panel, text="--", width=10, anchor="e", font=MONO)
    current.grid(row=row_index, column=5, padx=8, pady=5)
    current_labels.append(current)

    lo = tk.Label(left_panel, text=UNKNOWN_ROM_TEXT, width=10, anchor="e", font=MONO)
    lo.grid(row=row_index, column=6, padx=8, pady=5)
    rom_low_labels.append(lo)

    hi = tk.Label(left_panel, text=UNKNOWN_ROM_TEXT, width=10, anchor="e", font=MONO)
    hi.grid(row=row_index, column=7, padx=8, pady=5)
    rom_high_labels.append(hi)

servo_section_row = 10

tk.Label(left_panel, text="Servos", font=("Arial", 12, "bold")).grid(
    row=servo_section_row, column=0, columnspan=4, padx=8, pady=(20, 5), sticky="w"
)

servo_headers = ["Servo", "Target", "Send Both", "Last Target"]
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
        command=lambda idx=i: send_servo(idx),
    ).grid(row=row_index, column=2, padx=8, pady=5)

    pos = tk.Label(left_panel, text="90.0", width=10, anchor="e", font=MONO)
    pos.grid(row=row_index, column=3, padx=8, pady=5)
    servo_pos_labels.append(pos)

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

tk.Button(system_frame, text="Send All", width=12, command=send_all_targets).grid(
    row=0, column=2, padx=(20, 0), sticky="nw"
)

tk.Button(system_frame, text="Sync Enc", width=12, command=sync_targets_to_encoders).grid(
    row=0, column=3, padx=(8, 0), sticky="nw"
)

homing_label = tk.Label(home_frame, text="", font=("Arial", 10))
homing_label.grid(row=1, column=0, pady=5, sticky="w")

config_label = tk.Label(config_frame, text="", font=("Arial", 10))
config_label.grid(row=1, column=0, pady=5, sticky="w")

status_label = tk.Label(right_panel, textvariable=status_var, font=("Arial", 10), wraplength=450, justify="left")
status_label.grid(row=2, column=0, padx=20, pady=(2, 8), sticky="w")

snap_var = tk.BooleanVar(value=True)
controller_heartbeat_var = tk.BooleanVar(value=True)
slider_vars = []
disp_vars = []

snap_frame = tk.Frame(right_panel, bg=BG)
snap_frame.grid(row=3, column=0, padx=20, pady=(10, 0), sticky="w")
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
tk.Checkbutton(
    snap_frame,
    text="Controller heartbeat",
    variable=controller_heartbeat_var,
    bg=BG,
    fg=FG,
    selectcolor=PANEL_BG,
    activebackground=BG,
    activeforeground=FG,
    font=("Arial", 10),
).pack(side=tk.LEFT, padx=(16, 0))

ctrl = tk.LabelFrame(right_panel, text="Drive Command", font=("Arial", 10, "bold"))
ctrl.grid(row=4, column=0, padx=20, pady=(6, 4), sticky="w")

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
        ctrl, textvariable=dv, bg=BG, fg=VALUE_FG, font=MONO, width=8
    ).grid(row=row_idx, column=2, padx=(4, 8))

    def _update_slider_display(*_, slider_var=var, display_var=dv):
        display_var.set(f"{slider_var.get(): .2f}")

    var.trace_add("write", _update_slider_display)
    _update_slider_display()

message_frame = tk.LabelFrame(right_panel, text="CAN Message Map", font=("Arial", 10, "bold"))
message_frame.grid(row=5, column=0, padx=20, pady=(18, 4), sticky="w")

message_rows = [
    ("0x120", "TX", "Homing command, len 1, byte 0 = 1"),
    ("0x210", "TX", "Motor 0/1 absolute targets, int32 LE"),
    ("0x211", "TX", "Motor 2/3 absolute targets, int32 LE"),
    ("0x212", "TX", "Motor 4/5 absolute targets, int32 LE"),
    ("0x220", "TX", "Servo 0/1 degree targets, int32 LE"),
    ("0x110-0x115", "RX", "Encoder positions, int32 LE"),
    ("0x130-0x135", "RX", "ROM low/high limits, int32 LE"),
    ("0x140-0x145", "RX", "Motor current, mA, int32 LE"),
]

for row_idx, (can_id, direction, description) in enumerate(message_rows):
    tk.Label(message_frame, text=can_id, width=10, anchor="w", font=MONO).grid(
        row=row_idx, column=0, padx=(8, 4), pady=2, sticky="w"
    )
    tk.Label(message_frame, text=direction, width=4, anchor="w", font=MONO).grid(
        row=row_idx, column=1, padx=4, pady=2, sticky="w"
    )
    tk.Label(message_frame, text=description, anchor="w").grid(
        row=row_idx, column=2, padx=(4, 8), pady=2, sticky="w"
    )

apply_config(current_config, send=False)
update_display()
start_can_listener()
poll_can_queue()
update_targets_from_sliders.last_ms = int(root.tk.call("clock", "milliseconds"))
root.after(SLIDER_UPDATE_MS, update_targets_from_sliders)
root.after(CONTROLLER_HEARTBEAT_MS, update_controller_heartbeat)

root.mainloop()
