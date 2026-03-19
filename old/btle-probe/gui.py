import tkinter as tk
from tkinter import scrolledtext, ttk
import subprocess
import threading
import os
import struct
from collections import deque

BINARY_PATH = "./probe"
AVERAGE_WINDOW = 16  # Number of samples to average over

def decode_sticks(data):
    x = data[0] | ((data[1] & 0x0F) << 8)
    y = (data[1] >> 4) | (data[2] << 4)
    return x, y

def parse_imu_frame(data, offset):
    if offset + 18 > len(data):
        return None
    ts           = struct.unpack_from('<I', data, offset)[0]
    temp_raw     = struct.unpack_from('<h', data, offset + 0x04)[0]
    ax, ay, az   = struct.unpack_from('<3h', data, offset + 0x06)
    gx, gy, gz   = struct.unpack_from('<3h', data, offset + 0x0C)
    return {
        "timestamp_us": ts,
        "temperature":  25.0 + temp_raw / 127.0,
        "accel":  (ax, ay, az),
        "gyro":   (gx, gy, gz),
    }


# Scale factors — adjust these once the layout is confirmed
# int16 accel: ±8g range → 1g = 4096 LSB  → divide by 4096 for g, multiply by 9.81 for m/s²
# int16 gyro:  ±2000°/s  → 1°/s ≈ 16.4 LSB → divide by 16.4 for °/s
ACCEL_SCALE = 1.0 / 4096.0   # → g
GYRO_SCALE  = 1.0 / 16.4     # → °/s

def parse_bt_imu_frames(packet):
    """
    Decode the theoretical 3-frame BT IMU payload based on orientation analysis.

    Hypothesis (40 bytes starting at 0x46):
      [0x46–0x49]  uint32  batch timestamp (µs)
      [0x4A–0x55]  6×int16 frame 0: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
      [0x56–0x61]  6×int16 frame 1: same layout
      [0x62–0x6D]  6×int16 frame 2: same layout  (0x6D is just past packet end at 112)

    Note: frame 2 only has bytes up to 0x69 in a 112-byte packet (7 bytes short),
    so we only decode frame 2 if enough bytes exist.

    Returns list of frame dicts, or empty list if packet too short.
    """
    if len(packet) < 0x56:
        return []

    result = {}

    # Outer batch timestamp at 0x46
    result["bt_timestamp"] = struct.unpack_from('<I', packet, 0x46)[0]

    frames = []
    FRAME_START = 0x4A
    FRAME_SIZE  = 12  # 6 × int16

    for i in range(3):
        off = FRAME_START + i * FRAME_SIZE
        if off + FRAME_SIZE > len(packet):
            break
        ax, ay, az, gx, gy, gz = struct.unpack_from('<6h', packet, off)
        frames.append({
            "accel_raw": (ax, ay, az),
            "gyro_raw":  (gx, gy, gz),
            "accel_g":   (ax * ACCEL_SCALE, ay * ACCEL_SCALE, az * ACCEL_SCALE),
            "gyro_dps":  (gx * GYRO_SCALE,  gy * GYRO_SCALE,  gz * GYRO_SCALE),
        })

    result["frames"] = frames
    return result


def parse_switch2_report(packet):
    length = len(packet)

    if length == 64 and packet[0] == 0x09:
        buttons  = int.from_bytes(packet[3:6], 'little')
        l_stick  = decode_sticks(packet[6:9])
        r_stick  = decode_sticks(packet[9:12])
        imu_len  = packet[0x10]
        result = {
            "type":    "USB_PRO",
            "seq":     packet[1],
            "buttons": bin(buttons),
            "sticks":  {"left": l_stick, "right": r_stick},
        }
        if imu_len >= 18:
            frame = parse_imu_frame(packet, 0x11)
            if frame:
                result["imu"] = frame
            if imu_len >= 24:
                mx, my, mz = struct.unpack_from('<3h', packet, 0x11 + 18)
                result["mag"] = (mx, my, mz)
        return result

    elif length == 112:
        buttons  = int.from_bytes(packet[2:5], 'little')
        l_stick  = decode_sticks(packet[5:8])
        r_stick  = decode_sticks(packet[8:11])
        result = {
            "type":    "BTLE_PRO",
            "seq":     packet[0],
            "buttons": bin(buttons),
            "sticks":  {"left": l_stick, "right": r_stick},
        }
        ext_imu_len = packet[0x40]
        if ext_imu_len > 0:
            outer_clock = struct.unpack_from('<I', packet, 0x41)[0]
            result["imu_clock"] = outer_clock
            frame0 = parse_imu_frame(packet, 0x45)
            frame1 = parse_imu_frame(packet, 0x45 + 18)
            if frame0:
                result["imu"]  = frame0
                result["imu0"] = frame0
            if frame1:
                result["imu1"] = frame1
            # Theoretical BT 3-frame decode
            bt = parse_bt_imu_frames(packet)
            if bt.get("frames"):
                result["bt_imu"] = bt
        return result

    elif length == 63 or (length >= 62 and packet[1] == 0x20):
        buttons  = int.from_bytes(packet[0x04:0x08], 'little')
        l_stick  = decode_sticks(packet[0x0A:0x0D])
        r_stick  = decode_sticks(packet[0x0D:0x10])
        mouse_x, mouse_y = struct.unpack_from('<hh', packet, 0x10)
        mx, my, mz = struct.unpack_from('<3h', packet, 0x19)
        result = {
            "type":    "BTLE_JOYCON",
            "seq":     int.from_bytes(packet[0:4], 'little'),
            "buttons": bin(buttons),
            "sticks":  {"left": l_stick, "right": r_stick},
            "mouse":   (mouse_x, mouse_y),
            "mag":     (mx, my, mz),
        }
        if length >= 0x2A + 18:
            frame = parse_imu_frame(packet, 0x2A)
            if frame:
                result["imu"] = frame
        return result

    return {"error": "Unknown packet format", "length": length}


class ByteTracker:
    """
    Tracks rolling averages and change direction for each byte offset.
    Maintains a history window and compares current average to previous average.
    """
    def __init__(self, window=AVERAGE_WINDOW, stable_threshold=2.0, drift_threshold=10.0):
        # Per-offset rolling history of raw byte values
        self.histories: dict[int, deque] = {}
        # Previous average snapshot (for delta computation)
        self.prev_averages: dict[int, float] = {}
        # Current running averages
        self.curr_averages: dict[int, float] = {}
        # Per-offset min/max seen (for range display)
        self.mins: dict[int, float] = {}
        self.maxs: dict[int, float] = {}
        # How many total samples seen
        self.sample_count = 0

        self.window = window
        # Delta smaller than this = "stable"
        self.stable_threshold = stable_threshold
        # Delta larger than this = "drifting"
        self.drift_threshold = drift_threshold

    def update(self, packet: bytes):
        self.sample_count += 1
        # Save old averages before updating
        self.prev_averages = dict(self.curr_averages)

        for i, byte_val in enumerate(packet):
            if i not in self.histories:
                self.histories[i] = deque(maxlen=self.window)
                self.mins[i] = byte_val
                self.maxs[i] = byte_val

            self.histories[i].append(byte_val)
            self.mins[i] = min(self.mins[i], byte_val)
            self.maxs[i] = max(self.maxs[i], byte_val)
            self.curr_averages[i] = sum(self.histories[i]) / len(self.histories[i])

    def get_delta(self, offset: int) -> float:
        """Returns (current_avg - prev_avg). Zero if not enough history."""
        if offset not in self.curr_averages or offset not in self.prev_averages:
            return 0.0
        return self.curr_averages[offset] - self.prev_averages[offset]

    def get_trend(self, offset: int):
        """
        Returns a tuple: (symbol, color, magnitude_str)
          symbol: ▲ ▼ ~ (rising, falling, stable)
          color:  tkinter tag name
          magnitude: formatted string
        """
        if self.sample_count < 3:
            return ("?", "tag_neutral", "")

        delta = self.get_delta(offset)
        avg   = self.curr_averages.get(offset, 0)
        rng   = self.maxs.get(offset, 0) - self.mins.get(offset, 0)
        mag   = f"{delta:+.1f}"

        if abs(delta) <= self.stable_threshold:
            if rng <= self.stable_threshold * 2:
                return ("~", "tag_stable", mag)
            else:
                return ("~", "tag_noisy", mag)  # stable now but has varied
        elif delta > 0:
            if delta >= self.drift_threshold:
                return ("▲▲", "tag_rising_fast", mag)
            return ("▲", "tag_rising", mag)
        else:
            if delta <= -self.drift_threshold:
                return ("▼▼", "tag_falling_fast", mag)
            return ("▼", "tag_falling", mag)

    def get_avg(self, offset: int) -> float:
        return self.curr_averages.get(offset, 0.0)

    def get_range(self, offset: int) -> float:
        return self.maxs.get(offset, 0) - self.mins.get(offset, 0)

    def reset(self):
        self.histories.clear()
        self.prev_averages.clear()
        self.curr_averages.clear()
        self.mins.clear()
        self.maxs.clear()
        self.sample_count = 0


class ControllerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Switch 2 Pro Controller Engineering Tool")
        self.root.geometry("1400x900")

        self.tracker = ByteTracker()

        main_pane = tk.PanedWindow(root, orient=tk.VERTICAL)
        main_pane.pack(fill=tk.BOTH, expand=True)

        self.tabs = ttk.Notebook(root)
        main_pane.add(self.tabs, height=180)
        self.setup_tabs()

        log_pane = tk.PanedWindow(root, orient=tk.HORIZONTAL)
        main_pane.add(log_pane, height=500)

        self.hid_log = self.add_log(log_pane, "HID DATA (000E)", "#1e1e1e", "#569cd6")
        self.ack_log = self.add_log(log_pane, "COMMAND ACKS (001A)", "#1e1e1e", "#ce9178")
        self.sys_log = self.add_log(log_pane, "SYSTEM / STATUS", "#1e1e1e", "#d4d4d4")

        entry_frame = tk.Frame(root)
        main_pane.add(entry_frame, height=50)
        self.cmd_entry = tk.Entry(entry_frame, font=("Courier", 11))
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        self.cmd_entry.bind("<Return>", lambda e: self.send_manual())
        tk.Button(entry_frame, text="Send Raw", command=self.send_manual).pack(side=tk.RIGHT, padx=10)

        self.proc = subprocess.Popen([BINARY_PATH], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=self.reader, daemon=True).start()

    def add_log(self, parent, title, bg, fg):
        frame = tk.LabelFrame(parent, text=title)
        log = scrolledtext.ScrolledText(frame, bg=bg, fg=fg, font=("Courier", 9), state='disabled')
        log.pack(fill=tk.BOTH, expand=True)
        parent.add(frame)
        return log

    def write_log(self, widget, text):
        widget.configure(state='normal')
        widget.insert(tk.END, text + "\n")
        widget.see(tk.END)
        widget.configure(state='disabled')

    def send(self, hex_data):
        if self.proc.poll() is None:
            self.proc.stdin.write(hex_data + "\n")
            self.proc.stdin.flush()
            self.write_log(self.sys_log, f">> SENT: {hex_data}")

    def send_manual(self):
        val = self.cmd_entry.get()
        if val: self.send(val); self.cmd_entry.delete(0, tk.END)

    def reader(self):
        for line in iter(self.proc.stdout.readline, ''):
            l = line.strip()
            if l.startswith("RECV "):
                parts = l.split(" ")
                if len(parts) >= 3:
                    handle, raw_hex = parts[1], parts[2]
                    try:
                        packet_bytes = bytes.fromhex(raw_hex)
                    except ValueError:
                        continue

                    self.root.after(0, self.run_imu_debug, packet_bytes)

                    if handle in ("000E", "002E") or (len(packet_bytes) == 64 and packet_bytes[0] == 0x09):
                        decoded = parse_switch2_report(packet_bytes)
                        if "error" not in decoded:
                            self.root.after(0, self.write_log, self.hid_log, self.format_decoded_data(decoded))
                            self.root.after(0, self.update_live_ui, decoded)
                        else:
                            self.root.after(0, self.write_log, self.hid_log, f"RAW: {raw_hex}")
                    elif handle == "001A":
                        self.root.after(0, self.write_log, self.ack_log, f"ACK: {raw_hex}")
                    else:
                        self.root.after(0, self.write_log, self.sys_log, f"[{handle}] {raw_hex}")

                    # Always attempt BT IMU decode on any 112-byte packet
                    # regardless of handle or report ID — the IMU payload
                    # at 0x41 length + frames at 0x4A is consistent across IDs.
                    if len(packet_bytes) == 112 and len(packet_bytes) > 0x42 and packet_bytes[0x41] == 0x28:
                        bt = parse_bt_imu_frames(packet_bytes)
                        if bt.get("frames"):
                            synthetic = {
                                "type":    f"BT[{handle}]",
                                "seq":     packet_bytes[0x42],
                                "buttons": bin(0),
                                "sticks":  {},
                                "bt_imu":  bt,
                            }
                            self.root.after(0, self.update_live_ui, synthetic)
            else:
                self.root.after(0, self.write_log, self.sys_log, l)

    def format_decoded_data(self, d):
        out = f"[{d['type']}][Seq:{d['seq']:02X}] "
        if "sticks" in d:
            s = d["sticks"]
            if "left" in s:  out += f"L:({s['left'][0]:04},{s['left'][1]:04}) "
            if "right" in s: out += f"R:({s['right'][0]:04},{s['right'][1]:04}) "
        if "mouse" in d:
            out += f"M:({d['mouse'][0]},{d['mouse'][1]}) "
        out += f"BTN: {d['buttons']}"
        return out

    def build_packet(self, cid, sub, payload=[]):
        header = [cid, 0x91, 0x01, sub, 0x00, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF, 0x00]
        return " ".join(f"{b:02X}" for b in (header + payload))

    def update_live_ui(self, d):
        self.live_vars["type"].set(d.get("type", "UNK"))
        self.live_vars["seq"].set(f"{d.get('seq', 0):02X}")
        self.live_vars["btns"].set(d.get("buttons", "0"))

        sticks = d.get("sticks", {})
        if "left" in sticks:
            self.live_vars["lx"].set(f"X: {sticks['left'][0]}")
            self.live_vars["ly"].set(f"Y: {sticks['left'][1]}")
        if "right" in sticks:
            self.live_vars["rx"].set(f"X: {sticks['right'][0]}")
            self.live_vars["ry"].set(f"Y: {sticks['right'][1]}")

        # Legacy single-frame IMU (USB / old BT path)
        imu = d.get("imu")
        if isinstance(imu, dict):
            ax, ay, az = imu["accel"]
            gx, gy, gz = imu["gyro"]
            self.live_vars["accel"].set(f"X: {ax}  Y: {ay}  Z: {az}")
            self.live_vars["gyro"].set(f"X: {gx}  Y: {gy}  Z: {gz}")
            self.live_vars["temp"].set(f"{imu['temperature']:.1f} °C")
        mag = d.get("mag")
        if mag:
            mx, my, mz = mag
            self.live_vars["mag"].set(f"X: {mx}  Y: {my}  Z: {mz}")

        # Theoretical BT 3-frame IMU decode
        bt = d.get("bt_imu")
        if bt:
            ts = bt.get("bt_timestamp", 0)
            self.live_vars["bt_ts"].set(f"{ts}  ({ts/1e6:.3f} s)")
            frames = bt["frames"]
            for i, fv in enumerate(self.bt_frame_vars):
                if i < len(frames):
                    f = frames[i]
                    ax, ay, az = f["accel_raw"]
                    gx, gy, gz = f["gyro_raw"]
                    agx, agy, agz = f["accel_g"]
                    gdx, gdy, gdz = f["gyro_dps"]
                    fv["raw_a"].set( f"X:{ax:+6d}  Y:{ay:+6d}  Z:{az:+6d}")
                    fv["raw_g"].set( f"X:{gx:+6d}  Y:{gy:+6d}  Z:{gz:+6d}")
                    fv["phys_a"].set(f"X:{agx:+6.3f}g  Y:{agy:+6.3f}g  Z:{agz:+6.3f}g")
                    fv["phys_g"].set(f"X:{gdx:+7.2f}°/s  Y:{gdy:+7.2f}°/s  Z:{gdz:+7.2f}°/s")
                else:
                    for v in fv.values():
                        v.set("--")

    def setup_tabs(self):
        # --- Features Tab ---
        f_tab = ttk.Frame(self.tabs)
        self.tabs.add(f_tab, text="Features")
        self.f_vars = {
            0x01: tk.BooleanVar(), 0x02: tk.BooleanVar(), 0x04: tk.BooleanVar(),
            0x08: tk.BooleanVar(), 0x10: tk.BooleanVar(), 0x20: tk.BooleanVar(),
            0x40: tk.BooleanVar(), 0x80: tk.BooleanVar()
        }
        lbls = ["Button", "Stick", "Motion", "Unk08", "Mouse", "Current", "Unk40", "Mag"]
        for i, (val, var) in enumerate(self.f_vars.items()):
            tk.Checkbutton(f_tab, text=lbls[i], variable=var).grid(row=i//4, column=i%4, sticky='w', padx=5)

        def feat_cmd(sub):
            mask = sum(k for k, v in self.f_vars.items() if v.get())
            self.send(self.build_packet(0x0C, sub, [mask, 0, 0, 0]))

        btn_frame = tk.Frame(f_tab)
        btn_frame.grid(row=2, column=0, columnspan=4, pady=10)
        tk.Button(btn_frame, text="INIT",    command=lambda: feat_cmd(0x02)).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="ENABLE",  command=lambda: feat_cmd(0x04)).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="DISABLE", command=lambda: feat_cmd(0x05)).pack(side=tk.LEFT, padx=5)

        # --- Haptics Tab ---
        h_tab = ttk.Frame(self.tabs)
        self.tabs.add(h_tab, text="Haptics")
        for i, (n, v) in enumerate([("Buzz",1),("Find",2),("Connect",3),("Pairing",4),("Ding",7)]):
            tk.Button(h_tab, text=n, command=lambda val=v: self.send(self.build_packet(0x0A, 0x02, [val,0,0,0]))).grid(row=0, column=i, padx=5)

        # --- SPI Flash Tab ---
        s_tab = ttk.Frame(self.tabs)
        self.tabs.add(s_tab, text="SPI Flash")
        self.addr_in = tk.Entry(s_tab)
        self.addr_in.insert(0, "00300100")
        self.addr_in.grid(row=0, column=0)
        tk.Button(s_tab, text="Read 0x40", command=lambda: self.send(
            self.build_packet(0x02, 0x04, [0x40, 0x7E, 0, 0] + list(bytes.fromhex(self.addr_in.get())[::-1]))
        )).grid(row=0, column=1)

        # --- Live Data Tab ---
        self.live_tab = ttk.Frame(self.tabs)
        self.tabs.add(self.live_tab, text="Live Data")
        self.live_vars = {
            "type":  tk.StringVar(value="N/A"),
            "seq":   tk.StringVar(value="00"),
            "btns":  tk.StringVar(value="000000"),
            "lx":    tk.StringVar(value="0"),
            "ly":    tk.StringVar(value="0"),
            "rx":    tk.StringVar(value="0"),
            "ry":    tk.StringVar(value="0"),
            "accel": tk.StringVar(value="0  0  0"),
            "gyro":  tk.StringVar(value="0  0  0"),
            "temp":  tk.StringVar(value="--.- °C"),
            "mag":   tk.StringVar(value="--  --  --"),
            "bt_ts": tk.StringVar(value="--"),
        }

        row = 0

        # Header info row
        for col, (lbl, key) in enumerate([("Type:", "type"), ("Seq:", "seq"), ("Buttons:", "btns")]):
            tk.Label(self.live_tab, text=lbl, font=('Arial',9,'bold'), anchor='e').grid(
                row=row, column=col*2, sticky='e', padx=(8,2), pady=1)
            tk.Label(self.live_tab, textvariable=self.live_vars[key],
                     font=('Courier',9), anchor='w').grid(row=row, column=col*2+1, sticky='w')
        row += 1

        # Sticks
        tk.Label(self.live_tab, text="L-Stick:", font=('Arial',9,'bold')).grid(row=row, column=0, sticky='e', padx=(8,2))
        tk.Label(self.live_tab, textvariable=self.live_vars["lx"], font=('Courier',9)).grid(row=row, column=1, sticky='w')
        tk.Label(self.live_tab, textvariable=self.live_vars["ly"], font=('Courier',9)).grid(row=row, column=2, sticky='w')
        tk.Label(self.live_tab, text="R-Stick:", font=('Arial',9,'bold')).grid(row=row, column=3, sticky='e', padx=(8,2))
        tk.Label(self.live_tab, textvariable=self.live_vars["rx"], font=('Courier',9)).grid(row=row, column=4, sticky='w')
        tk.Label(self.live_tab, textvariable=self.live_vars["ry"], font=('Courier',9)).grid(row=row, column=5, sticky='w')
        row += 1

        # Legacy single-frame IMU (shown for USB packets)
        tk.Label(self.live_tab, text="── Legacy IMU (USB/old BT) ──",
                 font=('Arial',8), fg='#888').grid(row=row, column=0, columnspan=6, sticky='w', padx=8, pady=(6,1))
        row += 1
        for lbl, key in [("Accel:", "accel"), ("Gyro:", "gyro"), ("Temp:", "temp"), ("Mag:", "mag")]:
            tk.Label(self.live_tab, text=lbl, font=('Arial',9,'bold'), anchor='e').grid(
                row=row, column=0, sticky='e', padx=(8,2), pady=1)
            tk.Label(self.live_tab, textvariable=self.live_vars[key],
                     font=('Courier',9)).grid(row=row, column=1, columnspan=5, sticky='w')
            row += 1

        # BT Timestamp
        tk.Label(self.live_tab, text="── BT 3-Frame IMU (Theoretical) ──",
                 font=('Arial',8), fg='#5af').grid(row=row, column=0, columnspan=6, sticky='w', padx=8, pady=(8,1))
        row += 1
        tk.Label(self.live_tab, text="Batch TS:", font=('Arial',9,'bold'), anchor='e').grid(
            row=row, column=0, sticky='e', padx=(8,2))
        tk.Label(self.live_tab, textvariable=self.live_vars["bt_ts"],
                 font=('Courier',9), fg='#aaa').grid(row=row, column=1, columnspan=5, sticky='w')
        row += 1

        # BT 3 frames — each gets its own mini-panel
        FRAME_COLORS = ["#1a3a1a", "#1a1a3a", "#3a1a1a"]
        FRAME_FG     = ["#66ff88", "#6688ff", "#ff8866"]
        self.bt_frame_vars = []

        # Scale factor controls (live-adjustable)
        scale_frame = tk.Frame(self.live_tab)
        scale_frame.grid(row=row, column=0, columnspan=6, sticky='w', padx=8, pady=2)
        tk.Label(scale_frame, text="Accel scale (LSB/g):", font=('Arial',8)).pack(side=tk.LEFT)
        self.accel_scale_var = tk.DoubleVar(value=4096.0)
        tk.Spinbox(scale_frame, from_=256, to=32768, increment=256,
                   textvariable=self.accel_scale_var, width=7,
                   command=self._rescale).pack(side=tk.LEFT, padx=4)
        tk.Label(scale_frame, text="  Gyro scale (LSB/°/s):", font=('Arial',8)).pack(side=tk.LEFT)
        self.gyro_scale_var = tk.DoubleVar(value=16.4)
        tk.Spinbox(scale_frame, from_=1.0, to=200.0, increment=0.5,
                   textvariable=self.gyro_scale_var, width=7,
                   command=self._rescale).pack(side=tk.LEFT, padx=4)
        row += 1

        for fi in range(3):
            fvars = {
                "raw_a":  tk.StringVar(value="--"),
                "raw_g":  tk.StringVar(value="--"),
                "phys_a": tk.StringVar(value="--"),
                "phys_g": tk.StringVar(value="--"),
            }
            self.bt_frame_vars.append(fvars)

            panel = tk.LabelFrame(self.live_tab,
                                  text=f" Frame {fi}  (@ 0x{0x4A + fi*12:02X}–0x{0x55 + fi*12:02X}) ",
                                  font=('Courier', 8), fg=FRAME_FG[fi],
                                  bg=FRAME_COLORS[fi], padx=4, pady=2)
            panel.grid(row=row, column=0, columnspan=6, sticky='ew', padx=8, pady=2)
            panel.columnconfigure(1, weight=1)
            panel.columnconfigure(3, weight=1)

            tk.Label(panel, text="Accel raw:", font=('Arial',8,'bold'),
                     bg=FRAME_COLORS[fi], fg='#ccc').grid(row=0, column=0, sticky='e', padx=(2,4))
            tk.Label(panel, textvariable=fvars["raw_a"], font=('Courier',9),
                     bg=FRAME_COLORS[fi], fg=FRAME_FG[fi]).grid(row=0, column=1, sticky='w')

            tk.Label(panel, text="Accel (g):", font=('Arial',8,'bold'),
                     bg=FRAME_COLORS[fi], fg='#ccc').grid(row=0, column=2, sticky='e', padx=(12,4))
            tk.Label(panel, textvariable=fvars["phys_a"], font=('Courier',9),
                     bg=FRAME_COLORS[fi], fg="#ffffff").grid(row=0, column=3, sticky='w')

            tk.Label(panel, text="Gyro raw:", font=('Arial',8,'bold'),
                     bg=FRAME_COLORS[fi], fg='#ccc').grid(row=1, column=0, sticky='e', padx=(2,4))
            tk.Label(panel, textvariable=fvars["raw_g"], font=('Courier',9),
                     bg=FRAME_COLORS[fi], fg=FRAME_FG[fi]).grid(row=1, column=1, sticky='w')

            tk.Label(panel, text="Gyro (°/s):", font=('Arial',8,'bold'),
                     bg=FRAME_COLORS[fi], fg='#ccc').grid(row=1, column=2, sticky='e', padx=(12,4))
            tk.Label(panel, textvariable=fvars["phys_g"], font=('Courier',9),
                     bg=FRAME_COLORS[fi], fg="#ffffff").grid(row=1, column=3, sticky='w')

            row += 1

        # Sanity note
        tk.Label(self.live_tab,
                 text="⚠ BT frame layout is theoretical — adjust scale factors if values look wrong",
                 font=('Arial', 8), fg='#aa8800').grid(
            row=row, column=0, columnspan=6, sticky='w', padx=8, pady=(4, 0))

        # --- Bruteforce Diagnostics Tab ---
        debug_tab = ttk.Frame(self.tabs)
        self.tabs.add(debug_tab, text="Bruteforce Diagnostics")

        # Controls row
        ctrl_frame = tk.Frame(debug_tab)
        ctrl_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)

        tk.Label(ctrl_frame, text="Range:").pack(side=tk.LEFT)
        self.off_start = tk.IntVar(value=0)
        tk.Spinbox(ctrl_frame, from_=0, to=255, textvariable=self.off_start, width=4).pack(side=tk.LEFT, padx=2)
        tk.Label(ctrl_frame, text="to").pack(side=tk.LEFT)
        self.off_end = tk.IntVar(value=112)
        tk.Spinbox(ctrl_frame, from_=1, to=255, textvariable=self.off_end, width=4).pack(side=tk.LEFT, padx=2)

        tk.Label(ctrl_frame, text="  Window:").pack(side=tk.LEFT)
        self.avg_window = tk.IntVar(value=AVERAGE_WINDOW)
        tk.Spinbox(ctrl_frame, from_=2, to=256, textvariable=self.avg_window, width=4).pack(side=tk.LEFT, padx=2)

        tk.Label(ctrl_frame, text="  Stable±:").pack(side=tk.LEFT)
        self.stable_thresh = tk.DoubleVar(value=2.0)
        tk.Spinbox(ctrl_frame, from_=0.5, to=50.0, increment=0.5,
                   textvariable=self.stable_thresh, width=5).pack(side=tk.LEFT, padx=2)

        tk.Label(ctrl_frame, text="  Drift±:").pack(side=tk.LEFT)
        self.drift_thresh = tk.DoubleVar(value=10.0)
        tk.Spinbox(ctrl_frame, from_=1.0, to=200.0, increment=1.0,
                   textvariable=self.drift_thresh, width=5).pack(side=tk.LEFT, padx=2)

        self.show_floats = tk.BooleanVar(value=False)
        tk.Checkbutton(ctrl_frame, text="Try Floats", variable=self.show_floats).pack(side=tk.LEFT, padx=6)

        self.show_only_changing = tk.BooleanVar(value=False)
        tk.Checkbutton(ctrl_frame, text="Hide Stable", variable=self.show_only_changing).pack(side=tk.LEFT, padx=6)

        tk.Button(ctrl_frame, text="Reset Avg", bg="#5a1a1a", fg="white",
                  command=self.reset_tracker).pack(side=tk.RIGHT, padx=6)

        # Legend
        legend_frame = tk.Frame(debug_tab, bg="#111")
        legend_frame.pack(side=tk.TOP, fill=tk.X, padx=5)
        legend_items = [
            ("▲▲ Fast Rise",  "#ff4444"),
            ("▲ Rising",      "#ff9933"),
            ("~ Stable",      "#44bb44"),
            ("~ Noisy/Wide",  "#aaaa00"),
            ("▼ Falling",     "#4488ff"),
            ("▼▼ Fast Fall",  "#aa44ff"),
        ]
        for text, color in legend_items:
            tk.Label(legend_frame, text=f"  {text}  ", bg="#111", fg=color,
                     font=("Courier", 8)).pack(side=tk.LEFT)

        # Main display
        self.debug_disp = scrolledtext.ScrolledText(
            debug_tab, font=("Courier", 9), bg="#050505", fg="#cccccc"
        )
        self.debug_disp.pack(fill=tk.BOTH, expand=True)

        # Configure color tags
        self.debug_disp.tag_config("tag_rising_fast", foreground="#ff4444")
        self.debug_disp.tag_config("tag_rising",      foreground="#ff9933")
        self.debug_disp.tag_config("tag_stable",      foreground="#44bb44")
        self.debug_disp.tag_config("tag_noisy",       foreground="#aaaa00")
        self.debug_disp.tag_config("tag_falling",     foreground="#4488ff")
        self.debug_disp.tag_config("tag_falling_fast",foreground="#aa44ff")
        self.debug_disp.tag_config("tag_neutral",     foreground="#888888")
        self.debug_disp.tag_config("tag_header",      foreground="#ffffff")
        self.debug_disp.tag_config("tag_dim",         foreground="#444444")

    def _rescale(self):
        """Called when scale spinboxes change — updates the global scale factors."""
        global ACCEL_SCALE, GYRO_SCALE
        ACCEL_SCALE = 1.0 / max(1.0, self.accel_scale_var.get())
        GYRO_SCALE  = 1.0 / max(0.1, self.gyro_scale_var.get())

    def reset_tracker(self):
        self.tracker = ByteTracker(
            window=self.avg_window.get(),
            stable_threshold=self.stable_thresh.get(),
            drift_threshold=self.drift_thresh.get()
        )
        self.debug_disp.configure(state='normal')
        self.debug_disp.delete(1.0, tk.END)
        self.debug_disp.insert(tk.END, "[Averages reset]\n", "tag_neutral")
        self.debug_disp.configure(state='disabled')

    def run_imu_debug(self, packet_bytes: bytes):
        """
        Updates the ByteTracker and re-renders the diagnostics table with
        rolling averages and directional change indicators.
        """
        # Update tracker thresholds from UI
        self.tracker.window          = self.avg_window.get()
        self.tracker.stable_threshold = self.stable_thresh.get()
        self.tracker.drift_threshold  = self.drift_thresh.get()

        self.tracker.update(packet_bytes)

        start  = self.off_start.get()
        end    = min(self.off_end.get(), len(packet_bytes))
        n      = self.tracker.sample_count
        only_changing = self.show_only_changing.get()
        show_f = self.show_floats.get()

        self.debug_disp.configure(state='normal')
        self.debug_disp.delete(1.0, tk.END)

        # Header
        hdr = f"{'OFF':<5}{'HEX':<5}{'AVG':>7}{'RNG':>7}{'DELTA':>8}  {'TREND':<6}"
        hdr += f"{'INT16':>8}{'UINT16':>8}{'INT32':>12}"
        if show_f:
            hdr += f"{'FLOAT32':>14}"
        hdr += "\n" + "─" * (len(hdr)) + "\n"
        self.debug_disp.insert(tk.END, hdr, "tag_header")

        for i in range(start, end):
            symbol, color_tag, mag_str = self.tracker.get_trend(i)
            avg = self.tracker.get_avg(i)
            rng = self.tracker.get_range(i)
            raw = packet_bytes[i]

            if only_changing and symbol in ("~",) and rng <= self.stable_thresh.get() * 2:
                continue

            row = f"0x{i:02X} | {raw:02X}   | {avg:6.1f} | {rng:5.1f} | {self.tracker.get_delta(i):+7.1f}  {symbol:<4} "

            # int16 / uint16 / int32
            i16_str = u16_str = i32_str = f32_str = "-"
            if i + 2 <= len(packet_bytes):
                i16 = struct.unpack_from('<h', packet_bytes, i)[0]
                u16 = struct.unpack_from('<H', packet_bytes, i)[0]
                i16_str = str(i16)
                u16_str = str(u16)
            if i + 4 <= len(packet_bytes):
                i32 = struct.unpack_from('<i', packet_bytes, i)[0]
                i32_str = str(i32)
                if show_f:
                    f32 = struct.unpack_from('<f', packet_bytes, i)[0]
                    import math
                    f32_str = f"{f32:.4f}" if math.isfinite(f32) and abs(f32) < 1e9 else "[NF]"

            row += f"{i16_str:>8}{u16_str:>8}{i32_str:>12}"
            if show_f:
                row += f"{f32_str:>14}"
            row += "\n"

            self.debug_disp.insert(tk.END, row, color_tag)

        # Footer
        footer = f"\n{'─'*40}\n"
        footer += f"Samples: {n}  |  Window: {self.tracker.window}  |  "
        footer += f"Packet: {len(packet_bytes)} bytes  |  ID: 0x{packet_bytes[0]:02X}\n"
        footer += f"Stable ±{self.tracker.stable_threshold:.1f}  |  Drift ±{self.tracker.drift_threshold:.1f}\n"
        self.debug_disp.insert(tk.END, footer, "tag_dim")

        self.debug_disp.configure(state='disabled')


if __name__ == "__main__":
    root = tk.Tk()
    app = ControllerApp(root)
    root.mainloop()