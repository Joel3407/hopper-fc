#!/usr/bin/env python3
"""
EDF HOPPER GROUND STATION

Receives binary telemetry from the Teensy's second USB serial port
(SerialUSB1) and displays:
- Real-time plots (attitude, altitude, battery)
- Status text
- Command uplink capability

Frame format (little-endian, 24 bytes, sent at 10 Hz by send_telemetry()):
    0xAA 0x55 | state u8 | roll f32 | pitch f32 | altitude f32 |
    throttle f32 (0..1) | vbat f32 | XOR checksum of the 21 payload bytes
Frames that fail the checksum are dropped silently and the reader re-syncs.

Requirements:
    pip install -r requirements.txt

Usage:
    python ground_station.py /dev/ttyACM1  (Linux/Mac)
    python ground_station.py COM3          (Windows)

Author: Joel's Rocket Project
Date: November 2025
"""

import serial
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import sys
import threading

# ==================== CONFIGURATION ====================

BAUD_RATE = 115200  # Ignored by Teensy USB serial, but pyserial needs a value

SYNC = b'\xaa\x55'
PAYLOAD_SIZE = 21                                # state u8 + 5 x float32
FRAME_SIZE = len(SYNC) + PAYLOAD_SIZE + 1        # sync + payload + XOR = 24
MAX_BUFFER = FRAME_SIZE * 50                     # cap for runaway garbage

NO_BATTERY_V = 5.0   # below this, treat as "no battery connected"
MIN_SAFE_V = 19.8

# Plot history length (seconds)
HISTORY_TIME = 30

# State names
STATE_NAMES = {
    0: "PREFLIGHT",
    1: "ARMED",
    2: "TAKEOFF",
    3: "HOVER",
    4: "LANDING",
    5: "LANDED",
    6: "ABORT"
}

# ==================== DATA STORAGE ====================

class TelemetryData:
    """Stores incoming telemetry with time history"""
    
    def __init__(self, maxlen=3000):  # 30 sec at 100 Hz
        self.time = deque(maxlen=maxlen)
        self.state = deque(maxlen=maxlen)
        self.roll = deque(maxlen=maxlen)
        self.pitch = deque(maxlen=maxlen)
        self.altitude = deque(maxlen=maxlen)
        self.throttle = deque(maxlen=maxlen)
        self.voltage = deque(maxlen=maxlen)
        
        self.start_time = time.time()
        self.packet_count = 0
        self.last_update = 0
        
    def add_packet(self, state, roll, pitch, altitude, throttle, voltage):
        """Add new telemetry packet"""
        current_time = time.time() - self.start_time
        
        self.time.append(current_time)
        self.state.append(state)
        self.roll.append(roll)
        self.pitch.append(pitch)
        self.altitude.append(altitude)
        self.throttle.append(throttle)
        self.voltage.append(voltage)
        
        self.packet_count += 1
        self.last_update = time.time()
        
    def get_arrays(self):
        """Convert deques to numpy arrays for plotting"""
        return {
            'time': np.array(self.time),
            'state': np.array(self.state),
            'roll': np.array(self.roll),
            'pitch': np.array(self.pitch),
            'altitude': np.array(self.altitude),
            'throttle': np.array(self.throttle),
            'voltage': np.array(self.voltage)
        }
        
    def get_latest(self):
        """Get most recent values"""
        if len(self.time) == 0:
            return None
            
        return {
            'time': self.time[-1],
            'state': self.state[-1],
            'roll': self.roll[-1],
            'pitch': self.pitch[-1],
            'altitude': self.altitude[-1],
            'throttle': self.throttle[-1],
            'voltage': self.voltage[-1]
        }

# Global data object
telemetry = TelemetryData()

# ==================== SERIAL COMMUNICATION ====================

def xor_checksum(payload):
    """XOR of all payload bytes (sync bytes excluded)"""
    cs = 0
    for b in payload:
        cs ^= b
    return cs

class FrameParser:
    """Incremental frame parser: feed raw bytes, get back decoded packets.

    Scans for the sync bytes, verifies the XOR checksum, and silently drops
    bad frames. After a bad checksum it skips one byte and rescans, so a real
    frame hidden inside the corrupt one is still found.
    """

    def __init__(self):
        self.buf = bytearray()
        self.bad_frames = 0

    def feed(self, data):
        self.buf.extend(data)
        packets = []

        while True:
            idx = self.buf.find(SYNC)
            if idx < 0:
                # Keep a trailing 0xAA: its 0x55 may arrive in the next read
                keep = 1 if self.buf.endswith(SYNC[:1]) else 0
                del self.buf[:len(self.buf) - keep]
                break
            del self.buf[:idx]               # drop garbage before sync
            if len(self.buf) < FRAME_SIZE:
                break                        # wait for the rest of the frame

            payload = self.buf[len(SYNC):FRAME_SIZE - 1]
            if xor_checksum(payload) != self.buf[FRAME_SIZE - 1]:
                self.bad_frames += 1
                del self.buf[:1]             # bad frame: re-sync after this sync
                continue

            state, roll, pitch, altitude, throttle, voltage = struct.unpack(
                '<B5f', payload)
            packets.append({
                'state': state,
                'roll': roll,
                'pitch': pitch,
                'altitude': altitude,
                'throttle': throttle,
                'voltage': voltage
            })
            del self.buf[:FRAME_SIZE]

        if len(self.buf) > MAX_BUFFER:
            del self.buf[:-FRAME_SIZE]
        return packets

def serial_reader_thread(port):
    """Background thread to read serial data"""
    global telemetry
    parser = FrameParser()

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        print(f"Connected to {port}")

        while True:
            data = ser.read(max(1, ser.in_waiting))
            for pkt in parser.feed(data):
                telemetry.add_packet(
                    pkt['state'],
                    pkt['roll'],
                    pkt['pitch'],
                    pkt['altitude'],
                    pkt['throttle'],
                    pkt['voltage']
                )

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)

def send_command(ser, command):
    """Send command to rocket"""
    if ser and ser.is_open:
        ser.write(command.encode())
        print(f"Sent command: {command}")

# ==================== PLOTTING ====================

class RealtimePlotter:
    """Real-time telemetry plotting"""
    
    def __init__(self):
        # Create figure with subplots
        self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 10))
        self.fig.suptitle('EDF Hopper Telemetry', fontsize=16, fontweight='bold')
        
        # Attitude plot
        self.ax_attitude = self.axes[0]
        self.line_roll, = self.ax_attitude.plot([], [], 'r-', label='Roll', linewidth=2)
        self.line_pitch, = self.ax_attitude.plot([], [], 'b-', label='Pitch', linewidth=2)
        self.ax_attitude.set_ylabel('Angle (°)', fontsize=12)
        self.ax_attitude.set_title('Attitude', fontweight='bold')
        self.ax_attitude.legend(loc='upper right')
        self.ax_attitude.grid(True, alpha=0.3)
        self.ax_attitude.set_ylim(-45, 45)
        
        # Altitude & Throttle plot
        self.ax_altitude = self.axes[1]
        self.line_altitude, = self.ax_altitude.plot([], [], 'g-', label='Altitude', linewidth=2)
        self.ax_altitude.set_ylabel('Altitude (m)', fontsize=12, color='g')
        self.ax_altitude.tick_params(axis='y', labelcolor='g')
        self.ax_altitude.set_title('Altitude & Throttle', fontweight='bold')
        self.ax_altitude.grid(True, alpha=0.3)
        self.ax_altitude.set_ylim(-1, 10)
        
        # Twin axis for throttle
        self.ax_throttle = self.ax_altitude.twinx()
        self.line_throttle, = self.ax_throttle.plot([], [], 'm-', label='Throttle', linewidth=2)
        self.ax_throttle.set_ylabel('Throttle', fontsize=12, color='m')
        self.ax_throttle.tick_params(axis='y', labelcolor='m')
        self.ax_throttle.set_ylim(0, 1.2)
        
        # Battery voltage plot
        self.ax_voltage = self.axes[2]
        self.line_voltage, = self.ax_voltage.plot([], [], 'orange', linewidth=2)
        self.ax_voltage.set_xlabel('Time (s)', fontsize=12)
        self.ax_voltage.set_ylabel('Voltage (V)', fontsize=12)
        self.ax_voltage.set_title('Battery Voltage', fontweight='bold')
        self.ax_voltage.grid(True, alpha=0.3)
        self.ax_voltage.axhline(y=MIN_SAFE_V, color='r', linestyle='--', label='Min Safe (19.8V)')
        self.ax_voltage.legend(loc='lower left')
        
        # Status text
        self.status_text = self.fig.text(0.02, 0.02, '', fontsize=10, family='monospace',
                                         verticalalignment='bottom')
        
        plt.tight_layout()
        
    def update(self, frame):
        """Update plots with new data"""
        data = telemetry.get_arrays()
        latest = telemetry.get_latest()
        
        if latest is None or len(data['time']) < 2:
            return
            
        # Update attitude plot
        self.line_roll.set_data(data['time'], data['roll'])
        self.line_pitch.set_data(data['time'], data['pitch'])
        
        # Update altitude plot
        self.line_altitude.set_data(data['time'], data['altitude'])
        self.line_throttle.set_data(data['time'], data['throttle'])
        
        # Update voltage plot
        self.line_voltage.set_data(data['time'], data['voltage'])
        
        # Auto-scale x-axis (last 30 seconds)
        if data['time'][-1] > HISTORY_TIME:
            xlim_min = data['time'][-1] - HISTORY_TIME
            xlim_max = data['time'][-1] + 1
        else:
            xlim_min = 0
            xlim_max = HISTORY_TIME
            
        self.ax_attitude.set_xlim(xlim_min, xlim_max)
        self.ax_altitude.set_xlim(xlim_min, xlim_max)
        self.ax_voltage.set_xlim(xlim_min, xlim_max)
        
        # Auto-scale altitude y-axis
        if len(data['altitude']) > 0:
            alt_max = max(data['altitude']) if max(data['altitude']) > 5 else 5
            self.ax_altitude.set_ylim(-0.5, alt_max * 1.2)
        
        # Autoscale voltage axis over the visible window, always keeping the
        # safe-limit line in view (a 0.9 V no-battery reading would otherwise
        # be off the axis)
        visible = data['voltage'][data['time'] >= xlim_min]
        v_lo = min(visible.min(), MIN_SAFE_V)
        v_hi = max(visible.max(), MIN_SAFE_V)
        pad = max(0.5, (v_hi - v_lo) * 0.1)
        self.ax_voltage.set_ylim(v_lo - pad, v_hi + pad)

        # Update status text
        state_name = STATE_NAMES.get(latest['state'], 'UNKNOWN')
        if latest['voltage'] < NO_BATTERY_V:
            battery_str = f"NO BATTERY  ({latest['voltage']:5.2f} V)"
        else:
            battery_pct = ((latest['voltage'] - MIN_SAFE_V) / (25.2 - MIN_SAFE_V)) * 100
            battery_pct = max(0, min(100, battery_pct))  # Clamp 0-100%
            battery_str = f"{latest['voltage']:5.2f} V  ({battery_pct:5.1f}%)"
        
        status = f"""
STATE: {state_name:10s}  |  TIME: {latest['time']:6.1f} s  |  PACKETS: {telemetry.packet_count}
ROLL:  {latest['roll']:+6.1f}°  |  PITCH: {latest['pitch']:+6.1f}°
ALT:   {latest['altitude']:6.2f} m  |  THROTTLE: {latest['throttle']*100:5.1f}%
BATTERY: {battery_str}
        """
        self.status_text.set_text(status.strip())
        
        # Change background color based on state
        if latest['state'] == 6:  # ABORT
            self.fig.patch.set_facecolor('#ffcccc')
        elif latest['state'] in [2, 3]:  # TAKEOFF or HOVER
            self.fig.patch.set_facecolor('#ccffcc')
        else:
            self.fig.patch.set_facecolor('white')
            
    def start(self):
        """Start animation"""
        self.anim = FuncAnimation(self.fig, self.update, interval=100, blit=False)
        plt.show()

# ==================== COMMAND LINE INTERFACE ====================

def command_interface(ser):
    """Command line interface for sending commands"""
    print("\n=== GROUND STATION COMMANDS ===")
    print("  a - Arm")
    print("  d - Disarm")
    print("  t - Takeoff")
    print("  l - Land")
    print("  x - ABORT (emergency)")
    print("  s - Status")
    print("  q - Quit")
    print("================================\n")
    
    while True:
        cmd = input("Command: ").strip().lower()
        
        if cmd == 'q':
            print("Exiting...")
            sys.exit(0)
        elif cmd in ['a', 'd', 't', 'l', 'x', 's']:
            send_command(ser, cmd)
        else:
            print("Invalid command!")

# ==================== MAIN ====================

def main():
    if len(sys.argv) != 2:
        print(f"Usage: python {sys.argv[0]} <serial port>  (e.g. /dev/ttyACM1 or COM3)")
        sys.exit(2)
    serial_port = sys.argv[1]

    print("=== EDF HOPPER GROUND STATION ===")
    print(f"Connecting to {serial_port}...")
    
    # Start serial reader thread
    reader = threading.Thread(target=serial_reader_thread, args=(serial_port,), daemon=True)
    reader.start()
    
    # Wait for first packet
    print("Waiting for telemetry...")
    timeout = 10
    start = time.time()
    while len(telemetry.time) == 0:
        if time.time() - start > timeout:
            print("ERROR: No telemetry received!")
            print("Check:")
            print("  - Teensy USB connected (use the SECOND serial port, SerialUSB1)")
            print("  - Correct serial port")
            print("  - Flight computer powered on")
            sys.exit(1)
        time.sleep(0.1)
        
    print(f"Telemetry received! ({telemetry.packet_count} packets)")
    
    # Start command interface in background
    # (Disabled for now since it blocks - use keyboard commands in plot window instead)
    # cmd_thread = threading.Thread(target=command_interface, args=(ser,), daemon=True)
    # cmd_thread.start()
    
    # Start real-time plotting
    print("Starting plots...")
    plotter = RealtimePlotter()
    plotter.start()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nShutdown requested...")
        sys.exit(0)
