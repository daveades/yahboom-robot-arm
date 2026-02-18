#!/usr/bin/env python3
import serial
import time
import struct
import sys
import glob
# Protocol Constants
HEADER = 0xFF
DEVICE_ID = 0xFC
CMD_SERVO_WRITE = 0x10
def checksum(data):
    """Calculate checksum for the packet."""
    return sum(data) & 0xFF
def create_packet(servo_id, angle, time_ms):
    """
    Create a packet to move a single servo.
    Angle mapping logic based on Arm_Lib.py.
    """
    cmd_type = CMD_SERVO_WRITE + servo_id
    
    # Calibration mapping (from Arm_Lib.py)
    if servo_id in [1, 6]:
        pos = int((3100 - 900) * angle / 180 + 900)
    elif servo_id in [2, 3, 4]:
        # These are inverted
        inv_angle = 180 - angle
        pos = int((3100 - 900) * inv_angle / 180 + 900)
    elif servo_id == 5:
        # Servo 5 has 0-270 range
        pos = int((3700 - 380) * angle / 270 + 380)
    else:
        print(f"Invalid Servo ID: {servo_id}")
        return None
    # Construct Payload
    val_h = (pos >> 8) & 0xFF
    val_l = pos & 0xFF
    time_h = (time_ms >> 8) & 0xFF
    time_l = time_ms & 0xFF
    
    payload = [val_h, val_l, time_h, time_l]
    
    # Checksum calculation: sum of (Length + Type + Data) & 0xFF
    length = 7
    data_bytes = [length, cmd_type] + payload
    csum = sum(data_bytes) & 0xFF
    
    packet = [0xFF, 0xFC] + data_bytes + [csum]
    return packet
def read_version(ser):
    """Try to read the firmware version. Returns True if successful."""
    # Command: 0xFF 0xFC 0x03 0x01 [Checksum]
    # Checksum = (0x03 + 0x01) = 0x04
    cmd = [0xFF, 0xFC, 0x03, 0x01, 0x04]
    
    print(f"Sending Version Read Command: {[hex(x) for x in cmd]}")
    ser.reset_input_buffer()
    ser.write(bytearray(cmd))
    time.sleep(0.1)
    
    # Expecting response
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"Received ({len(response)} bytes): {[hex(x) for x in response]}")
        return True
    else:
        print("No response received.")
        return False
def set_torque(ser, on=True):
    """Enable or disable torque."""
    # Packet provided by Arm_Lib: FF FC 04 1A [01/00] Checksum
    # Data: Len=4, Type=0x1A, Val=0x01/00.
    # Checksum = 4 + 0x1A + Val.
    val = 0x01 if on else 0x00
    length = 4
    cmd_type = 0x1A
    csum = (length + cmd_type + val) & 0xFF
    packet = [0xFF, 0xFC, length, cmd_type, val, csum]
    ser.write(bytearray(packet))
    print(f"Torque {'ON' if on else 'OFF'} sent.")
def find_serial_port():
    """Try to find the correct serial port automatically."""
    candidates = ['/dev/ttyUSB0', '/dev/ttyAMA0', '/dev/serial0', '/dev/myserial']
    # Add any other ttyUSB* or ttyS* found
    candidates.extend(glob.glob('/dev/ttyUSB*'))
    candidates.extend(glob.glob('/dev/ttyS*'))
    
    # Filter unique and existing
    import os
    valid_ports = []
    for port in sorted(list(set(candidates))):
        if os.path.exists(port):
            valid_ports.append(port)
            
    return valid_ports
def interactive_mode(ser):
    print("\n--- Interactive Mode ---")
    print("Simulating Gamepad replacement. Enter commands:")
    print("  'center' : Reset all to 90")
    print("  'left'   : Base left (Servo 1 -> 135)")
    print("  'right'  : Base right (Servo 1 -> 45)")
    print("  'grab'   : Close claw (Servo 6 -> 180)")
    print("  'open'   : Open claw (Servo 6 -> 0)")
    print("  'q'      : Quit")
    
    while True:
        cmd = input("\nCommand > ").strip().lower()
        if cmd == 'q':
            break
        
        pkt = None
        if cmd == 'center':
            # Need to send packets for all servos or just loop 1-6
            print("Centering all servos...")
            for i in range(1, 7):
                pkt = create_packet(i, 90, 500)
                ser.write(bytearray(pkt))
                time.sleep(0.05)
            continue
            
        elif cmd == 'left':
            pkt = create_packet(1, 135, 500)
        elif cmd == 'right':
            pkt = create_packet(1, 45, 500)
        elif cmd == 'grab':
            pkt = create_packet(6, 150, 500) # 150 safe grab
        elif cmd == 'open':
            pkt = create_packet(6, 30, 500)  # 30 safe open
        else:
            print("Unknown command. Try 'center', 'left', 'right', 'grab', 'open'.")
            continue
            
        if pkt:
            ser.write(bytearray(pkt))
            print(f"Sent: {[hex(x) for x in pkt]}")
def test_connection(port_arg=None):
    port = port_arg
    
    if not port:
        print("No port specified. scanning...")
        ports = find_serial_port()
        if not ports:
            print("No serial ports found! Check your connections.")
            return
        print(f"Found ports: {ports}")
        # Default to the first likely candidate for external devices
        # Prefer ttyAMA (User specified), ttyUSB, or myserial
        preferred = [p for p in ports if 'AMA' in p or 'USB' in p or 'my' in p]
        port = preferred[0] if preferred else ports[0]
        
    print(f"\nAttempting to open: {port}")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        print(f"Success! Connected to {port}")
        
        # Try to read version to verify wiring
        print("\n--- Connection Test: Reading Version ---")
        if not read_version(ser):
            print("WARNING: Read failed. Check wiring (TX<->RX) or remove SPI gamepad if connected.")
            # We continue anyway just in case it's a receive-only issue
        
        # Enable Torque just in case
        set_torque(ser, True)
            
    except Exception as e:
        print(f"Failed to open {port}: {e}")
        print("Try specifying a port: python3 test_arm_protocol.py /dev/ttyAMA0")
        return
    # Run auto-test then interactive
    print("\nRunning quick functionality check (Center -> 45deg -> Center)...")
    try:
        # Move Servo 1 (Base)
        ser.write(bytearray(create_packet(1, 90, 500)))
        time.sleep(1)
        ser.write(bytearray(create_packet(1, 45, 500)))
        time.sleep(1)
        ser.write(bytearray(create_packet(1, 90, 500)))
        print("Check complete.")
        
        interactive_mode(ser)
        
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        ser.close()
        print("Connection closed.")
if __name__ == "__main__":
    if len(sys.argv) > 1:
        test_connection(sys.argv[1])
    else:
        test_connection()
