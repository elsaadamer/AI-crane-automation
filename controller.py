import serial
import time

class CraneController:
    def __init__(self):
        self.ser = None

    def connect(self, manual_port=None):
        target_port = "COM10"
        if self.ser and self.ser.is_open: return
        print(f"🔌 Connecting to {target_port}...")
        try:
            self.ser = serial.Serial(target_port, 115200, timeout=1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            print(f"✅ Connected to {target_port}")
        except Exception as e:
            print(f"❌ SERIAL ERROR: {e}")

    def wait_for_done(self, timeout_sec):
        start = time.time()
        while (time.time() - start) < timeout_sec:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if "<DONE>" in line: return True
            time.sleep(0.05)
        return False

    def drive_joystick(self, x, y, z, duration_ms=0):
        if not self.ser or not self.ser.is_open:
            print("❌ Not Connected!")
            return False

        cmd = f"<J,{int(x)},{int(y)},{int(z)},{int(duration_ms)}>"
        self.ser.write((cmd + '\n').encode())
        print(f"📡 SENT: {cmd}")

        wait_time = (duration_ms / 1000.0) + 1.5
        success = self.wait_for_done(wait_time)
        return success

    def set_magnet(self, is_on):
        if not self.ser or not self.ser.is_open:
            print("❌ Not Connected!")
            return False

        val = 1 if is_on else 0
        cmd = f"<M,{val}>"
        self.ser.write((cmd + '\n').encode())
        print(f"🧲 MAGNET SENT: {cmd}")
        time.sleep(0.2)
        return True

    def stop_all_motors(self):
        if self.ser: self.ser.write(b"<S>\n")
