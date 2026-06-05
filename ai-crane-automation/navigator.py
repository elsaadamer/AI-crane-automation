import time
import json
import os
import math

class CraneNavigator:
    def __init__(self, controller):
        self.controller = controller

        self.DIR_TROLLEY = 1
        self.DIR_BODY    = 1

        self.LIMIT_X_MIN = 0.4
        self.LIMIT_X_MAX = 31.0

        self.SAFETY_MARGIN = 4.0
        self.OBSTACLE_RADIUS = 4.0

        self.SPEED_TROLLEY_OUT = 200; self.CM_SEC_TROLLEY_OUT = 12.0
        self.SPEED_TROLLEY_RET = 255; self.CM_SEC_TROLLEY_RET = 13.0
        self.SPEED_BODY_POS_OUT = 200; self.CM_SEC_BODY_POS_OUT = 11.0
        self.SPEED_BODY_POS_RET = 255; self.CM_SEC_BODY_POS_RET = 8.0
        self.SPEED_BODY_NEG_OUT = 255; self.CM_SEC_BODY_NEG_OUT = 8.0
        self.SPEED_BODY_NEG_RET = 200; self.CM_SEC_BODY_NEG_RET = 11.0

        self.SPEED_HOOK_UP = 200
        self.SPEED_HOOK_DOWN = 200

        self.MEMORY_FILE = "crane_memory.json"
        self.current_trolley, self.current_body = self._load_position()

    def _load_position(self):
        if os.path.exists(self.MEMORY_FILE):
            try:
                with open(self.MEMORY_FILE, 'r') as f:
                    data = json.load(f)
                    x, y = data.get("x", 0.0), data.get("y", 0.0)
                    return x, y
            except: pass
        return 0.0, 0.0

    def _save_position(self):
        try:
            with open(self.MEMORY_FILE, 'w') as f:
                json.dump({"x": self.current_trolley, "y": self.current_body}, f)
        except: pass

    def _is_collision(self, start_x, start_y, end_x, end_y, obstacles):
        if not obstacles: return None

        for obs in obstacles:
            ox, oy = obs['x'], obs['y']

            dx = end_x - start_x
            dy = end_y - start_y

            if dx == 0 and dy == 0:
                dist = math.hypot(ox - start_x, oy - start_y)
            else:
                t = ((ox - start_x) * dx + (oy - start_y) * dy) / (dx*dx + dy*dy)
                t = max(0, min(1, t))
                closest_x = start_x + t * dx
                closest_y = start_y + t * dy
                dist = math.hypot(ox - closest_x, oy - closest_y)

            if dist < (self.OBSTACLE_RADIUS + self.SAFETY_MARGIN):
                return obs['name']
        return None

    def move_to(self, target_x, target_y, obstacles=None):
        if not self.controller.ser: return "❌ Hardware not connected"

        start_x, start_y = self.current_trolley, self.current_body
        print(f"   [Nav] Request: ({start_x:.1f}, {start_y:.1f}) -> ({target_x:.1f}, {target_y:.1f})")

        corner_a_x, corner_a_y = start_x, target_y
        block_a1 = self._is_collision(start_x, start_y, corner_a_x, corner_a_y, obstacles)
        block_a2 = self._is_collision(corner_a_x, corner_a_y, target_x, target_y, obstacles)

        path_a_safe = (block_a1 is None) and (block_a2 is None)

        corner_b_x, corner_b_y = target_x, start_y
        block_b1 = self._is_collision(start_x, start_y, corner_b_x, corner_b_y, obstacles)
        block_b2 = self._is_collision(corner_b_x, corner_b_y, target_x, target_y, obstacles)

        path_b_safe = (block_b1 is None) and (block_b2 is None)

        if path_a_safe:
            print("   [Nav] ✅ Path A (Y then X) is clear.")
            self._drive_axis("Y", target_y)
            self._drive_axis("X", target_x)
            return f"Moved (Path A) -> X:{target_x:.1f}, Y:{target_y:.1f}"

        elif path_b_safe:
            print("   [Nav] ✅ Path B (X then Y) is clear.")
            self._drive_axis("X", target_x)
            self._drive_axis("Y", target_y)
            return f"Moved (Path B) -> X:{target_x:.1f}, Y:{target_y:.1f}"

        else:
            blocker = block_a1 or block_a2 or block_b1 or block_b2
            return f"⛔ SAFETY HALT: Path blocked by {blocker}. Try moving manually via a waypoint (e.g., go to Y=15 first)."

    def _drive_axis(self, axis, target_val):
        if axis == "X":
            current = self.current_trolley
            limit_min = self.LIMIT_X_MIN
            limit_max = self.LIMIT_X_MAX
        else:
            current = self.current_body
            limit_min = -999.0
            limit_max = 999.0

        if axis == "X":
            if target_val < limit_min:
                print(f"   [Limit] Clamping {target_val} to {limit_min}")
                target_val = limit_min
            if target_val > limit_max:
                print(f"   [Limit] Clamping {target_val} to {limit_max}")
                target_val = limit_max

        delta = target_val - current
        if abs(delta) < 0.1: return

        if axis == "Y":
            if delta > 0:
                if current < -0.1: spd, cm_s = self.SPEED_BODY_NEG_RET, self.CM_SEC_BODY_NEG_RET
                else: spd, cm_s = self.SPEED_BODY_POS_OUT, self.CM_SEC_BODY_POS_OUT
            else:
                if current > 0.1: spd, cm_s = self.SPEED_BODY_POS_RET, self.CM_SEC_BODY_POS_RET
                else: spd, cm_s = self.SPEED_BODY_NEG_OUT, self.CM_SEC_BODY_NEG_OUT
            cmd_sign = 1 if delta > 0 else -1
            motor_cmd = f"<J,{spd*cmd_sign*self.DIR_BODY},0,0,"
        else:
            if delta < 0: spd, cm_s = self.SPEED_TROLLEY_RET, self.CM_SEC_TROLLEY_RET
            else: spd, cm_s = self.SPEED_TROLLEY_OUT, self.CM_SEC_TROLLEY_OUT
            cmd_sign = 1 if delta > 0 else -1
            motor_cmd = f"<J,0,0,{spd*cmd_sign*self.DIR_TROLLEY},"

        duration = int((abs(delta) / cm_s) * 1000)
        full_command = f"{motor_cmd}{duration}>"

        print(f"   [Motor] {axis}: Delta={delta:.1f}cm -> {duration}ms")
        self.controller.ser.write(full_command.encode())
        time.sleep((duration/1000) + 1.0)

        if axis == "X": self.current_trolley = target_val
        else: self.current_body = target_val
        self._save_position()

        try:
            self.controller.ser.close()
            time.sleep(0.3)
            self.controller.ser.open()
            time.sleep(3.0)
        except: pass

    def move_hook(self, direction, duration_sec):
        if not self.controller.ser: return "❌ Hardware not connected"
        if direction.lower() == "up": speed = self.SPEED_HOOK_UP
        elif direction.lower() == "down": speed = -self.SPEED_HOOK_DOWN
        else: return "❌ Invalid Direction"

        duration_ms = int(duration_sec * 1000)
        command = f"<J,0,{speed},0,{duration_ms}>"
        self.controller.ser.write(command.encode())
        time.sleep(duration_sec + 0.5)
        return f"Hook moved {direction} for {duration_sec}s"

def get_navigator(controller):
    return CraneNavigator(controller)
