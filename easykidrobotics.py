from machine import UART, Pin
from time import ticks_ms, ticks_diff, sleep_ms
from math import cos, sin, radians, pi, atan2
from board import motor, servo, rgbled_board
import bluepad32

uart = UART(2, baudrate=115200, tx=13, rx=26)
rx_buf = bytearray(8)

STATE_WAIT_HEADER = 0
STATE_READ_DATA = 1

parser_state = STATE_WAIT_HEADER
rx_index = 0
current_yaw = 0.0
yaw_offset = 0.0

def get_imu():
    global current_yaw, parser_state, rx_index
    
    while uart.any():
        b = uart.read(1)
        if not b: continue
        b = b[0]
        
        if parser_state == STATE_WAIT_HEADER:
            if b != 0xAA: continue
            rx_buf[0] = b
            rx_index = 1
            parser_state = STATE_READ_DATA
                
        elif parser_state == STATE_READ_DATA:
            rx_buf[rx_index] = b
            rx_index += 1
            
            if rx_index != 8: continue
            parser_state = STATE_WAIT_HEADER
            if rx_buf[7] != 0x55: continue

            yaw = (rx_buf[1] << 8) | rx_buf[2]
            if yaw >= 0x8000: yaw -= 0x10000
            
            current_yaw = (yaw / 100.0) - yaw_offset
            return True
    return False

def clamp(val, min_val=-1.0, max_val=1.0):
    return max(min(val, max_val), min_val)

def wrap_rads(angle):
    while angle > pi: angle -= 2 * pi
    while angle < -pi: angle += 2 * pi
    return angle

class PIDController:

    def __init__(self, kp, ki, kd, output_limit=1.0, integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = ticks_ms()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = ticks_ms()

    def calculate(self, setpoint, measurement):
        now = ticks_ms()
        dt = ticks_diff(now, self.prev_time) / 1000.0
        self.prev_time = now

        error = wrap_rads(setpoint - measurement)

        self.integral += error * dt
        self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return clamp(output, -self.output_limit, self.output_limit)

class TaskManager:

    def __init__(s):
        s.tasks = []

    def create(s, ms, callback, currPacket = -1):
        s.tasks.append({
            "t": ticks_ms() + ms,
            "f": callback,
            "p": currPacket
        })

    def update(s):
        now = ticks_ms()
        for i in range(len(s.tasks) - 1, -1, -1):
            if ticks_diff(s.tasks[i]["t"], now) <= 0:
                task = s.tasks.pop(i)
                task["f"](task["p"])

task = TaskManager()
controller = PIDController(kp=1, ki=0, kd=0.08)
setpoint = 0.0

def speed_control(value, onControlledR2, onControlledL2):
    abs_val = abs(value)
    if bluepad32.r2(): return value * onControlledR2
    if bluepad32.l2(): return value * onControlledL2
    elif abs_val <= 0.1: return 0
    elif abs_val <= 0.3: return value * 0.3
    else: return value * 0.65

last_rx = 0
def linear_heading(rx, ry, yaw):
    global last_rx, setpoint
    if bluepad32.r2(): rx = clamp(rx, -0.2, 0.2)
    else: rx = clamp(rx, -0.5, 0.5)
    r = controller.calculate(setpoint, yaw)
    if abs(wrap_rads(setpoint - yaw)) < 0.01: r = 0
    if abs(rx) > 0.1 or ticks_diff(ticks_ms(), last_rx) < 450:
        r = rx
        setpoint = yaw
    if abs(rx) > 0.1: last_rx = ticks_ms()
    return r

active = False
def atan2_heading(rx, ry, yaw):
    global setpoint, active

    if rx * rx + ry * ry > 0.8:
        active = True
        setpoint = atan2(rx, ry)
        r = controller.calculate(setpoint, yaw)
    else:
        if active:
            setpoint = yaw
            controller.reset()
        active = False
        r = 0

    return r

def movement():
    global setpoint

    rawx = -bluepad32.axisX() / 512
    rawy = -bluepad32.axisY() / 512
    lx = speed_control(rawx, 0.3, 1)
    ly = speed_control(rawy, 0.2, 0.9)
    rx = -bluepad32.axisRX() / 512.0
    ry = -bluepad32.axisRY() / 512.0
    yaw = radians(current_yaw)

    r = atan2_heading(rx, ry, yaw)
    if abs(lx) < 0.1 and abs(ly) < 0.1 and abs(r) < 0.01: r = 0

    # movement
    x = (cos(yaw) * lx) + (sin(yaw) * -ly)
    y = (sin(yaw) * lx) - (cos(yaw) * -ly)

    d = max(abs(x) + abs(y) + abs(r), 1)
    fl = (((y - x - r) / d))
    fr = (((y + x + r) / d))
    rl = (((y - x + r) / d))
    rr = (((y + x - r) / d))

    scale = max(abs(fl), abs(fr), abs(rl), abs(rr), 1)

    fl = int((fl / scale) * 100)
    fr = int((fr / scale) * 100)
    rl = int((rl / scale) * 100)
    rr = int((rr / scale) * 100)

    motor.wheel(fl, rr, fr, rl)

button_states = {}
def pressed_once(name, current):
    global button_states
    
    previous_state = button_states.get(name, False)
    clicked = current and not previous_state
    button_states[name] = current
    
    return clicked

artifactUp = False
pickUp = False
armUp = False

#SV4
HANGUP = 160
HANGMED = 120
HANGPICK = 130
HANGDOWN = 95

#SV5
ARMDOWN = 10
ARMMED = 50
ARMUP = 120

#SV6
RELEASE = 110
PICK = 85

pin14 = Pin(14, Pin.OUT)
pin15 = Pin(15, Pin.OUT)

liftLevel = 0
liftPacket = 0

UP_TRANSITIONS = {
    0: 1,
    1: 3,
    2: 3
}

DOWN_TRANSITIONS = {
    3: 2,
    2: 0,
    1: 0
}

def braking(currPacket):
    global liftPacket

    if currPacket != liftPacket: return
    pin14.value(1)
    pin15.value(1)

def lift(direction):
    global liftLevel, liftPacket

    transitions = UP_TRANSITIONS if direction > 0 else DOWN_TRANSITIONS
    if liftLevel not in transitions: return
    nextLevel = transitions[liftLevel]

    if direction > 0:
        pin14.value(0)
        pin15.value(1)
    else:
        pin14.value(1)
        pin15.value(0)

    liftPacket += 1
    packet = liftPacket

    delay = 100 if abs(nextLevel - liftLevel) == 1 else 300

    task.create(delay, braking, packet)
    liftLevel = nextLevel

def control():
    global setpoint, yaw_offset, artifactUp, pickUp, armUp

    if pressed_once("square", bluepad32.square()):
        yaw_offset += current_yaw
        setpoint = 0

    if pressed_once("r1", bluepad32.r1()):
        if not artifactUp:
            pickDelay = 100 if pickUp else 0
            servo.angle(servo.SV6, PICK)
            task.create(pickDelay, lambda x: servo.angle(servo.SV5, ARMUP))
            task.create(175 + pickDelay, lambda x: servo.angle(servo.SV4, HANGMED))
            task.create(275 + pickDelay, lambda x: servo.angle(servo.SV6, RELEASE))
            task.create(300 + pickDelay, lambda x: servo.angle(servo.SV4, HANGUP))
            task.create(300 + pickDelay, lambda x: servo.angle(servo.SV5, ARMDOWN))
            
            pickUp = False
            armUp = False
        else:
            servo.angle(servo.SV4, HANGPICK)
            servo.angle(servo.SV5, ARMUP)
            task.create(200, lambda x: servo.angle(servo.SV6, PICK))
            task.create(250, lambda x: servo.angle(servo.SV4, HANGDOWN))
            task.create(300, lambda x: servo.angle(servo.SV5, ARMDOWN))

            pickUp = True
            armUp = False
        artifactUp = not artifactUp

    if pressed_once("l1", bluepad32.l1()):
        if not pickUp: servo.angle(servo.SV6, PICK)
        else: servo.angle(servo.SV6, RELEASE)
        pickUp = not pickUp

    if pressed_once("circle", bluepad32.circle()):
        if not armUp: servo.angle(servo.SV5, ARMMED)
        else: servo.angle(servo.SV5, ARMDOWN)
        armUp = not armUp

    if pressed_once("triangle", bluepad32.triangle()): lift(1)
    if pressed_once("cross", bluepad32.cross()): lift(-1)

gamepad_state = True
started = False
team_switch = 0

def configuration():
    global gamepad_state, team_switch, started

    if pressed_once("circle", bluepad32.circle()):
        if team_switch == 0 or team_switch == 2:
            rgbled_board.fill("#67cfff")
            team_switch = 1
        else:
            rgbled_board.fill("#ff6767")
            team_switch = 2
        rgbled_board.show()

    if team_switch != 0:
        if pressed_once("square", bluepad32.square()):
            started = True
        return

    connection = bluepad32.is_connected()
    if connection != gamepad_state:
        gamepad_state = connection
        rgbled_board.set_color(2, "#6aff67" if gamepad_state else "#ff6767")
        rgbled_board.show()

def main():
    while uart.any(): uart.read()
    # while not started: configuration()
    while 1:
        task.update()
        get_imu()
        movement()
        control()

# -------------------- Start --------------------

print("initializing")

servo.angle(servo.SV6, RELEASE)
servo.angle(servo.SV5, ARMDOWN)
servo.angle(servo.SV4, HANGDOWN)

main()