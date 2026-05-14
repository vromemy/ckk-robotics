
#include <PTBOTAtomVX.h>
#include <Bluepad32.h>
#include <unordered_map>
#include <functional>
#include <string>

//SV1
#define RELEASE 120
#define PICK 90

//SV2
#define ARM_DOWN 10
#define ARM_MED 50
#define ARM_UP 120

//SV3
#define HANG_UP 160
#define HANG_MED 120
#define HANG_PICK 130
#define HANG_DOWN 90

typedef struct _ControllerData {
    int8_t index, l1, r1, up, down, left, right, triangle, cross, square, circle; // 0, 1 boolean
    double lx, ly, rx, ry, l2, r2; // 0.00 - 1.00 range
} ControllerData_t;

typedef struct _PIDController {
    double kp, ki, kd, integral, prev_err;
    uint32_t prev_time;
} PIDController_t;

typedef struct _TaskManager {
    std::function<void()> func;
    uint32_t triggerTime, interval;
    bool repeat, active;
} Tasks_t;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
std::unordered_map<std::string, Tasks_t> tasks;
ControllerData_t controller;
PIDController_t pid;

void taskCreate(const std::string& name, std::function<void()> func, uint32_t delay, bool repeat = false) {
    tasks[name] = {
        .func = func,
        .triggerTime = millis() + delay,
        .interval = delay,
        .repeat = repeat,
        .active = true
    };
}

void taskUpdate() {
    uint32_t now = millis();
    for (auto& [name, task] : tasks) {
        if (!task.active) continue;
        if ((now - task.triggerTime) >= 0) {
            task.func();
            if (task.repeat) task.triggerTime += task.interval;
            else task.active = false;
        }
    }
}

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (!myControllers[i]) {
            myControllers[i] = ctl;
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            return;
        }
    }
}

void getControllerData(ControllerData_t* ControllerData) {
    if (!BP32.update()) return;
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData() && ctl->isGamepad()) {
            uint8_t buffdpad = ctl->dpad();
            uint8_t buffbutton = ctl->buttons();

            ControllerData->index = ctl->index();
            ControllerData->lx = (double) ctl->axisX() / 512.0;
            ControllerData->ly = (double) ctl->axisY() / 512.0;
            ControllerData->rx = (double) ctl->axisRX() / 512.0;
            ControllerData->ry = (double) ctl->axisRY() / 512.0;
            ControllerData->l1 = ctl->l1();
            ControllerData->r1 = ctl->r1();
            ControllerData->l2 = (double) ctl->brake() / 1020.0;
            ControllerData->r2 = (double) ctl->throttle() / 1020.0;
            ControllerData->up = (buffdpad >> 0) & 1;
            ControllerData->down = (buffdpad >> 1) & 1;
            ControllerData->left = (buffdpad >> 3) & 1;
            ControllerData->right = (buffdpad >> 2) & 1;
            ControllerData->triangle = (buffbutton >> 3) & 1;
            ControllerData->cross = (buffbutton >> 0) & 1;
            ControllerData->square = (buffbutton >> 2) & 1;
            ControllerData->circle = (buffbutton >> 1) & 1;
        }
    }
}

inline void grip(int deg) { servoWrite(1, deg); }
inline void arm(int deg)  { servoWrite(2, deg); }
inline void hang(int deg) { servoWrite(3, deg); }
double maxf64(double val1, double val2) { return (val1 > val2) ? val1 : val2; }
double minf64(double val1, double val2) { return (val1 < val2) ? val1 : val2; }
double clampf64(double val, double min, double max) { return maxf64(minf64(val, max), min); }

double wrapRads(double val) {
    while (val > PI) val -= PI*2;
    while (val < -PI) val += PI*2;
    return val;
}

bool pressOnce(const std::string& name, bool current) {
    static std::unordered_map<std::string, bool> buttonStates;
    bool clicked = current && !buttonStates[name];
    buttonStates[name] = current;
    
    return clicked;
}

double yawOffset = 0.0;
double setpoint = 0.0;
double getYaw() { return wrapRads((angleRead(YAW) - yawOffset) * PI / 180.0); }

double calculatePID(PIDController_t* pid, double setpoint, double current) {
    uint32_t now = millis();
    double dt = (now - pid->prev_time) / 1000.0;
    pid->prev_time = now;
    
    if (dt <= 0 || dt > 0.1) dt = 0.01;

    double error = wrapRads(setpoint - current);

    pid->integral += error * dt;
    pid->integral = clampf64(pid->integral, -1, 1);

    double derivative = (error - pid->prev_err) / dt;
    pid->prev_err = error;
    
    return clampf64(pid->kp * error + pid->ki * pid->integral + pid->kd * derivative, -1, 1);
}

double linearHeading(double yaw) {
    static int last_rx = 0; 
    double rx = controller.rx;
    int now = millis();

    if (controller.r2) rx = clampf64(rx, -0.35, 0.35);

    double rotation = calculatePID(&pid, setpoint, yaw);
    if (fabs(wrapRads(setpoint - yaw)) < 0.01) rotation = 0;
    if (fabs(rx) > 0.1 || now - last_rx < 300) {
        rotation = rx;
        setpoint = yaw;
    }
    if (fabs(rx) > 0.1) last_rx = now;
    return rotation;
}

double speedControl(double value, double onControlled) {
    double absVal = fabs(value);
    if (controller.r2) return value * onControlled;
    if (absVal <= 0.1) return 0;
    if (absVal <= 0.4) return value * 0.35;
    return value;
}

void control() {
    static bool pickUp = false;
    static bool armUp = false;
    static bool artifactUp = false;

    if (pressOnce("reset", controller.square)) {
        yawOffset = angleRead(YAW);
        setpoint = 0.0;
    }

    if (pressOnce("r1", controller.r1)) {
        if (!artifactUp) {
            int pickDelay = (pickUp) ? 100 : 0;
            grip(PICK);
            taskCreate("sq1", []() {
                arm(ARM_UP);
            }, pickDelay);

            taskCreate("sq2", []() {
                hang(HANG_MED);
            }, 175 + pickDelay);

            taskCreate("sq3", []() {
                grip(RELEASE);
            }, 275 + pickDelay);

            taskCreate("sq4", []() {
                hang(HANG_UP);
            }, 300 + pickDelay);

            taskCreate("sq5", []() {
                arm(ARM_DOWN);
            }, 300 + pickDelay);

            pickUp = false;
            armUp = false;
        } else {
            hang(HANG_PICK);
            arm(ARM_UP);

            taskCreate("sq6", []() {
                grip(PICK);
            }, 200);

            taskCreate("sq7", []() {
                hang(HANG_DOWN);
            }, 250);

            taskCreate("sq8", []() {
                arm(ARM_DOWN);
            }, 300);

            pickUp = true;
            armUp = false;
        }

        artifactUp = not artifactUp;
    }
}

void movement() {
    double yaw = getYaw();

    double lx = speedControl(controller.lx, 0.35);
    double ly = -speedControl(controller.ly, 0.25);

    double x = (cos(yaw) * lx) - (sin(yaw) * ly);
    double y = (sin(yaw) * lx) + (cos(yaw) * ly);
    double r = linearHeading(yaw);
    double d = maxf64(fabs(x) + fabs(y) + fabs(r), 1);

    double fl = (((y + x - r) / d));
    double fr = (((y - x + r) / d));
    double rl = (((y - x - r) / d));
    double rr = (((y + x + r) / d));

    motorWrite(1, fl * 100);
    motorWrite(2, fr * 100);
    motorWrite(3, rl * 100);
    motorWrite(4, rr * 100);
}

void setup() {
    Serial.begin(115200);
    initialize();
    // calibrateIMU();

    BP32.setup(&onConnectedController, &onDisconnectedController);

    pid.kp = 1.5;
    pid.kd = 0.15;

    arm(ARM_DOWN);
    grip(RELEASE);
    hang(HANG_DOWN);
}

void loop() {
    getControllerData(&controller);
    taskUpdate();
    control();
    movement();
}
