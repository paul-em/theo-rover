SYSTEM_THREAD(ENABLED);

// Motor 1 (Left)
int ena = A5;
int in1 = D2;
int in2 = D3;

// Motor 2 (Right)
int enb = A4;
int in3 = D5;
int in4 = D4;

// HC-SR04 - power: many boards are unreliable on 3.3 V (weak / no echo). Prefer VIN (~5 V
// when USB-powered) + common GND; then Echo must see <=3.3 V (divider), Trig from Photon OK at 3.3 V.
int trigPin = D6;
int echoPin = D7;

unsigned long lastCommandTime = 0;
const unsigned long SAFETY_TIMEOUT = 2000;
bool motorsRunning = false;

unsigned long turnStopTime = 0;
bool turnPending = false;
const unsigned long TURN_DURATION = 400;

// Auto mode
bool autoMode = true;
enum AutoState { AUTO_FORWARD, AUTO_REVERSING, AUTO_TURNING };
AutoState autoState = AUTO_FORWARD;
unsigned long autoStateStart = 0;
const int OBSTACLE_DISTANCE_CM = 30;
const int AUTO_SPEED = 180;
const unsigned long REVERSE_DURATION = 500;
const unsigned long TURN_DURATION_AUTO = 600;
int autoTurnDirection = 0; // 0 = left, 1 = right

double lastDistanceCm = -1;

unsigned long lastUltrasonicMs = 0;
// HC-SR04 needs ~60 ms between triggers; also keeps loop free for cloud I/O.
const unsigned long ULTRASONIC_INTERVAL_MS = 100;
// ~2 cm round-trip ~= 116 us; shorter pulses are almost always noise / Trig coupling.
const uint32_t ECHO_MIN_VALID_US = 100;
// Ignore Echo for a few us after Trig (crosstalk); keep small so close range still works.
const uint32_t ECHO_BLANK_US = 25;

// Bounded echo measure: unsigned elapsed time only (avoids micros() deadline bugs).
// Skip short glitches; keep trying until total budget expires (no reflection / out of range).
static uint32_t measureEchoPulseMicros() {
    const uint32_t pulseWidthMaxUs = 26000;
    const uint32_t windowUs = 50000;

    uint32_t windowStart = micros();

    for (;;) {
        if (micros() - windowStart >= windowUs) {
            return 0;
        }
        while (digitalRead(echoPin) == LOW) {
            if (micros() - windowStart >= windowUs) {
                return 0;
            }
        }
        uint32_t pulseStart = micros();
        while (digitalRead(echoPin) == HIGH) {
            if (micros() - pulseStart > pulseWidthMaxUs) {
                return 0;
            }
            if (micros() - windowStart >= windowUs) {
                return 0;
            }
        }
        uint32_t width = micros() - pulseStart;
        if (width >= ECHO_MIN_VALID_US) {
            return width;
        }
    }
}

void setup() {
    pinMode(ena, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enb, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    Particle.function("motor", motorControl);
}

void sampleUltrasonic() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    delayMicroseconds(ECHO_BLANK_US);

    uint32_t duration = measureEchoPulseMicros();
    if (duration == 0) {
        lastDistanceCm = -1;
    } else {
        lastDistanceCm = duration / 58.0;
    }
}

void runAutoMode() {
    long dist = (lastDistanceCm >= 0) ? (long)(lastDistanceCm + 0.5) : -1;

    switch (autoState) {
        case AUTO_FORWARD:
            if (dist >= 0 && dist < OBSTACLE_DISTANCE_CM) {
                // Obstacle detected - reverse
                setMotor(ena, in1, in2, "reverse", AUTO_SPEED);
                setMotor(enb, in3, in4, "reverse", AUTO_SPEED);
                autoState = AUTO_REVERSING;
                autoStateStart = millis();
                Particle.publish("auto/obstacle", String(dist), PRIVATE);
            } else {
                setMotor(ena, in1, in2, "forward", AUTO_SPEED);
                setMotor(enb, in3, in4, "forward", AUTO_SPEED);
            }
            break;

        case AUTO_REVERSING:
            if (millis() - autoStateStart >= REVERSE_DURATION) {
                autoTurnDirection = random(2);
                if (autoTurnDirection == 0) {
                    setMotor(ena, in1, in2, "reverse", AUTO_SPEED);
                    setMotor(enb, in3, in4, "forward", AUTO_SPEED);
                } else {
                    setMotor(ena, in1, in2, "forward", AUTO_SPEED);
                    setMotor(enb, in3, in4, "reverse", AUTO_SPEED);
                }
                autoState = AUTO_TURNING;
                autoStateStart = millis();
            }
            break;

        case AUTO_TURNING:
            if (millis() - autoStateStart >= TURN_DURATION_AUTO) {
                autoState = AUTO_FORWARD;
            }
            break;
    }
}

void loop() {
    if (millis() - lastUltrasonicMs >= ULTRASONIC_INTERVAL_MS) {
        lastUltrasonicMs = millis();
        sampleUltrasonic();
    }

    if (autoMode) {
        runAutoMode();
        lastCommandTime = millis();
        motorsRunning = true;
    }

    if (turnPending && millis() >= turnStopTime) {
        stopMotors();
        motorsRunning = false;
        turnPending = false;
    }
    if (motorsRunning && millis() - lastCommandTime > SAFETY_TIMEOUT) {
        stopMotors();
        motorsRunning = false;
    }

    if (!Particle.connected()) {
        static uint32_t lastConnectionAttempt = 0;
        if (millis() - lastConnectionAttempt > 30000) {
            WiFi.off();
            delay(100);
            WiFi.on();
            Particle.connect();
            lastConnectionAttempt = millis();
        }
    }
}

void stopMotors() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(ena, 0);
    analogWrite(enb, 0);
}

void setMotor(int enPin, int fwdPin, int revPin, String action, int speed) {
    if (action == "forward") {
        digitalWrite(fwdPin, HIGH);
        digitalWrite(revPin, LOW);
        analogWrite(enPin, speed);
    } else if (action == "reverse") {
        digitalWrite(fwdPin, LOW);
        digitalWrite(revPin, HIGH);
        analogWrite(enPin, speed);
    } else {
        analogWrite(enPin, 0);
    }
}

int motorControl(String command) {
    lastCommandTime = millis();

    int commaIndex = command.indexOf(',');
    String action = command;
    int speed = 150;

    if (commaIndex > 0) {
        action = command.substring(0, commaIndex);
        speed = command.substring(commaIndex + 1).toInt();
        speed = constrain(speed, 0, 255);
    }

    if (action == "auto") {
        autoMode = !autoMode;
        if (autoMode) {
            autoState = AUTO_FORWARD;
            Particle.publish("auto/state", "on", PRIVATE);
        } else {
            stopMotors();
            motorsRunning = false;
            Particle.publish("auto/state", "off", PRIVATE);
        }
        return autoMode ? 1 : 0;
    }

    // Any manual command exits auto mode
    if (autoMode) {
        autoMode = false;
        Particle.publish("auto/state", "off", PRIVATE);
    }

    if (action == "forward") {
        setMotor(ena, in1, in2, "forward", speed);
        setMotor(enb, in3, in4, "forward", speed);
        motorsRunning = true;
        return speed;
    }
    if (action == "reverse") {
        setMotor(ena, in1, in2, "reverse", speed);
        setMotor(enb, in3, in4, "reverse", speed);
        motorsRunning = true;
        return speed;
    }
    if (action == "left") {
        setMotor(ena, in1, in2, "reverse", speed);
        setMotor(enb, in3, in4, "forward", speed);
        motorsRunning = true;
        turnPending = true;
        turnStopTime = millis() + TURN_DURATION;
        return speed;
    }
    if (action == "right") {
        setMotor(ena, in1, in2, "forward", speed);
        setMotor(enb, in3, in4, "reverse", speed);
        motorsRunning = true;
        turnPending = true;
        turnStopTime = millis() + TURN_DURATION;
        return speed;
    }
    if (action == "stop") {
        stopMotors();
        motorsRunning = false;
        return 0;
    }
    return -1;
}
