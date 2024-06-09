import numpy as np
import imutils
import cv2
import RPi.GPIO as GPIO
import time
from time import sleep
from picamera2 import Picamera2

# Servo Initialization
panServo = 18
tiltServo = 16
panAngle = 90
tiltAngle = 90

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(panServo, GPIO.OUT)
GPIO.setup(tiltServo, GPIO.OUT)


def setServoAngle(servo, angle):
    assert 30 <= angle <= 150, "Angle must be between 30 and 150 degrees"
    pwm = GPIO.PWM(servo, 50)
    pwm.start(8)
    dutyCycle = angle / 18. + 3.
    pwm.ChangeDutyCycle(dutyCycle)
    sleep(0.3)
    pwm.stop()


# Position servos to initial position
setServoAngle(panServo, panAngle)
setServoAngle(tiltServo, tiltAngle)

# Relay Initialization
RelayPIN1 = 15
GPIO.setup(RelayPIN1, GPIO.OUT)
GPIO.output(RelayPIN1, GPIO.LOW)
relayOn = False


def mapServoPosition(x_axis, y_axis):
    global panAngle, tiltAngle

    if x_axis < 180:
        panAngle += 10
        panAngle = min(panAngle, 140)
        setServoAngle(panServo, panAngle)
    elif x_axis > 220:
        panAngle -= 10
        panAngle = max(panAngle, 40)
        setServoAngle(panServo, panAngle)

    if y_axis < 92:
        tiltAngle += 10
        tiltAngle = min(tiltAngle, 140)
        setServoAngle(tiltServo, tiltAngle)
    elif y_axis > 132:
        tiltAngle -= 10
        tiltAngle = max(tiltAngle, 40)
        setServoAngle(tiltServo, tiltAngle)


# Camera Setting
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
picam2.start()

print("[INFO] warming up...")
time.sleep(2.5)

firstFrame = None
motionCounter = 0
minmotion = 4
showvideo = "yes"

# Animal color Range RGB
lower = np.array([65, 49, 28], dtype="uint8")
upper = np.array([187, 159, 174], dtype="uint8")
# Capture frames from the camera
try:
    while True:
        frame = picam2.capture_array()
        text = "No Motion"
        framecopy = frame.copy()
        frame = imutils.resize(frame, width=400)
        gray1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray1, (17, 17), 0)

        if firstFrame is None or motionCounter == 200:
            motionCounter = 0
            firstFrame = gray.copy()
            continue

        frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(firstFrame))
        thresh = cv2.threshold(frameDelta, 5, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        cnts, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in cnts:
            if cv2.contourArea(c) < 600:
                continue
            (x, y, w, h) = cv2.boundingRect(c)
            ccc = framecopy[y: y + h, x: x + w].copy()
            mask = cv2.inRange(ccc, lower, upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 1))
            detectarea = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(detectarea.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            for d in contours:
                if cv2.contourArea(d) < 3000:
                    continue
                text1 = "Detected"
                cv2.putText(frame, f"Animal: {text1}", (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                            (0, 0, 255), 1)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                x_axis = int(x + w / 2)
                y_axis = int(y + h / 2)
                cv2.circle(frame, (x_axis, y_axis), 5, (0, 0, 255), -1)

                motionCounter += 1

                if motionCounter > minmotion:
                    minmotion = 0
                    mapServoPosition(x_axis, y_axis)
                    if not relayOn:
                        GPIO.output(RelayPIN1, GPIO.HIGH)
                        relayOn = True

                    print("Servo Working")

            text = "Motion Detected"

        cv2.putText(frame, f"Status: {text}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        if showvideo == "yes":
            cv2.imshow("Security Feed", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

finally:
    GPIO.output(RelayPIN1, GPIO.LOW)
    GPIO.cleanup()
    cv2.destroyAllWindows()
    picam2.stop()
