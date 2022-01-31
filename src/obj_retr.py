import os
import smtplib
import time
from datetime import datetime
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

import RPi.GPIO as gpio
import cv2
import numpy as np
import serial

smtpUser = 'enter your email'
smtpPass = 'enter your password'

cap = cv2.VideoCapture(0)
time.sleep(0.1)

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('video.avi', fourcc, 10, (640, 480))

m1 = "picked"
m2 = "delivered"

kp = 10
kd = 100
ki = 0.07
dia = 6.5
ser = serial.Serial('/dev/ttyUSB0', 9600)
xlist = list()
ylist = list()

upper = np.array([179, 213, 163])
lower = np.array([0, 140, 64])


def send_email(m):
    pic_time = datetime.now().strftime('%Y%m%d%H%M%S')
    command = 'raspistill -w 640 -h 480 -vf -hf -o ' + pic_time + '.jpg'
    os.system(command)

    toAdd = 'recipient email'
    cc = 'cc1 email (separated by a comma), cc2 email'
    fromAdd = smtpUser
    subject = 'Vial ' + m + ' at ' + pic_time
    rec = cc.split(",") + [toAdd]
    msg = MIMEMultipart()
    msg['Subject'] = subject
    msg['From'] = fromAdd
    msg['To'] = toAdd
    msg['CC'] = cc

    msg.preamble = "Image recorded at " + pic_time
    body = MIMEText("Vial " + m + " by Rajan's Barron at " + pic_time)
    msg.attach(body)

    fp = open(pic_time + '.jpg', 'rb')
    img = MIMEImage(fp.read())
    fp.close()
    msg.attach(img)

    s = smtplib.SMTP('smtp.gmail.com', 587)

    s.ehlo()
    s.starttls()
    s.ehlo()

    s.login(smtpUser, smtpPass)
    s.sendmail(fromAdd, rec, msg.as_string())
    s.quit()
    print('Email sent')


def imu():
    count = 0

    while True:

        if (ser.in_waiting > 0):

            count += 1
            line = ser.readline()
            if count > 10:

                line = line.rstrip().lstrip()

                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                line = float(line)

                print(line)
                return line


def next_time():
    line = ser.readline()
    line = line.rstrip().lstrip()
    line = str(line)
    line = line.strip("'")
    line = line.strip("b'")
    try:
        line = float(line)
    except:
        return next_time()
    return line


def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)
    gpio.setup(7, gpio.IN, pull_up_down=gpio.PUD_UP)
    gpio.setup(12, gpio.IN, pull_up_down=gpio.PUD_UP)


def gameover():
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)


def motorcontrol(desired_speed, error):
    ierr = 0
    preverror = 0
    ierr = error + ierr
    motor_speed_l = desired_speed - (kp * error + kd * (preverror - error) + ki * ierr)
    preverror = error
    ierr = error + ierr
    motor_speed_r = desired_speed + (kp * error + kd * (preverror - error) + ki * ierr)
    preverror = error
    return motor_speed_l, motor_speed_r


def right(diff_avg1):
    desired_speed = 75

    current_orient = imu()
    new_orient = current_orient + (0.061 * abs(diff_avg1))
    if new_orient > 360:
        new_orient = new_orient - 360
    init()
    pwm1 = gpio.PWM(31, 50)  # BackRight motor
    pwm2 = gpio.PWM(35, 50)  # FrontLeft motor
    val = desired_speed
    if current_orient < new_orient - 2.5 or current_orient > new_orient + 2.5:
        pwm1.start(val)
        pwm2.start(val)
    while True:

        current_orient1 = next_time()
        if new_orient - 3 <= current_orient1 <= new_orient + 3:
            pwm1.stop()
            pwm2.stop()
            gameover()
            break
    gpio.cleanup()


def left(diff_avg1):
    desired_speed = 75

    current_orient = imu()
    new_orient = current_orient - (0.061 * abs(diff_avg1))
    if new_orient < 0:
        new_orient = new_orient + 360
    init()
    pwm1 = gpio.PWM(33, 50)  # BackRight motor
    pwm2 = gpio.PWM(37, 50)  # FrontLeft motor
    val = desired_speed
    if current_orient < new_orient - 2.5 or current_orient > new_orient + 2.5:
        pwm1.start(val)
        pwm2.start(val)
    while True:

        current_orient1 = next_time()
        if new_orient - 3 <= current_orient1 <= new_orient + 3:
            pwm1.stop()
            pwm2.stop()
            gameover()
            break

    gpio.cleanup()


def forward(x):
    theta = next_time()
    desired_speed = 50
    ticks = 19 * (x / (np.pi * int(dia)))
    init()
    counterBR = np.uint64(0)
    counterFL = np.uint64(0)
    buttonBR = int(0)
    buttonFL = int(0)
    pwm1 = gpio.PWM(33, 50)  # BackRight motor
    pwm2 = gpio.PWM(35, 50)  # FrontLeft motor
    val = desired_speed
    pwm1.start(val)
    pwm2.start(val)
    time.sleep(0)
    while True:
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        if int(gpio.input(7)) != int(buttonFL):
            buttonFL = int(gpio.input(7))
            counterFL += 1
        error = counterBR - counterFL

        motor_speed_l, motor_speed_r = motorcontrol(desired_speed, error)

        if motor_speed_l > 100:
            motor_speed_l = 100
        if motor_speed_r > 100:
            motor_speed_r = 100

        if motor_speed_l < 0:
            motor_speed_l = 0
        if motor_speed_r < 0:
            motor_speed_r = 0

        pwm1.ChangeDutyCycle(motor_speed_r)
        pwm2.ChangeDutyCycle(motor_speed_l)
        if counterBR >= ticks:
            pwm1.stop()
        if counterFL >= ticks:
            pwm2.stop()
        if counterBR >= ticks or counterFL >= ticks:
            gameover()
            break
    actual_dist = (counterBR * np.pi * int(dia)) / 19
    if xlist == []:
        xcoor = 0
        ycoor = 0
        xlist.append(0)
        ylist.append(0)
    else:
        xcoor = xlist[-1]
        ycoor = ylist[-1]
    newx = xcoor + (actual_dist * np.cos((theta * np.pi) / 180))
    newy = ycoor + (actual_dist * np.sin((theta * np.pi) / 180))
    xlist.append(newx)
    ylist.append(newy)
    gpio.cleanup()


def reverse(x):
    theta = next_time()
    desired_speed = 50
    ticks = 19 * (x / (np.pi * int(dia)))
    init()
    counterBR = np.uint64(0)
    counterFL = np.uint64(0)
    buttonBR = int(0)
    buttonFL = int(0)
    pwm1 = gpio.PWM(31, 50)  # BackRight motor
    pwm2 = gpio.PWM(37, 50)  # FrontLeft motor
    val = desired_speed
    pwm1.start(val)
    pwm2.start(val)
    time.sleep(0)
    while True:
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        if int(gpio.input(7)) != int(buttonFL):
            buttonFL = int(gpio.input(7))
            counterFL += 1

        error = counterBR - counterFL

        motor_speed_l, motor_speed_r = motorcontrol(desired_speed, error)

        if motor_speed_l > 100:
            motor_speed_l = 100
        if motor_speed_r > 100:
            motor_speed_r = 100

        if motor_speed_l < 0:
            motor_speed_l = 0
        if motor_speed_r < 0:
            motor_speed_r = 0

        pwm1.ChangeDutyCycle(motor_speed_r)
        pwm2.ChangeDutyCycle(motor_speed_l)
        if counterBR >= ticks:
            pwm1.stop()
        if counterFL >= ticks:
            pwm2.stop()
        if counterBR >= ticks or counterFL >= ticks:
            gameover()
            break

    actual_dist = (counterBR * np.pi * int(dia)) / 19
    if xlist == []:
        xcoor = 0
        ycoor = 0
        xlist.append(0)
        ylist.append(0)
    else:
        xcoor = xlist[-1]
        ycoor = ylist[-1]
    newx = xcoor + (actual_dist * np.cos((theta * np.pi) / 180))
    newy = ycoor + (actual_dist * np.sin((theta * np.pi) / 180))
    xlist.append(newx)
    ylist.append(newy)
    gpio.cleanup()


def loose(tf):
    gpio.setmode(gpio.BOARD)
    gpio.setup(36, gpio.OUT)
    pwm = gpio.PWM(36, 50)
    pwm.start(7)
    pwm.ChangeDutyCycle(13)
    time.sleep(tf)
    pwm.stop()
    gpio.output(36, False)
    gpio.cleanup()


def grab(tf):
    gpio.setmode(gpio.BOARD)
    gpio.setup(36, gpio.OUT)
    pwm = gpio.PWM(36, 50)
    pwm.start(7)
    pwm.ChangeDutyCycle(7)
    time.sleep(tf)
    pwm.stop()
    gpio.output(36, False)
    gpio.cleanup()


diff_lis = list()
count = 0
while True:
    d, img = cap.read()
    img = cv2.resize(img, (640, 480))
    img = cv2.flip(img, 0)
    img = cv2.flip(img, 1)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    cv2.waitKey(1)
    if count % 8 != 0:
        count += 1
        continue
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:5]
    try:
        cnt = contours[0]
    except:
        continue
    M = cv2.moments(cnt)

    (x, y), radius = cv2.minEnclosingCircle(cnt)
    center = (int(x), int(y))
    radius = int(radius)

    circle1 = cv2.circle(img, center, radius, (0, 0, 255), 3)
    circle2 = cv2.circle(img, center, 1, (0, 0, 255), 3)
    circle3 = cv2.circle(img, (280, 240), 1, (0, 0, 255), 3)
    circle4 = cv2.circle(img, (360, 240), 1, (0, 0, 255), 3)
    circle5 = cv2.circle(img, (320, 240), 1, (0, 0, 255), 3)

    diff_avg = 320 - center[0]
    if radius < 170:
        if diff_avg > 40:
            left(diff_avg)
        elif diff_avg < -40:
            right(diff_avg)

    if radius < 100:
        loose(1)
        forward(15)
        loose(1)
    elif radius < 172:
        loose(1)
        forward(1)
        loose(1)
    else:
        grab(1)
        cap.release()
        send_email(str(m1))
        right(2951)
        forward(5)
        loose(1)
        time.sleep(1)
        reverse(5)
        send_email(str(m2))
        left(2951)
        break

    diff_lis = list()
    count = 1
    cv2.imshow("result", img)
    cv2.waitKey(1)

out.release()
cv2.destroyAllWindows()
