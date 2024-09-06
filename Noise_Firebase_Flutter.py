import sounddevice as sd
import numpy as np
import threading
import firebase_admin
from firebase_admin import credentials, db
import smbus
from time import sleep
import math
import time
from datetime import datetime
import RPi.GPIO as GPIO
from collections import deque

# 최근 가속도 값 저장을 위한 큐
acc_queue_1 = deque(maxlen=5)  # 최근 10개의 가속도 값만 저장
acc_queue_2 = deque(maxlen=5)

def calculate_vibration_from_queue(acc_queue):
    if len(acc_queue) < 2:
        return 0

    vibration_sum = 0
    for i in range(len(acc_queue) - 1):
        ax1, ay1, az1 = acc_queue[i]
        ax2, ay2, az2 = acc_queue[i + 1]
        delta_ax = ax2 - ax1
        delta_ay = ay2 - ay1
        delta_az = az2 - az1
        vibration_sum += math.sqrt(delta_ax ** 2 + delta_ay ** 2 + delta_az ** 2)

    return vibration_sum / (len(acc_queue) - 1)

"""
# GPIO 핀 설정
LED_R = 22
LED_G = 27
LED_B = 17

# GPIO 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)  # B
GPIO.setup(27, GPIO.OUT)  # G
GPIO.setup(22, GPIO.OUT)  # R
BUZZER_PIN = 18 #부저
GPIO.setup(BUZZER_PIN, GPIO.OUT)

buzzer = GPIO.PWM(BUZZER_PIN, 1)  # 부저 PWM 인스턴스 생성

def beep(frequency, duration):
    buzzer.ChangeFrequency(frequency)
    buzzer.start(50)  # 듀티 사이클 50%로 설정
    time.sleep(duration)
    buzzer.stop()
"""
# Firebase 초기화
cred = credentials.Certificate("/home/pi/testkey.json")
name = firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://whfdjq-64525-default-rtdb.firebaseio.com/'
})
ref = db.reference('/')

# 레지스터 주소 정의
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
ACCEL_XOUT_H_2 = 0x3B
ACCEL_YOUT_H_2 = 0x3D
ACCEL_ZOUT_H_2 = 0x3F
GYRO_XOUT_H_2 = 0x43
GYRO_YOUT_H_2 = 0x45
GYRO_ZOUT_H_2 = 0x47

# 센서 데이터를 저장할 리스트 초기화
mic1_data = []
mic2_data = []
mpu1_data = []
mpu2_data = []

def update_firebase(vibration_int_1, vibration_int_2):
    ref.update({'MPU1': vibration_int_1, 'MPU2': vibration_int_2})

def MPU_Init(Device_Address_1):
    bus.write_byte_data(Device_Address_1, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address_1, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address_1, CONFIG, 0)
    bus.write_byte_data(Device_Address_1, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address_1, INT_ENABLE, 1)

def read_raw_data(Device_Address_1, addr):
    high = bus.read_byte_data(Device_Address_1, addr)
    low = bus.read_byte_data(Device_Address_1, addr + 1)
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

bus = smbus.SMBus(1)
Device_Address_1 = 0x68
MPU_Init(Device_Address_1)
Device_Address_2 = 0x69
MPU_Init(Device_Address_2)

acc_x = read_raw_data(Device_Address_1, ACCEL_XOUT_H)
acc_y = read_raw_data(Device_Address_1, ACCEL_YOUT_H)
acc_z = read_raw_data(Device_Address_1, ACCEL_ZOUT_H)
last_ax = acc_x / 16384.0
last_ay = acc_y / 16384.0
last_az = acc_z / 16384.0

acc_x_2 = read_raw_data(Device_Address_2, ACCEL_XOUT_H_2)
acc_y_2 = read_raw_data(Device_Address_2, ACCEL_YOUT_H_2)
acc_z_2 = read_raw_data(Device_Address_2, ACCEL_ZOUT_H_2)
last_ax_2 = acc_x_2 / 16384.0
last_ay_2 = acc_y_2 / 16384.0
last_az_2 = acc_z_2 / 16384.0

last_time = time.time()

def calculate_vibration(ax, ay, az, last_ax, last_ay, last_az, delta_t):
    delta_ax = (ax - last_ax) / delta_t
    delta_ay = (ay - last_ay) / delta_t
    delta_az = (az - last_az) / delta_t

    return math.sqrt(delta_ax**2 + delta_ay**2 + delta_az**2)


# 1분마다 평균값 계산 및 Firebase 업로드 함수
def calculate_and_upload_avg():
    prev_minute = datetime.now().minute  # 현재 분 저장
    while True:
        current_minute = datetime.now().minute  # 현재 분 확인
        if current_minute != prev_minute:  # 현재 분이 이전과 다를 때만 실행
            # 평균값 계산
            mic_avg = int(sum(mic1_data) / len(mic1_data)) if mic1_data else 0
            mic_avg_2 = int(sum(mic2_data) / len(mic2_data)) if mic2_data else 0
            mpu1_avg = int(sum(mpu1_data) / len(mpu1_data)) if mpu1_data else 0
            mpu2_avg = int(sum(mpu2_data) / len(mpu2_data)) if mpu2_data else 0

            # Firebase에 평균값 업로드
            ref.update({'AVG/MIC1': mic_avg, 'AVG/MIC2': mic_avg_2, 'AVG/MPU1': mpu1_avg, 'AVG/MPU2': mpu2_avg})

            # 리스트 초기화
            mic1_data.clear()
            mic2_data.clear()
            mpu1_data.clear()
            mpu2_data.clear()
            
            prev_minute = current_minute  # 현재 분을 이전 분으로 갱신

        time.sleep(1)  # 1초 대기


# 1분마다 평균값 계산 및 Firebase 업로드 함수
def calculate_and_upload_avg():
    prev_minute = datetime.now().minute  # 현재 분 저장
    while True:
        current_minute = datetime.now().minute  # 현재 분 확인
        if current_minute != prev_minute:  # 현재 분이 이전과 다를 때만 실행
            # 평균값 계산
            mic_avg = int(sum(mic1_data) / len(mic1_data)) if mic1_data else 0
            mic_avg_2 = int(sum(mic2_data) / len(mic2_data)) if mic2_data else 0
            mpu1_avg = int(sum(mpu1_data) / len(mpu1_data)) if mpu1_data else 0
            mpu2_avg = int(sum(mpu2_data) / len(mpu2_data)) if mpu2_data else 0

            # Firebase에 평균값 업로드 (시간 태그 사용)
            current_time = datetime.now().strftime('%H:%M')
            ref.update({f'AVG/MIC1/{current_time}': mic_avg,
                        f'AVG/MIC2/{current_time}': mic_avg_2,
                        f'AVG/MPU1/{current_time}': mpu1_avg,
                        f'AVG/MPU2/{current_time}': mpu2_avg})

            # 리스트 초기화
            mic1_data.clear()
            mic2_data.clear()
            mpu1_data.clear()
            mpu2_data.clear()
            
            prev_minute = current_minute  # 현재 분을 이전 분으로 갱신

        time.sleep(1)  # 1초 대기

# 타이머 스레드 생성 및 시작
timer_thread = threading.Thread(target=calculate_and_upload_avg)
timer_thread.start()
"""
# LED 상태 변수
led_status = 'off'
led_thread = None

def alert(noise1, noise2, vibration_int_1, vibration_int_2):
    led_on = False
    if 7000 <= noise1 <= 10000 or 7000 <= noise2 <= 10000 or  2000<= vibration_int_1 <= 4000 or 2000 <= vibration_int_2 <= 4000:
        # 주황색 LED 켜기
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.LOW)
        buzzer_thread = threading.Thread(target=beep, args=(262,1))  # 주파수 262Hz, 3초 동안 울림
        buzzer_thread.start()
        led_on = True
    elif noise1 > 10000 or noise2 > 10000 or vibration_int_1 > 4000 or vibration_int_2 > 4000:
        # 빨간색 LED 켜기
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.LOW)
        buzzer_thread = threading.Thread(target=beep, args=(523,1))  # 주파수 523Hz, 3초 동안 울림
        buzzer_thread.start()
        led_on = True

    if led_on:
        # LED를 3초 동안 켜진 상태로 유지
        start_time = time.time()
        while time.time() - start_time < 2:
            pass

    # LED 끄기
    GPIO.output(LED_R, GPIO.LOW)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.LOW)

def start_alert_thread(noise1, noise2, vibration_int_1, vibration_int_2):
    led_thread = threading.Thread(target=alert, args=(noise1, noise2, vibration_int_1, vibration_int_2))
    led_thread.start()
"""
def upload_noise1_history(noise1):
    if noise1 >= 30000:
        current_date = datetime.now().date().strftime('%Y-%m-%d')
        current_time = datetime.now().strftime('%H:%M:%S')
        ref.update({f"history/mic1/{current_date}/{current_time}": noise1})

def upload_noise2_history(noise2):
    if noise2 >= 30000:
        current_date = datetime.now().date().strftime('%Y-%m-%d')
        current_time = datetime.now().strftime('%H:%M:%S')
        ref.update({f"history/mic2/{current_date}/{current_time}": noise2})

def upload_vib1_history(vibration_int_1):
    if vibration_int_1 >= 4000:
        current_date = datetime.now().date().strftime('%Y-%m-%d')
        current_time = datetime.now().strftime('%H:%M:%S')
        ref.update({f"history/vib1/{current_date}/{current_time}": vibration_int_1})

def upload_vib2_history(vibration_int_2):
    if vibration_int_2 >= 4000:
        current_date = datetime.now().date().strftime('%Y-%m-%d')
        current_time = datetime.now().strftime('%H:%M:%S')
        ref.update({f"history/vib2/{current_date}/{current_time}": vibration_int_2})


# 첫번째 소음 수준을 측정하는 함수를 정의합니다.
def measure_noise_1(indata, frames):
    noise1 = int(np.linalg.norm(indata) * 1000)  # noise 정수로 변환합니다.
    print("Noise1 Level:", noise1)
    mic1_data.append(noise1) # 수집한 데이터를 리스트에 추가

    # Firebase Realtime Database에 noise 값을 업로드하는 작업을 별도의 스레드에서 실행합니다.
    threading.Thread(target=upload_to_firebase, args=(noise1,)).start()
    threading.Thread(target=upload_noise1_history, args=(noise1,)).start()

# 두번째 노이즈를 측정하는 함수를 정의합니다.
def measure_noise_2(indata, frames):
    noise2 = int(np.linalg.norm(indata) * 1000)  # noise 정수로 변환합니다.
    print("Noise2 Level:", noise2)
    mic2_data.append(noise2) # 수집한 데이터를 리스트에 추가

    # Firebase Realtime Database에 noise 값을 업로드하는 작업을 별도의 스레드에서 실행합니다.
    threading.Thread(target=upload_to_firebase_2, args=(noise2,)).start()
    threading.Thread(target=upload_noise2_history, args=(noise2,)).start()

# Firebase Realtime Database에 noise 값을 업로드하는 함수를 정의합니다.
def upload_to_firebase(noise1):
    ref.update({'MIC1': noise1})

# Firebase Realtime Database에 noise 값을 업로드하는 함수를 정의합니다.
def upload_to_firebase_2(noise2):
    ref.update({'MIC2': noise2})

# 마이크를 위한 스트림을 생성합니다. 카드 번호를 지정합니다.
stream = sd.InputStream(device=1)
stream_2 = sd.InputStream(device=2)

# 마이크 스트림을 시작합니다.
stream.start()
stream_2.start()

while True:  # 무한 루프를 생성합니다.
    acc_x = read_raw_data(Device_Address_1, ACCEL_XOUT_H)
    acc_y = read_raw_data(Device_Address_1, ACCEL_YOUT_H)
    acc_z = read_raw_data(Device_Address_1, ACCEL_ZOUT_H)
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0

    acc_x_2 = read_raw_data(Device_Address_2, ACCEL_XOUT_H_2)
    acc_y_2 = read_raw_data(Device_Address_2, ACCEL_YOUT_H_2)
    acc_z_2 = read_raw_data(Device_Address_2, ACCEL_ZOUT_H_2)
    Ax_2 = acc_x_2 / 16384.0
    Ay_2 = acc_y_2 / 16384.0
    Az_2 = acc_z_2 / 16384.0

    current_time = time.time()
    delta_t = current_time - last_time
    
    # 큐에 가속도 값 추가
    acc_queue_1.append((Ax, Ay, Az))
    acc_queue_2.append((Ax_2, Ay_2, Az_2))

    # 큐에 있는 가속도 값들의 변화량을 계산하여 진동값 결정
    vibration_1 = calculate_vibration_from_queue(acc_queue_1)
    vibration_int_1 = int(1000 * vibration_1)
    vibration_int_1 = min(1, vibration_int_1) if vibration_int_1 >= 40000 else vibration_int_1

    vibration_2 = calculate_vibration_from_queue(acc_queue_2)
    vibration_int_2 = int(1000 * vibration_2)
    vibration_int_2 = min(1, vibration_int_2) if vibration_int_2 >= 40000 else vibration_int_2

    print("MPU1 : %d" % vibration_int_1)
    print("MPU2 : %d" % vibration_int_2)

    indata, frames = stream.read(int(stream.samplerate * 0.1))
    indata_2, frames_2 = stream_2.read(int(stream_2.samplerate * 0.1))  # 추가된 마이크 데이터 읽기
    measure_noise_1(indata, frames)  # 첫 번째 마이크 데이터 처리
    measure_noise_2(indata_2, frames_2)  # 추가된 마이크 데이터 처리
    noise1 = int(np.linalg.norm(indata) * 1000)
    noise2 = int(np.linalg.norm(indata_2) * 1000)

    #start_alert_thread(noise1, noise2, vibration_int_1, vibration_int_2)

    # 수집한 데이터를 리스트에 추가
    mpu1_data.append(vibration_int_1)
    mpu2_data.append(vibration_int_2)

    # 잔동 값을 Firebase Realtime Database 업로드를 위한 스레드 생성
    firebase_thread = threading.Thread(target=update_firebase,args=(vibration_int_1, vibration_int_2))
    firebase_thread.start()

    threading.Thread(target=upload_vib1_history, args=(vibration_int_1,)).start()
    threading.Thread(target=upload_vib2_history, args=(vibration_int_2,)).start()

    last_ax, last_ay, last_az = Ax, Ay, Az
    last_ax_2, last_ay_2, last_az_2 = Ax_2, Ay_2, Az_2
    last_time = current_time

    sleep(0.1)
