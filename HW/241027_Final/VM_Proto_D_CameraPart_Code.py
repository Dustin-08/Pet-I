import machine
import os
import sdcard
from pyb import Pin, SPI
import uos
import sensor
import urequests
import network
import time
import json

# Wi-Fi 설정
SSID = "SW"  # 연결할 Wi-Fi 네트워크 이름
PASSWORD = "20240704"  # Wi-Fi 비밀번호

def connect_to_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)

    print("Wi-Fi 연결 중...")
    while not wlan.isconnected():
        machine.idle()  # 연결 대기

    print("Wi-Fi 연결 성공!")
    print("IP 주소:", wlan.ifconfig()[0])

# Wi-Fi에 연결
connect_to_wifi()

# 서버로부터 제어 신호 확인
def get_control_signal():
    url = "http://DevSe.gonetis.com:12478/get-signal"
    try:
        response = urequests.get(url)
        if response.status_code == 200:
            signal_data = response.json()
            return signal_data.get("signal", "0")  # 신호 값 반환, 없으면 기본값 '0'
    except Exception as e:
        print("제어 신호 수신 실패:", e)
    return "0"  # 기본값으로 '0' 반환

# SD 카드 핀 설정
cs = Pin("CS", Pin.OUT_OD)  # CS pin = PE11
spi = SPI(4, SPI.MASTER, baudrate=int(480000000 / 256), polarity=0, phase=0)

# SD 카드 초기화
try:
    sd = sdcard.SDCard(spi, cs)
    vfs = uos.VfsFat(sd)
    uos.mount(vfs, "/sd")
    print("SD 카드가 마운트되었습니다.")
except Exception as e:
    print("SD 카드 초기화 실패:", e)

# 카메라 초기화
sensor.reset()  # 카메라 모듈 초기화
sensor.set_pixformat(sensor.RGB565)  # 픽셀 포맷 설정 (RGB565 또는 GRAYSCALE)
sensor.set_framesize(sensor.QVGA)  # 프레임 사이즈 설정 (QVGA, VGA 등)
sensor.skip_frames(time=2000)  # 카메라 안정화 대기

# HTTP 요청을 통해 서버에 이미지 파일 전송 (재시도 로직 포함)
def send_image_to_server(file_path, retries=2):
    url = "http://DevSe.gonetis.com:12478/upload"  # 서버의 업로드 엔드포인트 URL
    headers = {"Content-Type": "image/jpeg"}

    for attempt in range(retries):
        try:
            # 파일을 열고 읽기
            with open(file_path, "rb") as file:
                file_data = file.read()

            # HTTP POST 요청으로 파일 전송
            response = urequests.post(url, data=file_data, headers=headers)

            # 서버 응답 확인
            if response.status_code == 200:
                print("파일 업로드 성공")
                break
            else:
                print(f"파일 업로드 실패, 상태 코드: {response.status_code}, 재시도: {attempt + 1}/{retries}")

        except OSError as e:
            print(f"서버 연결 오류: {e}, 재시도: {attempt + 1}/{retries}")
            time.sleep(2)  # 2초 대기 후 재시도

        if attempt == retries - 1:
            print("재시도 횟수 초과, 업로드 실패")

# 주기적으로 제어 신호 확인 및 이미지 캡처 및 전송
while True:
    control_signal = get_control_signal()
    print(f"현재 제어 신호: {control_signal}")

    if control_signal == "1":
        # 이미지 캡처
        img = sensor.snapshot()
        img_path = "/sd/captured_image.jpg"
        
        try:
            img.save(img_path, quality=90)  # quality 옵션을 통해 JPEG 품질 조정 (0-100, 기본값 90)
            print("이미지가 SD 카드에 JPEG 형식으로 저장되었습니다:", img_path)
        except Exception as e:
            print("이미지 저장 실패:", e)
            continue  # 오류 시 재시도하지 않고 다음 신호 확인으로 이동

        # SD 카드에 저장된 이미지를 서버로 전송
        send_image_to_server(img_path)
    
    time.sleep(5)  # 5초 대기 후 다시 신호 확인

# SD 카드 마운트 해제
try:
    uos.umount("/sd")
    print("SD 카드 마운트 해제 성공")
except Exception as e:
    print("SD 카드 마운트 해제 실패:", e)
