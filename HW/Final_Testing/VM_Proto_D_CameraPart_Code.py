import struct
import asyncio
import aioble
import bluetooth
import sensor, image, machine, sdcard, uos
from machine import LED
from pyb import Pin, SPI
from micropython import const

# UUIDs
_LED_CONTROL_UUID = bluetooth.UUID(0x1815)
_LED_CONTROL_CHAR_UUID = bluetooth.UUID(0x2A56)
_ADV_APPEARANCE_GENERIC_TAG = const(512)
_ADV_INTERVAL_MS = 250_000

# SD 카드 핀 설정
SCK_PIN = 9   # SCK 핀
MISO_PIN = 10  # MISO 핀
MOSI_PIN = 8   # MOSI 핀
CS_PIN = 7     # Chip Select 핀

# CS 핀 설정
cs = Pin("PG12", Pin.OUT_PP, Pin.PULL_UP)

# SD 카드 초기화
spi = machine.SPI(4)
sd = sdcard.SDCard(spi, machine.Pin(cs))

# 파일 시스템 마운트
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")

# 카메라 설정
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # RGB565 포맷 설정
sensor.set_framesize(sensor.QVGA)  # 해상도 설정
sensor.skip_frames(time=2000)  # 카메라 설정 적용 대기

# 블루투스 장치 설정
led = LED("LED_BLUE")

# 사진 촬영 카운트 초기화
photo_count = 0

# GATT 서버 설정
led_service = aioble.Service(_LED_CONTROL_UUID)
led_characteristic = aioble.Characteristic(led_service, _LED_CONTROL_CHAR_UUID, write=True, capture=True)
aioble.register_services(led_service)

# 사진 촬영 함수
def capture_image():
    global photo_count  # 전역 변수를 사용하여 카운트를 유지
    photo_count += 1  # 사진 촬영 시마다 카운트를 증가
    file_name = f"/sd/snapshot_{photo_count}.jpg"  # 카운트를 파일 이름에 반영
    img = sensor.snapshot()
    img.save(file_name)  # SD 카드에 이미지 저장
    print(f"이미지를 저장했습니다: {file_name}")

# 블루투스를 통해 데이터를 수신하고, 1을 받으면 사진을 찍고 SD 카드에 저장하는 함수
async def led_control_task():
    while True:
        conn, data = await led_characteristic.written()
        received_value = struct.unpack("<B", data)[0]  # 1바이트 데이터 해석
        if received_value == 1:
            led.on()  # LED 켜기 (작업 중 표시)
            capture_image()  # 사진 한 장 촬영
            led.off()  # LED 끄기 (작업 완료 표시)

# BLE 장치 광고 및 연결 처리 함수
async def peripheral_task():
    while True:
        async with await aioble.advertise(
            _ADV_INTERVAL_MS,
            name="mpy-blinky",
            services=[_LED_CONTROL_UUID],
            appearance=_ADV_APPEARANCE_GENERIC_TAG,
        ) as connection:
            print("Connection from", connection.device)
            await connection.disconnected()

# 메인 함수
async def main():
    await asyncio.gather(led_control_task(), peripheral_task())

# 프로그램 시작
asyncio.run(main())
