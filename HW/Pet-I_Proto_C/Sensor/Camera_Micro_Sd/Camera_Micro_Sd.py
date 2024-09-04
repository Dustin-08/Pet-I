import machine
import os
import time
import sdcard
from pyb import Pin, SPI
import uos  # uos 모듈을 가져옵니다.

# SD 카드 핀 설정
SCK_PIN = 9   # SCK 핀
MISO_PIN = 10  # MISO 핀
MOSI_PIN = 8   # MOSI 핀
CS_PIN = 7     # Chip Select 핀
#cs = Pin("7", Pin.OUT_OD) # CS pin = PE11
cs = Pin("PG12", Pin.OUT_PP, Pin.PULL_UP)

# SD 카드 초기화
sd = sdcard.SDCard(machine.SPI(4), machine.Pin(cs))

# 파일 시스템 마운트
vfs = uos.VfsFat(sd)  # uos 모듈에서 VfsFat을 가져와 사용합니다.
uos.mount(vfs, "/sd")

# 파일 생성 및 쓰기
with open("/sd/test.txt", "w") as f:
    f.write("안녕하세요, VisionMesh SD 카드 테스트 중입니다! 😊")

# 파일 읽기
with open("/sd/test.txt", "r") as f:
    content = f.read()
    print("파일 내용:", content)

# SD 카드 마운트 해제
uos.umount("/sd")
