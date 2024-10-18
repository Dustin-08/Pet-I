import machine
import os
import sdcard
from pyb import Pin, SPI
import uos  # uos 모듈을 가져옵니다.
import sensor  # 카메라 모듈을 가져옵니다.

# SD 카드 핀 설정
cs = Pin("CS", Pin.OUT_OD)  # CS pin = PE11
spi = SPI(4, SPI.MASTER, baudrate=int(480000000 / 256), polarity=0, phase=0)

# SD 카드 초기화
sd = sdcard.SDCard(spi, cs)

# 파일 시스템 마운트
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")

# 카메라 초기화
sensor.reset()  # 카메라 모듈 초기화
sensor.set_pixformat(sensor.RGB565)  # 픽셀 포맷 설정 (RGB565 또는 GRAYSCALE)
sensor.set_framesize(sensor.QVGA)  # 프레임 사이즈 설정 (QVGA, VGA 등)
sensor.skip_frames(time=2000)  # 카메라 안정화 대기

# 이미지 캡처
img = sensor.snapshot()  # 카메라에서 이미지를 캡처합니다.

# 이미지 파일로 저장 (JPEG 포맷)
img_path = "/sd/captured_image.jpg"
img.save(img_path, quality=90)  # quality 옵션을 통해 JPEG 품질 조정 (0-100, 기본값 90)

print("이미지가 SD 카드에 JPEG 형식으로 저장되었습니다:", img_path)

# SD 카드 마운트 해제
uos.umount("/sd")
