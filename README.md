# ESP32-H2 Zigbee – Relay (On/Off Output)

Firmware **Zigbee 3.0** cho **ESP32-H2** dùng **ESP-IDF + esp-zigbee-sdk** (`esp-zigbee-lib` / `esp-zboss-lib`)
để bật/tắt một relay. Thiết bị là **Zigbee Router** (mở rộng mạng mesh vì cấp nguồn lưới), một endpoint
HA **On/Off Output (0x0002)** gồm các cluster Basic, Identify, Groups, Scenes, On/Off.

Dùng được với Zigbee2MQTT, ZHA (Home Assistant), deCONZ/Phoscon, SmartThings, Hue Bridge... và
mọi coordinator Zigbee 3.0.

> Repo còn có project **cảm biến hiện diện Zigbee dùng radar LD2410** trong thư mục
> [`ld2410_presence/`](ld2410_presence/README.md).

## Phần cứng

Mặc định cho **ESP32-H2-MINI-1 / ESP32-H2-DevKitM-1** (flash 4 MB).

| Chức năng | GPIO mặc định | Cấu hình (`idf.py menuconfig` → *Zigbee Relay Configuration*) |
|---|---|---|
| Relay (chân IN module relay) | **GPIO12**, mức cao = đóng | `CONFIG_RELAY_GPIO`, `CONFIG_RELAY_ACTIVE_HIGH` |
| Nút nhấn (về GND) | **GPIO9** (nút BOOT) | `CONFIG_BUTTON_GPIO` (-1 để tắt) |
| LED trạng thái (theo relay) | tắt (-1) | `CONFIG_STATUS_LED_GPIO`, `CONFIG_STATUS_LED_ACTIVE_LOW` |
| Nhớ trạng thái khi mất điện | bật | `CONFIG_RESTORE_RELAY_STATE` |

```
ESP32-H2                 Module relay
  GPIO12 ───────────────  IN
  5V     ───────────────  VCC
  GND    ───────────────  GND
```

- Module relay "low level trigger": tắt `CONFIG_RELAY_ACTIVE_HIGH`.
- Relay trần: GPIO → 1 kΩ → transistor NPN kéo cuộn relay, **bắt buộc có diode** ngược song song cuộn relay.
- Tránh dùng cho relay: GPIO2/3/8/9/25 (strapping), GPIO26/27 (USB), GPIO23/24 (UART0).
- DevKitM-1 có LED RGB WS2812 trên GPIO8 (LED địa chỉ hoá) → để `CONFIG_STATUS_LED_GPIO=-1`.

## Build & nạp

Yêu cầu ESP-IDF **v5.3** trở lên. Các thư viện Zigbee được Component Manager tự tải
(xem `main/idf_component.yml`).

```bash
. $IDF_PATH/export.sh
idf.py set-target esp32h2
idf.py menuconfig          # (tuỳ chọn) đổi GPIO
idf.py build
idf.py -p /dev/ttyUSB0 erase-flash flash monitor
```

## Ghép nối (pairing)

1. Bật chế độ cho phép thiết bị tham gia (*Permit join*) trên coordinator.
2. Cấp nguồn cho ESP32-H2: thiết bị mới (factory-new) tự động tìm và tham gia mạng (network steering),
   thử lại mỗi giây cho tới khi thành công.
3. Coordinator sẽ thấy thiết bị `haxeko` / `ESP32H2-Relay` với một công tắc On/Off.

## Nút nhấn

| Thao tác | Tác dụng |
|---|---|
| Nhấn ngắn | Đảo trạng thái relay và báo về coordinator |
| Giữ ≥ 5 s | Rời mạng, xoá thông tin mạng (factory reset) và khởi động lại để ghép nối lại |

## Cấu trúc

```
CMakeLists.txt         project ESP-IDF
sdkconfig.defaults     target esp32h2, Zigbee router, radio native
partitions.csv         nvs, factory app, zb_storage, zb_fct
main/app_main.c        stack Zigbee, data model, xử lý lệnh ZCL
main/app_driver.c      relay, LED, nút nhấn, lưu trạng thái NVS
main/app_priv.h        cấu hình mạng Zigbee, tên nhà sản xuất/model
main/Kconfig.projbuild tuỳ chọn GPIO
```
