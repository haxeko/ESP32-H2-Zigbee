# ESP32-H2 Zigbee – Cảm biến hiện diện LD2410

Firmware **Zigbee 3.0** cho **ESP32-H2** + radar mmWave 24 GHz **HLK-LD2410** (LD2410 / LD2410B / LD2410C),
viết bằng **ESP-IDF + esp-zigbee-sdk**. Thiết bị là **Zigbee Router**, khai báo kiểu
**Occupancy Sensor (0x0107)**, gồm các cluster Basic, Identify và **Occupancy Sensing (0x0406)**.

- Radar phát hiện người **đang di chuyển hoặc đứng yên** → `occupancy = 1`, không còn người → `0`.
- Mỗi lần trạng thái đổi, thiết bị **tự gửi báo cáo** về coordinator, không cần cấu hình reporting.
- Dùng được với Zigbee2MQTT, ZHA (Home Assistant), deCONZ, SmartThings...

## Đấu dây

| LD2410 | ESP32-H2 | Ghi chú |
|---|---|---|
| **TX** | **GPIO4** (UART1 RX) | `CONFIG_LD2410_UART_RX_GPIO` |
| **RX** | **GPIO3** (UART1 TX) | `CONFIG_LD2410_UART_TX_GPIO` |
| VCC | **5V** | LD2410 cần 5 V (dòng ~80 mA) |
| GND | GND | |
| OUT | — | không dùng |

```
 HLK-LD2410               ESP32-H2
   TX  ──────────────────  GPIO4  (RX)
   RX  ──────────────────  GPIO3  (TX)
   VCC ──────────────────  5V
   GND ──────────────────  GND
```

- Mức logic UART của LD2410 là 3.3 V, nối thẳng vào ESP32-H2 được.
- UART mặc định của LD2410: **256000 baud, 8N1** (`CONFIG_LD2410_BAUD_RATE`).
- GPIO3 là chân strapping của ESP32-H2, nhưng chỉ được đọc lúc reset và chân RX của LD2410 không kéo nó,
  nên dùng làm TX an toàn.
- Nút BOOT (GPIO9): **giữ ≥ 5 s** để rời mạng Zigbee và factory reset.

## Cấu hình radar

Trong `idf.py menuconfig` → *LD2410 Presence Sensor Configuration*:

| Tuỳ chọn | Mặc định | Ý nghĩa |
|---|---|---|
| `LD2410_APPLY_CONFIG` | bật | Ghi cấu hình xuống radar mỗi lần khởi động |
| `LD2410_MAX_GATE` | 8 | Cổng xa nhất (1 cổng = 0,75 m → 8 = 6 m) |
| `LD2410_NO_ONE_DURATION` | 10 s | Thời gian không còn ai trước khi báo *unoccupied* |

Tắt `LD2410_APPLY_CONFIG` nếu muốn tinh chỉnh độ nhạy từng cổng bằng app **HLKRadarTool** (Bluetooth của LD2410B/C).

## Build & nạp

Đã build thử với **ESP-IDF v5.3.2** (bản esp-zigbee-sdk khuyến nghị) + `esp-zigbee-lib`/`esp-zboss-lib` **1.6.4**
(ghim trong `main/idf_component.yml`, Component Manager tự tải).

```bash
cd ld2410_presence
. $IDF_PATH/export.sh
idf.py set-target esp32h2
idf.py menuconfig          # (tuỳ chọn)
idf.py build
idf.py -p /dev/ttyUSB0 erase-flash flash monitor
```

Log mẫu:

```
I ld2410: UART1 @ 256000 baud, TX GPIO3 -> LD2410 RX, RX GPIO4 <- LD2410 TX
I ld2410: Configured: max gate 8 (6.00 m), no-one duration 10 s
I zb_presence: Joined network (PAN ID: 0x1a62, Channel: 11, Short Address: 0x3f21)
I zb_presence: OCCUPIED (state 2, distance 135 cm)
I zb_presence: state 2, moving 0 cm (0), stationary 135 cm (47), detect 135 cm
```

## Ghép nối

1. Bật *Permit join* trên coordinator.
2. Cấp nguồn: thiết bị mới tự động tìm và tham gia mạng, thử lại mỗi giây cho tới khi thành công.
3. Coordinator nhận thiết bị `haxeko` / `ESP32H2-LD2410` với thuộc tính **occupancy**.

## Cấu trúc

```
main/app_main.c        stack Zigbee, cluster Occupancy Sensing, báo cáo trạng thái, nút reset
main/ld2410.c/.h       driver UART LD2410: phân tích frame dữ liệu, lệnh cấu hình 0x0060
main/Kconfig.projbuild chân UART, baud, cấu hình radar, nút nhấn
```
