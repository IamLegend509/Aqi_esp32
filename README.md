AQI_ESP32 — Wiring, BOM and Assembly Notes

Overview

This README documents wiring, pinouts, BOM and perf‑board assembly tips for the aqi_esp32 project. It summarizes connections required to wire the four sensor components (MQ7, MQ135, GP2Y1010 dust, BMP280) plus the DHT22 and the ESP32 module.

Pinouts (connect exactly as shown in code)

- ESP32 WROOM-32E signals used:
  - 3.3V: ESP32 3.3V rail (power for BMP280, DHT22, ESP32)
  - GND: common ground
  - GPIO21 (I2C SDA) → BMP280 SDA (pull-up 4.7k to 3.3V)
  - GPIO22 (I2C SCL) → BMP280 SCL (pull-up 4.7k to 3.3V)
  - GPIO14 → DHT22 DATA (4.7k pull-up to 3.3V)
  - GPIO33 → GP2Y1010 Vo (ADC1_CH5) (use divider)
  - GPIO32 → GP2Y1010 LED control (drive LOW to pulse)
  - GPIO34 → MQ135 AOUT (ADC1_CH6) (use divider)
  - GPIO35 → MQ7 AOUT (ADC1_CH7) (use divider)

Power and rails

- Use a single common GND for ESP32 and all sensors.
- 5V rail (external regulated) for MQ7, MQ135 and GP2Y1010 Vcc and heater/LED pins.
- 3.3V rail for BMP280, DHT22 and ESP32. Do NOT power BMP280 with 5V.
- Place decoupling: 0.1µF ceramic close to each sensor VCC pin. GP2Y1010 also needs a 220µF near its V-LED pin.

ADC and level shifting

- All analog inputs must be ≤3.3V. Use resistor dividers as shown in source code:
  - Typical divider: 10k (top) / 20k (bottom) → DIVIDER_FACTOR = 1.5
  - Connect AOUT → divider → ADC pin (ADC sees Vgpio)
- Use ADC1 channels only (safe with Wi‑Fi):
  - GPIO33 : ADC1_CH5 (dust)
  - GPIO34 : ADC1_CH6 (MQ135)
  - GPIO35 : ADC1_CH7 (MQ7)
- Keep ADC traces short and away from noisy 5V or switching lines. Add 100nF + optional 10µF decoupling.

MQ7 & MQ135 specifics

- Both outputs are 0–5V. Use divider to scale to ≤3.3V.
- MQ7 needs heater switching (5V ↔ ~1.4V) for accurate CO measurements. Implement heater driver (low‑side MOSFET or high‑side switch) and timing (see MQ7 datasheet) if you need correct ppm readings.
- MQ135 RL depends on breakout; measure RL physically. Code assumes common breakouts (1k or 10k) — check R0 calibrations in clean air.

GP2Y1010 (dust) specifics

- Follow Sharp timing: pulse LED low for ~280µs, sample after ~40µs, then rest ~9680µs.
- V-LED: 5V through 150Ω; tie LED‑GND and S‑GND to system ground. Place 220µF cap near V-LED to stabilize.
- Vo must be divided down to ADC input. Use same 10k/20k example or tune for accuracy.

BMP280 (I2C)

- VCC → 3.3V
- GND → GND
- SDA → GPIO21, SCL → GPIO22
- SDO pin sets I2C address: GND → 0x76, 3.3V → 0x77

DHT22

- VCC → 3.3V
- Data → GPIO14 with 4.7k pull-up to 3.3V
- DHT22 requires >=2s between readings; code uses ~3.5s between samples.

BOM (suggested)

- 3× 10kΩ 1% resistors (divider tops)
- 3× 20kΩ 1% resistors (divider bottoms) or 2×10k in series for the 20k value
- 4.7kΩ ×2 pull-up resistors (I2C and DHT22)
- 150Ω resistor for GP2Y1010 V-LED
- 0.1µF ceramic decoupling caps × per sensor
- 220µF electrolytic for GP2Y1010 V-LED
- Optional: MOSFET + driver parts for MQ7 heater switching (IRLZ44/logic MOSFET or similar, gate resistor, diode if needed)
- Perfboard, headers, wire, test points, screw terminal block for 5V/3.3V/GND

Perfboard / PCB tips

- Place ESP32 and analog dividers close together; route ADC traces as short as possible.
- Star ground: run a single ground bus to which each sensor ground ties, minimizing loops.
- Place decoupling caps right beside sensor VCC pins.
- Add test points at ADC nodes both before and after dividers.
- Keep 5V high‑current lines (heater, LED) physically separated from sensitive ADC traces.
- If making a PCB, include footprints for removable sensor cables/connectors.

Calibration and testing

- Power up with serial logging enabled (115200). Use idf.py build and idf.py -p <PORT> flash; open serial monitor via idf.py monitor.
- Calibrate: measure actual ADC raw and verify divider factor; adjust code DIVIDER_FACTOR constants if needed.
- For MQ sensors: warmup for 5+ minutes before calibration. MQ7 requires heater cycling before trusting readings.

Quick commands

- Build: idf.py build
- Flash: idf.py -p /dev/ttyUSB0 flash
- Monitor: idf.py monitor
