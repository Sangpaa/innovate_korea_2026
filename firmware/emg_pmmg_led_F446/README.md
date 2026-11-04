# emg_pmmg_led_F446

NUCLEO-F446RE 펌웨어. `reference/pmmg_HND_STM`(NUCLEO-G474RE)을 F446RE로 포팅하고, LED 출력 두 개를 추가했다.

- 왼팔·오른팔 EMG envelope(A0, A1) → 각 팔의 WS2812 2줄(144구씩, 144구/m 1 m)에 충전 게이지
- pMMG J1 심박 → 아크 리액터(WS2812 링 24·16·12·8구 동심원, 60구)가 박동마다 빛남
- 전체 배선도, 효과 미리보기, 구매 목록: 아티팩트 "EMG·pMMG LED 배선도" (https://claude.ai/artifact/6bRHv5xasxSt1ucHRmLvBD)

## 빌드와 플래시

```sh
make                              # build/emg_pmmg_led_F446.elf, .bin
make flash                        # ST-LINK(SWD)로 쓰고 리셋
make WS2812_LEDS_PER_STRIP=60     # 줄당 LED 개수 변경 (기본 144)
```

- 플래시 전 원래 보드 펌웨어는 `../backup/f446re_before_flash_20261022.bin`에 있다.
- 복구 명령: `STM32_Programmer_CLI -c port=SWD -w ../backup/f446re_before_flash_20261022.bin 0x08000000 -rst`

## 핀 배선

| 기능 | Nucleo 핀 | 연결 |
|---|---|---|
| EMG envelope 입력 | A0 (PA0) 왼팔, A1 (PA1) 오른팔 | PSL-iEMG2 Ch2. **0–3.3 V만 허용**한다. 보드 전원은 Nucleo 5V. |
| WS2812 데이터 | D10 (PB6) 왼팔, PB7 (CN7-21) 오른팔, D15 (PB8) 링 | Pixel Shifter 2개(PS1: PB6→D5, PB7→C5 / PS2: PB8→D5, V = 3V3) → 각 330 Ω → 줄 1 / 바깥 링(24구) DIN. 예비: 74HCT125N(5 V 전원) |
| pMMG 센서 | Nucleo 핀에 직접 | 레퍼런스와 같다 (SPI1/2/3). 단 J7 CS는 PC9 (F446RE에는 PB11이 없다). J1 = D3 SCK, D5 MISO, D4 MOSI, PC8 CS, 3V3, GND. J12(D6 PB10)는 XM 동기 입력이다. |
| PC 스트림 | USB (ST-LINK VCP) | 921600 8N1, 레퍼런스 v5 패킷 형식 그대로 |

각 팔의 2줄은 직렬로 연결한다(줄 1 DOUT → 줄 2 DIN). 1 m 줄은 끝에도 5V·GND를 연결한다. 아크 리액터는 바깥에서 안쪽으로 직렬 연결한다(24 → 16 → 12 → 8). 링 크기와 순서는 `fx.h`의 `FX_RING_SIZES`이고, 합이 `WS2812_RING_LEDS`(60)와 같아야 한다.

전원은 USB-C PD 보조배터리 → PD 트리거(20 V) → 5 V 강압 모듈이다. LED는 강압 모듈에서 직접 받고, Nucleo는 같은 5 V를 E5V(CN7-6, JP5 → E5V)로 받는다. 자세한 배선은 아티팩트를 본다.

## 동작

| 출력 | 입력 | 표시 (`fx.c`) |
|---|---|---|
| 팔 게이지 (왼팔, 오른팔) | EMG level ≥ 0.10 동안 충전. level 1이면 3 s에 가득. 힘을 빼면 0.4 s 유지 후 1.5 s에 비움 | 초록→빨강→자홍 그라데이션, 물결, 흰 머리, 반짝임. 100 %에서 흰 섬광 후 무지개 |
| 아크 리액터 | pMMG J1 심박 (`pmmg_light.c`가 박동·BPM 검출) | 박동마다 중심에서 바깥으로 섬광이 퍼짐(쿵), 이어서 약한 두 번째 섬광(쾅). 색은 BPM을 따름(60 청록 → 90 보라 → 120 빨강). 박동 사이에는 희미한 혜성이 BPM에 맞춰 돎. 맥박이 안정되기 전에는 약하게 깜빡이고, 박동이 없으면 천천히 숨 쉬듯 빛남 |

- **기준값:** 부팅 직후 EMG는 2 s, pMMG는 1 s 동안 측정한다. 이때 힘을 빼야 한다. 측정 중 팔은 파란 점이 움직이고 링은 파랗게 숨 쉰다.
- **B1 버튼:** 센서 재초기화(레퍼런스 동작), EMG·pMMG 기준값 재측정, 게이지 비움.
- **링 빨간 깜빡임:** 바깥 링 첫 LED. J1 데이터가 없거나 유효하지 않다.
- **프레임:** 팔 한 줄 체인(288구)을 보내는 데 약 8.9 ms가 걸린다. 100 Hz 주기(10 ms) 안에 들어간다.
- **전류 제한:** `ws2812.c`가 프레임마다 출력별 전류를 추정한다. 감마 2.2와 밝기 상한(128)을 먼저 적용한다. 그다음 팔마다 3.5 A(`WS2812_ARM_LIMIT_MA`), 아크 리액터 1.8 A(`WS2812_RING_LIMIT_MA`)를 넘으면 그 출력을 어둡게 한다. 마지막으로 전체가 8 A(`WS2812_POWER_LIMIT_MA`)를 넘으면 모두 같은 비율로 어둡게 한다. 이 값은 색 전류만 센다. 대기 전류(LED당 약 1 mA)는 따로 더해진다. 팔 퓨즈 5 A 기준: 3.5 A + 대기 0.3 A = 3.8 A로 퓨즈 정격의 80 % 안이다.

## 조정 값

| 파일 | 설정 |
|---|---|
| `Core/Inc/ws2812.h` | `WS2812_ARM_STRIPS`, `WS2812_LEDS_PER_STRIP`, `WS2812_RING_LEDS`, `WS2812_MAX_BRIGHTNESS`(128), `WS2812_ARM_LIMIT_MA`(3500), `WS2812_RING_LIMIT_MA`(1800), `WS2812_POWER_LIMIT_MA`(8000), `WS2812_OPEN_DRAIN` |
| `Core/Inc/fx.h` | 게이지 경로(`FX_ARM_SEQUENTIAL`, `FX_ARM_SERPENTINE`), 충전·유지·방전 시간, 임계값, 색, 아크 리액터 링 크기(`FX_RING_SIZES`)와 충격파 임계값 |
| `Core/Inc/emg_input.h` | `EMG_DEADBAND_MV`, `EMG_FULL_SCALE_MV`, `EMG_SMOOTH_MS`, `EMG_REST_CAL_MS` |
| `Core/Inc/pmmg_light.h` | `PMMG_LIGHT_CHANNEL`(0 = J1), `PMMG_PRESSURE_SIGN`, 심박 검출: `PMMG_PULSE_MIN_PA`(3 Pa / 50 ms), `PMMG_PULSE_THRESHOLD`, `PMMG_BPM_MIN`/`MAX`(35–180), `PMMG_PULSE_LOST_MS` |
| `Core/Inc/ms5607.h` | 사용할 pMMG 커넥터, 샘플링 주파수 (레퍼런스와 같다) |

값을 조정할 때는 디버거 Live Expressions에서 `app_monitor`를 본다. EMG mV·기준값·level(왼팔, 오른팔), 게이지, pMMG Δ Pa, 맥박 신호(`pmmg_pulse_pa`), 상승 기울기 최대값(`pmmg_slope_peak_pa`), BPM, 박동 수, 추정 LED 전류(전체 `led_current_ma`, 출력별 `led_output_ma`)가 들어 있다.

**심박 검출 조정:** 센서를 대고 가만히 있을 때 `pmmg_beats`가 맥박마다 1씩 늘고 `pmmg_bpm`이 맞으면 된다. 박동을 놓치면 `pmmg_slope_peak_pa`를 보고 `PMMG_PULSE_MIN_PA`를 그 값의 절반 정도로 낮춘다. 잡음에 박동이 생기면 올린다. 맥박에서 압력이 내려가면 `PMMG_PRESSURE_SIGN`을 -1로 바꾼다.

## 레퍼런스(G474) 대비 변경점

| 항목 | G474RE | F446RE |
|---|---|---|
| 클럭 | HSE 24 MHz, PLL 없음 | ST-LINK MCO 8 MHz → PLL 180 MHz. MCO가 없으면 HSI로 대체한다. |
| SPI 속도 | 375 kHz | 352 kHz (SPI1 /256, SPI2·3 /128) |
| VCP UART | LPUART1 + DMA1 Ch1 (DMAMUX) | USART2 + DMA1 Stream6 Ch4 |
| SPI RX 비우기 | `SR.FRLVL` (FIFO) | `SR.RXNE` (F4에는 FIFO 없음) |
| BSP | STM32G4 Nucleo BSP | 사용하지 않음. LD2 = PA5, B1 = PC13(눌림 = low, falling edge) |
| ms5607 | — | `ms5607_peek_channel()` 추가. 스트림의 프레임 소비와 상관없이 읽는다. |
| J7 CS | PB11 | PC9 (CN10-1). F446RE LQFP64에는 PB11 핀이 없다. |

## DMA/타이머 자원

| 자원 | 사용처 |
|---|---|
| TIM2 | 1 MHz 타임베이스 + J12 동기 캡처 |
| TIM4 CH1/CH2/CH3 + DMA1 Stream0/3/7 Ch2 | WS2812 왼팔/오른팔/링 (IRQ 없음, CCDS=1) |
| ADC1 (IN0·IN1 스캔) + DMA2 Stream0 Ch0 | EMG 두 채널 |
| USART2 + DMA1 Stream6 Ch4 | 스트림 |
| SPI1/2/3 IRQ | pMMG |

NVIC 우선순위: SysTick 0, SPI 1, 스트림 2, B1 15.
