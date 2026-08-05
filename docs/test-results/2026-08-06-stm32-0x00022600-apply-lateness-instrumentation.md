# 0x00022600 apply lateness 계측 배포와 buffered startup 중단 분석

## 결론

apply lateness 분포 계측 firmware `0x00022600`을 플래시하고 검증까지 통과했다.
그러나 첫 실측을 위한 q0 복귀 Action이 startup에서 `missed_apply_tick`으로
중단되어 **histogram은 아직 얻지 못했다**(`applied=0`이라 집계될 sample이 없었다).
팔은 움직이지 않았고 fail-closed는 설계대로 동작했다.

중단 원인을 조사해 **serial 링크와 buffered 교환 경로는 무결함을 실측으로
확인했고**, 남은 원인을 host 수신 경로의 heartbeat 예산으로 좁혔다. 원인을
확정하기 위한 계측을 transport에 추가했다.

## 0x00022600 계측 내용

lateness는 순수 C core에서 계산되므로 계측도 그 자리에 넣었다.

- `apply_lateness_histogram[6]` — 적용된 모든 sample을 tick 단위 bucket에 집계.
  허용치를 넘으면 실행이 종료되므로 5 ms 운영 한계에서 6개 bucket이 허용 범위
  전체를 정확히 덮는다. 마지막 bucket은 saturating이다.
- `maximum_apply_lateness_sample_index` — 최대가 마지막으로 갱신된 applied
  sample 위치(1-based). 최악 tick이 궤적의 어디였는지 알려준다.

`BUFFERED_STATUS` payload가 `32 → 60` bytes로 늘었고 bridge가 성공 terminal에
붙인다. host는 `16 / 32 / 60`을 모두 수용하므로 구버전 firmware도 계속 해석된다.
status encoder는 작은 버퍼를 조용히 truncate하지 않고 거부한다.

## 0x00022600 물리 검증

| 항목 | 결과 |
|---|---|
| identity | `0x00022600 / 0x00000FFF / 0x8AD27897`, protocol 1, joints 6 |
| reset 후 counter 14개 | 전부 `0` |
| bus health schema | `2` |
| `lazy_arm_count == transaction_count` | `42 == 42` |
| 70초 soak | `PASSED=1`, 12개 counter delta `0`, snapshot 전부 `armed=False` |
| torque / 전압 / 온도 | 전부 OFF / `12.2~12.5 V` / `27~30 °C` |

UART 수명주기는 이번 변경에서 건드리지 않았으므로 0x225의 300초 soak을
반복하지 않고 경로 무변경 확인만 했다.

플래시 시 발견한 절차 결함: bridge를 `ros2 run ... &`로 띄우고 `$!`를 저장했는데
`ros2 run`은 실제 노드를 자식으로 실행한다. 저장된 PID는 wrapper였고 노드는
살아남아 플래시 내내 리셋된 MCU를 폴링했다. 이후 정지는 프로세스 패턴으로 한다.

## q0 복귀 startup 중단

```
+0.000s  connected firmware=0x00022600 mode=MOTION_ENABLED
+7.882s  WARN  transient heartbeat delay (1/3): timeout waiting for STATE_FEEDBACK
+8.283s  ERROR aborted  reason=missed_apply_tick
         precompute_ms=155.864 first_sample_lead_ms=95
         accepted=16 applied=0 queued=16
+8.300s  ERROR heartbeat rejected latched=1
```

마지막 줄은 결과이지 원인이 아니다. `missed_apply_tick`이 `safe_stop_required`를
걸어 latch가 서고 그 뒤 heartbeat가 거부됐다.

START는 수락됐고(`first_sample_lead_ms=95`, 하한 80) 팔은 한 sample도 적용하지
못한 채 중단됐다.

## 기각된 원인

| 가설 | 기각 근거 |
|---|---|
| USB kworker 정체 | `kworker/0:4`의 `+usb`/`+pm` 접미사는 그 순간 처리 중인 workqueue일 뿐이다. `wchan=0`, 스택 없음 |
| USB / serial 오류 | 커널 USB 로그는 부팅 시각 카메라 열거뿐. ST-LINK 오류 `0` |
| serial 링크 정체 | 아래 실측 |
| 0x00022600 회귀 | 같은 경고가 `0x00022500` Motion-11에서도 1회 발생했다. status payload 증가분은 프레임당 약 2.4 ms(115200 baud)로 250 ms를 설명하지 못한다 |

`kworker` 접미사 한 글자로 USB 원인을 성급히 결론냈다가 증거로 기각했다.

## 실측: buffered 교환 경로는 무결

무동작 `capture_buffered_validation_timing.py`로 buffered frame을 1000회 교환했다.
ARM, ENABLE, CLEAR_FAULT, SAFE_STOP, 실행 가능한 setpoint를 보내지 않는다.

| 지표 | min | p50 | p95 | max |
|---|---:|---:|---:|---:|
| `serial_round_trip_ms` | 19.652 | 19.911 | 20.016 | **20.145** |
| `host_command_jitter_ms` | 0.001 | 0.059 | 0.155 | 0.336 |
| `delivery_lateness_ms` | 0.000 | 0.000 | 0.000 | 0.000 |

`TRANSPORT_ERRORS=0`. **최대 20.1 ms로 250 ms 근처도 가지 않는다.**
heartbeat/position만 도는 soak도 깨끗했다(0x226 70초 700회, 0x225 300초 3000회,
최대 6.2 ms). 즉 정체는 두 트래픽이 섞이는 실제 실행에서만 나타난다.

## 좁혀진 원인과 예산

```
serial port read timeout   = 120 ms   (bridge_node.py:100)
heartbeat 응답 timeout     = 250 ms   (transport.py)
heartbeat timer 주기       = 100 ms   (bridge_node.py:149)
실측 buffered 왕복 max     =  20.1 ms

250 ms 안에 가능한 read_until 반복 = 2회
```

`_receive_matching`은 매칭되지 않는 프레임을 소비하고 루프를 계속한다.
비매칭 프레임 2개를 소비하거나 빈 read가 2번이면 예산이 끝난다. 링크가
건강해도 그렇다. `applied=0 / queued=16`은 host가 prime frame을 밀어 넣는
바로 그 구간에서 터졌음을 뜻한다.

transport에는 이미 "MCU가 최종 servo 검증 중 GET_STATE 응답을 생략할 수 있다"는
주석과 `defer_state_after_motion_result` 처리가 있다. MCU가 특정 구간에서 늦게
답하는 것은 알려진 동작인데 heartbeat 예산이 이를 감당하지 못한다.

## 추가한 계측

타임아웃 하나로는 두 원인을 구분할 수 없다.

- **host가 다른 트래픽에 예산을 소진** → 수신 경로 demultiplexing 문제
- **MCU가 조용한 링크에서 늦게 응답** → demultiplexing으로는 해결되지 않음

`ResponseTimeoutError`가 관측 내역을 함께 보고하도록 했다.

```
MCU 지연:   ... elapsed_ms=251.0 budget_ms=250.0 empty_reads=2 undecodable=0 observed=none
host 소진:  ... elapsed_ms=252.0 budget_ms=250.0 empty_reads=0 undecodable=0 observed=SETPOINT_STATUS#118,SETPOINT_STATUS#119
```

`TransportError`의 하위 클래스이므로 bridge의 기존 처리 경로는 그대로다.
관측 프레임 목록은 8개로 제한해 예외 메시지가 무한히 길어지지 않는다.
시험 5개가 두 시나리오와 경계를 덮는다.

## 로컬 회귀

```
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
python3 -m pytest -q
```
**`637 passed`**

## 다음 gate

1. 계측이 붙은 상태로 q0 복귀를 재시도해 `empty_reads` / `observed` 를 확보한다.
   fresh anchor로 계획을 다시 만들어야 한다(이전 시도 잠금이 남아 있다).
2. 그 증거로 수신 경로 설계를 확정한다. `observed`가 채워지면 demultiplexing,
   비어 있고 `empty_reads`만 늘면 MCU 응답 지연이므로 예산이나 priming 구간의
   heartbeat 정책을 다뤄야 한다.
3. 해결 후 Motion-11을 같은 궤적으로 재실행해 lateness histogram 첫 실측을 얻는다.
   `maximum_apply_lateness_ms=5`가 특정 구간에 몰리는지 전 구간에 퍼지는지는
   그 분포로만 알 수 있다.
