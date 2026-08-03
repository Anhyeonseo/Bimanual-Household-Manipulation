# Motion-8 G474 buffered 물리 실행 후보 로컬 결과

## 결론

G474에 validation-only route와 분리된 physical buffered execution route를
구현했다. 로컬 후보 identity는 `0x00022000`, capabilities는
`0x00000FFF`이며 새 bit `0x00000800`이 물리 실행 후보를 나타낸다.

ROS Action runtime은 아직 연결하지 않았고, Pi 전송·serial 접근·STM32
flash/reset·12 V 제어·로봇 이동은 모두 0회다. 따라서
`motion_authorized=false`, `deployed=false`다.

## 구현 결과

- trajectory `t=0` pose를 첫 wire sample과 interpolation anchor에 함께 사용
- servo read sweep 없이 fresh start pose를 anchor로 고정
- reviewed timing `20 ms`, lead `60..400 ms`, prime/watermark/refill
  `16/10/16` 적용
- executor는 1 ms tick, 6축 SYNC_WRITE는 5 ms와 마지막 sample에서 실행
- queue underflow·missed apply tick·cancel·connection loss·tracking fault를
  extended terminal과 physical safe-stop 경로에 연결
- BEGIN 뒤 START frame이 오지 않으면 anchor deadline에서 tracking fault로
  종료해 PRIMING 무기한 유지를 차단
- validation-only candidate는 기존처럼 무동작 route로 분리 유지
- host physical exchange는 matching one-shot 응답만 허용하고 자동 재전송 금지

## 검증 결과

- 전체 Python/ROS 회귀: `494 passed`
- STM32 actuator C: ctest `2/2 passed`
- C fault injection: queue underflow와 missed apply tick 모두
  `HOLD + safe_stop_required`
- `single_arm_bridge` symlink-install rebuild: PASS
- Cortex-M4 Release clean cross-build: PASS, compiler warning 0
- ELF text/data/bss: `35456 / 112 / 5392` bytes
- HEX SHA-256:
  `b5b1780bfdf5fdfb9b90637b26925fef58a68b150321f0d307a85ebcedef4bee`

HEX:
`artifacts/firmware/2026-08-03/stm32_g474_single_arm_0x00022000.hex`

## 다음 gate

1. 명시 승인 아래 Pi host/HEX 전송과 기존 host backup
2. `0x00021900` flash 512 KiB backup과 SHA 생성
3. 별도 승인으로 program/verify/reset 1회
4. identity/capability, READ_ONLY, MOTION_ENABLED 무동작 검증
5. torque ON 고정자세 no-setpoint 안정성 확인
6. 별도 승인으로 단일 관절 최소 변위 buffered 실행 1회
7. 성공 뒤에만 ROS Action runtime 연결과 연속 Pick/Place로 확대
