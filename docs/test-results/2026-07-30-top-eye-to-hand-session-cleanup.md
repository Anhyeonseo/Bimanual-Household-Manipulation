# Top eye-to-hand 보정 세션 정리 기록

## 목적

최종 보정에 사용하지 않는 대형 원본 프레임을 저장소 작업 디렉터리에서
제거하되, 실패 수치와 의사결정은 Git에 남긴다. 원본 `artifacts/`는 Git
제외 대상이며 최종 채택 세트만 로컬에 보존한다.

## 세션 판정

| 세션 | 입력 규모 | 판정 | 보존 방식 |
|---|---:|---|---|
| session01 | training 4개, 80 frames | 조립된 session/candidate 없음, 초기 비강체 trial | raw 제거, 상태만 문서화 |
| session02 | training 5개, 100 frames | 조립된 session/candidate 없음, 불완전 trial | raw 제거, 상태만 문서화 |
| session03 | training 8개 + validation 2개, 360 frames | REJECTED | candidate YAML 보존, raw 제거 |
| session04 initial | training 8개 + validation 2개 | REJECTED | candidate YAML 보존 |
| session04 r1 | 교체 training 1개 포함 8개 + validation 2개 | REJECTED | candidate YAML 보존 |
| session04 q0 refined | 같은 r1 세트에 wrist-flex q0 보정 적용 | VALIDATED | candidate와 참조 raw 유지 |
| independent held-out | heldout 2개, 40 frames | VALIDATED | session/candidate와 raw 유지 |

session03 거부 수치:

- training translation RMS/max: `7.749 / 12.001 mm`
- training rotation RMS/max: `1.645 / 2.373 deg`
- validation translation max: `6.072 mm`

session04 initial 거부 수치:

- training translation RMS/max: `3.235 / 6.364 mm`
- validation translation max: `5.763 mm`

session04 r1 거부 수치:

- training translation RMS/max: `3.201 / 6.228 mm`
- training rotation RMS: `1.002 deg`
- validation translation max: `6.081 mm`

세 후보 모두 `motion_authorized=false`,
`robot_target_available=false`였다. q0 metrology 보정과 독립 held-out 검증을
통과한 세트만 최종 계측 근거로 채택했다.

## 제거 범위

- `session01/`, `session02/`, `session03/`의 raw frame과 diagnostics
- session04의 교체 전 `training_01/`
- 대체된 초기 orientation 포함 stage7 plan-only JSON
- 미리보기 `tmp/`, 재생성 가능한 `output/`, `*.orig` 백업
- Python/pytest 캐시

최종 session04 r1이 참조하는 `training_01_replacement`, training 02..08,
repeat validation diagnostics와 독립 held-out 원본은 보존한다.

## 영구 증거

- [session03 rejected candidate](evidence/2026-07-29-top-eye-to-hand-session03-rejected.yaml)
- [session04 initial rejected candidate](evidence/2026-07-29-top-eye-to-hand-session04-initial-rejected.yaml)
- [session04 r1 rejected candidate](evidence/2026-07-29-top-eye-to-hand-session04-r1-rejected.yaml)
- [q0-refined candidate](evidence/2026-07-30-wrist-flex-q0-refined.yaml)
- [independent validation](evidence/2026-07-30-top-eye-to-hand-independent-validation.yaml)
