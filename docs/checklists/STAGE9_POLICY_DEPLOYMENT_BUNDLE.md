# 단계 9 Policy ONNX 배포 번들 계약

## 목적

Isaac Sim/Isaac Lab 학습은 데스크탑에서 끝내고 Pi 5에는 검증된 ONNX와
배포 manifest만 전달한다. 실제 정책 파일이 준비되기 전에는 입력·출력이나
action scale을 추측해 채우지 않는다.

## 현재 상태

- 저장소와 홈 디렉터리를 검색한 결과 실제 policy ONNX/체크포인트는 없다.
- 발견된 ONNX/PT 파일은 모두 단계 8 Top 펜 YOLO-OBB 검출 모델이다.
- 따라서 이 이슈는 배포 계약과 fail-closed 검증기까지만 구현한다.
- Pi 전송, policy inference, ROS node, command arbiter와 로봇 이동은 제외한다.

## 번들 필수 파일

하나의 bundle 디렉터리 안에 다음 파일을 둔다.

1. `policy.onnx`
2. `observation_contract.json`
3. `policy_bundle.json`

`policy_bundle.json`은 다음을 고정한다.

- model 상대 경로, SHA-256, opset와 모든 ONNX output name/dtype/shape
- deployment contract SHA-256
- observation contract 상대 경로와 SHA-256
- `SHADOW`, `onnxruntime_cpu`, `control_dt_s`, target rate와 deadline
- action output name, representation, 순서, scale, lower/upper
- checkpoint, training config와 export config SHA-256
- command publication 금지와 모든 fail-closed 정책

`observation_contract.json`은 실제 학습과 동일한 다음 값을 고정한다.

- `structured_state`, `rgb_tensor` 또는 `hybrid`
- 모든 ONNX input name/dtype/static shape
- structured feature 순서와 affine normalization
- RGB camera order, encoding, resize/layout/color order/scale/mean/std
- observation 최대 age와 source skew, 위반 시 reject

## 검증

검증기는 상대 경로가 bundle 밖으로 나가는 것을 거부하고 모든 SHA를
재계산한다. 검증된 export 환경의 `onnx` 패키지로 graph를 검사하고 manifest
및 observation contract의 input/output name, dtype, static shape와 실제 graph가
정확히 같은지 비교한다.

~~~bash
/path/to/export/python tools/validate_policy_deployment_bundle.py \
  --manifest /path/to/bundle/policy_bundle.json \
  --contract config/policy_deployment_contract.json \
  --output /path/to/bundle/validation.json
~~~

합격 출력은 다음과 같다.

~~~text
POLICY_DEPLOYMENT_BUNDLE_PASS
POLICY_DEPLOYMENT_BUNDLE_ARTIFACT=...
~~~

## 완료 조건

- [x] 배포 계약이 motion authority와 command publication을 금지한다.
- [x] stale/skew/deadline/nonfinite/out-of-bounds/manifest mismatch가 모두
  fail-closed다.
- [x] 경로 탈출, SHA 변조, ONNX I/O 불일치와 action dimension 불일치를
  단위 테스트로 거부한다.
- [ ] 실제 학습 policy ONNX와 export metadata를 확보한다.
- [ ] 실제 bundle validation artifact와 SHA를 생성한다.
- [ ] 다음 이슈에서 Pi 단일 inference shadow smoke를 수행한다.
