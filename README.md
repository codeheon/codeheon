# Realistic FPV Drone Simulator

브라우저에서 실행되는 FPV 드론 시뮬레이터입니다. Acro 스타일 조작과 간단한 물리 모델(중력, 추력, 항력, 바람, 배터리 전압 강하)을 반영했습니다.

## 실행 방법

```bash
python3 -m http.server 8000
```

브라우저에서 `http://localhost:8000` 접속.

## 조작

- `W/S`: Pitch
- `A/D`: Roll
- `Q/E`: Yaw
- `Shift/Ctrl`: Throttle 증가/감소
- `R`: 기체 리셋
- `G`: 게이트 표시 토글

## 튜닝

- **Rates**: 회전 응답 속도
- **Camera Tilt**: FPV 카메라 각도
- **Wind**: 측풍 강도
