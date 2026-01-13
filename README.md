# Highway-Drowsiness-Prevention
SeSAC 임베디드 모빌리티 교육 과정 프로젝트의 고속도로 주행 졸음 방지 및 운전 보조 시스템

Project_Root/
├── 📂 Docs/               # 요구사항 명세서, ICD 엑셀, 회로도, 데이터시트
├── 📂 Firmware/           # STM32 (Nucleo) 소스코드
│   ├── 📂 Chassis/        # 핸들/엔코더 제어
│   ├── 📂 Body/           # 초음파/터치 센서
│   ├── 📂 Gateway/        # 게이트웨이
│   └── 📂 Control/        # 모터 제어
├── 📂 Software/           # RPi (Vision) 소스코드
│   ├── 📂 Vision/         # 파이썬, OpenCV 코드
│   └── 📂 Tests/          # 단위 테스트용 스크립트
└── 📄 README.md           # 프로젝트 개요
