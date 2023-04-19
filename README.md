# 포트폴리오

<aside>
💡 🥇 **최우수상 수상** 🥇

</aside>

![Untitled](assets/Untitled.png)

![Untitled](assets/Untitled%201.png)

## 돌봇

서비스 내용 : 홈IoT 로봇을 이용한 독거 노인 돌봄 서비스

- 서비스 주요 기능 :
    - 로봇
        - 자율주행, 로봇 제어(SLAM, Path Planning, Follow the carrot)
        - 원격 IoT 가전 제어(UDP)
        - 음성을 통한 가전 기기 제어
        - 실시간 일정 알림
        - 응급 상황 감지 및 보호자 연락
        - 객체 인식 및 객체 추종
    - 어플
        - 실시간 스트리밍을 통한 영상 송출
        - 가전 기기 제어
        - 일 단위 로그 확인
        - 일정 등록
        
- 개발기간 : 2022.02.20~2022.04.07 (7주)
- 플랫폼 : 어플
- 담당 역할 : 6명
    - 정찬영 (팀장)  : ROS2기반 로봇 시스템 제어/알고리즘 구현 (SLAM, Path Planning, Follow the carrot), UDP통신 기반 IoT 기기 제어 기능 구현, 앱 - 로봇간 Socket 통신 구현
    - 김정민 : Backend(로그인, 로봇 세팅, 스케줄, 테스트 코드)
    - 기성도 : Frontend(UI/UX 담당)
    - 김원혁 : Unity 객체 생성 및 애니메이션 구현 , openCV, Lidar 센서를 이용한 객체 추종 구현, Dijkstra 알고리즘 구현
    - 이가옥 : ROS2, 일정알람, 음성제어 (TTS/STT), 임베디드-백엔드 연동(REST API), 임베디드-프론트 연동(Socketio), 영상 송출 (CCTV 기능)
    - 김도원 : Backend(스케줄, 로그, 소켓 서버), 배포
    
- 개발 환경
    - 언어 : Java11, Python, Dart
    - 서버 : AWS EC2, Ubuntu
    - 프레임워크 : Spring, Flutter, ROS2
    - DB : Mysql, S3
    - IDE : IntelliJ, VS code
    - API, 라이브러리 : Google Cloud Speech-to-text API, Socket IO
    - 통신 프로토콜 : WS, HTTPS, UDP

- 시연 영상 : [https://www.youtube.com/watch?v=JUCa4dl0fJo](https://www.youtube.com/watch?v=JUCa4dl0fJo)
- UCC : [https://youtu.be/yobItHNKP1M](https://www.youtube.com/watch?v=U6sROxQxg14)

## 서비스 소개

❓전자기기를 다루기 어려운 노인 분들을 돕는 로봇이 있을까요?

❓말로 설명해도 알아 듣는 로봇이 있을까요?

❓앱으로 어르신의 집을 관리 할 수 있을까요? 

❓집에 어르신이 잘 계신지 직접 눈으로 확인 할 수 있을까요?

![Untitled](assets/Untitled%202.png)

이 모든 기능을 가진 스마트홈 로봇 서비스 프로젝트

## 아키텍처(기술스택)

![아키텍처.png](assets/%25EC%2595%2584%25ED%2582%25A4%25ED%2585%258D%25EC%25B2%2598.png)

## 서비스 특장점

- 보호자를 위한 서비스
    - 원격 가전 제어(앱)
    - 보호 대상자 스케줄 관리
    - 알림 및 로그 제공
    - CCTV 열람
    - 보호 대상자의 응급상황 파악
- 어르신을 위한 서비스
    - 원격 가전 제어 (음성)
    - 일정 알림
    - 응급 상황을 보호자에게 알림
    - 24시 돌봄 서비스 (추종)
- 3자 통신
    - 프론트엔드 - 백엔드 - 로봇 3자간 통신

![Untitled](assets/Untitled%203.png)

## 기능 명세서

![Untitled](assets/Untitled%204.png)

![Untitled](assets/Untitled%205.png)

## ERD

![돌봇 (3).png](assets/%25EB%258F%258C%25EB%25B4%2587_(3).png)

## API 설계서

### REST API 명세서

![Untitled](assets/Untitled%206.png)

### SOCKET TOPIC

![Untitled](assets/Untitled%207.png)

## 서비스 목업

![Untitled](assets/Untitled.jpeg)

## 페이지 나열

- 로그인

![login.png](assets/login.png)

- 메인페이지

![main.png](assets/main.png)

- 방별 가전 목록 및 상태

![main2.png](assets/main2.png)

- 에어컨 상세 작동 설정

![aircon.png](assets/aircon.png)

- CCTV(수동모드)

![cctv1.png](assets/cctv1.png)

- CCTV(수동모드)

![cctv2.png](assets/cctv2.png)

- CCTV(수동모드)

![cctv3.png](assets/cctv3.png)

- 일정 리스트

![schedule3.png](assets/schedule3.png)

- 스케줄러

![schedule1.png](assets/schedule1.png)

- 일정 추가 및 수정

![schedule2.png](assets/schedule2.png)

- 일정 알림 시간 설정

![schedule4.png](assets/schedule4.png)

- 긴급 알림

![emergency.png](assets/emergency.png)

- 일 단위 로봇 작동 로그
    
    ![log.png](assets/log.png)
    

## 프로젝트 일정 관리

![Untitled](assets/Untitled%208.png)

![간트차트_백엔드.png](assets/%25EA%25B0%2584%25ED%258A%25B8%25EC%25B0%25A8%25ED%258A%25B8_%25EB%25B0%25B1%25EC%2597%2594%25EB%2593%259C.png)

![간트차트_임베디드 (2).png](assets/%25EA%25B0%2584%25ED%258A%25B8%25EC%25B0%25A8%25ED%258A%25B8_%25EC%259E%2584%25EB%25B2%25A0%25EB%2594%2594%25EB%2593%259C_(2).png)
