# 돌봇 포팅 메뉴얼

![Untitled](assets/Untitled.png)


### A708 : 돌봇

삼성청년SW 아카데미 8기 서울캠퍼스
특화 프로젝트 (7주, 2022.02.20 -- 2022.04.07)

담당 컨설턴드 : 고성현
정찬영(팀장), 기성도, 김도원, 김원혁, 김정민, 이가옥





## 1. 프론트 포팅 메뉴얼

- APK파일 다운로드
- 무시하고 설치

![Untitled](assets/Untitled%201.png)

- 바탕화면에 설치된 APK 파일 확인

![Untitled](assets/Untitled%202.png)

- 실행

![Untitled](assets/Untitled%203.png)

## 2. 백엔드 포팅 메뉴얼

### ubuntu Linux 환경 설정

- sudo apt-get install git 명령어를 입력하여 패키지 리스트를 업데이트

- sudo apt install git 명령어를 입력하여 깃을 설치
- sudo snap install docker 명령어로 도커 설치

### Git clone

- git clone https://lab.ssafy.com/s08-mobility-smarthome-sub2/S08P22A708.git

### SSL 인증서 발급

- cd 명령으로 클론한 S08P22A708 디렉토리로 이동
- init-letsencrypt.sh 내부에 적용되어 잇는 도메인을 사용할 도메인으로 변경 후 적용
- sudo docker compose build 로 도커이미지 빌드
- chmod +x init-letsencrypt.sh 명령어로 권한 부여
- ./init-letsencrypt.sh 명령어로 실행
- 실행 이후 sudo docker comopse down으로 실행된 container들 종료

### 실행

- sudo docker compose build 로 도커 이미지 빌드
- sudo docker compose up 명령으로 실행

### 계정 및 정의 파일 목록

    1. SSL 인증서 발급을 위한 실행 파일 ./init-letsencrypt.sh
    2. spring boot 서버를 위한 ./backend/A708_backend_spring/src/main/resources/application.properties
    3. 도커 컴포즈 파일 ./docker-compose.yml
    4. 도커 파일들 
    - ./backend/A708_backend_spring/Dockerfile
    - ./backend/A708_backend_proxyserver/Dockerfile
    - ./backend/A708_backend_db/MYSQL/Dockerfile

    5. mysql ID: A708 PW: A708

### DB 덤프 파일

-  [Dump20230407.sql](Dump20230407.sql) 

## 3. IoT 포팅 메뉴얼

### 설치 라이브러리 및 환경 변수 설정

- 환경 변수 설정
GOOGLE_APPLICATION_CREDENTIALS = C:/TTS/vibrant-period-381607-92ab31325bad.json
- [socket.io](http://socket.io) 버전 맞추기
    - pip install python-socketio==5.8.0
- pip install playsound==1.2.2
- pip install gTTS==2.3.1
- pip install --upgrade google-cloud-speech
- pip install pyaudio==0.2.13
- pip install pynput
- pip install keyboard
- C:\ 드라이브에 TTS 파일 생성 후 아래 파일 넣기
  
    [vibrant-period-381607-92ab31325bad (1).json](%E1%84%83%E1%85%A9%E1%86%AF%E1%84%87%E1%85%A9%E1%86%BA%20%E1%84%91%E1%85%A9%E1%84%90%E1%85%B5%E1%86%BC%20%E1%84%86%E1%85%A6%E1%84%82%E1%85%B2%E1%84%8B%E1%85%A5%E1%86%AF%20f136d44c54034decb7fab33cc23a6bc1/vibrant-period-381607-92ab31325bad_(1).json)
    

```json
{
  "type": "service_account",
  "project_id": "vibrant-period-381607",
  "private_key_id": "92ab31325bad9744bce53ab1b222e814dde36084",
  "private_key": "-----BEGIN PRIVATE KEY-----\nMIIEvAIBADANBgkqhkiG9w0BAQEFAASCBKYwggSiAgEAAoIBAQDJjn0QFqNC3u4M\nVEtiil4ETYHi8O3m8D/cwyGkn0tm7wc5o0ICj2Wwk3BO891QFaJ8K+7/ArTJEsoE\n8vWyOAEzaa86ObH7wSixW1cu3AUy+DBZy+Q3Roa1yKl04PbJXPSuVz2LDxRXleIw\nKaEA7PSnAxp0jxdPDAWcxhnWCWd9jOd2Q4F14JOCE/SAJLrIYp77hrtj+K6ivyZS\nWA4OWH2lTK1Dd+8ITuG6l7ctvZV+v1BfKoNLN1uGZ2c1EBlzAlwgKvA/58mHpMGp\nB3Of3TmKvF+qufhrNfztTixLtnK4Rn5MqCE2gXAlPrXzmRnX+O4XkiNjygEftTMe\nAokRfttZAgMBAAECggEABAUzPaKULt2dfUWwuCOvE7sd01mRXFn//6iChrozkrdD\nZfbO6F9GwFGuxlt+rLbRwoz4At7TSBqT83H8vp6+sSosKZkLdT4yUF9UYGl1h8GD\nzWFlM9dGZpmgYXQbu59iQ1+I/p0yuFehypVBGXCgiRhI3OqMa+sblKmDy/fsuDLn\nOFJTVbLLw6uC54Ab9rajjlqw0OLASnxqzEmqj5i2cn8xc3JG/jtq03QVfWeSD6yf\nnbfiYP9wODeqzed3E7WFiwu97O9jVDAQrk9uB3V3DgmS658PxINacdDQdTLX2vsX\nWib1CqWSOQ+xyjGoy7Szp9TC2jLbqa9KQw6wEyc1lQKBgQD0CzgUJcNQp6UQZUJ5\nUsyxYG19DovvCE6qENQmPs74NkM8gNUN5YiZlePNRAEajeL6Ch7bpf2pOCNoX+53\njlgtqm+r5Sf1Wcm1s+VQXLn3RvRCa31F6keG3eU++sO1DD1lsPESaCgLTodj00ui\niAviUQO6cP20kVC6gS/yBJ1p8wKBgQDTbmXGN6cgHvcJSiSH+RGfE8sjIVonLOTo\net6KoPY+oYYh4zhH5J1MIP/jF/wqfeFUHHAvr9YVKTs+bq25NP4DyYlg0FdgERPo\nezypTPEGQzkvROXo4HQdtJm5MyTiyNsCeYkhExN8Cn9+UdkTpczWJJsoX83qJAUL\na749pmzMgwKBgCxS6VGHdkzs1eb/bS5Z4oGI0Pn4rWOxr4/l1JJJsA+W02lmwMtw\nV/oFJkW+xzCKeqINOoOMR2D3qD6pxccDHHLW1Md0rGg8cY1F6i1JWiFWSsDRgwQy\nw+Dz5lIk+2yjl3cjxvUf474B+kcgNkmDyhzFuBahtQKZrf7hvdKIEgsPAoGAKvtX\nb0tjC1GgMOBEuuu0XF1+v16JYEttIaHQ2efNB4B4ryztBRiBrbdLrdPzbIP4qcu6\nNseNQvIF3DRN1sfiF80Bw4+3I411OZ7L/zLoELjbyDM+dfAjSuKrGY4/ImdotoEU\nkoe5P083CewDsE3VbXc1n6iSSNYXvHGj1A6gMZcCgYBsxcTLhUAbDaig12wwq7W2\n6kVQVV03AXYmctqbk8ow61qcEP8nohhj3F/jSpNGMjTvC1GpRjyrWsezxvjsq87x\nV+zu7iiK3vT+Fpq2d+Y6RlY3Dp2UMVC6mBPmFhTYz4hFw+xk14wrNsR6Soq6Oc2h\nnf32mwx+PIfx9ThTYg7PSA==\n-----END PRIVATE KEY-----\n",
  "client_email": "ssafy-a708@vibrant-period-381607.iam.gserviceaccount.com",
  "client_id": "103684847748474924736",
  "auth_uri": "https://accounts.google.com/o/oauth2/auth",
  "token_uri": "https://oauth2.googleapis.com/token",
  "auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
  "client_x509_cert_url": "https://www.googleapis.com/robot/v1/metadata/x509/ssafy-a708%40vibrant-period-381607.iam.gserviceaccount.com"
}
```

### 시뮬레이터 다운로드

⬇ 시뮬레이터 다운 링크

- 개발 환경 구축 메뉴얼 & SSAFY 시뮬레이터 메뉴얼 참고 환경 구축
- smarthome3를 사용하기 때문에 명세서 중 Udp or smarthome3에 관한 설정은 꼼꼼히 설정해주어야한다.

[모빌리티(스마트홈) - Google Drive](https://drive.google.com/drive/folders/11r-nBPexjBe9cpDHlt6Q4ur0fdvXVFEI)

### Github clone

⬇ Gitlab Clone 주소

[](https://lab.ssafy.com/s08-mobility-smarthome-sub2/S08P22A708.git)

- 마스터 로컬로 Clone
  
    ![Untitled](assets/Untitled%204.png)
    
- 다음과 같이 embedded\A708_embedded 폴더로 이동
  
    ![Untitled](assets/Untitled%205.png)
    
- 실행시킬 수 있는 환경을 만들기 위해 build.bat 파일 실행 ( 관리자 권환으로 실행된 Native tools Command Prompt for VS2019 실행 후 해당 폴더로 이동 후 실행)
- 다음과 같이 세 개의 파일이 finished되면 다음으로 넘어가면 된다.

![Untitled](assets/Untitled%206.png)

### Unity Asset 다운로드

### 에셋 이동

- 다운 받은 에셋들을 압축 해제 후 지정된 경로로 이동시켜주어야한다.
- C:\Users\SSAFY\Desktop\SSAFYLauncher_SSAFY_Win\SSAFYLauncher_SSAFY_Win_Data\Bundle\v.4.5.210806.H4
- 해당 경로에 압축 해제해주면 된다.

### 인테리어 파일 이동

- home.json 파일도 해당 경로로 이동시켜주어야한다.
- C:\Users\SSAFY\Desktop\SSAFYLauncher_SSAFY_Win\SSAFYLauncher_SSAFY_Win_Data\SaveFile\Scenario\V_IND_SmartHome3
- home.json 파일을 다음과 같이 위치시켜주면 완료

![Untitled](assets/Untitled%207.png)

![Untitled](assets/Untitled%208.png)

### 시뮬레이터 실행 및 코드 작동

- 시뮬레이터 실행 후 주어진 아이디 비밀번호 입력 후 최신 버전으로 실행
  
    ![Untitled](assets/Untitled%209.png)
    

![Untitled](assets/Untitled%2010.png)

- Map : Smarthome3 선택
- Custom : TurtleBot 선택

![Untitled](assets/Untitled%2011.png)

![Untitled](assets/Untitled%2012.png)

- 위와 같이 SSAFY >> Save & Load >> home.json 로드해주면 기본 세팅은 완료된다.

![Untitled](assets/Untitled%2013.png)

(ps. 정신 없이 돌아다니는 ai를 억제하기 위해서는 Edit >> Scenario >> Scenario Edit mode 이동 후 도둑 객체를 직접 클릭 >> moving object setting 을 Manual Mode 로 바꿔주면 된다.

## 시연 시나리오

1. 시연
    - 사전 설명
        - 구성한 맵 (집) 소개 (아주 간략하게 언급)
        - 움직이는 객체를 넣지 못함을 소개 (유니티에서 할아버지 움직이는 움짤 넣기)
    - 직접 시연
        1. **간단한 시뮬레이터 환경 보여주기**
            1. WASD 방향키 조작으로 Unity 환경에서 구성한 asset들 보여주기
        2. **앱을 통한 가전 제어 ( 로봇 시점으로 고정, 로봇이 주행모습/ 로봇cam 함께 보여주기, TV를 켜는게 베스트)**
            1. 다운 받은 APK 로그인 후 메인 홈 화면에서 ON/OFF 조절로 명령 실행
        3. **STT를 통한 가전 제어 (자유시점 (공기청정기 바라보고있는 시점) /가전제어에 집중)**
            1. 돌쇠야 ! 호출 시 돌봇이 반응 
            2. 작은 방/ 거실/ 서재/ 안방 안의 에어컨/ TV/ 공기청정기의 키고 끄는 것을 명령으로 확인 가능 ( ex : 작은 방 TV 꺼 줘)
        4. **로그 기록** 
            1. 로그 기록이 남아 있는 파트로 이동하여 확인
        5. **CCTV 시연**
            1. CCTV 들어갈 시 CCTV 화면이 잘 나오고 있는 화면을 확인
            2. 수동으로 전환 후 각 방을 클릭하여 해당 방으로 이동하는 상황을 시연
