DROP DATABASE IF EXISTS dolbot;

CREATE DATABASE dolbot;

USE dolbot;

CREATE TABLE user_info (
  user_id INT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
  user_name VARCHAR(50) NOT NULL,
  kakao_id INT UNSIGNED NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

CREATE TABLE home (
  home_id INT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
  robot_number INT UNSIGNED NOT NULL,
  location VARCHAR(50) NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

CREATE TABLE user_home (
  user_home_id INT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
  user_id INT UNSIGNED NOT NULL,
  home_id INT UNSIGNED NOT NULL,
  nickname VARCHAR(50) NOT NULL,
  is_alarm BOOLEAN NOT NULL DEFAULT false,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  FOREIGN KEY (user_id) REFERENCES user_info(user_id),
  FOREIGN KEY (home_id) REFERENCES home(home_id)
);

CREATE TABLE schedule_info (
  schedule_id INT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
  home_id INT UNSIGNED NOT NULL,
  schedule_time TIMESTAMP NOT NULL,
  content VARCHAR(100) NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  FOREIGN KEY (home_id) REFERENCES home(home_id)
);

CREATE TABLE appliance (
  appliance_id INT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
  appliance_name VARCHAR(50) NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

CREATE TABLE emergency (
  emergency_id INT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
  content VARCHAR(50) NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

CREATE TABLE room (
  room_id INT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
  room_name VARCHAR(20) NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

CREATE TABLE log_list (
  log_list_id INT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
  home_id INT UNSIGNED NOT NULL,
  log_date DATE NOT NULL,
  picture_url VARCHAR(500) NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  FOREIGN KEY (home_id) REFERENCES home(home_id)
);

CREATE TABLE appliance_log (
  appliance_log_id INT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
  log_list_id INT UNSIGNED NOT NULL,
  appliance_id INT UNSIGNED NOT NULL,
  room_id INT UNSIGNED NOT NULL,
  log_time TIME NOT NULL,
  is_on BOOLEAN NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  FOREIGN KEY (log_list_id) REFERENCES log_list(log_list_id),
  FOREIGN KEY (appliance_id) REFERENCES appliance(appliance_id),
  FOREIGN KEY (room_id) REFERENCES room(room_id)
);

CREATE TABLE emergency_log (
  emergency_log_id INT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
  log_list_id INT UNSIGNED NOT NULL,
  emergency_id INT UNSIGNED NOT NULL,
  log_time TIME NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  FOREIGN KEY (log_list_id) REFERENCES log_list(log_list_id),
  FOREIGN KEY (emergency_id) REFERENCES emergency(emergency_id)
);

CREATE TABLE schedule_log (
  schedule_log_id INT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY ,
  log_list_id INT UNSIGNED NOT NULL,
  content VARCHAR(100) NOT NULL,
  log_time TIME NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  FOREIGN KEY (log_list_id) REFERENCES log_list(log_list_id)
);

INSERT INTO user_info (user_name, kakao_id)
VALUES ('John Doe', 1),('KIM DO WON', 2);

INSERT INTO home (robot_number, location)
VALUES (1, 'Living Room'),
       (2, 'Bedroom'),
       (3, 'Kitchen');

INSERT INTO user_home (user_id, home_id, nickname)
VALUES (1, 1, '김원혁'),
       (2, 2, '김정민'),
       (1, 3, '기성도');

INSERT INTO schedule_info (home_id, schedule_time, content)
VALUES (1, '2023-03-22 08:00:00', '청소하기'),
       (1, '2023-03-23 10:00:00', '병원가기'),
       (1, '2023-03-23 11:00:00', '이불 정리 하기'),
       (1, '2023-03-24 15:00:00', '약 먹기');

INSERT INTO appliance (appliance_name)
VALUES ('TV'),
       ('Air conditioner'),
       ('Refrigerator');

INSERT INTO emergency (content)
VALUES ('화재'),
       ('기절'),
       ('쓰러짐');

INSERT INTO room (room_name)
VALUES ('Living Room'),
       ('Bedroom'),
       ('Kitchen');

INSERT INTO log_list (home_id, log_date, picture_url)
VALUES (1, '2023-03-22', 'https://dolbot.s3.ap-northeast-2.amazonaws.com/1679576099594.jpg'),
       (2, '2023-03-23', 'https://dolbot.s3.ap-northeast-2.amazonaws.com/1679578161983.jpg'),
       (3, '2023-03-24', 'https://dolbot.s3.ap-northeast-2.amazonaws.com/1679578161983.jpg');

INSERT INTO appliance_log (log_list_id, appliance_id, room_id, log_time, is_on)
VALUES (1, 1, 1, '08:30:00', true),
       (1, 2, 1, '09:00:00', false),
       (2, 3, 2, '10:30:00', true);

INSERT INTO emergency_log (log_list_id, emergency_id, log_time)
VALUES (1, 1, '10:00:00'),
       (2, 2, '11:00:00'),
       (3, 3, '12:00:00');

INSERT INTO schedule_log (log_list_id, content, log_time)
VALUES (1, '청소하기', '09:00:00'),
       (2, '병원가기', '11:00:00'),
       (3, '약먹기', '16:00:00');
