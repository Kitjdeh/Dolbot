DROP DATABASE IF EXISTS dolbot;

CREATE DATABASE dolbot;

USE dolbot;

CREATE TABLE user_info (
  user_id INT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
  user_name VARCHAR(50) NOT NULL,
  profile_image_url VARCHAR(500) NOT NULL,
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

INSERT INTO user_info (user_name, profile_image_url)
VALUES ('John Doe', 'https://example.com/profile.jpg'),
       ('Jane Doe', 'https://example.com/profile.jpg');

INSERT INTO home (robot_number, location)
VALUES (1, 'Living Room'),
       (2, 'Bedroom'),
       (3, 'Kitchen');

INSERT INTO user_home (user_id, home_id, nickname)
VALUES (1, 1, 'Living Room'),
       (2, 2, 'Bedroom'),
       (1, 3, 'Kitchen');

INSERT INTO schedule_info (home_id, schedule_time, content)
VALUES (1, '2023-03-22 08:00:00', 'Clean living room'),
       (1, '2023-03-23 10:00:00', 'Change bed sheets'),
       (1, '2023-03-23 11:00:00', 'Change bed sheets2'),
       (1, '2023-03-24 15:00:00', 'Prepare dinner');

INSERT INTO appliance (appliance_name)
VALUES ('TV'),
       ('Air conditioner'),
       ('Refrigerator');

INSERT INTO emergency (content)
VALUES ('Fire'),
       ('Flood'),
       ('Earthquake');

INSERT INTO room (room_name)
VALUES ('Living Room'),
       ('Bedroom'),
       ('Kitchen');

INSERT INTO log_list (home_id, log_date, picture_url)
VALUES (1, '2023-03-22', 'https://example.com/picture.jpg'),
       (2, '2023-03-23', 'https://example.com/picture.jpg'),
       (3, '2023-03-24', 'https://example.com/picture.jpg');

INSERT INTO appliance_log (log_list_id, appliance_id, room_id, log_time, is_on)
VALUES (1, 1, 1, '08:30:00', 1),
       (1, 2, 1, '09:00:00', 0),
       (2, 3, 2, '10:30:00', 1);

INSERT INTO emergency_log (log_list_id, emergency_id, log_time)
VALUES (1, 1, '10:00:00'),
       (2, 2, '11:00:00'),
       (3, 3, '12:00:00');

INSERT INTO schedule_log (log_list_id, content, log_time)
VALUES (1, 'Cleaning done', '09:00:00'),
       (2, 'Sheets changed', '11:00:00'),
       (3, 'Dinner prepared', '16:00:00');

SELECT * FROM schedule_info
WHERE DATE(schedule_time) = '2023-03-17'
AND home_id = 1;

SELECT * FROM schedule_info;

select current_date, current_timestamp from dual;

select * from user_info;

select * from home;

select * from user_home;