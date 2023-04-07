-- MySQL dump 10.13  Distrib 8.0.32, for Win64 (x86_64)
--
-- Host: localhost    Database: dolbot
-- ------------------------------------------------------
-- Server version	8.0.32

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `appliance`
--

DROP TABLE IF EXISTS `appliance`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `appliance` (
  `appliance_id` int unsigned NOT NULL AUTO_INCREMENT,
  `appliance_name` varchar(50) NOT NULL,
  `created_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`appliance_id`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `appliance`
--

LOCK TABLES `appliance` WRITE;
/*!40000 ALTER TABLE `appliance` DISABLE KEYS */;
INSERT INTO `appliance` VALUES (1,'조명','2023-04-03 06:43:03','2023-04-03 06:43:03'),(2,'에어컨','2023-04-03 06:43:03','2023-04-03 06:43:03'),(3,'TV','2023-04-03 06:43:03','2023-04-03 06:43:03'),(4,'공기청정기','2023-04-03 06:43:03','2023-04-03 06:43:03');
/*!40000 ALTER TABLE `appliance` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `appliance_log`
--

DROP TABLE IF EXISTS `appliance_log`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `appliance_log` (
  `appliance_log_id` int unsigned NOT NULL AUTO_INCREMENT,
  `log_list_id` int unsigned NOT NULL,
  `appliance_id` int unsigned NOT NULL,
  `room_id` int unsigned NOT NULL,
  `log_time` time NOT NULL,
  `is_on` tinyint(1) NOT NULL,
  `created_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`appliance_log_id`),
  KEY `log_list_id` (`log_list_id`),
  KEY `appliance_id` (`appliance_id`),
  KEY `room_id` (`room_id`),
  CONSTRAINT `appliance_log_ibfk_1` FOREIGN KEY (`log_list_id`) REFERENCES `log_list` (`log_list_id`),
  CONSTRAINT `appliance_log_ibfk_2` FOREIGN KEY (`appliance_id`) REFERENCES `appliance` (`appliance_id`),
  CONSTRAINT `appliance_log_ibfk_3` FOREIGN KEY (`room_id`) REFERENCES `room` (`room_id`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `appliance_log`
--

LOCK TABLES `appliance_log` WRITE;
/*!40000 ALTER TABLE `appliance_log` DISABLE KEYS */;
INSERT INTO `appliance_log` VALUES (1,1,1,1,'08:30:00',1,'2023-04-03 06:43:03','2023-04-03 06:43:03'),(2,1,2,1,'09:00:00',0,'2023-04-03 06:43:03','2023-04-03 06:43:03'),(3,2,3,2,'10:30:00',1,'2023-04-03 06:43:03','2023-04-03 06:43:03');
/*!40000 ALTER TABLE `appliance_log` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `emergency`
--

DROP TABLE IF EXISTS `emergency`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `emergency` (
  `emergency_id` int unsigned NOT NULL AUTO_INCREMENT,
  `content` varchar(50) NOT NULL,
  `created_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`emergency_id`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `emergency`
--

LOCK TABLES `emergency` WRITE;
/*!40000 ALTER TABLE `emergency` DISABLE KEYS */;
INSERT INTO `emergency` VALUES (1,'구조요청','2023-04-03 06:43:03','2023-04-03 06:43:03'),(2,'태풍','2023-04-03 06:43:03','2023-04-03 06:43:03'),(3,'화재','2023-04-03 06:43:03','2023-04-03 06:43:03'),(4,'쓰러짐','2023-04-03 06:43:03','2023-04-03 06:43:03');
/*!40000 ALTER TABLE `emergency` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `emergency_log`
--

DROP TABLE IF EXISTS `emergency_log`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `emergency_log` (
  `emergency_log_id` int unsigned NOT NULL AUTO_INCREMENT,
  `log_list_id` int unsigned NOT NULL,
  `emergency_id` int unsigned NOT NULL,
  `log_time` time NOT NULL,
  `created_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`emergency_log_id`),
  KEY `log_list_id` (`log_list_id`),
  KEY `emergency_id` (`emergency_id`),
  CONSTRAINT `emergency_log_ibfk_1` FOREIGN KEY (`log_list_id`) REFERENCES `log_list` (`log_list_id`),
  CONSTRAINT `emergency_log_ibfk_2` FOREIGN KEY (`emergency_id`) REFERENCES `emergency` (`emergency_id`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `emergency_log`
--

LOCK TABLES `emergency_log` WRITE;
/*!40000 ALTER TABLE `emergency_log` DISABLE KEYS */;
INSERT INTO `emergency_log` VALUES (1,1,1,'10:00:00','2023-04-03 06:43:03','2023-04-03 06:43:03'),(2,2,2,'11:00:00','2023-04-03 06:43:03','2023-04-03 06:43:03'),(3,3,3,'12:00:00','2023-04-03 06:43:03','2023-04-03 06:43:03');
/*!40000 ALTER TABLE `emergency_log` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `home`
--

DROP TABLE IF EXISTS `home`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `home` (
  `home_id` int unsigned NOT NULL AUTO_INCREMENT,
  `robot_number` int unsigned NOT NULL,
  `location` varchar(50) NOT NULL,
  `created_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`home_id`),
  UNIQUE KEY `robot_number` (`robot_number`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `home`
--

LOCK TABLES `home` WRITE;
/*!40000 ALTER TABLE `home` DISABLE KEYS */;
INSERT INTO `home` VALUES (1,708001,'서울','2023-04-03 06:43:03','2023-04-03 06:43:03'),(2,708002,'부산','2023-04-03 06:43:03','2023-04-03 06:43:03'),(3,708003,'제주도','2023-04-03 06:43:03','2023-04-03 06:43:03');
/*!40000 ALTER TABLE `home` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `log_list`
--

DROP TABLE IF EXISTS `log_list`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `log_list` (
  `log_list_id` int unsigned NOT NULL AUTO_INCREMENT,
  `home_id` int unsigned NOT NULL,
  `log_date` date NOT NULL,
  `picture_url` varchar(500) NOT NULL,
  `created_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`log_list_id`),
  KEY `home_id` (`home_id`),
  CONSTRAINT `log_list_ibfk_1` FOREIGN KEY (`home_id`) REFERENCES `home` (`home_id`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `log_list`
--

LOCK TABLES `log_list` WRITE;
/*!40000 ALTER TABLE `log_list` DISABLE KEYS */;
INSERT INTO `log_list` VALUES (1,1,'2023-03-22','https://dolbot.s3.ap-northeast-2.amazonaws.com/1679576099594.jpg','2023-04-03 06:43:03','2023-04-03 06:43:03'),(2,2,'2023-03-23','https://dolbot.s3.ap-northeast-2.amazonaws.com/1679578161983.jpg','2023-04-03 06:43:03','2023-04-03 06:43:03'),(3,3,'2023-03-24','https://dolbot.s3.ap-northeast-2.amazonaws.com/1679578161983.jpg','2023-04-03 06:43:03','2023-04-03 06:43:03');
/*!40000 ALTER TABLE `log_list` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `room`
--

DROP TABLE IF EXISTS `room`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `room` (
  `room_id` int unsigned NOT NULL AUTO_INCREMENT,
  `room_name` varchar(20) NOT NULL,
  `created_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`room_id`)
) ENGINE=InnoDB AUTO_INCREMENT=7 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `room`
--

LOCK TABLES `room` WRITE;
/*!40000 ALTER TABLE `room` DISABLE KEYS */;
INSERT INTO `room` VALUES (1,'거실','2023-04-03 06:43:03','2023-04-03 06:43:03'),(2,'안방','2023-04-03 06:43:03','2023-04-03 06:43:03'),(3,'서재','2023-04-03 06:43:03','2023-04-03 06:43:03'),(4,'작은방','2023-04-03 06:43:03','2023-04-03 06:43:03'),(5,'화장실','2023-04-03 06:43:03','2023-04-03 06:43:03'),(6,'현관','2023-04-03 06:43:03','2023-04-03 06:43:03');
/*!40000 ALTER TABLE `room` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `schedule_info`
--

DROP TABLE IF EXISTS `schedule_info`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `schedule_info` (
  `schedule_id` int unsigned NOT NULL AUTO_INCREMENT,
  `home_id` int unsigned NOT NULL,
  `schedule_time` timestamp NOT NULL,
  `content` varchar(100) NOT NULL,
  `created_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`schedule_id`),
  KEY `home_id` (`home_id`),
  CONSTRAINT `schedule_info_ibfk_1` FOREIGN KEY (`home_id`) REFERENCES `home` (`home_id`)
) ENGINE=InnoDB AUTO_INCREMENT=23 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `schedule_info`
--

LOCK TABLES `schedule_info` WRITE;
/*!40000 ALTER TABLE `schedule_info` DISABLE KEYS */;
INSERT INTO `schedule_info` VALUES (1,1,'2023-03-21 23:00:00','청소하기','2023-04-03 06:43:03','2023-04-03 06:43:03'),(2,1,'2023-03-23 01:00:00','병원가기','2023-04-03 06:43:03','2023-04-03 06:43:03'),(3,1,'2023-03-23 02:00:00','이불 정리 하기','2023-04-03 06:43:03','2023-04-03 06:43:03'),(4,1,'2023-03-24 06:00:00','약 먹기','2023-04-03 06:43:03','2023-04-03 06:43:03'),(5,1,'2023-03-24 15:25:00','약 먹기','2023-04-03 06:44:52','2023-04-03 06:44:52'),(6,1,'2023-03-25 15:25:00','약 먹기','2023-04-03 06:44:52','2023-04-03 06:44:52'),(7,1,'2023-03-24 15:25:00','약 먹기','2023-04-03 07:12:55','2023-04-03 07:12:55'),(8,1,'2023-03-25 15:25:00','약 먹기','2023-04-03 07:12:55','2023-04-03 07:12:55'),(9,1,'2023-03-25 14:25:00','약 먹기','2023-04-03 07:13:22','2023-04-03 07:13:22'),(10,1,'2023-03-26 14:25:00','약 먹기','2023-04-03 07:13:22','2023-04-03 07:13:22'),(11,1,'2023-03-25 14:25:00','약 먹기','2023-04-03 07:13:35','2023-04-03 07:13:35'),(12,1,'2023-03-26 14:25:00','약 먹기','2023-04-03 07:13:35','2023-04-03 07:13:35'),(13,1,'2023-03-25 14:25:00','약 먹기','2023-04-03 07:17:11','2023-04-03 07:17:11'),(14,1,'2023-03-26 14:25:00','약 먹기','2023-04-03 07:17:11','2023-04-03 07:17:11'),(15,1,'2023-03-24 15:25:00','약 먹기','2023-04-03 07:21:18','2023-04-03 07:21:18'),(16,1,'2023-03-25 15:25:00','약 먹기','2023-04-03 07:21:18','2023-04-03 07:21:18'),(17,1,'2023-03-24 15:25:00','약 먹기','2023-04-03 08:19:40','2023-04-03 08:19:40'),(18,1,'2023-03-25 15:25:00','약 먹기','2023-04-03 08:19:40','2023-04-03 08:19:40'),(19,1,'2023-03-24 15:25:00','약 먹기','2023-04-03 08:20:14','2023-04-03 08:20:14'),(20,1,'2023-03-25 15:25:00','약 먹기','2023-04-03 08:20:14','2023-04-03 08:20:14'),(21,1,'2023-03-25 00:25:00','약 먹기','2023-04-03 08:20:31','2023-04-03 08:20:31'),(22,1,'2023-03-26 00:25:00','약 먹기','2023-04-03 08:20:31','2023-04-03 08:20:31');
/*!40000 ALTER TABLE `schedule_info` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `schedule_log`
--

DROP TABLE IF EXISTS `schedule_log`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `schedule_log` (
  `schedule_log_id` int unsigned NOT NULL AUTO_INCREMENT,
  `log_list_id` int unsigned NOT NULL,
  `content` varchar(100) NOT NULL,
  `log_time` time NOT NULL,
  `created_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`schedule_log_id`),
  KEY `log_list_id` (`log_list_id`),
  CONSTRAINT `schedule_log_ibfk_1` FOREIGN KEY (`log_list_id`) REFERENCES `log_list` (`log_list_id`)
) ENGINE=InnoDB AUTO_INCREMENT=4 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `schedule_log`
--

LOCK TABLES `schedule_log` WRITE;
/*!40000 ALTER TABLE `schedule_log` DISABLE KEYS */;
INSERT INTO `schedule_log` VALUES (1,1,'청소하기','09:00:00','2023-04-03 06:43:03','2023-04-03 06:43:03'),(2,2,'병원가기','11:00:00','2023-04-03 06:43:03','2023-04-03 06:43:03'),(3,3,'약먹기','16:00:00','2023-04-03 06:43:03','2023-04-03 06:43:03');
/*!40000 ALTER TABLE `schedule_log` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `user_home`
--

DROP TABLE IF EXISTS `user_home`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `user_home` (
  `user_home_id` int unsigned NOT NULL AUTO_INCREMENT,
  `user_id` int unsigned NOT NULL,
  `home_id` int unsigned NOT NULL,
  `nickname` varchar(50) NOT NULL,
  `is_alarm` tinyint(1) NOT NULL DEFAULT '0',
  `created_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`user_home_id`),
  KEY `user_id` (`user_id`),
  KEY `home_id` (`home_id`),
  CONSTRAINT `user_home_ibfk_1` FOREIGN KEY (`user_id`) REFERENCES `user_info` (`user_id`),
  CONSTRAINT `user_home_ibfk_2` FOREIGN KEY (`home_id`) REFERENCES `home` (`home_id`)
) ENGINE=InnoDB AUTO_INCREMENT=7 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `user_home`
--

LOCK TABLES `user_home` WRITE;
/*!40000 ALTER TABLE `user_home` DISABLE KEYS */;
INSERT INTO `user_home` VALUES (1,1,1,'김원혁',0,'2023-04-03 06:43:03','2023-04-03 06:43:03'),(2,1,2,'이가옥',0,'2023-04-03 06:43:03','2023-04-03 06:43:03'),(3,1,3,'정찬영',0,'2023-04-03 06:43:03','2023-04-03 06:43:03'),(4,2,1,'김정민',0,'2023-04-03 06:43:03','2023-04-03 06:43:03'),(5,2,2,'김도원',0,'2023-04-03 06:43:03','2023-04-03 06:43:03'),(6,2,3,'기성도',0,'2023-04-03 06:43:03','2023-04-03 06:43:03');
/*!40000 ALTER TABLE `user_home` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `user_info`
--

DROP TABLE IF EXISTS `user_info`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `user_info` (
  `user_id` int unsigned NOT NULL AUTO_INCREMENT,
  `kakao_id` varchar(20) NOT NULL,
  `main_home_id` int unsigned NOT NULL DEFAULT '0',
  `created_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `updated_at` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (`user_id`),
  UNIQUE KEY `kakao_id` (`kakao_id`)
) ENGINE=InnoDB AUTO_INCREMENT=3 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `user_info`
--

LOCK TABLES `user_info` WRITE;
/*!40000 ALTER TABLE `user_info` DISABLE KEYS */;
INSERT INTO `user_info` VALUES (1,'1111111111',1,'2023-04-03 06:43:03','2023-04-03 06:43:03'),(2,'2222222222',2,'2023-04-03 06:43:03','2023-04-03 06:43:03');
/*!40000 ALTER TABLE `user_info` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2023-04-07 11:37:55
