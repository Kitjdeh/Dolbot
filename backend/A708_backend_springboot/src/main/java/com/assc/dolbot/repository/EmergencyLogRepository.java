package com.assc.dolbot.repository;

import com.assc.dolbot.entity.EmergencyLog;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.List;

public interface EmergencyLogRepository extends JpaRepository<EmergencyLog, Integer> {
    List<EmergencyLog> findByLogListId(int logListId);
}
