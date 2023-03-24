package com.assc.dolbot.repository;

import com.assc.dolbot.entity.EmergencyLog;
import com.assc.dolbot.entity.ScheduleLog;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.List;

public interface ScheduleLogRepository extends JpaRepository<ScheduleLog, Integer> {
    List<ScheduleLog> findByLogListId(int logListId);
}
