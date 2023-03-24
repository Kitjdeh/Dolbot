package com.assc.dolbot.repository;


import com.assc.dolbot.entity.ApplianceLog;
import com.assc.dolbot.entity.ScheduleInfo;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.List;

public interface ApplianceLogRepository extends JpaRepository<ApplianceLog, Integer> {
    List<ApplianceLog> findByLogListId(int logListId);
}
