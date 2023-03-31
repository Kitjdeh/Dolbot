package com.assc.dolbot.repository;

import com.assc.dolbot.entity.ApplianceLog;
import com.assc.dolbot.entity.LogList;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.sql.Date;

public interface LogListRepository extends JpaRepository<LogList, Integer> {

    @Query(nativeQuery = true, value = "SELECT * FROM log_list WHERE home_id = :homeId AND log_date = :logDate")
    LogList findByHomeIdAndLogDate(int homeId, Date logDate);

    @Query(nativeQuery = true, value = "SELECT COUNT(*) FROM log_list WHERE home_id = :homeId AND log_date = :logDate")
    long countLogListByHomeIdAndLogDate(int homeId, Date logDate);
}
