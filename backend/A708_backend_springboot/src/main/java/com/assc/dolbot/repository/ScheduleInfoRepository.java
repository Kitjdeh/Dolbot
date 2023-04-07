package com.assc.dolbot.repository;

import java.time.LocalDate;
import java.util.List;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import com.assc.dolbot.entity.ScheduleInfo;

public interface ScheduleInfoRepository extends JpaRepository<ScheduleInfo, Integer> {
	@Query(nativeQuery = true, value = "SELECT * FROM schedule_info WHERE home_id = :homeId AND DATE(schedule_time) = :localDate ORDER BY schedule_time")
	List<ScheduleInfo> findByHomeIdAndDate(int homeId, LocalDate localDate);
}
