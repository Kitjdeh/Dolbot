package com.assc.dolbot.repository;

import java.time.LocalDate;
import java.util.List;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import com.assc.dolbot.dto.ScheduleInfoDto;
import com.assc.dolbot.entity.ScheduleInfo;

public interface ScheduleInfoRepository extends JpaRepository<ScheduleInfo, Integer> {
	@Query("SELECT s FROM ScheduleInfo s "
		+ "WHERE FUNCTION('DATE_TRUNC', 'day', s.scheduleTime) = :date "
		+ "AND s.homeId = :homeId")
	List<ScheduleInfo> findByHomeIdAndDate(int homeId, LocalDate date);
}
