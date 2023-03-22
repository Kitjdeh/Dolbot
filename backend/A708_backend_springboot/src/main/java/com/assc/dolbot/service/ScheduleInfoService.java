package com.assc.dolbot.service;

import java.time.LocalDate;
import java.util.List;

import com.assc.dolbot.dto.ScheduleInfoDto;

public interface ScheduleInfoService {
	// 스케줄 추가
	public void addScheduleInfo(ScheduleInfoDto scheduleInfoDto) throws Exception;
	// 날짜에 맞는 스케줄 선택
	public List<ScheduleInfoDto> findScheduleInfoList(int homeId, LocalDate localDate) throws Exception;
	// 스케줄 수정
	public void modifyScheduleInfo(int scheduleInfoId, ScheduleInfoDto scheduleInfoDto) throws Exception;
	// 스케줄 제거
	public void removeScheduleInfo(int scheduleId) throws Exception;
}
