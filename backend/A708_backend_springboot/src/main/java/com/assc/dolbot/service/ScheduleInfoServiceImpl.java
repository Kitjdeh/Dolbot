package com.assc.dolbot.service;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import com.assc.dolbot.dto.ScheduleInfoDto;
import com.assc.dolbot.entity.ScheduleInfo;
import com.assc.dolbot.repository.ScheduleInfoRepository;


@Service
public class ScheduleInfoServiceImpl implements ScheduleInfoService{
	@Autowired
	private ScheduleInfoRepository scheduleInfoRepository;

	@Override
	public void addScheduleInfo(ScheduleInfoDto scheduleInfoDto) throws Exception {
		ZonedDateTime scheduleTime = scheduleInfoDto.getScheduleTime().toInstant().atZone(ZoneId.of("Asia/Seoul"));
		LocalDate startDate = scheduleInfoDto.getStartDate();
		LocalDate endDate = scheduleInfoDto.getEndDate();

		// startDate부터 endDate까지 반복
		for (LocalDate date = startDate; !date.isAfter(endDate); date = date.plusDays(1)) {
			ZonedDateTime dateTime = LocalDateTime.of(date, scheduleTime.toLocalTime()).atZone(ZoneId.of("Asia/Seoul"));
			scheduleInfoDto.setScheduleTime(Date.from(dateTime.toInstant()));
			scheduleInfoRepository.save(scheduleInfoDto.toEntity());
		}
	}

	@Override
	public List<ScheduleInfoDto> findScheduleInfoList(int homeId, LocalDate localDate) throws Exception {
		List<ScheduleInfo> scheduleInfoList = scheduleInfoRepository.findByHomeIdAndDate(homeId, localDate);
		List<ScheduleInfoDto> scheduleInfoDtoList = new ArrayList<>();
		for(int i=0; i<scheduleInfoList.size(); i++){
			scheduleInfoDtoList.add(scheduleInfoList.get(i).toDto());
		}
		return scheduleInfoDtoList;
	}

	@Override
	public void modifyScheduleInfo(int scheduleInfoId, ScheduleInfoDto scheduleInfoDto) throws Exception {
		scheduleInfoDto.setScheduleId(scheduleInfoId);
		ScheduleInfo scheduleInfo = scheduleInfoRepository.findById(scheduleInfoId).get();
		scheduleInfo.setContent(scheduleInfoDto.getContent());
		scheduleInfo.setScheduleTime(scheduleInfoDto.getScheduleTime());
		scheduleInfoRepository.save(scheduleInfo);
	}

	@Override
	public void removeScheduleInfo(int scheduleId) throws Exception {
		scheduleInfoRepository.deleteById(scheduleId);
	}

}
