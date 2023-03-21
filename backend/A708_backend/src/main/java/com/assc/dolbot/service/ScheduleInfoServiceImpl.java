package com.assc.dolbot.service;

import java.time.LocalDate;
import java.util.ArrayList;
import java.util.Calendar;
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
		Date scheduleTime = scheduleInfoDto.getScheduleTime();
		// LocalDateTime localDateTime = scheduleTime.toInstant().atZone(ZoneId.systemDefault()).toLocalDateTime();
		// LocalDateTime updatedDateTime = localDateTime.minusHours(9);
		// ZonedDateTime zonedDateTime = updatedDateTime.atZone(ZoneId.systemDefault());
		// scheduleTime = Date.from(zonedDateTime.toInstant());
		LocalDate startDate = scheduleInfoDto.getStartDate();
		LocalDate endDate = scheduleInfoDto.getEndDate();

		// startDate부터 endDate까지 반복
		for (LocalDate date = startDate; !date.isAfter(endDate); date = date.plusDays(1)) {
			Calendar cal = Calendar.getInstance();
			cal.setTime(scheduleTime);
			cal.set(Calendar.YEAR, date.getYear());
			cal.set(Calendar.MONTH, date.getMonthValue()-1);
			cal.set(Calendar.DAY_OF_MONTH, date.getDayOfMonth());
			scheduleTime = cal.getTime();
			scheduleInfoDto.setScheduleTime(scheduleTime);
			System.out.println("cal" + cal.toString());
			System.out.println("cal.get"+cal.getTime());
			System.out.println("st"+scheduleTime);
			System.out.println("dto"+scheduleInfoDto);
			scheduleInfoRepository.save(scheduleInfoDto.toEntity());
		}
	}

	@Override
	public List<ScheduleInfoDto> findScheduleInfoList(int homeId, LocalDate localDate) throws Exception {
		List<ScheduleInfo> list = scheduleInfoRepository.findByHomeIdAndDate(homeId, localDate);
		List<ScheduleInfoDto> dtoList = new ArrayList<>();
		for(int i=0; i<list.size(); i++){
			dtoList.add(list.get(i).toDto());
		}
		return dtoList;
	}

	// 모든 entity 추가후 save 한번으로 업데이트 가능한지 확인 필요!!
	@Override
	public void modifyScheduleInfo(int scheduleInfoId, ScheduleInfoDto scheduleInfoDto) throws Exception {
		scheduleInfoDto.setScheduleId(scheduleInfoId);
		ScheduleInfo scheduleInfo = scheduleInfoRepository.findById(scheduleInfoId).get();
		scheduleInfo.setContent(scheduleInfoDto.getContent());
		scheduleInfo.setScheduleTime(scheduleInfoDto.getScheduleTime());
		System.out.println(scheduleInfo);
		scheduleInfoRepository.save(scheduleInfo);
	}

	@Override
	public void removeScheduleInfo(int scheduleId) throws Exception {
		scheduleInfoRepository.deleteById(scheduleId);
	}

}
