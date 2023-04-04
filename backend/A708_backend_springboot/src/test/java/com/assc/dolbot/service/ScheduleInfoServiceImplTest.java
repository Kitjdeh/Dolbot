package com.assc.dolbot.service;

import static org.assertj.core.api.Assertions.*;
import static org.mockito.BDDMockito.*;

import java.time.LocalDate;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.springframework.boot.test.context.SpringBootTest;

import com.assc.dolbot.dto.ScheduleInfoDto;
import com.assc.dolbot.entity.ScheduleInfo;
import com.assc.dolbot.repository.ScheduleInfoRepository;

@SpringBootTest
@DisplayName("ScheduleInfoService")
class ScheduleInfoServiceImplTest {

	@Mock
	private ScheduleInfoRepository scheduleInfoRepository;

	@InjectMocks
	private ScheduleInfoServiceImpl scheduleInfoService;

	@Test
	@DisplayName("스케줄 추가")
	public void addScheduleInfo() throws Exception {
		// Given
		ScheduleInfoDto scheduleInfoDto = ScheduleInfoDto.builder()
			.startDate(LocalDate.parse("2023-03-25"))
			.endDate(LocalDate.parse("2023-03-25"))
			.scheduleTime(Date.from(ZonedDateTime.parse("2023-03-25T09:00:00.000+09:00").toInstant()))
			.content("약 먹기")
			.homeId(1)
			.build();

		ScheduleInfo scheduleInfo = ScheduleInfo.builder()
			.scheduleId(5)
			.scheduleTime(Date.from(ZonedDateTime.parse("2023-03-25T09:00:00.000+09:00").toInstant()))
			.content("약 먹기")
			.homeId(1)
			.build();

		given(scheduleInfoRepository.save(any())).willReturn(scheduleInfo);

		// When
		scheduleInfoService.addScheduleInfo(scheduleInfoDto);

		// Then
		verify(scheduleInfoRepository).save(any(ScheduleInfo.class));
	}

	@Test
	@DisplayName("날짜에 맞는 스케줄 선택")
	public void findScheduleInfoList() throws Exception {
		//given
		List<ScheduleInfo> list = new ArrayList<>();
		ScheduleInfo scheduleInfo1 = ScheduleInfo.builder()
			.scheduleId(2)
			.homeId(1)
			.scheduleTime(new Date(1679533200000L))
			.content("병원 가기")
			.build();
		ScheduleInfo scheduleInfo2 = ScheduleInfo.builder()
			.scheduleId(3)
			.homeId(1)
			.scheduleTime(new Date(1679536800000L))
			.content("약 먹기")
			.build();

		list.add(scheduleInfo1);
		list.add(scheduleInfo2);

		List<ScheduleInfoDto> listDto = new ArrayList<>();
		ScheduleInfoDto scheduleInfoDto1 = ScheduleInfoDto.builder()
			.scheduleId(2)
			.homeId(1)
			.scheduleTime(new Date(1679533200000L))
			.content("병원 가기")
			.build();
		ScheduleInfoDto scheduleInfoDto2 = ScheduleInfoDto.builder()
			.scheduleId(3)
			.homeId(1)
			.scheduleTime(new Date(1679536800000L))
			.content("약 먹기")
			.build();

		listDto.add(scheduleInfoDto1);
		listDto.add(scheduleInfoDto2);

		given(scheduleInfoRepository.findByHomeIdAndDate(1, LocalDate.parse("2023-02-20"))).willReturn(list);

		// when
		List<ScheduleInfoDto> result = scheduleInfoService.findScheduleInfoList(1, LocalDate.parse("2023-02-20"));

		// then
		assertThat(result.toString()).isEqualTo(listDto.toString());

	}

	@Test
	@DisplayName("스케줄 수정")
	public void modifyScheduleInfo() throws Exception {
		ScheduleInfoDto scheduleInfoDto = ScheduleInfoDto.builder()
			.scheduleTime(Date.from(ZonedDateTime.parse("2023-03-25T09:00:00.000+09:00").toInstant()))
			.content("약 먹기")
			.build();

		ScheduleInfo scheduleInfo = ScheduleInfo.builder()
			.scheduleId(1)
			.homeId(1)
			.scheduleTime(Date.from(ZonedDateTime.parse("2023-03-25T09:00:00.000+09:00").toInstant()))
			.content("약 먹기")
			.build();

		given(scheduleInfoRepository.findById(1)).willReturn(java.util.Optional.ofNullable(scheduleInfo));

		// when
		scheduleInfoService.modifyScheduleInfo(1, scheduleInfoDto);

		// then
		verify(scheduleInfoRepository).save(scheduleInfo);
	}

	@Test
	@DisplayName("스케줄 제거")
	public void removeScheduleInfo() throws Exception {
		// Given
		int scheduleId = 1;

		// When
		scheduleInfoService.removeScheduleInfo(scheduleId);

		// Then
		verify(scheduleInfoRepository).deleteById(scheduleId);
	}
}