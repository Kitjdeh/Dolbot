package com.assc.dolbot.controller;

import static org.mockito.BDDMockito.*;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.*;

import java.time.LocalDate;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.WebMvcTest;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.data.jpa.mapping.JpaMetamodelMappingContext;
import org.springframework.http.MediaType;
import org.springframework.test.web.servlet.MockMvc;
import org.springframework.test.web.servlet.ResultActions;
import org.springframework.test.web.servlet.request.MockMvcRequestBuilders;

import com.assc.dolbot.dto.ScheduleInfoDto;
import com.assc.dolbot.service.ScheduleInfoService;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.datatype.jsr310.JavaTimeModule;

@MockBean(JpaMetamodelMappingContext.class)
@WebMvcTest(ScheduleInfoController.class)
@DisplayName("ScheduleInfoController")
class ScheduleInfoControllerTest {

	@SuppressWarnings("SpringJavaInjectionPointsAutowiringInspection")
	@Autowired
	private MockMvc mockMvc;

	@MockBean
	ScheduleInfoService scheduleInfoService;

	@Test
	@DisplayName("스케줄 추가하기")
	public void scheduleInfoAdd() throws Exception {
		ScheduleInfoDto scheduleInfoDto = ScheduleInfoDto.builder()
			.startDate(LocalDate.parse("2023-03-25"))
			.endDate(LocalDate.parse("2023-03-26"))
			.scheduleTime(Date.from(ZonedDateTime.parse("2023-03-25T09:00:00.000+09:00").toInstant()))
			.content("약 먹기")
			.homeId(1)
			.build();

		String body = (new ObjectMapper().registerModule(new JavaTimeModule())).writeValueAsString(scheduleInfoDto);

		//when
		ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.post("/api/v1/schedule-info")
			.content(body)
			.contentType(MediaType.APPLICATION_JSON)
			.accept(MediaType.APPLICATION_JSON)
		);

		//then
		resultActions
			.andExpect(status().isOk());
	}

	@Test
	@DisplayName("날짜에 맞는 스케줄 불러오기")
	public void scheduleInfoList() throws Exception {
		List<ScheduleInfoDto> list = new ArrayList<>();
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

		list.add(scheduleInfoDto1);
		list.add(scheduleInfoDto2);

		given(scheduleInfoService.findScheduleInfoList(1, LocalDate.parse("2023-02-20"))).willReturn(list);

		//when
		ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.get("/api/v1/schedule-info//{home_id}", 1)
			.param("localDate", LocalDate.parse("2023-02-20").toString())
			.contentType(MediaType.APPLICATION_JSON)
			.accept(MediaType.APPLICATION_JSON)
		);

		//then
		resultActions
			.andExpect(status().isOk());
	}

	@Test
	@DisplayName("스케줄 수정하기")
	public void scheduleInfoModify() throws Exception {
		ScheduleInfoDto scheduleInfoDto = ScheduleInfoDto.builder()
			.scheduleTime(Date.from(ZonedDateTime.parse("2023-03-25T09:00:00.000+09:00").toInstant()))
			.content("약 먹기")
			.build();

		String body = (new ObjectMapper().registerModule(new JavaTimeModule())).writeValueAsString(scheduleInfoDto);

		//when
		ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.patch("/api/v1/schedule-info/{schedule_id}", 1)
			.content(body)
			.contentType(MediaType.APPLICATION_JSON)
			.accept(MediaType.APPLICATION_JSON)
		);

		//then
		resultActions
			.andExpect(status().isOk());
	}

	@Test
	@DisplayName("스케줄 삭제하기")
	public void scheduleInfoRemove() throws Exception {
		//when
		ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.delete("/api/v1/schedule-info/{schedule_id}", 1)
			.contentType(MediaType.APPLICATION_JSON)
			.accept(MediaType.APPLICATION_JSON)
		);

		//then
		resultActions
			.andExpect(status().isOk());
	}
}