package com.assc.dolbot.controller;

import static org.mockito.BDDMockito.*;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.WebMvcTest;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.data.jpa.mapping.JpaMetamodelMappingContext;
import org.springframework.http.MediaType;
import org.springframework.test.web.servlet.MockMvc;
import org.springframework.test.web.servlet.ResultActions;
import org.springframework.test.web.servlet.request.MockMvcRequestBuilders;

import com.assc.dolbot.dto.UserHomeDto;
import com.assc.dolbot.service.UserHomeService;
import com.fasterxml.jackson.databind.ObjectMapper;

@MockBean(JpaMetamodelMappingContext.class)
@WebMvcTest(UserHomeController.class)
@DisplayName("UserHomeController")
class UserHomeControllerTest {

	@SuppressWarnings("SpringJavaInjectionPointsAutowiringInspection")
	@Autowired
	private MockMvc mockMvc;

	@MockBean
	UserHomeService userHomeService;

	@Test
	@DisplayName("로봇등록")
	public void userHomeAdd() throws Exception {
		UserHomeDto userHomeDto1 = UserHomeDto.builder()
			.userId(1)
			.nickname("할머니")
			.robotNumber(708001)
			.build();

		UserHomeDto userHomeDto2 = UserHomeDto.builder()
			.userHomeId(7)
			.userId(1)
			.homeId(1)
			.nickname("할머니")
			.status(0)
			.build();

		String body = (new ObjectMapper()).writeValueAsString(userHomeDto1);

		given(userHomeService.addUserHome(any(UserHomeDto.class))).willReturn(userHomeDto2);

		//when
		ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.post("/api/v1/user-homes")
			.content(body)
			.contentType(MediaType.APPLICATION_JSON)
			.accept(MediaType.APPLICATION_JSON)
		);

		//then
		resultActions
			.andExpect(status().isOk());

	}

	@Test
	@DisplayName("등록된 로봇들 정보 가져오기")
	public void userHomeListByUserId() throws Exception {
		List<UserHomeDto> list = new ArrayList<>();
		UserHomeDto userHomeDto1 = UserHomeDto.builder()
			.userHomeId(1)
			.userId(1)
			.homeId(1)
			.nickname("김원혁")
			.isAlarm(false)
			.build();
		UserHomeDto userHomeDto2 = UserHomeDto.builder()
			.userHomeId(2)
			.userId(1)
			.homeId(2)
			.nickname("이가옥")
			.isAlarm(false)
			.build();
		UserHomeDto userHomeDto3 = UserHomeDto.builder()
			.userHomeId(3)
			.userId(1)
			.homeId(3)
			.nickname("정찬영")
			.isAlarm(false)
			.build();

		list.add(userHomeDto1);
		list.add(userHomeDto2);
		list.add(userHomeDto3);

		given(userHomeService.findUserHomeListByUserId(Mockito.anyInt())).willReturn(list);

		//when
		ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.get("/api/v1/user-homes/robots/{user_id}", 1)
			.contentType(MediaType.APPLICATION_JSON)
			.accept(MediaType.APPLICATION_JSON)
		);

		//then
		resultActions
			.andExpect(status().isOk());

	}

	@Test
	@DisplayName("등록된 유저들 정보 가져오기 만들기")
	public void userHomeListByRobotNumber() throws Exception {
		List<UserHomeDto> list = new ArrayList<>();
		UserHomeDto userHomeDto1 = UserHomeDto.builder()
			.userHomeId(1)
			.userId(1)
			.homeId(1)
			.nickname("김원혁")
			.isAlarm(false)
			.build();
		UserHomeDto userHomeDto2 = UserHomeDto.builder()
			.userHomeId(4)
			.userId(2)
			.homeId(1)
			.nickname("김정민")
			.isAlarm(false)
			.build();
		list.add(userHomeDto1);
		list.add(userHomeDto2);

		given(userHomeService.findUserHomeListByRobotNumber(Mockito.anyInt())).willReturn(list);

		//when
		ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.get("/api/v1/user-homes/users/{robot_number}", 1)
			.contentType(MediaType.APPLICATION_JSON)
			.accept(MediaType.APPLICATION_JSON)
		);

		//then
		resultActions
			.andExpect(status().isOk());

	}

	@Test
	@DisplayName("로봇의 알람 설정")
	public void userHomeModify() throws Exception {
		UserHomeDto userHomeDto = UserHomeDto.builder()
			.isAlarm(true)
			.build();

		String body = (new ObjectMapper()).writeValueAsString(userHomeDto);

		//when
		ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.patch("/api/v1/user-homes/{user_home_id}", 1)
			.content(body)
			.contentType(MediaType.APPLICATION_JSON)
			.accept(MediaType.APPLICATION_JSON)
		);

		//then
		resultActions
			.andExpect(status().isOk());
	}

	@Test
	@DisplayName("로봇해제")
	public void userHomeRemove() throws Exception {
		//when
		ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.delete("/api/v1/user-homes/{user_home_id}", 1)
			.contentType(MediaType.APPLICATION_JSON)
			.accept(MediaType.APPLICATION_JSON)
		);

		//then
		resultActions
			.andExpect(status().isOk());
	}
}