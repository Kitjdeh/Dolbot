// package com.assc.dolbot.controller;
//
// import static org.mockito.BDDMockito.*;
// import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.*;
//
// import org.junit.jupiter.api.DisplayName;
// import org.junit.jupiter.api.Test;
// import org.springframework.beans.factory.annotation.Autowired;
// import org.springframework.boot.test.autoconfigure.web.servlet.WebMvcTest;
// import org.springframework.boot.test.mock.mockito.MockBean;
// import org.springframework.data.jpa.mapping.JpaMetamodelMappingContext;
// import org.springframework.http.MediaType;
// import org.springframework.test.web.servlet.MockMvc;
// import org.springframework.test.web.servlet.ResultActions;
// import org.springframework.test.web.servlet.request.MockMvcRequestBuilders;
//
// import com.assc.dolbot.dto.UserInfoDto;
// import com.assc.dolbot.service.UserInfoService;
// import com.fasterxml.jackson.databind.ObjectMapper;
//
// @MockBean(JpaMetamodelMappingContext.class)
// @WebMvcTest(UserInfoController.class)
// @DisplayName("UserInfoController")
// class UserInfoControllerTest {
//
// 	@SuppressWarnings("SpringJavaInjectionPointsAutowiringInspection")
// 	@Autowired
// 	private MockMvc mockMvc;
//
// 	@MockBean
// 	UserInfoService userInfoService;
//
// 	@Test
// 	@DisplayName("로그인")
// 	public void userLogin() throws Exception {
// 		UserInfoDto userInfoDto = UserInfoDto.builder()
// 			.kakaoId(1)
// 			.build();
// 		UserInfoDto user = UserInfoDto.builder()
// 			.userId(1)
// 			.kakaoId(1)
// 			.build();
//
// 		String body = (new ObjectMapper()).writeValueAsString(userInfoDto);
// 		given(userInfoService.loginUserInfo(userInfoDto)).willReturn(user);
//
// 		//when
// 		ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.post("/api/v1/login")
// 			.content(body)
// 			.contentType(MediaType.APPLICATION_JSON)
// 			.accept(MediaType.APPLICATION_JSON)
// 		);
//
// 		//then
// 		resultActions
// 			.andExpect(status().isOk());
//
// 	}
//
// 	@Test
// 	@DisplayName("회원가입")
// 	public void userJoin() throws Exception {
// 		UserInfoDto userInfoDto = UserInfoDto.builder()
// 			.kakaoId(3)
// 			.build();
// 		UserInfoDto user = UserInfoDto.builder()
// 			.userId(3)
// 			.kakaoId(3)
// 			.build();
//
// 		String body = (new ObjectMapper()).writeValueAsString(userInfoDto);
// 		given(userInfoService.loginUserInfo(userInfoDto)).willReturn(user);
//
// 		//when
// 		ResultActions resultActions = mockMvc.perform(MockMvcRequestBuilders.post("/api/v1/login")
// 			.content(body)
// 			.contentType(MediaType.APPLICATION_JSON)
// 			.accept(MediaType.APPLICATION_JSON)
// 		);
//
// 		//then
// 		resultActions
// 			.andExpect(status().isOk());
//
// 	}
// }