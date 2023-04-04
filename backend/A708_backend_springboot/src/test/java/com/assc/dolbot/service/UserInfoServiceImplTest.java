package com.assc.dolbot.service;

import static org.assertj.core.api.Assertions.*;
import static org.mockito.BDDMockito.*;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.springframework.boot.test.context.SpringBootTest;

import com.assc.dolbot.dto.UserInfoDto;
import com.assc.dolbot.entity.UserInfo;
import com.assc.dolbot.repository.UserInfoRepository;

@SpringBootTest
@DisplayName("UserInfoService")
class UserInfoServiceImplTest {

	@Mock
	private UserInfoRepository userInfoRepository;

	@InjectMocks
	private UserInfoServiceImpl userInfoService;

	@Test
	@DisplayName("로그인: 새로운 유저인 경우")
	public void testLoginUserInfoNewUser() throws Exception {
		// given
		UserInfoDto userInfoDto = UserInfoDto.builder().kakaoId("3333333333").build();
		given(userInfoRepository.findByKakaoId(userInfoDto.getKakaoId())).willReturn(null);
		given(userInfoRepository.save(any(UserInfo.class)))
			.willReturn(UserInfo.builder().userId(3).kakaoId("3333333333").mainHomeId(0).build());

		// when
		UserInfoDto result = userInfoService.loginUserInfo(userInfoDto);

		// then
		assertThat(result).isNotNull();
		assertThat(result.getUserId()).isEqualTo(3);
		assertThat(result.getKakaoId()).isEqualTo("3333333333");
		assertThat(result.isNew()).isTrue();
	}

	@Test
	@DisplayName("로그인: 기존 유저인 경우")
	public void testLoginUserInfoExistingUser() throws Exception {
		// given
		UserInfoDto userInfoDto = UserInfoDto.builder().kakaoId("1111111111").build();
		given(userInfoRepository.findByKakaoId(userInfoDto.getKakaoId()))
			.willReturn(UserInfo.builder().userId(1).kakaoId("1111111111").mainHomeId(0).build());

		// when
		UserInfoDto result = userInfoService.loginUserInfo(userInfoDto);

		// then
		assertThat(result).isNotNull();
		assertThat(result.getUserId()).isEqualTo(1);
		assertThat(result.getKakaoId()).isEqualTo("1111111111");
		assertThat(result.isNew()).isFalse();
	}

	@Test
	@DisplayName("사용자 기본 집 변경")
	public void modifyUserInfo() throws Exception {
		// given
		UserInfoDto userInfoDto = UserInfoDto.builder().mainHomeId(2).build();
		UserInfo userInfo = UserInfo.builder().userId(1).kakaoId("1111111111").mainHomeId(1).build();
		given(userInfoRepository.findById(1)).willReturn(java.util.Optional.ofNullable(userInfo));

		// when
		userInfoService.modifyUserInfo(1, userInfoDto);

		// then
		verify(userInfoRepository).save(userInfo);
	}
}