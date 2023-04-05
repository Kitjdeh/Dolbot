package com.assc.dolbot.service;

import static org.assertj.core.api.Assertions.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.BDDMockito.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.Mockito;
import org.springframework.boot.test.context.SpringBootTest;

import com.assc.dolbot.dto.UserHomeDto;
import com.assc.dolbot.entity.Home;
import com.assc.dolbot.entity.UserHome;
import com.assc.dolbot.entity.UserInfo;
import com.assc.dolbot.repository.HomeRepository;
import com.assc.dolbot.repository.UserHomeRepository;

@SpringBootTest
@DisplayName("UserHomeService")
class UserHomeServiceImplTest {

	@Mock
	private UserHomeRepository userHomeRepository;

	@Mock
	private HomeRepository homeRepository;

	@Mock
	private UserHomeRepository userInfoRepository;

	@InjectMocks
	private UserHomeServiceImpl userHomeService;

	@Test
	@DisplayName("로봇 추가")
	public void addUserHome() throws Exception {
		// Given
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

		UserInfo userInfo = UserInfo.builder().userId(1).kakaoId("1111111111").mainHomeId(0).build();

		Home home = Home.builder().homeId(1).robotNumber(708001).build();

		List<UserHome> list = new ArrayList<>();
		UserHome userHome1 = UserHome.builder()
			.userHomeId(1)
			.userId(1)
			.homeId(1)
			.nickname("김원혁")
			.isAlarm(false)
			.build();
		UserHome userHome2 = UserHome.builder()
			.userHomeId(2)
			.userId(1)
			.homeId(2)
			.nickname("이가옥")
			.isAlarm(false)
			.build();
		UserHome userHome3 = UserHome.builder()
			.userHomeId(3)
			.userId(1)
			.homeId(3)
			.nickname("정찬영")
			.isAlarm(false)
			.build();

		list.add(userHome1);
		list.add(userHome2);
		list.add(userHome3);

		when(homeRepository.findByRobotNumber(708001)).thenReturn(home);
		given(userHomeRepository.findByUserIdAndHomeId(anyInt(),anyInt())).willReturn(null);
		given(userHomeRepository.findByUserId(userHomeDto1.getUserId())).willReturn(list);

		given(userHomeRepository.save(any())).willReturn(userHomeDto2.toEntity());

		// When
		userHomeService.addUserHome(userHomeDto1);

		// Then
		verify(homeRepository).findByRobotNumber(708001);
		assertEquals(1, userHomeDto1.getHomeId());
	}

	@Test
	@DisplayName("사용자에 맞는 로봇 정보들 선택")
	public void findUserHomeListByUserId() throws Exception {
		// given
		List<UserHome> list = new ArrayList<>();
		UserHome userHome1 = UserHome.builder()
			.userHomeId(1)
			.userId(1)
			.homeId(1)
			.nickname("김원혁")
			.isAlarm(false)
			.build();
		UserHome userHome2 = UserHome.builder()
			.userHomeId(2)
			.userId(1)
			.homeId(2)
			.nickname("이가옥")
			.isAlarm(false)
			.build();
		UserHome userHome3 = UserHome.builder()
			.userHomeId(3)
			.userId(1)
			.homeId(3)
			.nickname("정찬영")
			.isAlarm(false)
			.build();

		list.add(userHome1);
		list.add(userHome2);
		list.add(userHome3);

		List<UserHomeDto> listDto = new ArrayList<>();
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

		listDto.add(userHomeDto1);
		listDto.add(userHomeDto2);
		listDto.add(userHomeDto3);

		given(userHomeRepository.findByUserId(Mockito.anyInt())).willReturn(list);

		// when
		List<UserHomeDto> result = userHomeService.findUserHomeListByUserId(1);

		// then
		assertThat(result.toString()).isEqualTo(listDto.toString());
	}

	@Test
	@DisplayName("로봇에 맞는 사용자 정보들 선택")
	public void findUserHomeListByRobotNumber() throws Exception {
		// given
		List<UserHome> list = new ArrayList<>();
		UserHome userHome1 = UserHome.builder()
			.userHomeId(1)
			.userId(1)
			.homeId(1)
			.nickname("김원혁")
			.isAlarm(false)
			.build();
		UserHome userHome2 = UserHome.builder()
			.userHomeId(4)
			.userId(2)
			.homeId(1)
			.nickname("김정민")
			.isAlarm(false)
			.build();

		list.add(userHome1);
		list.add(userHome2);

		List<UserHomeDto> listDto = new ArrayList<>();
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

		listDto.add(userHomeDto1);
		listDto.add(userHomeDto2);

		Home home = new Home();
		home.setHomeId(1);

		given(homeRepository.findByRobotNumber(Mockito.anyInt())).willReturn(home);
		given(userHomeRepository.findByHomeId(Mockito.anyInt())).willReturn(list);

		// when
		List<UserHomeDto> result = userHomeService.findUserHomeListByRobotNumber(10);

		// then
		assertThat(result.toString()).isEqualTo(listDto.toString());
	}

	@Test
	@DisplayName("로봇정보 수정")
	public void modifyUserHome() throws Exception {
		UserHome userHome = UserHome.builder()
			.userHomeId(1)
			.userId(1)
			.homeId(1)
			.nickname("김원혁")
			.isAlarm(true)
			.build();

		UserHomeDto userHomeDto = UserHomeDto.builder()
			.userHomeId(1)
			.userId(1)
			.homeId(1)
			.nickname("김원혁")
			.isAlarm(true)
			.build();

		given(userHomeRepository.findById(userHomeDto.getUserHomeId())).willReturn(
			java.util.Optional.ofNullable(userHome));

		// when
		userHomeService.modifyUserHome(1, userHomeDto);

		// then
		verify(userHomeRepository).save(userHome);
	}

	@Test
	@DisplayName("로봇 제거")
	public void removeUserHome() throws Exception {
		// Given
		int userHomeId = 1;

		// When
		userHomeService.removeUserHome(userHomeId);

		// Then
		verify(userHomeRepository).deleteById(userHomeId);
	}
}