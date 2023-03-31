package com.assc.dolbot.service;

import com.assc.dolbot.dto.UserInfoDto;

public interface UserInfoService {
	// 사용자 로그인, 회원가입
	public UserInfoDto loginUserInfo(UserInfoDto userInfoDto) throws Exception;

	// 사용자 기본 집 변경
	public void modifyUserInfo(int userId, UserInfoDto userInfoDto) throws Exception;
}
