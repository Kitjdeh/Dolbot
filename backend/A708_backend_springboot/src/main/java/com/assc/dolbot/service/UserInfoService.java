package com.assc.dolbot.service;

import com.assc.dolbot.dto.UserInfoDto;

public interface UserInfoService {
	// 사용자 로그인, 회원가입
	public UserInfoDto loginUserInfo(UserInfoDto userInfoDto) throws Exception;
}
