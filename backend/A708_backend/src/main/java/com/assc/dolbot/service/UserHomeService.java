package com.assc.dolbot.service;

import java.util.List;

import com.assc.dolbot.dto.UserHomeDto;

public interface UserHomeService {
	// 로봇 추가
	public void addUserHome(UserHomeDto userHomeDto) throws Exception;
	// 사용자에 맞는 로봇정보들 선택
	public List<UserHomeDto> findUserHomeList(int userId) throws Exception;
	// 로봇정보 수정
	public void modifyUserHome(int userHomeId, UserHomeDto userHomeDto) throws Exception;
	// 로봇 제거
	public void removeUserHome(int userHomeId) throws Exception;
}
