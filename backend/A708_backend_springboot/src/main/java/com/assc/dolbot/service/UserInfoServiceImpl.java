package com.assc.dolbot.service;

import org.apache.catalina.User;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import com.assc.dolbot.dto.UserInfoDto;
import com.assc.dolbot.entity.UserInfo;
import com.assc.dolbot.repository.UserInfoRepository;

@Service
public class UserInfoServiceImpl implements UserInfoService{
	@Autowired
	private UserInfoRepository userInfoRepository;

	@Override
	public UserInfoDto loginUserInfo(UserInfoDto userInfoDto) throws Exception {
		UserInfo userInfo = userInfoRepository.findByEmail(userInfoDto.getEmail());
		if(userInfo == null){
			userInfo = userInfoRepository.save(userInfoDto.toEntity());
			userInfoDto = userInfo.toDto();
			userInfoDto.setNew(true);
		}else {
			userInfoDto = userInfo.toDto();
		}
		return userInfoDto;
	}

	@Override
	public void modifyUserInfo(int userId, UserInfoDto userInfoDto) throws Exception {
		UserInfo userInfo = userInfoRepository.findById(userId).get();
		userInfo.setMain_home_id(userInfoDto.getMain_home_id());
		userInfoRepository.save(userInfo);
	}
}
