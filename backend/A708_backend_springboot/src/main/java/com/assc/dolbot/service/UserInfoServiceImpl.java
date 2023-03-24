package com.assc.dolbot.service;

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
		UserInfo userInfo = userInfoRepository.findByKakaoId(userInfoDto.getKakaoId());
		UserInfoDto dto;
		if(userInfo == null){
			System.out.println("회원가입");
			userInfoRepository.save(userInfoDto.toEntity());
			userInfo = userInfoRepository.findByKakaoId(userInfoDto.getKakaoId());
			dto = userInfo.toDto();
			dto.setNew(true);
		}else {
			dto = userInfo.toDto();
		}
		return dto;
	}
}
