package com.assc.dolbot.repository;

import org.springframework.data.jpa.repository.JpaRepository;

import com.assc.dolbot.entity.UserInfo;

public interface UserInfoRepository extends JpaRepository<UserInfo, Integer> {
	UserInfo findByKakaoId(String kakaoId);
}
