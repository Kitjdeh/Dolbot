package com.assc.dolbot.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import com.assc.dolbot.dto.UserInfoDto;
import com.assc.dolbot.service.UserInfoService;

@RestController
@RequestMapping("/api/v1")
@CrossOrigin(origins = "*")
public class UserInfoController {
	private static final String SUCCESS = "success";
	private static final String FAIL = "fail";

	@Autowired
	private UserInfoService userInfoService;

	// 로그인, 회원가입
	@PostMapping("/login")
	public ResponseEntity<UserInfoDto> userLogin(@RequestBody UserInfoDto userInfoDto) {
		UserInfoDto user = new UserInfoDto();
		try{
			user = userInfoService.loginUserInfo(userInfoDto);
			return new ResponseEntity<UserInfoDto>(user, HttpStatus.OK);
		}catch(Exception e){
			return new ResponseEntity<UserInfoDto>(user, HttpStatus.INTERNAL_SERVER_ERROR);
		}
	}

}
