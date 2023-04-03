package com.assc.dolbot.controller;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import com.assc.dolbot.dto.UserInfoDto;
import com.assc.dolbot.service.UserInfoService;

@RestController
@RequestMapping("/api/v1/user")
@CrossOrigin(origins = "*")
public class UserInfoController {
	private static final String SUCCESS = "success";
	private static final String FAIL = "fail";

	@Autowired
	private UserInfoService userInfoService;

	// 로그인, 회원가입
	@PostMapping("/login")
	public ResponseEntity<UserInfoDto> userLogin(@RequestBody UserInfoDto userInfoDto) {
		try{
			userInfoDto = userInfoService.loginUserInfo(userInfoDto);
			return new ResponseEntity<UserInfoDto>(userInfoDto, HttpStatus.OK);
		}catch(Exception e){
			e.printStackTrace();
			return new ResponseEntity<UserInfoDto>(userInfoDto, HttpStatus.INTERNAL_SERVER_ERROR);
		}
	}

	// 회원 기본 집 변경
	@PatchMapping("/{user_id}")
	public ResponseEntity<String> userModify(@PathVariable("user_id") int userId, @RequestBody UserInfoDto userInfoDto){
		try{
			userInfoService.modifyUserInfo(userId, userInfoDto);
			return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
		}catch(Exception e){
			e.printStackTrace();
			return new ResponseEntity<>(FAIL, HttpStatus.INTERNAL_SERVER_ERROR);
		}
	}
}
