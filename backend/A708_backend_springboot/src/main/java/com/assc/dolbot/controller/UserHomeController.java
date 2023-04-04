package com.assc.dolbot.controller;

import java.util.ArrayList;
import java.util.List;

import org.apache.catalina.User;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import com.assc.dolbot.dto.UserHomeDto;
import com.assc.dolbot.entity.Home;
import com.assc.dolbot.service.UserHomeService;

@RestController
@RequestMapping("/api/v1/user-homes")
@CrossOrigin(origins = "*")
public class UserHomeController {
	private static final String SUCCESS = "success";
	private static final String FAIL = "fail";

	@Autowired
	private UserHomeService userHomeService;

	//로봇등록
	@PostMapping
	public ResponseEntity<UserHomeDto> userHomeAdd(@RequestBody UserHomeDto userHomeDto) {
		try{
			userHomeDto = userHomeService.addUserHome(userHomeDto);
			if(userHomeDto.getStatus() == 1){
				return new ResponseEntity<>(userHomeDto, HttpStatus.NOT_FOUND);
			}else if(userHomeDto.getStatus() == 2){
				return new ResponseEntity<>(userHomeDto, HttpStatus.CONFLICT);
			}

			return new ResponseEntity<>(userHomeDto, HttpStatus.OK);
		}catch(Exception e){
			e.printStackTrace();
			return new ResponseEntity<>(userHomeDto, HttpStatus.INTERNAL_SERVER_ERROR);
		}
	}

	//등록된 로봇들 정보 가져오기
	@GetMapping("/robots/{user_id}")
	public ResponseEntity<List<UserHomeDto>> userHomeListByUserId(@PathVariable("user_id") int userId){
		List<UserHomeDto> userHomeDtoList = new ArrayList<>();
		try{
			userHomeDtoList = userHomeService.findUserHomeListByUserId(userId);
			return new ResponseEntity<List<UserHomeDto>>(userHomeDtoList, HttpStatus.OK);
		}catch(Exception e){
			e.printStackTrace();
			return new ResponseEntity<List<UserHomeDto>>(userHomeDtoList, HttpStatus.INTERNAL_SERVER_ERROR);
		}
	}

	//등록된 유저들 정보 가져오기 만들기
	@GetMapping("/users/{robot_number}")
	public ResponseEntity<List<UserHomeDto>> userHomeListByRobotNumber(@PathVariable("robot_number") int robotNumber){
		List<UserHomeDto> userHomeDtoList = new ArrayList<>();
		try{
			userHomeDtoList = userHomeService.findUserHomeListByRobotNumber(robotNumber);
			return new ResponseEntity<List<UserHomeDto>>(userHomeDtoList, HttpStatus.OK);
		}catch(Exception e){
			e.printStackTrace();
			return new ResponseEntity<List<UserHomeDto>>(userHomeDtoList, HttpStatus.INTERNAL_SERVER_ERROR);
		}
	}


	//로봇의 알람 설정
	@PatchMapping("/{user_home_id}")
	public ResponseEntity<String> userHomeModify(@PathVariable("user_home_id") int userHomeId, @RequestBody UserHomeDto userHomeDto){
		try{
			userHomeService.modifyUserHome(userHomeId, userHomeDto);
			return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
		}catch(Exception e){
			e.printStackTrace();
			return new ResponseEntity<>(FAIL, HttpStatus.INTERNAL_SERVER_ERROR);
		}
	}

	//로봇해제
	@DeleteMapping("/{user_home_id}")
	public ResponseEntity<String> userHomeRemove(@PathVariable("user_home_id") int userHomeId){
		try{
			userHomeService.removeUserHome(userHomeId);
			return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
		}catch(Exception e){
			e.printStackTrace();
			return new ResponseEntity<>(FAIL, HttpStatus.INTERNAL_SERVER_ERROR);
		}
	}
}
