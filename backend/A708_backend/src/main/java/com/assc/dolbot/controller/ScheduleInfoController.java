package com.assc.dolbot.controller;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import com.assc.dolbot.dto.ScheduleInfoDto;
import com.assc.dolbot.service.ScheduleInfoService;

@RestController
@RequestMapping("/api/v1/schedule-info")
@CrossOrigin(origins = "*")
public class ScheduleInfoController {
	private static final String SUCCESS = "success";
	private static final String FAIL = "fail";

	@Autowired
	private ScheduleInfoService scheduleInfoService;

	// 스케줄 추가하기
	@PostMapping
	public ResponseEntity<String> scheduleInfoAdd(@RequestBody ScheduleInfoDto scheduleInfoDto) {
		try{
			System.out.println("SUCCESS");
			scheduleInfoService.addScheduleInfo(scheduleInfoDto);
			return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
		}catch(Exception e){
			System.out.println("FAIL");
			return new ResponseEntity<>(FAIL, HttpStatus.OK);
		}
	}

	// 날짜에 맞는 스케줄 불러오기
	// Map 구성요소 : date
	@GetMapping("/{home_id}")
	public ResponseEntity<List<ScheduleInfoDto>> scheduleInfoList(@PathVariable("home_id") int homeId, @RequestBody Map<String, String> map){
		List<ScheduleInfoDto> list = new ArrayList<>();
		try{
			list = scheduleInfoService.findScheduleInfoList(homeId, map);
			return new ResponseEntity<List<ScheduleInfoDto>>(list, HttpStatus.OK);
		}catch(Exception e){
			return new ResponseEntity<List<ScheduleInfoDto>>(list, HttpStatus.OK);
		}
	}

	// 스케줄 수정하기
	@PatchMapping("/{schedule_id}")
	public ResponseEntity<String> scheduleInfoModify(@RequestBody ScheduleInfoDto scheduleInfoDto){
		try{
			scheduleInfoService.modifyScheduleInfo(scheduleInfoDto);
			return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
		}catch(Exception e){
			return new ResponseEntity<>(FAIL, HttpStatus.OK);
		}
	}

	// 스케줄 삭제하기
	@DeleteMapping("/{schedule_id}")
	public ResponseEntity<String> scheduleInfoRemove(@PathVariable("schedule_id") int scheduleInfoId){
		try{
			scheduleInfoService.removeScheduleInfo(scheduleInfoId);
			return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
		}catch(Exception e){
			return new ResponseEntity<>(FAIL, HttpStatus.OK);
		}
	}
}
