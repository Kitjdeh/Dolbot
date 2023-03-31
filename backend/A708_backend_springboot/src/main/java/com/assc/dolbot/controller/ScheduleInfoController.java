package com.assc.dolbot.controller;

import java.time.LocalDate;
import java.util.ArrayList;
import java.util.List;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.format.annotation.DateTimeFormat;
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
			scheduleInfoService.addScheduleInfo(scheduleInfoDto);
			return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
		}catch(Exception e){
			e.printStackTrace();
			return new ResponseEntity<>(FAIL, HttpStatus.INTERNAL_SERVER_ERROR);
		}
	}

	// 날짜에 맞는 스케줄 불러오기
	@GetMapping("/{home_id}")
	public ResponseEntity<List<ScheduleInfoDto>> scheduleInfoList(@PathVariable("home_id") int homeId, @RequestParam("localDate") @DateTimeFormat(iso = DateTimeFormat.ISO.DATE) LocalDate localDate){

		List<ScheduleInfoDto> scheduleInfoDtoList = new ArrayList<>();
		try{
			scheduleInfoDtoList = scheduleInfoService.findScheduleInfoList(homeId, localDate);
			return new ResponseEntity<List<ScheduleInfoDto>>(scheduleInfoDtoList, HttpStatus.OK);
		}catch(Exception e){
			e.printStackTrace();
			return new ResponseEntity<List<ScheduleInfoDto>>(scheduleInfoDtoList, HttpStatus.INTERNAL_SERVER_ERROR);
		}
	}

	// 스케줄 수정하기
	@PatchMapping("/{schedule_id}")
	public ResponseEntity<String> scheduleInfoModify(@PathVariable("schedule_id") int scheduleInfoId, @RequestBody ScheduleInfoDto scheduleInfoDto){
		try{
			scheduleInfoService.modifyScheduleInfo(scheduleInfoId, scheduleInfoDto);
			return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
		}catch(Exception e){
			e.printStackTrace();
			return new ResponseEntity<>(FAIL, HttpStatus.INTERNAL_SERVER_ERROR);
		}
	}

	// 스케줄 삭제하기
	@DeleteMapping("/{schedule_id}")
	public ResponseEntity<String> scheduleInfoRemove(@PathVariable("schedule_id") int scheduleInfoId){
		try{
			scheduleInfoService.removeScheduleInfo(scheduleInfoId);
			return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
		}catch(Exception e){
			e.printStackTrace();
			return new ResponseEntity<>(FAIL, HttpStatus.INTERNAL_SERVER_ERROR);
		}
	}
}
