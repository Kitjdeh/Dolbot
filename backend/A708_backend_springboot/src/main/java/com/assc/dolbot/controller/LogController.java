package com.assc.dolbot.controller;

import com.assc.dolbot.dto.LogDto;
import com.assc.dolbot.dto.LogListDto;
import com.assc.dolbot.dto.ScheduleInfoDto;
import com.assc.dolbot.entity.ApplianceLog;
import com.assc.dolbot.entity.LogList;
import com.assc.dolbot.service.LogService;
import com.assc.dolbot.service.ScheduleInfoService;
import io.swagger.annotations.ApiOperation;
import io.swagger.annotations.ApiResponse;
import io.swagger.annotations.ApiResponses;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.format.annotation.DateTimeFormat;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.multipart.MultipartFile;

import java.time.LocalDate;
import java.util.ArrayList;
import java.util.List;

@RestController
@RequestMapping("/api/v1/log")
@CrossOrigin(origins = "*")
public class LogController {
    private static final String SUCCESS = "success";
    private static final String FAIL = "fail";

    @Autowired
    private LogService logService;

    // 로그리스트 만들기 (사진 최초 저장하기)
    // 같은 날짜로 한번 더 실행시 예외를 일으킴
    @PostMapping(value = "/log-lists", consumes = "application/json")
    @ApiOperation(value = "사진 최초 등록, 로그 리스트 생성", notes = "LogList를 생성하는 API, 같은 날짜로 두번 실행 불가" +
            "\n request : { \n" +
            "\t\"robotId\" : 1\n" +
            "\t\"picture\" : \"인코딩된 이미지의 문자열\",\n" +
            "\t\"logDate\" : \"2023-01-19\"\n" +
            "}")
    public ResponseEntity<LogList> logListAdd(@RequestBody LogListDto logListDto) {
        LogList logList = null;
        try{
            logList = logService.addLogList(logListDto);
        }catch(Exception e){
            e.printStackTrace();
            return new ResponseEntity<LogList>(logList, HttpStatus.INTERNAL_SERVER_ERROR);
        }

        return new ResponseEntity<LogList>(logList, HttpStatus.OK);
    }

    // 사진 바꾸기
    @PatchMapping("/log-lists/{log_list_id}")
    @ApiOperation(value = "사진 변경", notes = " 해당 날짜의 사진 변경, /api/v1/log/logs/{home_id}?localDate=yyyy-MM-dd) " +
            "\nrequest :{\n" +
            "\t\"robotId\" : 1\n" +
            "\t\"picture\" : \"인코딩된 이미지의 문자열\"\n" +
            "}")
    public ResponseEntity<String> logListModify(@PathVariable("log_list_id") int logListId, @RequestBody LogListDto logListDto){
        try{
            logService.modifyLogList(logListId,logListDto);
            return new ResponseEntity<String>("success", HttpStatus.OK);
        }catch(Exception e){
            e.printStackTrace();
            return new ResponseEntity<String>("fail", HttpStatus.INTERNAL_SERVER_ERROR);
        }

    }
    //
    // 날짜에 맞는 로그리스트 불러오기
    @GetMapping("/logs/{home_id}")
    @ApiOperation(value = "날짜에 맞는 로그리스트를 불러옴", notes = "/api/v1/log/logs/{home_id}?localDate=yyyy-MM-dd)")
    public ResponseEntity<LogListDto> logList(@PathVariable("home_id") int homeId, @RequestParam("localDate") @DateTimeFormat(iso = DateTimeFormat.ISO.DATE) LocalDate localDate){

        LogListDto logList = new LogListDto();
        try{
            logList = logService.findLogList(homeId, localDate);
            if(logList == null) {
                logList = new LogListDto();
                return new ResponseEntity<LogListDto>(logList, HttpStatus.NOT_FOUND);
            }
            return new ResponseEntity<LogListDto>(logList, HttpStatus.OK);
        }catch(Exception e){
            e.printStackTrace();
            return new ResponseEntity<LogListDto>(logList, HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    // 기기 동작 로그
    @PostMapping("log/appliance-log")
    @ApiOperation(value = "기기 동작 로그 저장", notes = "기기 동작 로그를 저장하는 api" +
            "{\n" +
            "  \"applianceId\": 1,\n" +
            "  \"logListId\": 1,\n" +
            "  \"logTime\": \"09:50:00\",\n" +
            "  \"on\": true,\n" +
            "  \"roomId\": 1\n" +
            "}")
    public ResponseEntity<String> applianceLogAdd(@RequestBody LogDto logDto){
        try{
            logService.addApplianceLog(logDto);
            return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
        }catch(Exception e){
            e.printStackTrace();
            return new ResponseEntity<>(FAIL, HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    // 비상 로그 저장
    @PostMapping("log/emergency-log")
    @ApiOperation(value = "비상 상황 로그 저장", notes = "비상 상황 로그를 저장하는 api" +
            "{\n" +
            "   \"emergencyId\": 1,\n" +
            "   \"logListId\": 1,\n" +
            "   \"logTime\": \"09:50:15\"\n" +
            "    }")
    public ResponseEntity<String> emergencyLogAdd(@RequestBody LogDto logDto){
        try{
            logService.addEmergencyLog(logDto);
            return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
        }catch(Exception e){
            e.printStackTrace();
            return new ResponseEntity<>(FAIL, HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    @PostMapping("log/schedule-log")
    @ApiOperation(value = "일정 실행 로그 저장", notes = "일정 실행행 로그 저장하는 api" +
            "{\n" +
            "  \"logListId\": 1,\n" +
            "  \"logTime\": \"11:12:12\",\n" +
            "  \"scheduleContent\": \"약 먹기\"\n" +
            "}")
    public ResponseEntity<String> scheduleLogAdd(@RequestBody LogDto logDto){
        try{
            logService.addScheduleLog(logDto);
            return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
        }catch(Exception e){
            e.printStackTrace();
            return new ResponseEntity<>(FAIL, HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

}
