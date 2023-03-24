package com.assc.dolbot.controller;

import com.assc.dolbot.dto.LogListDto;
import com.assc.dolbot.dto.ScheduleInfoDto;
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
    @ApiOperation(value = "로그리스트 생성", notes = "Form Data 값을 받아와서 LogList를 생성하는 API, 같은 날짜로 두번 실행시 예외를 일으킴")
    public ResponseEntity<LogList> logListAdd(@RequestBody LogListDto logListDto) {
        LogList logList = null;
        try{
            logList = logService.addLogList(logListDto);
        }catch(Exception e){
            e.printStackTrace();
            System.out.println(logListDto);
            return new ResponseEntity<LogList>(logList, HttpStatus.INTERNAL_SERVER_ERROR);
        }

        return new ResponseEntity<LogList>(logList, HttpStatus.OK);
    }

    // 사진 바꾸기
    @PatchMapping("/log-lists/{log_list_id}")
    public ResponseEntity<String> logListModify(@PathVariable("log_list_id") int logListId, @RequestBody LogListDto logListDto){
        try{
            logService.modifyLogList(logListId,logListDto);
        }catch(Exception e){
            e.printStackTrace();
            System.out.println(logListDto);
            return new ResponseEntity<String>("fail", HttpStatus.INTERNAL_SERVER_ERROR);
        }

        return new ResponseEntity<String>("success", HttpStatus.OK);
    }
//
//    // 날짜에 맞는 스케줄 불러오기
//    // Map 구성요소 : date
//    @GetMapping("/log/logs/{home_id}")
//    public ResponseEntity<List<ScheduleInfoDto>> scheduleInfoList(@PathVariable("home_id") int homeId, @RequestParam("localDate") @DateTimeFormat(iso = DateTimeFormat.ISO.DATE) LocalDate localDate){
//
//        List<ScheduleInfoDto> list = new ArrayList<>();
//        try{
//            list = scheduleInfoService.findScheduleInfoList(homeId, localDate);
//            return new ResponseEntity<List<ScheduleInfoDto>>(list, HttpStatus.OK);
//        }catch(Exception e){
//            return new ResponseEntity<List<ScheduleInfoDto>>(list, HttpStatus.INTERNAL_SERVER_ERROR);
//        }
//    }
//
//    // 스케줄 삭제하기
//    @PostMapping("log/appliance-log")
//    public ResponseEntity<String> scheduleInfoRemove(@PathVariable("schedule_id") int scheduleInfoId){
//        try{
//            scheduleInfoService.removeScheduleInfo(scheduleInfoId);
//            return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
//        }catch(Exception e){
//            return new ResponseEntity<>(FAIL, HttpStatus.INTERNAL_SERVER_ERROR);
//        }
//    }
//
//    @PostMapping("log/emergency-log")
//    public ResponseEntity<String> scheduleInfoRemove(@PathVariable("schedule_id") int scheduleInfoId){
//        try{
//            scheduleInfoService.removeScheduleInfo(scheduleInfoId);
//            return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
//        }catch(Exception e){
//            return new ResponseEntity<>(FAIL, HttpStatus.INTERNAL_SERVER_ERROR);
//        }
//    }
//
//    @PostMapping("log/schedule-log")
//    public ResponseEntity<String> scheduleInfoRemove(@PathVariable("schedule_id") int scheduleInfoId){
//        try{
//            scheduleInfoService.removeScheduleInfo(scheduleInfoId);
//            return new ResponseEntity<>(SUCCESS, HttpStatus.OK);
//        }catch(Exception e){
//            return new ResponseEntity<>(FAIL, HttpStatus.INTERNAL_SERVER_ERROR);
//        }
//    }

}
