package com.assc.dolbot.service;

import com.assc.dolbot.dto.LogDto;
import com.assc.dolbot.dto.LogListDto;
import com.assc.dolbot.entity.*;
import com.assc.dolbot.repository.*;
import com.assc.dolbot.util.AmazonS3ResourceStorage;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.util.Base64Utils;
import org.springframework.web.multipart.MultipartFile;
import org.springframework.mock.web.MockMultipartFile;

import java.sql.Time;
import java.text.SimpleDateFormat;
import java.time.Instant;
import java.time.LocalDate;
import java.util.*;
import java.sql.Date;

@Service
public class LogServiceImpl implements LogService{

    @Autowired
    HomeRepository homeRepository;
    @Autowired
    LogListRepository logListRepository;
    @Autowired
    ApplianceLogRepository applianceLogRepository;
    @Autowired
    ApplianceRepository applianceRepository;
    @Autowired
    RoomRepository roomRepository;
    @Autowired
    EmergencyLogRepository emergencyLogRepository;
    @Autowired
    EmergencyRepository emergencyRepository;
    @Autowired
    ScheduleLogRepository scheduleLogRepository;

    @Autowired
    AmazonS3ResourceStorage amazonS3ResourceStorage;


    // loglist를 만들고 사진을 저장하는 함수 같은 날짜로 한번 더 실행시 예외를 일으킴
    @Override
    public LogList addLogList(LogListDto logListDto) throws Exception {
        Home home = homeRepository.findByRobotNumber(logListDto.getRobotId());
        LogList logList = logListDto.toEntity();
        LogList newLogList;
        if(0  < logListRepository.countLogListByHomeIdAndLogDate(home.getHomeId(),logList.getLogDate())){
            System.out.println("already saved infomation : check your date)");
            throw new Exception();
        }
        Instant now = Instant.now();
        long uniqueValue = now.toEpochMilli();
        byte[] decodedImageBytes = Base64Utils.decodeFromUrlSafeString(logListDto.getPicture());
        System.out.println(123);
        String fileName = Long.valueOf(uniqueValue) + ".jpg";
        String contentType = "image/jpeg";
        logList.setHome(home);
        MultipartFile file = new MockMultipartFile(fileName,fileName,contentType,decodedImageBytes);

        String url = amazonS3ResourceStorage.store(fileName, file);
        logList.setPictureUrl(url);
        logListRepository.save(logList);
        newLogList = logListRepository.findByHomeIdAndLogDate(home.getHomeId(), logList.getLogDate());
        return newLogList;
    }

    // loglist의 사진 수정
    @Override
    public void modifyLogList(int logListId, LogListDto logListDto) throws Exception {
        logListDto.setLogDate(LocalDate.now());
        LogList logList = logListDto.toEntity();
        logList.setLogListId(logListId);
        Instant now = Instant.now();
        long uniqueValue = now.toEpochMilli();
        byte[] decodedImageBytes = Base64Utils.decodeFromUrlSafeString(logListDto.getPicture());
        String fileName = Long.valueOf(uniqueValue) + ".jpg";
        String contentType = "image/jpeg";
        MultipartFile file = new MockMultipartFile(fileName,fileName,contentType,decodedImageBytes);

        String url = amazonS3ResourceStorage.store(fileName, file);
        logList.setPictureUrl(url);
        logListRepository.save(logList);
    }

    @Override
    public LogListDto findLogList(int homeId, LocalDate localDate) throws Exception {
        LogList logList = logListRepository.findByHomeIdAndLogDate(homeId,Date.valueOf(localDate));
        List<LogDto> logDtoList = new ArrayList<>();
        System.out.println(logList);
        List<ApplianceLog> applianceLogList = applianceLogRepository.findByLogListId(logList.getLogListId());
        for(int i = 0; i < applianceLogList.size(); i++){
            logDtoList.add(applianceLogList.get(i).toDto());
        }

        List<EmergencyLog> emergencyLogList = emergencyLogRepository.findByLogListId(logList.getLogListId());
        for(int i = 0; i < emergencyLogList.size(); i++){
            logDtoList.add(emergencyLogList.get(i).toDto());
        }

        List<ScheduleLog> scheduleLogsList = scheduleLogRepository.findByLogListId(logList.getLogListId());
        for(int i = 0; i < scheduleLogsList.size(); i++){
            logDtoList.add(scheduleLogsList.get(i).toDto());
        }

        LogListDto logListDto = logList.toDto();
        Collections.sort(logDtoList, new Comparator<LogDto>() {
            @Override
            public int compare(LogDto o1, LogDto o2) {
                SimpleDateFormat dateFormat = new SimpleDateFormat("hh:mm:ss");
                Time t1 = Time.valueOf(o1.getLogTime());
                Time t2 = Time.valueOf(o2.getLogTime());

                return t1.compareTo(t2);
            }
        });


        logListDto.setLogs(logDtoList);


        return logListDto;
    }

    @Override
    public void addApplianceLog(LogDto logDto) throws Exception {
        ApplianceLog applianceLog = logDto.toApplianceLog();
        applianceLog.setAppliance(applianceRepository.findById(logDto.getApplianceId()).get());
        applianceLog.setRoom(roomRepository.findById(logDto.getRoomId()).get());

        System.out.println(applianceLog);

        applianceLogRepository.save(applianceLog);
    }

    @Override
    public void addEmergencyLog(LogDto logDto) throws Exception {
        EmergencyLog emergencyLog = logDto.toEmergencyLog();
        emergencyLog.setEmergency(emergencyRepository.findById(logDto.getEmergencyId()).get());
        emergencyLogRepository.save(emergencyLog);
    }

    @Override
    public void addScheduleLog(LogDto logDto) throws Exception {
        ScheduleLog scheduleLog = logDto.toScheduleLog();
        scheduleLogRepository.save(scheduleLog);
    }

}
