package com.assc.dolbot.service;

import com.assc.dolbot.dto.LogListDto;
import com.assc.dolbot.entity.Home;
import com.assc.dolbot.entity.LogList;
import com.assc.dolbot.repository.HomeRepository;
import com.assc.dolbot.repository.LogListRepository;
import com.assc.dolbot.util.AmazonS3ResourceStorage;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.util.Base64Utils;
import org.springframework.web.multipart.MultipartFile;
import org.springframework.mock.web.MockMultipartFile;
import java.time.Instant;
import java.time.LocalDate;
import java.util.Base64;
import java.sql.Date;
import java.util.Properties;

@Service
public class LogServiceImpl implements LogService{

    @Autowired
    HomeRepository homeRepository;
    @Autowired
    LogListRepository logListRepository;
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
}
