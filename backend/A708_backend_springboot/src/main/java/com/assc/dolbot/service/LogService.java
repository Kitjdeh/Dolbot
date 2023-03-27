package com.assc.dolbot.service;

import com.assc.dolbot.dto.LogDto;
import com.assc.dolbot.dto.LogListDto;
import com.assc.dolbot.entity.ApplianceLog;
import com.assc.dolbot.entity.LogList;

import java.time.LocalDate;

public interface LogService {

    public LogList addLogList(LogListDto logListDto) throws Exception;
    public void modifyLogList(int logListId, LogListDto logListDto) throws Exception;
    public LogListDto findLogList(int homeId, LocalDate localDate) throws Exception;
    public void addApplianceLog(LogDto logDto) throws Exception;
    public void addEmergencyLog(LogDto logDto) throws Exception;
    public void addScheduleLog(LogDto logDto) throws Exception;
}
