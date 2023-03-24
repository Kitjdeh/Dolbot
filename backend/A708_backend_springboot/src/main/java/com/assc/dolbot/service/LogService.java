package com.assc.dolbot.service;

import com.assc.dolbot.dto.LogListDto;
import com.assc.dolbot.entity.LogList;

public interface LogService {

    public LogList addLogList(LogListDto logListDto) throws Exception;
    public void modifyLogList(int logListId, LogListDto logListDto) throws Exception;
}
