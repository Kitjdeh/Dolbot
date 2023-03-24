package com.assc.dolbot.dto;

import com.assc.dolbot.entity.ApplianceLog;
import com.assc.dolbot.entity.LogList;
import lombok.*;

import java.sql.Date;
import java.sql.Time;

@Setter
@Getter
@ToString
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class LogDto {
    private String type;
    private String logTime;
    private boolean isOn;
    private String applianceName;
    private String roomName;
    private String emergencyContent;
    private String scheduleContent;
    private int logListId;
    private int applianceId;
    private int roomId;

    public ApplianceLog toApplianceLog(){
        ApplianceLog applianceLog = ApplianceLog.builder()
                .logListId(this.logListId)
                .isOn(this.isOn)
                .logTime(Time.valueOf(logTime))
                .build();
        return applianceLog;
    }
}
