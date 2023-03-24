package com.assc.dolbot.dto;

import lombok.*;

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
}
