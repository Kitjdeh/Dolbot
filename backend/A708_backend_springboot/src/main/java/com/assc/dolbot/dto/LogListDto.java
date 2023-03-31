package com.assc.dolbot.dto;

import com.assc.dolbot.entity.Home;
import com.assc.dolbot.entity.LogList;
import lombok.*;
import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.format.annotation.DateTimeFormat;
import org.springframework.web.multipart.MultipartFile;

import javax.persistence.*;
import java.sql.Date;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;

@Setter
@Getter
@ToString
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class LogListDto {
    private int robotId;
    private int logListId;
    @DateTimeFormat(iso = DateTimeFormat.ISO.DATE)
    private LocalDate logDate;
    private String pictureUrl;
    private String picture;
    private List<LogDto> logs;


    public LogList toEntity(){
        LogList logList = LogList.builder()
                .logListId(this.logListId)
                .logDate(Date.valueOf(logDate))
                .build();
        return logList;
    }
}
