package com.assc.dolbot.entity;

import com.assc.dolbot.dto.LogDto;
import lombok.*;
import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

import javax.persistence.*;
import java.sql.Time;
import java.text.SimpleDateFormat;
import java.time.LocalDateTime;

@EntityListeners(AuditingEntityListener.class)
@Getter
@Setter
@Entity
@Table(name="schedule_log")
@NoArgsConstructor
@AllArgsConstructor
@Builder
@ToString
public class ScheduleLog {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private int scheduleLogId;

    @Column(nullable = false)
    private Time logTime;

    @Column(nullable = false, length = 100)
    private String content;

    @Column(nullable = false)
    private int logListId;

    @CreatedDate
    @Column(updatable = false, nullable = false)
    private LocalDateTime createdAt;
    @LastModifiedDate
    private LocalDateTime updatedAt;


    public LogDto toDto(){
        SimpleDateFormat sdf = new SimpleDateFormat("HH:mm:ss");
        String timeString = sdf.format(logTime);
        LogDto logDto = LogDto.builder()
                .type("일정")
                .logTime(timeString)
                .applianceName("")
                .isOn(false)
                .emergencyContent("")
                .roomName("일정")
                .scheduleContent(this.content)
                .build();
        return logDto;
    }
}
