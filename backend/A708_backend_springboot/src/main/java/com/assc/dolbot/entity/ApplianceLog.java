package com.assc.dolbot.entity;

import com.assc.dolbot.dto.LogDto;
import com.assc.dolbot.dto.ScheduleInfoDto;
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
@Table(name="appliance_log")
@NoArgsConstructor
@AllArgsConstructor
@Builder
@ToString
public class ApplianceLog {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private int applianceLogId;

    @Column(nullable = false)
    private Time logTime;

    @Column(nullable = false)
    private boolean isOn;

    @Column(nullable = false)
    private int logListId;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name="room_id")
    private Room room;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name="appliance_id")
    private Appliance appliance;


    @CreatedDate

    @Column(updatable = false, nullable = false)
    private LocalDateTime createdAt;
    @LastModifiedDate
    private LocalDateTime updatedAt;

    public LogDto toDto(){
        SimpleDateFormat sdf = new SimpleDateFormat("HH:mm:ss");
        String timeString = sdf.format(logTime);
        LogDto logDto = LogDto.builder()
                .type("가전")
                .logTime(timeString)
                .applianceName(this.appliance.getApplianceName())
                .isOn(this.isOn)
                .emergencyContent("")
                .roomName(this.room.getRoomName())
                .scheduleContent("")
                .build();
        return logDto;
    }
}
