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
@Table(name="emergency_log")
@NoArgsConstructor
@AllArgsConstructor
@Builder
@ToString
public class EmergencyLog {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private int emergencyLogId;

    @Column(nullable = false)
    private Time logTime;

    @Column(nullable = false)
    private int logListId;

    @ManyToOne
    @JoinColumn(name="emergency_id")
    private Emergency emergency;

    @CreatedDate
    @Column(updatable = false, nullable = false)
    private LocalDateTime createdAt;
    @LastModifiedDate
    private LocalDateTime updatedAt;

    public LogDto toDto(){
        SimpleDateFormat sdf = new SimpleDateFormat("HH:mm:ss");
        String timeString = sdf.format(logTime);
        LogDto logDto = LogDto.builder()
                .type("비상")
                .logTime(timeString)
                .applianceName("")
                .isOn(false)
                .emergencyContent(this.emergency.getContent())
                .roomName("비상")
                .scheduleContent("")
                .build();
        return logDto;
    }
}
