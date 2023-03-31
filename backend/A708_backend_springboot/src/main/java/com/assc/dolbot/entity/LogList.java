package com.assc.dolbot.entity;

import com.assc.dolbot.dto.LogDto;
import com.assc.dolbot.dto.LogListDto;
import lombok.*;
import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;
import org.springframework.format.annotation.DateTimeFormat;
import org.springframework.web.multipart.MultipartFile;

import javax.persistence.*;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.sql.Date;
import java.text.SimpleDateFormat;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

@EntityListeners(AuditingEntityListener.class)
@Getter
@Setter
@Entity
@Table(name="logList")
@NoArgsConstructor
@AllArgsConstructor
@Builder
@ToString
public class LogList {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private int logListId;
    @Column(updatable = false,nullable = false)
    private Date logDate;
    @Column(nullable = false, length = 500)
    private String pictureUrl;

    @ManyToOne
    @JoinColumn(updatable = false, name="homeId")
    private Home home;

    @CreatedDate
    @Column(updatable = false, nullable = false)
    private LocalDateTime createdAt;
    @LastModifiedDate
    private LocalDateTime updatedAt;

    public LogListDto toDto(){
        LogListDto logListDto = LogListDto.builder()
                .logListId(this.logListId)
                .logDate(this.logDate.toLocalDate())
                .pictureUrl(this.pictureUrl)
                .logs(new ArrayList<LogDto>())
                .build();
        return logListDto;
    }
}
