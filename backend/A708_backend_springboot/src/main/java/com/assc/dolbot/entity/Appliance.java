package com.assc.dolbot.entity;

import lombok.*;
import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

import javax.persistence.*;
import java.time.LocalDateTime;
import java.util.List;

@EntityListeners(AuditingEntityListener.class)
@Getter
@Setter
@Entity
@Table(name="appliance")
@NoArgsConstructor
@AllArgsConstructor
@Builder
@ToString
public class Appliance {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private int applianceId;
    @Column(nullable = false, length = 50)
    private String applianceName;

    @CreatedDate
    @Column(updatable = false, nullable = false)
    private LocalDateTime createdAt;
    @LastModifiedDate
    private LocalDateTime updatedAt;
}
