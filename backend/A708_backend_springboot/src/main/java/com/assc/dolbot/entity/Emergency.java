package com.assc.dolbot.entity;

import lombok.*;
import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

import javax.persistence.*;
import java.time.LocalDateTime;

@EntityListeners(AuditingEntityListener.class)
@Getter
@Setter
@Entity
@Table(name="emergency")
@NoArgsConstructor
@AllArgsConstructor
@Builder
@ToString
public class Emergency {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private int emergencyId;
    @Column(nullable = false, length = 50)
    private String content;

    @CreatedDate
    @Column(updatable = false, nullable = false)
    private LocalDateTime createdAt;
    @LastModifiedDate
    private LocalDateTime updatedAt;
}
