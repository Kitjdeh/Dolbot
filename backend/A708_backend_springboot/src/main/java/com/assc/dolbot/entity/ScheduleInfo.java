package com.assc.dolbot.entity;

import java.time.LocalDateTime;
import java.util.Date;

import javax.persistence.Column;
import javax.persistence.Entity;
import javax.persistence.EntityListeners;
import javax.persistence.GeneratedValue;
import javax.persistence.GenerationType;
import javax.persistence.Id;
import javax.persistence.Table;
import javax.persistence.Temporal;
import javax.persistence.TemporalType;

import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

import com.assc.dolbot.dto.ScheduleInfoDto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import lombok.ToString;

@EntityListeners(AuditingEntityListener.class)
@Getter
@Setter
@Entity
@Table(name="schedule_info")
@NoArgsConstructor
@AllArgsConstructor
@Builder
@ToString
public class ScheduleInfo {
	@Id
	@GeneratedValue(strategy = GenerationType.IDENTITY)
	private int scheduleId;

	@Column(nullable = false)
	private int homeId;

	@Temporal(TemporalType.TIMESTAMP)
	@Column(nullable = false)
	private Date scheduleTime;

	@Column(nullable = false, length = 100)
	private String content;

	@CreatedDate
	@Column(updatable = false, nullable = false)
	private LocalDateTime createdAt;

	@LastModifiedDate
	@Column(nullable = false)
	private LocalDateTime updatedAt;

	public ScheduleInfoDto toDto(){
		ScheduleInfoDto scheduleInfoDto = ScheduleInfoDto.builder()
			.scheduleId(this.scheduleId)
			.homeId(this.homeId)
			.scheduleTime(this.scheduleTime)
			.content(this.content)
			.build();
		return scheduleInfoDto;
	}
}
