package com.assc.dolbot.entity;

import java.time.LocalDateTime;
import java.util.Date;

import javax.persistence.Column;
import javax.persistence.Entity;
import javax.persistence.GeneratedValue;
import javax.persistence.GenerationType;
import javax.persistence.Id;
import javax.persistence.Table;
import javax.persistence.Temporal;
import javax.persistence.TemporalType;

import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;

import com.assc.dolbot.dto.ScheduleInfoDto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@Entity
@Table(name="schedule_info")
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class ScheduleInfo {
	@Id
	@GeneratedValue(strategy = GenerationType.IDENTITY)
	private int scheduleId;

	@Column
	private int homeId;

	@Temporal(TemporalType.TIMESTAMP)
	private Date scheduleTime;

	@Column
	private String content;

	@CreatedDate
	private LocalDateTime createdAt;

	@LastModifiedDate
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
