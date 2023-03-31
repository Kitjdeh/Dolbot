package com.assc.dolbot.dto;

import java.time.LocalDate;
import java.util.Date;

import org.springframework.format.annotation.DateTimeFormat;

import com.assc.dolbot.entity.ScheduleInfo;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import lombok.ToString;

@Setter
@Getter
@ToString
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class ScheduleInfoDto {
	private int scheduleId;
	private int homeId;
	private Date scheduleTime;
	private String content;
	private LocalDate startDate;
	private LocalDate endDate;

	public ScheduleInfo toEntity(){
		ScheduleInfo build = ScheduleInfo.builder()
			.scheduleId(this.scheduleId)
			.homeId(this.homeId)
			.scheduleTime(this.scheduleTime)
			.content(this.content)
			.build();
		return build;
	}
}
