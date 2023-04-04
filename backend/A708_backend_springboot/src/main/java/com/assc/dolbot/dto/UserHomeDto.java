package com.assc.dolbot.dto;

import com.assc.dolbot.entity.UserHome;

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
public class UserHomeDto {
	private int userHomeId;
	private int userId;
	private int homeId;
	private String nickname;
	private boolean isAlarm;
	private int robotNumber;
	private int status;

	public UserHome toEntity(){
		UserHome build = UserHome.builder()
			.userHomeId(this.userHomeId)
			.userId(this.userId)
			.homeId(this.homeId)
			.nickname(this.nickname)
			.isAlarm(this.isAlarm)
			.build();
		return build;
	}
}
