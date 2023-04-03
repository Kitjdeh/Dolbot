package com.assc.dolbot.dto;

import com.assc.dolbot.entity.UserInfo;

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
public class UserInfoDto {
	private int userId;
	private String kakaoId;
	private int mainHomeId;
	private boolean isNew;

	public UserInfo toEntity() {
		UserInfo build = UserInfo.builder()
			.userId(this.userId)
			.kakaoId(this.kakaoId)
			.mainHomeId(this.mainHomeId)
			.build();
		return build;
	}
}
