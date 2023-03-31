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
	private String email;
	private int main_home_id;
	private boolean isNew;

	public UserInfo toEntity() {
		UserInfo build = UserInfo.builder()
			.userId(this.userId)
			.email(this.email)
			.main_home_id(this.main_home_id)
			.build();
		return build;
	}
}
