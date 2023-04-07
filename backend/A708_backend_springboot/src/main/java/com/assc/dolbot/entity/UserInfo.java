package com.assc.dolbot.entity;
import java.time.LocalDateTime;

import javax.persistence.Column;
import javax.persistence.Entity;
import javax.persistence.EntityListeners;
import javax.persistence.GeneratedValue;
import javax.persistence.GenerationType;
import javax.persistence.Id;
import javax.persistence.Table;

import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

import com.assc.dolbot.dto.UserInfoDto;

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
@Table(name="user_info")
@NoArgsConstructor
@AllArgsConstructor
@Builder
@ToString
public class UserInfo {
	@Id
	@GeneratedValue(strategy = GenerationType.IDENTITY)
	private int userId;

	@Column(nullable = false, unique = true, length = 20)
	private String kakaoId;

	@Column(nullable = false)
	private int mainHomeId;

	@CreatedDate
	@Column(updatable = false, nullable = false)
	private LocalDateTime createdAt;

	@LastModifiedDate
	@Column(nullable = false)
	private LocalDateTime updatedAt;

	public UserInfoDto toDto() {
		UserInfoDto userInfoDto = UserInfoDto.builder()
			.userId(this.userId)
			.kakaoId(this.kakaoId)
			.mainHomeId(this.mainHomeId)
			.build();
		return userInfoDto;
	}
}
