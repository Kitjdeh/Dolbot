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

import com.assc.dolbot.dto.UserHomeDto;

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
@Table(name="user_home")
@NoArgsConstructor
@AllArgsConstructor
@Builder
@ToString
public class UserHome {
	@Id
	@GeneratedValue(strategy = GenerationType.IDENTITY)
	private int userHomeId;

	@Column(nullable = false)
	private int userId;

	@Column(nullable = false)
	private int homeId;

	@Column(nullable = false, length = 50)
	private String nickname;

	@Column(nullable = false)
	private boolean isAlarm;

	@CreatedDate
	@Column(updatable = false, nullable = false)
	private LocalDateTime createdAt;

	@LastModifiedDate
	@Column(nullable = false)
	private LocalDateTime updatedAt;

	public UserHomeDto toDto(){
		UserHomeDto userHomeDto = UserHomeDto.builder()
			.userHomeId(this.userHomeId)
			.userId(this.userId)
			.homeId(this.homeId)
			.nickname(this.nickname)
			.isAlarm(this.isAlarm)
			.build();
		return userHomeDto;
	}
}
