package com.assc.dolbot.repository;

import java.util.List;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import com.assc.dolbot.entity.UserHome;

public interface UserHomeRepository extends JpaRepository<UserHome, Integer> {
	List<UserHome> findByUserId(int userId);
	List<UserHome> findByHomeId(int homeId);
	@Query(nativeQuery = true, value = "SELECT * FROM user_home WHERE user_id = :userId AND home_id = :homeId")
	UserHome findByUserIdAndHomeId(int userId, int homeId);
}
