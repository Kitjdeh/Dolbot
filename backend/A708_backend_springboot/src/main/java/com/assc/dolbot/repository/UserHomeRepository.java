package com.assc.dolbot.repository;

import java.util.List;

import org.springframework.data.jpa.repository.JpaRepository;

import com.assc.dolbot.entity.UserHome;

public interface UserHomeRepository extends JpaRepository<UserHome, Integer> {
	List<UserHome> findByUserId(int userId);
}
