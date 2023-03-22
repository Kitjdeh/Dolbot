package com.assc.dolbot.repository;

import org.springframework.data.jpa.repository.JpaRepository;

import com.assc.dolbot.entity.Home;

public interface HomeRepository extends JpaRepository<Home, Integer> {
	Home findByRobotNumber(int robotNumber);
}
