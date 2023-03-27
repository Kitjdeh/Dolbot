package com.assc.dolbot.repository;

import com.assc.dolbot.entity.Emergency;
import org.springframework.data.jpa.repository.JpaRepository;

public interface EmergencyRepository extends JpaRepository<Emergency, Integer> {

}
