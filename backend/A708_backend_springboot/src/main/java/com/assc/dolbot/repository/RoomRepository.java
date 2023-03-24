package com.assc.dolbot.repository;

import com.assc.dolbot.entity.EmergencyLog;
import com.assc.dolbot.entity.Room;
import org.springframework.data.jpa.repository.JpaRepository;

public interface RoomRepository extends JpaRepository<Room, Integer> {
}
