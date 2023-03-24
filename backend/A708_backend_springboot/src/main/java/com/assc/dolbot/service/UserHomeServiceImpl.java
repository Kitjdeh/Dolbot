package com.assc.dolbot.service;

import java.util.ArrayList;
import java.util.List;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import com.assc.dolbot.dto.UserHomeDto;
import com.assc.dolbot.entity.UserHome;
import com.assc.dolbot.repository.HomeRepository;
import com.assc.dolbot.repository.UserHomeRepository;

@Service
public class UserHomeServiceImpl implements UserHomeService{
	@Autowired
	private UserHomeRepository userHomeRepository;

	@Autowired
	private HomeRepository homeRepository;

	@Override
	public void addUserHome(UserHomeDto userHomeDto) throws Exception {
		int homeId = homeRepository.findByRobotNumber(userHomeDto.getRobotNumber()).getHomeId();
		userHomeDto.setHomeId(homeId);
		userHomeRepository.save(userHomeDto.toEntity());
	}

	@Override
	public List<UserHomeDto> findUserHomeListByUserId(int userId) throws Exception {
		List<UserHome> list = userHomeRepository.findByUserId(userId);
		List<UserHomeDto> dtoList = new ArrayList<>();
		for(int i=0; i<list.size(); i++){
			dtoList.add(list.get(i).toDto());
		}
		return dtoList;
	}

	@Override
	public List<UserHomeDto> findUserHomeListByRobotNumber(int robotNumber) throws Exception {
		int homeId = homeRepository.findByRobotNumber(robotNumber).getHomeId();
		List<UserHome> list = userHomeRepository.findByHomeId(homeId);
		List<UserHomeDto> dtoList = new ArrayList<>();
		for(int i=0; i<list.size(); i++){
			dtoList.add(list.get(i).toDto());
		}
		return dtoList;
	}

	@Override
	public void modifyUserHome(int userHomeId, UserHomeDto userHomeDto) throws Exception {
		UserHome userHome = userHomeRepository.findById(userHomeId).get();
		userHome.setAlarm(userHomeDto.isAlarm());
		System.out.println("isAlarm"+userHomeDto.isAlarm());
		userHomeRepository.save(userHome);
	}

	@Override
	public void removeUserHome(int userHomeId) throws Exception {
		userHomeRepository.deleteById(userHomeId);
	}
}
