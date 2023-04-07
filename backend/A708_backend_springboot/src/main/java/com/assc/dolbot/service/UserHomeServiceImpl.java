package com.assc.dolbot.service;

import java.util.ArrayList;
import java.util.List;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import com.assc.dolbot.dto.UserHomeDto;
import com.assc.dolbot.entity.Home;
import com.assc.dolbot.entity.UserHome;
import com.assc.dolbot.entity.UserInfo;
import com.assc.dolbot.repository.HomeRepository;
import com.assc.dolbot.repository.UserHomeRepository;
import com.assc.dolbot.repository.UserInfoRepository;

@Service
public class UserHomeServiceImpl implements UserHomeService{
	@Autowired
	private UserHomeRepository userHomeRepository;

	@Autowired
	private HomeRepository homeRepository;

	@Autowired
	private UserInfoRepository userInfoRepository;

	@Override
	public UserHomeDto addUserHome(UserHomeDto userHomeDto) throws Exception {
		Home home = homeRepository.findByRobotNumber(userHomeDto.getRobotNumber());
		if(home == null){
			userHomeDto.setStatus(1);
		}else {
			UserHome userHome = userHomeRepository.findByUserIdAndHomeId(userHomeDto.getUserId(), home.getHomeId());

			if(userHome != null){
				userHomeDto.setStatus(2);
			}else {
				if(userHomeRepository.findByUserId(userHomeDto.getUserId()).size()==0){
					UserInfo userInfo = userInfoRepository.findById(userHomeDto.getUserId()).get();
					userInfo.setMainHomeId(home.getHomeId());
					userInfoRepository.save(userInfo);
				}
				userHomeDto.setHomeId(home.getHomeId());
				userHomeDto = userHomeRepository.save(userHomeDto.toEntity()).toDto();
			}
		}

		return userHomeDto;
	}

	@Override
	public List<UserHomeDto> findUserHomeListByUserId(int userId) throws Exception {
		List<UserHome> userHomeList = userHomeRepository.findByUserId(userId);
		List<UserHomeDto> userHomeDtoList = new ArrayList<>();
		for(int i=0; i<userHomeList.size(); i++){
			userHomeDtoList.add(userHomeList.get(i).toDto());
		}
		return userHomeDtoList;
	}

	@Override
	public List<UserHomeDto> findUserHomeListByRobotNumber(int robotNumber) throws Exception {
		int homeId = homeRepository.findByRobotNumber(robotNumber).getHomeId();
		List<UserHome> userHomeList = userHomeRepository.findByHomeId(homeId);
		List<UserHomeDto> userHomeDtoList = new ArrayList<>();
		for(int i=0; i<userHomeList.size(); i++){
			userHomeDtoList.add(userHomeList.get(i).toDto());
		}
		return userHomeDtoList;
	}

	@Override
	public void modifyUserHome(int userHomeId, UserHomeDto userHomeDto) throws Exception {
		UserHome userHome = userHomeRepository.findById(userHomeId).get();
		userHome.setAlarm(userHomeDto.isAlarm());
		userHomeRepository.save(userHome);
	}

	@Override
	public void removeUserHome(int userHomeId) throws Exception {
		userHomeRepository.deleteById(userHomeId);
	}
}
