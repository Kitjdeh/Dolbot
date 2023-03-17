package com.assc.dolbot.service;

import java.time.LocalDate;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import javax.transaction.Transactional;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import com.assc.dolbot.dto.ScheduleInfoDto;
import com.assc.dolbot.entity.ScheduleInfo;
import com.assc.dolbot.repository.ScheduleInfoRepository;


@Service
public class ScheduleInfoServiceImpl implements ScheduleInfoService{
	@Autowired
	private ScheduleInfoRepository scheduleInfoRepository;

	@Override
	public void addScheduleInfo(ScheduleInfoDto scheduleInfoDto) throws Exception {
		scheduleInfoRepository.save(scheduleInfoDto.toEntity());
	}

	@Override
	public List<ScheduleInfoDto> findScheduleInfoList(int homeId, Map<String, String> map) throws Exception {
		LocalDate date = LocalDate.parse(map.get("date"), DateTimeFormatter.ISO_DATE);
		List<ScheduleInfo> list = scheduleInfoRepository.findByHomeIdAndDate(homeId, date);
		List<ScheduleInfoDto> dtoList = new ArrayList<>();
		for(int i=0; i<list.size(); i++){
			dtoList.add(list.get(i).toDto());
		}
		return dtoList;
	}

	@Override
	public void modifyScheduleInfo(ScheduleInfoDto scheduleInfoDto) throws Exception {
		scheduleInfoRepository.save(scheduleInfoDto.toEntity());
	}

	@Override
	public void removeScheduleInfo(int scheduleId) throws Exception {
		scheduleInfoRepository.deleteById(scheduleId);
	}

	// // 강의 노트 찾기
	// @Override
	// public LectureNoteDto findLectureNote(int lectureNoteId) {
	// 	Optional<LectureNote> lectureNoteWrapper = lectureNoteRepository.findByQuestionId(lectureNoteId);
	// 	if(lectureNoteWrapper.isPresent()){
	// 		LectureNote lectureNote = lectureNoteWrapper.get();
	// 		LectureNoteDto dto = LectureNoteDto.builder()
	// 			.lectureNoteId(lectureNote.getLectureNoteId())
	// 			.questionId(lectureNote.getQuestionId())
	// 			.lectureTime(lectureNote.getLectureTime())
	// 			.pdfUrl(lectureNote.getPdfUrl())
	// 			.build();
	// 		return dto;
	// 	}
	// 	return null;
	// }
	//
	// // 강의 노트 삽입, 수정
	// @Override
	// @Transactional
	// public void saveLectureNote(LectureNoteDto lectureNoteDto) {
	// 	lectureNoteRepository.save(lectureNoteDto.toEntity());
	// }
	//
	// // 강의 노트 제거
	// @Override
	// public void removeLectureNote(int lectureNoteId) {
	// 	Optional<LectureNote> lectureNoteWrapper = lectureNoteRepository.findById(lectureNoteId);
	// 	if(lectureNoteWrapper.isPresent()){
	// 		LectureNote lectureNote = lectureNoteWrapper.get();
	// 		lectureNoteRepository.deleteById(lectureNote.getLectureNoteId());
	// 	}
	// }
}
