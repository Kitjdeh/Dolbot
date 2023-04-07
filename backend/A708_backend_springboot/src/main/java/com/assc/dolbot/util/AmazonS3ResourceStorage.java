package com.assc.dolbot.util;

import com.amazonaws.services.s3.AmazonS3;
import com.amazonaws.services.s3.AmazonS3Client;
import com.amazonaws.services.s3.model.CannedAccessControlList;
import com.amazonaws.services.s3.model.PutObjectRequest;
import lombok.RequiredArgsConstructor;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;
import org.springframework.web.multipart.MultipartFile;

import java.io.File;

@Component
@RequiredArgsConstructor
public class AmazonS3ResourceStorage {
    @Value("${cloud.aws.s3.bucket}")
    private String bucket;
    private final AmazonS3Client amazonS3Client;

    public String store(String fullPath, MultipartFile multipartFile) {
        File file = new File(MultipartUtil.getLocalHomeDirectory(), fullPath);
        String url = "";
        try {
            multipartFile.transferTo(file);
            amazonS3Client.putObject(new PutObjectRequest(bucket, fullPath, file)
                    .withCannedAcl(CannedAccessControlList.PublicRead));
            url = amazonS3Client.getUrl(bucket,fullPath).toString();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException();
        } finally {
            if (file.exists()) {
                file.delete();
            }
        }
        return url;
    }

}
