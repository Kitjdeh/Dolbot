FROM gradle:jdk11-alpine
LABEL name = "KIMDOWON"
LABEL email = "kdw3238@naver.com"

RUN mkdir /srv/docker-server
WORKDIR /srv/docker-server
CMD chmod +x ./gradlew && ./gradlew bootJar && java -jar ./build/libs/dolbot-0.0.1-SNAPSHOT.jar
