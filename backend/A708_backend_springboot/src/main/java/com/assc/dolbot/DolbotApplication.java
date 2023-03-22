package com.assc.dolbot;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.data.jpa.repository.config.EnableJpaAuditing;

@EnableJpaAuditing
@SpringBootApplication
public class DolbotApplication {

	public static void main(String[] args) {
		SpringApplication.run(DolbotApplication.class, args);
	}

}
