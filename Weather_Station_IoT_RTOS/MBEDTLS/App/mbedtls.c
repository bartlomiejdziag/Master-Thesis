/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : mbedtls.c
  * Description        : This file provides code for the configuration
  *                      of the mbedtls instances.
  ******************************************************************************
  ******************************************************************************
   * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include <string.h>
#include "printf.h"
#include "mbedtls/memory_buffer_alloc.h"
#include "mbedtls/net_sockets.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "mbedtls.h"

/* USER CODE BEGIN 0 */
#define SERVER_NAME  "a3fkdk3h9bh19m-ats.iot.eu-west-1.amazonaws.com"
//#define SERVER_NAME  "3.248.137.106"

//Amazon ECC 256 certificate
const char mbedtls_aws_root_certificate [ ]  =
		"-----BEGIN CERTIFICATE-----\r\n"
		"MIIBtjCCAVugAwIBAgITBmyf1XSXNmY/Owua2eiedgPySjAKBggqhkjOPQQDAjA5\r\n"
		"MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6b24g\r\n"
		"Um9vdCBDQSAzMB4XDTE1MDUyNjAwMDAwMFoXDTQwMDUyNjAwMDAwMFowOTELMAkG\r\n"
		"A1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJvb3Qg\r\n"
		"Q0EgMzBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABCmXp8ZBf8ANm+gBG1bG8lKl\r\n"
		"ui2yEujSLtf6ycXYqm0fc4E7O5hrOXwzpcVOho6AF2hiRVd9RFgdszflZwjrZt6j\r\n"
		"QjBAMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/BAQDAgGGMB0GA1UdDgQWBBSr\r\n"
		"ttvXBp43rDCGB5Fwx5zEGbF4wDAKBggqhkjOPQQDAgNJADBGAiEA4IWSoxe3jfkr\r\n"
		"BqWTrBqYaGFy+uGh0PsceGCmQ5nFuMQCIQCcAu/xlJyzlvnrxir4tiz+OpAUFteM\r\n"
		"YyRIHN8wfdVoOw==\r\n"
		"-----END CERTIFICATE-----\r\n" ;

//client certificate here
const char mbedtls_client_certificate [ ]  =
		"-----BEGIN CERTIFICATE-----\r\n"
		"MIIDWjCCAkKgAwIBAgIVAJK5UDanSCTMduI7tQGxOTU4ELP4MA0GCSqGSIb3DQEB\r\n"
		"CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t\r\n"
		"IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yMjA3MTcxOTMy\r\n"
		"MzFaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh\r\n"
		"dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDLx5T9YNBAEWRVEojw\r\n"
		"hAgN9kX/9kXpfjj5x0CKNF7qUvRWgJXka6v57e9/jBj2aKLutp740Q0NMWZLu+S3\r\n"
		"zo+CKUpNX2+/l/6fDLgY/eNuQkYJ0cWBBTZjix7P/0FEPLVqTrADZg2Zwb08/QlC\r\n"
		"WY9MRotmjBsn7EGgO82lcSPTHmTU+ADzi5xaaMjJmoJrJxOwfTAq2QmGPbIBpLno\r\n"
		"6S4Qg/rq7+66VhXDIG8l0kONnfBxh8+n1GeLSYn0fiW9UmKsqR8YbdyZ/BpMMixM\r\n"
		"IYfPdsX76KRenjqHP8Oy5PWTo67KEuCVrBEllKWje4FJh2Hof0ET2pUYSDPTdgb+\r\n"
		"B+5dAgMBAAGjYDBeMB8GA1UdIwQYMBaAFOesuEUsgJ00WoR2+Nb9Wex/TiG9MB0G\r\n"
		"A1UdDgQWBBTCkMvimy4EHE/4Bmi+573trhXnNTAMBgNVHRMBAf8EAjAAMA4GA1Ud\r\n"
		"DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAmldB7V5hQLqiVRJrC0xBx5Cg\r\n"
		"iUh7uTuCFGkQvsd6SY2gscLZ63Kb4xBQxzZNZnGrrOZ75SeMv6mAUfQGjbHk+bC8\r\n"
		"8anyb7r0mgtlqKSml56bovlNdWzfVmradLH8ejEBe7p6rrrtZf5ep+8M4MT3EVhY\r\n"
		"9Qrgg4vF2+Fw4Gthb59iwMyvdADfA/q+jgABT4/uzZ5I24uMKb0de835tYr9CDSL\r\n"
		"xGi86rPEEmCmcTjXP9ODl+9rPtZIU0HgCRl/o88nQtdgetljwNfuU3vxWdjb/m1X\r\n"
		"/KxJh+qnRcNaxojxyK/8Qawgub5Td91G9ezjP118w52onaGItb1wz+TKnnwuTA==\r\n"
		"-----END CERTIFICATE-----\r\n" ;

//client private key here
const char mbedtls_client_key [ ]  =
		"-----BEGIN RSA PRIVATE KEY-----\r\n"
		"MIIEogIBAAKCAQEAy8eU/WDQQBFkVRKI8IQIDfZF//ZF6X44+cdAijRe6lL0VoCV\r\n"
		"5Gur+e3vf4wY9mii7rae+NENDTFmS7vkt86PgilKTV9vv5f+nwy4GP3jbkJGCdHF\r\n"
		"gQU2Y4sez/9BRDy1ak6wA2YNmcG9PP0JQlmPTEaLZowbJ+xBoDvNpXEj0x5k1PgA\r\n"
		"84ucWmjIyZqCaycTsH0wKtkJhj2yAaS56OkuEIP66u/uulYVwyBvJdJDjZ3wcYfP\r\n"
		"p9Rni0mJ9H4lvVJirKkfGG3cmfwaTDIsTCGHz3bF++ikXp46hz/DsuT1k6OuyhLg\r\n"
		"lawRJZSlo3uBSYdh6H9BE9qVGEgz03YG/gfuXQIDAQABAoIBAHHPQ7zHZ0eA7yI8\r\n"
		"OdLqSn1RwAKWjrE7V6LYz4jgeuov7lqpcg2ivOfXJV185ywngGgGAn9AciOeJV3C\r\n"
		"0Jh/etLdMKxJ0uWUhBZcSi1bDrwzs14klEPcn2W0+z57dv4fUcrPq6oaQxpOg9we\r\n"
		"Dh/KWSfmdlCNi1UetABDpjTyHWn4ncomaCZh20rSDnyhYj96VYtxqJkmk2lRB/wR\r\n"
		"HgBwJTWESQEKVSRsBzD6gEf35tifqin1IKvCzlbv0mQS/lw52Ng4ItsxecC5jaRU\r\n"
		"AMkohXbnCNHJcNQGQSuT8ItdGWF9WYreqmrWXqpHl4m0MFj2cbt21RkmC/+J0vv6\r\n"
		"y9iIfNkCgYEA9kcIVgDxankrMZ9FENtIecSEvUNZsrdI3nweTu8jwOstGkahY1cs\r\n"
		"dg1rzTpe0u8QlexqH6aX+/uex1DraSG28dLG9WHgJLJgcWRMLSguzL6dChIZZqpe\r\n"
		"6tXkcgoXZFhp2BGlEeL482ThawKBW0/87o4LG9L5AG6FIxxEjZ2MgEcCgYEA09MN\r\n"
		"Ea+upeTlZijWge+vdBlTf5maMVWh9uPsqG3t104z5zFs7QGn6bqW9Fl7T0R960J3\r\n"
		"dcd+9jqlHR21m0d3UMaESzvYiQO3JO1Qe1k03PMUdXn4oM0OE/3rwunhDD2K+kqJ\r\n"
		"DDGxqjgBLFLeTBlNf6nnQFlTITh1Ow9ATY5jsjsCgYAYaihQZl2IgaSfkbuGFUcV\r\n"
		"Ez9Zh/C/f94QeFuE1b5EpMve2/up3n2A4om6WbwGyz5ornxC0QAmMeamucXssTPy\r\n"
		"u0OQFfjjim93LktC+sXQ9GCbG/o6rE8mlrfD+m4hO4aarf0gTdECSBD0y9XJGJ/p\r\n"
		"glllk7+tpEc8mNPO+jxwhwKBgDQlswP1oiNJFIhv6U8XuLP2QC4bnHAkyrjDJ9HX\r\n"
		"LkRhBjoWWuCGFdZejyccmFTNYcp3FH20XTc0/M0MMVdG4sLJ/GuhEg+5EG9Q1Q1o\r\n"
		"yrDSTukU2/aKRLr7Of1EIpkm1X4Et3R/D2P+YZuN0xgc6o1RQJMA6ow1H/coLAZr\r\n"
		"1XNdAoGAG1pwzhyOVsgmUBTlYTBgspIbEt2Iz7RIOKk9ZHey4TeLBZ4kzRdMXjWT\r\n"
		"iSdzVYu+E7wqGNxrpFQMJriWWz68KHEwrGs8qgeCSCyJH6EtFVfHoHxdDajkDNZ9\r\n"
		"Lfq+u16lWoR6B+AeVXgo97mGGFjJj2NrXXBCgZBe9eW65eMhLNY=\r\n"
		"-----END RSA PRIVATE KEY-----\r\n" ;

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Global variables ---------------------------------------------------------*/
mbedtls_ssl_context ssl;
mbedtls_ssl_config conf;
mbedtls_x509_crt cert;
mbedtls_ctr_drbg_context ctr_drbg;
mbedtls_entropy_context entropy;

/* USER CODE BEGIN 2 */
int ret;
mbedtls_net_context server_fd;
mbedtls_pk_context cli_key;
mbedtls_x509_crt cli_cert;
const char *pers = "mbedtls";

const size_t mbedtls_aws_root_certificate_len = sizeof(mbedtls_aws_root_certificate);
const size_t mbedtls_client_certificate_len = sizeof(mbedtls_client_certificate);
const size_t mbedtls_client_key_len = sizeof(mbedtls_client_key);
/* USER CODE END 2 */

/* MBEDTLS init function */
void MX_MBEDTLS_Init(void)
{
   /**
  */
  mbedtls_ssl_init(&ssl);
  mbedtls_ssl_config_init(&conf);
  mbedtls_x509_crt_init(&cert);
  mbedtls_ctr_drbg_init(&ctr_drbg);
  mbedtls_entropy_init( &entropy );
  /* USER CODE BEGIN 3 */
  dns_init();
  mbedtls_pk_init(&cli_key);
  mbedtls_x509_crt_init(&cli_cert);

  ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy, (const unsigned char*) pers, strlen(pers));

  if (ret < 0) {
		printf("mbedtls_crt_drbg_seed failed.\n");
	}

  ret = mbedtls_x509_crt_parse(&cert, (const unsigned char*) mbedtls_aws_root_certificate , mbedtls_aws_root_certificate_len);

  if (ret < 0) {
		printf("mbedtls_x509_crt_parse failed.\n");
	}

  ret = mbedtls_x509_crt_parse(&cli_cert, (const unsigned char*)mbedtls_client_certificate , mbedtls_client_certificate_len);

	if (ret < 0) {
		printf("mbedtls_x509_crt_parse failed.\n");
	}

	ret = mbedtls_pk_parse_key(&cli_key, (const unsigned char*) mbedtls_client_key, mbedtls_client_key_len, ( unsigned char const  * ) "" , 0);
	if (ret < 0) {
		printf("mbedtls_pk_parse_key failed.\n");
	}

	ret = mbedtls_ssl_config_defaults(&conf, MBEDTLS_SSL_IS_CLIENT,
	MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT);
	if (ret < 0) {
		printf("mbedtls_ssl_config_defaults failed.\n");
	}

	mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_OPTIONAL);
	mbedtls_ssl_conf_ca_chain(&conf, &cert, NULL);
	mbedtls_ssl_conf_rng(&conf, mbedtls_ctr_drbg_random, &ctr_drbg);

	//config client certificate & key
	ret = mbedtls_ssl_conf_own_cert(&conf, &cli_cert, &cli_key);
	if (ret < 0) {
		printf("mbedtls_ssl_conf_own_cert failed.\n");
	}

	//set timeout 1000ms, mbedtls_ssl_conf_read_timeout has problem with accurate timeout
	mbedtls_ssl_conf_read_timeout(&conf, 1000);

	//ssl setup
	ret = mbedtls_ssl_setup(&ssl, &conf);
	if (ret < 0) {
		printf("mbedtls_ssl_setup failed.\n");
	}

	//set hostname
	ret = mbedtls_ssl_set_hostname(&ssl, SERVER_NAME);
	if (ret < 0) {
		printf("mbedtls_ssl_set_hostname failed.\n");
	}

	//set bio
	mbedtls_ssl_set_bio(&ssl, &server_fd, mbedtls_net_send, mbedtls_net_recv, NULL);

  /* USER CODE END 3 */

}

/* USER CODE BEGIN 4 */
int net_connect(void) {
	int ret;

	ret = mbedtls_net_connect(&server_fd, SERVER_NAME, (const char*)MQTT_TLS_PORT, MBEDTLS_NET_PROTO_TCP);
	printf("ret value : %d\n\r", ret);
	if (ret < 0) {
		printf("mbedtls_net_connect failed.\n");
		return -1;
	}

	while ((ret = mbedtls_ssl_handshake(&ssl)) != 0) {
		if (ret != MBEDTLS_ERR_SSL_WANT_READ
				&& ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
			printf("mbedtls_ssl_handshake failed.\n");
			return -1;
		}
	}

	ret = mbedtls_ssl_get_verify_result(&ssl);
	if (ret < 0) {
		printf("mbedtls_ssl_get_verify_result failed.\n");
		return -1;
	}

	return 0;
}

int net_read(unsigned char *buffer, int len, int timeout_ms) {
	int ret;
	int received = 0;
	int error = 0;
	int complete = 0;

	//set timeout
	if (timeout_ms != 0) {
		mbedtls_ssl_conf_read_timeout(&conf, timeout_ms);
	}

	//read until received length is bigger than variable len
	do {
		ret = mbedtls_ssl_read(&ssl, buffer, len);
		if (ret > 0) {
			received += ret;
		} else if (ret != MBEDTLS_ERR_SSL_WANT_READ) {
			error = 1;
		}
		if (received >= len) {
			complete = 1;
		}
	} while (!error && !complete);

	return received;
}

int net_write(unsigned char *buffer, int len, int timeout_ms) {
	int ret;
	int written;

	//check all bytes are written
	for (written = 0; written < len; written += ret) {
		while ((ret = mbedtls_ssl_write(&ssl, buffer + written, len - written))
				<= 0) {
			if (ret != MBEDTLS_ERR_SSL_WANT_READ
					&& ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
				return ret;
			}
		}
	}

	return written;
}

void net_disconnect(void) {
	int ret;

	do {
		ret = mbedtls_ssl_close_notify(&ssl);
	} while (ret == MBEDTLS_ERR_SSL_WANT_WRITE);

	mbedtls_ssl_session_reset(&ssl);
	mbedtls_net_free(&server_fd);
}

void net_clear() {
	mbedtls_net_free(&server_fd);
	mbedtls_ssl_free(&ssl);
	mbedtls_ssl_config_free(&conf);
	mbedtls_x509_crt_free(&cert);
	mbedtls_ctr_drbg_free(&ctr_drbg);
	mbedtls_entropy_free(&entropy);
	mbedtls_x509_crt_free(&cli_cert);
	mbedtls_pk_free(&cli_key);
}
/* USER CODE END 4 */

/**
  * @}
  */

/**
  * @}
  */

