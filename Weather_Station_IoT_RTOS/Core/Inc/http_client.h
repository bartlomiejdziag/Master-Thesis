/*
 * http_client.h
 *
 *  Created on: 25 lip 2022
 *      Author: dzion
 */

#ifndef INC_HTTP_CLIENT_H_
#define INC_HTTP_CLIENT_H_

extern QueueHandle_t xWeatherQueue;

typedef struct weather {
    float temperature;                         // in celsius
    float feels_like;                          // in celsius
    uint32_t pressure;                          // hPa
    uint32_t humidity;                          // %
    float visibility;                          // km
    float wind_speed;                          // m/s
    time_t sunrise;                             // linux timestamp
    time_t sunset;                              // linux timestamp
} weather_t;

void openweather_init(void);
void openweather_task(weather_t *w, struct netconn *con, struct netbuf *buffer);

#endif /* INC_HTTP_CLIENT_H_ */
