#include <stdarg.h>
#include <string.h>
#include <ip_addr.h>

#include "FreeRTOS.h"
#include "task.h"

#include "lwip.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/api.h"

#include "semphr.h"

#include "printf.h"

#include "http_client.h"
#include "parson.h"

#define MAX_PRIORITY 4

SemaphoreHandle_t xTCPSem;
TaskHandle_t xTCPHandle = NULL;
QueueHandle_t xWeatherQueue;

ip_addr_t *addr, dest_addr;
unsigned short dest_port;
char msgc[512];
char smsgc[512];

const char *http_command = "GET http://api.openweathermap.org/data/2.5/weather?q=";
const char *weather_request = "&units=metric&lang=pl&appid=839d6df972338bb98ae9a6dbf710ad81\r\n\r\n";
const char *city = "Lodz";

extern void vHTTPTask(void *pvParameters);

static void tcpsend(char *data, struct netconn *con)
{

	sprintf(smsgc, "%s%s%s", http_command, city ,weather_request);
	// semaphore must be taken before accessing the tcpsend function
	xSemaphoreTake(xTCPSem, pdMS_TO_TICKS(500));
	// send the data to the connected connection
	netconn_write(con, data, strlen(data), NETCONN_COPY);
	// relaese the semaphore
	xSemaphoreGive(xTCPSem);
}

static int parse_OWM_data(char *data, weather_t *w) {
    JSON_Value *jsonValue = json_parse_string(data);
    JSON_Object *jsonObject = json_value_get_object(jsonValue);

    JSON_Object *main = json_object_get_object(jsonObject, "main");
    float temperature = json_object_get_number(main, "temp");
    float feels_like = json_object_get_number(main, "feels_like");
    float pressure = json_object_get_number(main, "pressure");
    float humidity = json_object_get_number(main, "humidity");

    float visibility = json_object_get_number(jsonObject, "visibility");
    float wind_speed = json_object_dotget_number(jsonObject, "wind.speed");

    w->temperature = temperature;
    w->feels_like = feels_like;
    w->pressure = (uint32_t) pressure;
    w->humidity = (uint32_t) humidity;
    w->visibility = (uint32_t) visibility / 1000.0;
    w->wind_speed = wind_speed;

    printf("temp: %0.2f feels: %0.2f pressure: %u humidity: %u wind_speed: %0.2f\r\n", w->temperature, w->feels_like, w->pressure, w->humidity, w->wind_speed);
    return 0;
}

void openweather_init(void)
{
	xTCPSem = xSemaphoreCreateMutex();  // the semaphore would prevent simultaneous access to tcpsend
	xTaskCreate(vHTTPTask, "HTTPTask", 1024, (void*) 1, MAX_PRIORITY, &xTCPHandle);
	vTaskSuspend(xTCPHandle);
}

void openweather_task(weather_t *w, struct netconn *con, struct netbuf *buffer) {
	err_t err, connect_error;
	ip4_addr_t serverIP;

	/* Create a new connection identifier. */
	con = netconn_new(NETCONN_TCP);

	if (con != NULL) {
		/* Bind connection to the port number 7 (port of the Client). */
		err = netconn_bind(con, IP_ADDR_ANY, 7);

		if (err == ERR_OK) {
			/* The desination IP adress of the computer */

			if (netconn_gethostbyname("api.openweathermap.org", &serverIP) != ERR_OK) {
				printf("Error getting weather server IP address\n");
			}

			dest_port = LWIP_IANA_PORT_HTTP;  // server port

			/* Connect to the TCP Server */
			connect_error = netconn_connect(con, &serverIP, dest_port);

			// If the connection to the server is established, the following will continue, else delete the connection
			if (connect_error == ERR_OK) {
				// Release the semaphore once the connection is successful
				xSemaphoreGive(xTCPSem);

				// semaphore must be taken before accessing the tcpsend function
				xSemaphoreTake(xTCPSem, pdMS_TO_TICKS(0));

				// send the data to the TCP Server
				tcpsend(smsgc, con);
				/* wait until the data is sent by the server */
				if (netconn_recv(con, &buffer) == ERR_OK) {
					strncpy(msgc, buffer->p->payload, buffer->p->len); // get the message from the server
					parse_OWM_data(msgc, w);

					xQueueSend(xWeatherQueue, &w, 0);
					memset(msgc, '\0', 512);  // clear the buffer
				}
			}
			/* Close connection and discard connection identifier. */
			netconn_close(con);
			netconn_delete(con);
		}
	} else {
		// if the binding wasn't successful, delete the netconn connection
		netconn_delete(con);
	}
	netbuf_delete(buffer);
}
