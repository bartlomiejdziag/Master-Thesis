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

#define HIGH_PRIORITY 3

SemaphoreHandle_t xTCPSem;
TaskHandle_t xTCPHandle = NULL;

struct netconn *conn;
struct netbuf *buf;
ip_addr_t *addr, dest_addr;
unsigned short port, dest_port;
char msgc[512];
char smsgc[512];
int indx = 0;

const char *firstPart = "GET http://api.openweathermap.org/data/2.5/weather?q=";
const char *secondPart = "&units=metric&lang=pl&appid=839d6df972338bb98ae9a6dbf710ad81\r\n\r\n";
const char *city = "Lodz";

extern void vHTTPTask(void *pvParameters);

void tcpclient_init (void)
{
	xTCPSem = xSemaphoreCreateMutex();  // the semaphore would prevent simultaneous access to tcpsend
	xTaskCreate(vHTTPTask, "HTTPTask", 1024, (void*) 1, HIGH_PRIORITY, &xTCPHandle);
	vTaskSuspend(xTCPHandle);
}

static void tcpsend(char *data)
{

	sprintf(smsgc, "%s%s%s", firstPart, city ,secondPart);
	// semaphore must be taken before accessing the tcpsend function
	xSemaphoreTake(xTCPSem, pdMS_TO_TICKS(500));
	// send the data to the connected connection
	netconn_write(conn, data, strlen(data), NETCONN_COPY);
	// relaese the semaphore
	xSemaphoreGive(xTCPSem);
}

void tcpinit(void) {
	err_t err, connect_error;
	ip4_addr_t serverIP;

	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_TCP);

	if (conn != NULL) {
		/* Bind connection to the port number 7 (port of the Client). */
		err = netconn_bind(conn, IP_ADDR_ANY, 7);

		if (err == ERR_OK) {
			/* The desination IP adress of the computer */

		    if (netconn_gethostbyname("api.openweathermap.org", &serverIP) != ERR_OK) {
		        printf("Error getting weather server IP address\n");
		    }

			dest_port = LWIP_IANA_PORT_HTTP;  // server port

			/* Connect to the TCP Server */
			connect_error = netconn_connect(conn, &serverIP, dest_port);

			// If the connection to the server is established, the following will continue, else delete the connection
			if (connect_error == ERR_OK) {
				// Release the semaphore once the connection is successful
				xSemaphoreGive(xTCPSem);
				while (1) {

					// semaphore must be taken before accessing the tcpsend function
					xSemaphoreTake(xTCPSem, pdMS_TO_TICKS(5000));

					// send the data to the TCP Server
					tcpsend(smsgc);
					/* wait until the data is sent by the server */
					if (netconn_recv(conn, &buf) == ERR_OK)
					{
						/* Extract the address and port in case they are required */
							addr = netbuf_fromaddr(buf);  // get the address of the client
							port = netbuf_fromport(buf);  // get the Port of the client

							strncpy(msgc, buf->p->payload, buf->p->len);   // get the message from the server
							printf("Received message from HTTP server: %s", msgc);

							memset(msgc, '\0', 200);  // clear the buffer
					}
						netbuf_delete(buf);
				}
			} else {
				/* Close connection and discard connection identifier. */
				netconn_close(conn);
				netconn_delete(conn);
			}
		} else {
			// if the binding wasn't successful, delete the netconn connection
			netconn_delete(conn);
		}
	}
}
