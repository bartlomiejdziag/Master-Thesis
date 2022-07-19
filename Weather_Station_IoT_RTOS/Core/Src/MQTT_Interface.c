#include <stdarg.h>
#include <string.h>
#include "lwip.h"
#include "lwip/apps/mqtt.h"
#include "MQTT_Interface.h"
#include "printf.h"


static int inpub_id;
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
//  printf("Incoming publish at topic %s with total length %u\n\r", topic, (unsigned int)tot_len);

  /* Decode topic string into a user defined reference */
  if(strcmp(topic, "print_payload") == 0) {
    inpub_id = 0;
  } else if(topic[0] == 'A') {
    /* All topics starting with 'A' might be handled at the same way */
    inpub_id = 1;
  } else {
    /* For all other topics */
    inpub_id = 2;
  }
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{

  if(flags & MQTT_DATA_FLAG_LAST) {
    /* Last fragment of payload received (or whole part if payload fits receive buffer
       See MQTT_VAR_HEADER_BUFFER_LEN)  */

    /* Call function or do action depending on reference, in this case inpub_id */
    if(inpub_id == 0) {
      /* Don't trust the publisher, check zero termination */
      if(data[len-1] == 0) {

      }
    } else if(inpub_id == 1) {
      /* Call an 'A' function... */
    } else {

    }
  } else {
    /* Handle fragmented payload, store in buffer, write to file or whatever */
  }
}

static void mqtt_sub_request_cb(void *arg, err_t result)
{
  /* Just print the result code here for simplicity,
     normal behaviour would be to take some action if subscribe fails like
     notifying user, retry subscribe or disconnect from server */
  printf("Subscribe result: %d\n\r", result);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
  err_t err;
  if(status == MQTT_CONNECT_ACCEPTED) {
    printf("mqtt_connection_cb: Successfully connected\n\r");

    /* Setup callback for incoming publish requests */
    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);

    /* Subscribe to a topic named "subtopic" with QoS level 1, call mqtt_sub_request_cb with result */
    err = mqtt_subscribe(client, "lwip_test", 1, mqtt_sub_request_cb, arg);

    if(err != ERR_OK) {
      printf("mqtt_subscribe return: %d\n\r", err);
    }
  } else {
    printf("mqtt_connection_cb: Disconnected, reason: %d\n\r", status);

    /* Its more nice to be connected, so try to reconnect */
    mqtt_user_connect(client);
  }
}

/* Called when publish is complete either with sucess or failure */
static void mqtt_pub_request_cb(void *arg, err_t result)
{
  if(result != ERR_OK) {
    printf("Publish result: %d\n\r", result);
  }
}

err_t mqtt_user_connect(mqtt_client_t *client)
{

    struct mqtt_connect_client_info_t ci;
    err_t err;
    ip_addr_t server;

    memset(&ci, 0, sizeof(ci));

    ci.client_id = "lwip_test";
    ci.client_user = "sammy";
    ci.client_pass = "Password";

    ip4_addr_set_u32(&server, ipaddr_addr("192.168.1.13"));

    err = mqtt_client_connect(client, &server, MQTT_TLS_PORT, mqtt_connection_cb, 0, &ci);

    if (err != ERR_OK) {
        printf("mqtt_connect return %d\n\r", err);
    }

    return err;
}

void mqtt_user_publish(mqtt_client_t *client, void *arg, char const *format, ...) {

	err_t err;
	u8_t qos = 1; /* 0 1 or 2, see MQTT specification */
	u8_t retain = 0; /* No don't retain such crappy payload... */

	va_list args;
	char buf[512];

	va_start(args, format);
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);

	err = mqtt_publish(client, "lwip_test", buf, strlen(buf), qos, retain, mqtt_pub_request_cb, arg);
	if (err != ERR_OK) {
		printf("Publish err: %d\n\r", err);
	}
}

