#ifndef INC_MQTT_INTERFACE_H_
#define INC_MQTT_INTERFACE_H_

extern mqtt_client_t static_client;

err_t mqtt_user_connect(mqtt_client_t *client);
err_t mqtt_user_publish(mqtt_client_t *client, void *arg, char const *format, ...);

#endif /* INC_MQTT_INTERFACE_H_ */
