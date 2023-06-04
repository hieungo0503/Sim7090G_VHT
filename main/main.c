#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <math.h>
#include <sys/time.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"


#define NUMERO_PORTA_SERIALE2 UART_NUM_2
#define BUF_SIZE (1024 * 2)
//#define RD_BUF_SIZE (1024)
#define U2RXD 16
#define U2TXD 17
#define BAUDRATE 115200
#define Client_ID "da73a39c-ee99-44b6-aa4c-86cb553d712f"
#define PUBLISH_TOPIC "messages/6b88a402-9679-499b-83fc-9502d113df22/attributets"

typedef struct simcom_t
{
 uart_port_t uart_num;
  char RSRP[10];
  char RSRQ[10];
  char SINR[10];
  char PCI[10];
  char cellID[10];
  int tx_io_num;
  int rx_io_num;
  int baud_rate;
  int timestamp;
  int tp;
  bool AT_buff_avai;
  uint8_t AT_buff[BUF_SIZE];
  void (*mqtt_CB)(char * data);
}simcom;
simcom simcom_7090G;
extern uint8_t tp_mess[10];

typedef struct client_t
{
   char client_id[50];
   char url[51];
   char user_name[50];
   char password[50];
   int port;
   char mqtt_id_device[50];
}client;

client client_MQTT=
{
	Client_ID,
	"mqtt.innoway.vn",
	"batky",
	"sLmYvvEAUrrfNjZKVX2xI7CPfiW2fojh",
	1883,
	"dc9d5717-2522-4f39-a899-cce286152284"
	};

 typedef enum
{
 AT_OK,
  AT_ERROR,
  AT_TIMEOUT,
}AT_flag;

AT_flag _readFeedback(uint32_t timeout, char *expect) {
  uint64_t timeCurrent = esp_timer_get_time() / 1000;
  while(esp_timer_get_time() / 1000 < (timeout + timeCurrent)) {
    //delay(10);
    if(simcom_7090G.AT_buff_avai) {
      if(strstr((char *)simcom_7090G.AT_buff, "ERROR"))
      {//delay(1000);
    	  return AT_ERROR;
      }

      else if(strstr((char *)simcom_7090G.AT_buff, expect)) return AT_OK;
    }
  }
  return AT_TIMEOUT;
}


static const char * TAG = "Sim7090G";
void UART_RX(void *pvParameters);
void init_simcom(uart_port_t uart_num, int tx_io_num, int rx_io_num, int baud_rate)
{
  uart_config_t uart_config =
  {
    .baud_rate = baud_rate,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  uart_param_config(uart_num, &uart_config);
  uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_set_pin(uart_num, tx_io_num, rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  xTaskCreate(UART_RX, "uart_task1_receive_data", 4096, NULL, 10, NULL);
}
void UART_RX(void *pvParameters){
  uint8_t data[512];
  while (1) {
    int len = uart_read_bytes(UART_NUM_2, data,BUF_SIZE, 100 / portTICK_PERIOD_MS);
    // Write data back to the UART
    if (len) {
      data[len] = '\0';
      ESP_LOGI(TAG, "Receive: %s", (char*) data);
      if(strstr((char*)data, "+CMQPUB:"))
      {
        simcom_7090G.mqtt_CB((char*)data);
      }
      else if(strstr((char*)data, "+CMQPUB:")) {}
      else
      {
        memcpy(simcom_7090G.AT_buff, data, len);
        simcom_7090G.AT_buff_avai = true;
      }
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}
static void send_ATComand(char *ATcommand) {
  ESP_LOGI(TAG, "Send: %s", ATcommand);
  simcom_7090G.AT_buff_avai = false;
  memset(simcom_7090G.AT_buff, 0, BUF_SIZE);
  uart_write_bytes(UART_NUM_2, (char *)ATcommand, strlen((char *) ATcommand));
  uart_write_bytes(UART_NUM_2, "\r\n", strlen("\r\n"));
  vTaskDelay(100/portTICK_PERIOD_MS);
}

bool isRegistered(int retry)
{
  while(retry--)
  {
    vTaskDelay(1000/portTICK_PERIOD_MS);
    send_ATComand("AT+CREG?");
    //if(_readSerial(1000) == false) continue;
    if(strstr((char*)simcom_7090G.AT_buff, "0,1") || strstr((char*)simcom_7090G.AT_buff, "0,5") || strstr((char*)simcom_7090G.AT_buff, "1,1") || strstr((char*)simcom_7090G.AT_buff, "1,5")|| strstr((char*)simcom_7090G.AT_buff, "2,1") || strstr((char*)simcom_7090G.AT_buff, "2,5")) return true;
    else continue;
  }
  return false;
}
bool isInit(int retry) {
  AT_flag res;
  while(retry--) {
    send_ATComand("AT");
    res = _readFeedback(1000, "OK");
    if(res == AT_OK) return true;
    else if(res == AT_ERROR) return false;
  }
  return false;
}
bool isConnected_network(int retry)
{
	AT_flag res;
	while(retry--)
	{
		send_ATComand("AT+CNACT=0,1");
		res = _readFeedback(1000, "+APP PDP");
		if(res == AT_OK){
		if(strstr((char*)simcom_7090G.AT_buff,"DEACTIVE"))
			return false;
		else
			return true;
		}
		else if(res == AT_ERROR) return false;
	}
	return false;
}

bool mqtt_start(client clientMQTT,int keeptime,int cleanss, int Qos, int Retain, int retry)
{
	if(!isConnected_network(2)) return false;
	AT_flag res;
	int count = 8;
	char buf[300];
	while(retry --)
	{
		while(count --)
		{
			if(count==7)
			{
				sprintf(buf,"AT+SMCONF=\"CLIENTID\",\"%s\"",clientMQTT.client_id);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR || res == AT_TIMEOUT) return false;
			}
			else if(count==6)
			{
				sprintf(buf,"AT+SMCONF=\"URL\",\"%s\",\"%s\"",clientMQTT.url,"1883");
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR || res == AT_TIMEOUT) return false;
			}
			else if(count==5)
			{
				sprintf(buf,"AT+SMCONF=\"USERNAME\",\"%s\"",clientMQTT.user_name);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR || res == AT_TIMEOUT) return false;
			}
			else if(count==4)
			{
				sprintf(buf,"AT+SMCONF=\"PASSWORD\",\"%s\"",clientMQTT.password);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR || res == AT_TIMEOUT) return false;
			}
			else if(count==3)
			{
				sprintf(buf,"AT+SMCONF=\"KEEPTIME\",\"%d\"",keeptime);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR || res == AT_TIMEOUT) return false;
			}
			else if(count==2)
			{
				sprintf(buf,"AT+SMCONF=\"CLEANSS\",\"%d\"",cleanss);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR || res == AT_TIMEOUT) return false;
			}
			else if(count==1)
			{
				sprintf(buf,"AT+SMCONF=\"QOS\",\"%d\"",Qos);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR || res == AT_TIMEOUT) return false;
			}
			else if(count==0)
			{
				sprintf(buf,"AT+SMCONF=\"RETAIN\",\"%d\"",Retain);
				send_ATComand(buf);
				res = _readFeedback(1000, "OK");
				if(res == AT_ERROR || res == AT_TIMEOUT) return false;
				else if(res == AT_OK) return true;
			}
		}

	}
	return true;
}
bool Connect_MQTT(int retry)
{
	AT_flag res;
		while(retry--)
		{
			send_ATComand("AT+SMCONN");
			res = _readFeedback(5000, "OK");
			if(res == AT_OK) return true;
			else if(res == AT_ERROR) return false;
		}
		return false;
}
bool MQTT_PUBLISH(char*topic,char* data,int retry,int retain,int Qos)
{
	AT_flag res;
	char buff[200];
	sprintf(buff,"AT+SMPUB=\"%s\",%d,%d,%d",topic,strlen(data),Qos,retain);
	while(retry --)
	{
		send_ATComand(buff);
		send_ATComand(data);

		res = _readFeedback(1000,"OK");
		if(res== AT_OK) return true;
		else if(res == AT_ERROR) return false;
	}
	return false;
}

// subscribe one topic to server
/*
bool mqtt_subscribe(client clientMQTT, char *topic, int qos, int retry,  void (*mqttSubcribeCB)(char * data)) {
  AT_flag res;
  char buf[200];
  sprintf(buf, "AT+SMSUB=\"%s\",%d",topic, qos);
  while(retry--) {
    send_ATComand(buf);
    res = _readFeedback(3000, "OK");
    if(res == AT_OK)
    {
      simcom_7090G.mqtt_CB = mqttSubcribeCB;
      return true;
    }
    //else if(res == AT_ERROR) return false;
  }
  return false;
}
*/
static int filter_comma(char *respond_data, int begin, int end, char *output)
{
    memset(output, 0, strlen(output));
    int count_filter = 0;
    int lim = 0;
    int start = 0;
    int finish = 0;
    int i = 0;
    for (i = 0; i < strlen(respond_data); i++)
    {
        if ( respond_data[i] == ',')
        {
            count_filter ++;
            if (count_filter == begin)          start = i+1;
            if (count_filter == end)            finish = i;
        }

    }
    lim = finish - start;
    for (i = 0; i < lim; i++){
        output[i] = respond_data[start];
        start ++;
    }
    output[i] = 0;
    return 0;
}

bool get_RSRP_RSRQ_SINR_PCI_cellID(int retry)
{
	AT_flag res;
	while(retry --)
	{
		send_ATComand("AT+CENG?");
		res = _readFeedback(1000, "LTE NB-IOT");
		if(res == AT_OK)
		{
			filter_comma((char*)simcom_7090G.AT_buff, 6, 7, simcom_7090G.RSRP);
			filter_comma((char*)simcom_7090G.AT_buff, 8, 9, simcom_7090G.RSRQ);
			filter_comma((char*)simcom_7090G.AT_buff, 9, 10, simcom_7090G.SINR);
			filter_comma((char*)simcom_7090G.AT_buff, 5, 6, simcom_7090G.PCI);
			filter_comma((char*)simcom_7090G.AT_buff, 11, 12, simcom_7090G.cellID);
			return true;
		}
		else
			return false;

	}
	return false;
}

void app_main(void)
{
	init_simcom(UART_NUM_2,U2TXD,U2RXD,BAUDRATE);

//    ESP_LOGI(TAG,"IS CONNECT MQTT Success? %d",mqtt_start(client_MQTT, 60, 0, 0, 0, 1));
//    Connect_MQTT(1);
    //MQTT_PUBLISH(PUBLISH_TOPIC, "{\"longtitude\":106.674141,\"latitude\":10.789906}",1,0,0);

    ESP_LOGI(TAG,"GET DATA SUCCESS? %d",get_RSRP_RSRQ_SINR_PCI_cellID(1));
    ESP_LOGI(TAG,"RSRP=%s,RSRQ=%s,SINR=%s,PCI=%s,CELLID=%s",simcom_7090G.RSRP,simcom_7090G.RSRQ,simcom_7090G.SINR,simcom_7090G.PCI,simcom_7090G.cellID);

    while (true) {


       vTaskDelay(5000/portTICK_PERIOD_MS);
    }
}
