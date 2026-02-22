///*
// * ESP-01.c
// *
// *  Created on: Jan 28, 2026
// *      Author: ASUS
// */
//
//#include "ESP-01.h"
//
///* -- Helper: build & send Web page via ESP-01 (AT) -- */
///* Sends header + body as a single payload length indicated in AT+CIPSEND */
//void send_web_page_via_esp(int channel, int wheelState, int brushState, int relayState)
//{
//    char body_buf[1400];
//    const char *body_template =
//      "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
//      "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
//      "<style>body{background:#000;color:#fff;font-family:Arial;padding:12px}"
//      ".button{display:inline-block;padding:10px 14px;margin:6px;border-radius:6px;text-decoration:none;background:#444;color:#fff}"
//      ".on{background:#2e8b57}.off{background:#8b0000}</style>"
//      "</head><body>"
//      "<h3>STM32 - Cleaning Robot</h3>"
//      "<p>Controls:</p>"
//      "<a class=\"button %s\" href=\"/wheelon\">WHEEL ON</a>"
//      "<a class=\"button %s\" href=\"/wheeloff\">WHEEL OFF</a><br/>"
//      "<a class=\"button %s\" href=\"/brushon\">BRUSH ON</a>"
//      "<a class=\"button %s\" href=\"/brushoff\">BRUSH OFF</a><br/>"
//      "<a class=\"button %s\" href=\"/relayon\">RELAY ON</a>"
//      "<a class=\"button %s\" href=\"/relayoff\">RELAY OFF</a>"
//      "<p>Status: Wheel: <b>%s</b> &nbsp; Brush: <b>%s</b> &nbsp; Relay: <b>%s</b></p>"
//      "</body></html>";
//
//    const char *wheel_on_cls = wheelState==0 ? "on" : "off";
//    const char *wheel_off_cls = wheelState==1 ? "on" : "off";
//    const char *brush_on_cls = brushState==0 ? "on" : "off";
//    const char *brush_off_cls = brushState==1 ? "on" : "off";
//    const char *relay_on_cls = relayState==1 ? "on" : "off";
//    const char *relay_off_cls = relayState==0 ? "on" : "off";
//    const char *wheel_status = wheelState==0 ? "ON" : "OFF";
//    const char *brush_status = brushState==0 ? "ON" : "OFF";
//    const char *relay_status = relayState==1 ? "ON" : "OFF";
//
//    int body_len = snprintf(body_buf, sizeof(body_buf), body_template,
//      wheel_on_cls, wheel_off_cls, brush_on_cls, brush_off_cls,
//      relay_on_cls, relay_off_cls,
//      wheel_status, brush_status, relay_status);
//
//    if (body_len < 0) return;
//    if (body_len >= (int)sizeof(body_buf)) body_len = sizeof(body_buf) - 1;
//
//    char header[200];
//    int header_len = snprintf(header, sizeof(header),
//        "HTTP/1.1 200 OK\r\n"
//        "Content-Type: text/html; charset=utf-8\r\n"
//        "Content-Length: %d\r\n"
//        "Connection: close\r\n"
//        "\r\n",
//        body_len);
//
//    if (header_len < 0) return;
//
//    int total_len = header_len + body_len;
//
//    char atcmd[64];
//    int atcmd_len = snprintf(atcmd, sizeof(atcmd), "AT+CIPSEND=%d,%d\r\n", channel, total_len);
//    if (atcmd_len < 0) return;
//
//    uint8_t rxTmp[512];
//    memset(rxTmp,0,sizeof(rxTmp));
//    // send CIPSEND
//    HAL_UART_Transmit(&huart1, (uint8_t*)atcmd, atcmd_len, 1000);
//    // wait for '>' prompt (small timeout)
//    HAL_UART_Receive(&huart1, rxTmp, sizeof(rxTmp), 200);
//    if (strchr((char*)rxTmp, '>') == NULL) {
//      // try small delay then continue anyway
//      HAL_Delay(30);
//    }
//
//    // send header then body
//    HAL_UART_Transmit(&huart1, (uint8_t*)header, header_len, 2000);
//    HAL_UART_Transmit(&huart1, (uint8_t*)body_buf, body_len, 2000);
//
//    // read ESP response (SEND OK)
//    memset(rxTmp,0,sizeof(rxTmp));
//    HAL_UART_Receive(&huart1, rxTmp, sizeof(rxTmp), 300);
//}
//
//void wifiConfiguration(uint8_t rxBuffer[], uint8_t ATisOK, int channel, char ATcommand[]){
//
//  // Reset module
//  sprintf(ATcommand,"AT+RST\r\n");
//  memset(rxBuffer,0,sizeof(rxBuffer));
//  HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//  HAL_UART_Receive (&huart1, rxBuffer, 512, 200);
//  HAL_Delay(500);
//
//  // CWMODE -> AP
//  ATisOK = 0;
//  while(!ATisOK){
//	sprintf(ATcommand,"AT+CWMODE_CUR=2\r\n");
//	  memset(rxBuffer,0,sizeof(rxBuffer));
//	  HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//	  HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
//	if(strstr((char *)rxBuffer,"OK")){
//	  ATisOK = 1;
//	}
//	HAL_Delay(200);
//  }
//
//  // SoftAP config: SSID "STM32", password "cleaningrobot"
//  ATisOK = 0;
//  while(!ATisOK){
//	sprintf(ATcommand,"AT+CWSAP_CUR=\"STM32\",\"cleaningrobot\",1,3,4,0\r\n");
//	  memset(rxBuffer,0,sizeof(rxBuffer));
//	  HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//	  HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
//	if(strstr((char *)rxBuffer,"OK")){
//	  ATisOK = 1;
//	}
//	HAL_Delay(200);
//  }
//
//  // IP
//  ATisOK = 0;
//  while(!ATisOK){
//	sprintf(ATcommand,"AT+CIPAP_CUR=\"192.168.51.1\"\r\n");
//	memset(rxBuffer,0,sizeof(rxBuffer));
//	HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//	HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
//	if(strstr((char *)rxBuffer,"OK")){
//	  ATisOK = 1;
//	}
//	HAL_Delay(200);
//  }
//
//  // enable multiple connections
//  ATisOK = 0;
//  while(!ATisOK){
//	sprintf(ATcommand,"AT+CIPMUX=1\r\n");
//	  memset(rxBuffer,0,sizeof(rxBuffer));
//	  HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//	  HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
//	  if(strstr((char *)rxBuffer,"OK")){
//		ATisOK = 1;
//	  }
//	  HAL_Delay(200);
//  }
//
//  // start server port 80
//  ATisOK = 0;
//  while(!ATisOK){
//	sprintf(ATcommand,"AT+CIPSERVER=1,80\r\n");
//	memset(rxBuffer,0,sizeof(rxBuffer));
//	HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//	HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
//	if(strstr((char *)rxBuffer,"OK")){
//		ATisOK = 1;
//	}
//	HAL_Delay(200);
//  }
//  HAL_Delay(500);
//}
//
//
//void channelDetermination(uint8_t rxBuffer[], int channel){
//  if(strstr((char *)rxBuffer,"+IPD,0")) channel = 0;
//  else if(strstr((char *)rxBuffer,"+IPD,1")) channel = 1;
//  else if(strstr((char *)rxBuffer,"+IPD,2")) channel = 2;
//  else if(strstr((char *)rxBuffer,"+IPD,3")) channel = 3;
//  else if(strstr((char *)rxBuffer,"+IPD,4")) channel = 4;
//  else if(strstr((char *)rxBuffer,"+IPD,5")) channel = 5;
//  else if(strstr((char *)rxBuffer,"+IPD,6")) channel = 6;
//  else if(strstr((char *)rxBuffer,"+IPD,7")) channel = 7;
//}
//
//void closeConnection(char ATcmdClose, uint8_t rxBuffer[], int channel){
//  sprintf(ATcmdClose, "AT+CIPCLOSE=%d\r\n", channel);
//  memset(rxBuffer,0,sizeof(rxBuffer));
//  HAL_UART_Transmit(&huart1,(uint8_t *)ATcmdClose,strlen(ATcmdClose),1000);
//  HAL_UART_Receive (&huart1, rxBuffer, 512, 200);
//}
