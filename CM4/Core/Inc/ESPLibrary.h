/*
 * ESPLibrary.h
 *
 *  Created on: Oct 1, 2024
 *      Author: andra
 */


#ifndef INC_ESPLIBRARY_H_
#define INC_ESPLIBRARY_H_


#include <WiFi.h>
#include "BluetoothSerial.h"


class CustomCommunication {
	public:

 CustomCommunication(const char* ssid, const char* password,const char* BT_name); // Costruttore
 void begin(); // Inizializza la comunicazione
 void handleWiFiCommunication(); // Gestisce la comunicazione WiFi
 void handleBluetoothCommunication(); // Gestisce la comunicazione

 private:
 const char* _ssid;
 const char* _password;
 const char* _BT_name;
 //Inizializzo un oggetto bluetooth
 BluetoothSerial _serialBT;
 //Dichiarazione del server WiFi: oggetto WiFiServer.
 WiFiServer _server;
 }



#endif /* INC_ESPLIBRARY_H_ */
