/* Author : Xylerk
* Description : Envoi d'une trame contenant une température et une humidité générées aléatoirement
* Licence : CC-BY-SA
*/

#include <TimerOne.h>
#include "Ydle.h"
#define RX_PIN 2
#define TX_PIN 10
#define BT_PIN 3

// Init de la node, 
ydle y(RX_PIN, TX_PIN, BT_PIN);

void setup()
{
	y.init_timer();
        Serial.begin(115200);
	Serial.println("Envoi température et humidité dans une seule trame");    		
}

void loop()
{
	// Génération température alétoire et affichage moniteur série
        float temperature=(random(100)+.01*random(100))*(-1)*pow(2,random(2));
        Serial.print("Température : ");
        Serial.println(temperature);
        // Génération humidité alétoire et affichage moniteur série
        float humidity=random(100)+.01*random(100);
        Serial.print("Humidité : ");
        Serial.println(humidity);
        // Construction trame
        Frame_t frame;
        //Déclaration type de trame
        y.dataToFrame(&frame, YDLE_TYPE_STATE);
        //Ajout température
        y.addData(&frame, YDLE_DATA_DEGREEC, temperature);
        //Ajout humidité
        y.addData(&frame, YDLE_DATA_HUMIDITY, humidity);
        //envoi de la trame
        y.send(&frame);
        delay(5000);
}