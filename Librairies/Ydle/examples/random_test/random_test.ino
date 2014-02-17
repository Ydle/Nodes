/* Author : Xylerk
* Description : Test envoi valeur aléatoire
* Licence : CC-BY-SA
*/

#include <TimerOne.h>
#include "Ydle.h"
#define RX_PIN 2
#define TX_PIN 10
#define BT_PIN 3

// Init de la node, 
ydle y(RX_PIN, TX_PIN, BT_PIN);

float test=50;
void setup()
{
	y.init_timer();
        Serial.begin(115200);
	// Insérer votre code d'init après
		
}

void loop()
{
	Frame_t frame;
        y.dataToFrame(&frame, YDLE_TYPE_STATE);
        test=(random(100)+.1*random(10))*(-1)*pow(2,random(2));
        y.addData(&frame, YDLE_DATA_DEGREEC, test);
        y.send(&frame);
        delay(1000);
}
