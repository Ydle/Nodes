/* Author : Fabrice Scheider AKA Denia
* Description : Sktech de test de la librairie
* Licence : CC-BY-SA
*/

#include <TimerOne.h>
#include "Ydle.h"

#define DELAY_SEND 15000

static ydle y(12, 10, 3);

unsigned long last_send, cur_time;
int i = 0;
// User callback to handle the new order
void dummy_callback(Frame_t *frame){
	Serial.println("Hey, i'm the callback dude !");
	Serial.print("Paquet recu de :");
	Serial.println(frame->sender);
	Serial.print("Type de trame:");
	Serial.println(frame->type);
}

void setup()
{
	Serial.begin(115200);
	Serial.println("init complete");

	y.init_timer();
	y.attach(dummy_callback);
	
	cur_time = 0;
	last_send = cur_time;
}

void loop()
{
	y.receive();
	if(y.initialized()){
		cur_time = millis();
		if(cur_time - last_send >= DELAY_SEND){
			last_send = cur_time;
			Frame_t f;
			y.dataToFrame(&f, 4);
			y.addData(&f, YDLE_DATA_DEGREEC, i++);
			y.send(&f);
		}
	}
}

