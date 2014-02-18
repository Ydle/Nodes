/* Author : Fabrice Scheider AKA Denia
* Modified by Xylerk
* Description : Node envoyant humidité et température en utilisant le protocole radio ydle.
* Note : J'utilise dans cette librairie un DHT11, il est donc nécessaire que vous l'ayez dans 
* votre répertoire librairies pour que le sketch compile.
* Licence : CC-BY-SA
*/

#include <TimerOne.h>
#include "Ydle.h"
#include "dht11.h"

#define RX_PIN 12
#define TX_PIN 10
#define BT_PIN 3

#define DELAY_SEND 15000

dht11 dht;

#define DHT_PIN 9

ydle y(RX_PIN, TX_PIN, BT_PIN);

unsigned long last_send, cur_time;

void setup()
{
	Serial.begin(115200);
	Serial.println("init complete");

	y.init_timer();

	cur_time = millis();
	last_send = cur_time;
}

void loop()
{
	y.receive();
	if(y.initialized()){
		cur_time = millis();
		if(cur_time - last_send >= DELAY_SEND){
			last_send = cur_time;
			// code de récupération des informations venant du capteurs
			if(dht.read(DHT11_PIN) == DHTLIB_OK){
				
				
				// Création de la frame qui va être envoyée
				Frame_t frame;
				// On y insère les différentes valeurs dont nous avons besoin
			
				/** Nous demandons un accusé réception.
				 * La bibliothèque va s'occuper seule de l'acquittement
				 * Si aucun acquittement n'est reçu au bout d'une seconde
				 * elle procédera à un nouvel envois. Si au bout de 3 envois successifs
				 * aucun acquittement n'est reçu, nous considérons la trame comme perdue.
				 */
				y.dataToFrame(&frame, YDLE_TYPE_STATE_ACK);
			
				/** Nous ajoutons les valeurs que nous avons récupérées auprès du capteur	
				 */
				y.addData(&frame, YDLE_DATA_DEGREEC, dht.temperature);
				y.addData(&frame, YDLE_DATA_HUMIDITY, dht.humidity);
				
				/** Nous procédons à l'envois de la trame.
				 *
				 */ 
				y.send(&frame);
			}
		}
	}
}

