/* Author : Fabrice Scheider AKA Denia
* Description : Template avec les d�clarations minimales pour le fonctionnement de la biblioth�que ydle_lib
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
	// Insérer votre code d'init après
		
}

void loop()
{
	y.receive();
	if(y.initialized()){
		// Insérer le code utilisateur ici
	}
}
