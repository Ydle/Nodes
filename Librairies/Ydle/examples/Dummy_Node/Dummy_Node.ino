/* Author : Fabrice Scheider AKA Denia
 * Modified by Xylerk
* Description : Template avec les déclarations minimales pour le fonctionnement de la bibliothèque ydle 
* Licence : CC-BY-SA
*/

#include <TimerOne.h>
#include "Ydle.h"
#define RX_PIN 12
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
		
		//
		Frame_t frame;
		// Choisir le type de trame à envoyer
		y.dataToFrame(&frame, /* Choisir le type de trame */);
		// Ajouter les données dans la trame
		y.addData(&frame, /* type de donnée */, /* donnée */);
		y.addData(&frame, /* type de donnée */, /* donnée */);
		// Envoyer la trame
		y.send();
	}
}
