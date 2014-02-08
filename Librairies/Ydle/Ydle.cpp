// Ydle.cpp
//
// Ydle implementation for Arduino
// See the README file in this directory for documentation
// For changes, look at Ydle.h
//
// Authors:
// Fabrice Scheider AKA Denia,
// Manuel Esteban AKA Yaug
// Matthieu Desgardin AKA Zescientist
// Yargol AKA Yargol
//
// WebPage: http://www.ydle.fr/index.php
// Contact: http://forum.ydle.fr/index.php
// Licence: CC by sa (http://creativecommons.org/licenses/by-sa/3.0/fr/)
// Pll function inspired on VirtualWire library


#include <TimerOne.h>
#include "Ydle.h"
#include <avr/eeprom.h>


const PROGMEM prog_uchar _atm_crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};


static Frame_t g_m_receivedframe;  // received frame
static Frame_t g_frameBuffer[YDLE_MAX_FRAME];
static Frame_t g_sendFrameBuffer; // Une seule pour le moment

static uint8_t m_data[YDLE_MAX_SIZE_FRAME]; // data + crc

static uint8_t pinRx = 12;		// Le numéro de la broche IO utilisée pour le module récepteur
static uint8_t pinTx = 10;		// Le numéro de la broche IO utilisée pour le module émetteur
static uint8_t pinLed = 13;		// Le numéro de la broche IO utilisée pour la Led de statut
static uint8_t pinCop = 8;		// Permet la recopie du signal sur une sortie pour vérification à l'oscilloscope
static uint8_t pinButton = 3;	// Le numéro de la broche IO utilisée pour l'installation du boutton de resettage

static unsigned char start_bit2 = 0b01000010; // Octet de start

volatile uint8_t sample_value = 0;		// Disponibilité d'un sample
volatile uint8_t sample_count = 1;		// Nombre de samples sur la période en cours
volatile uint8_t last_sample_value = 0; // La valeur du dernier sample reéu

// La rampe PLL, varie entre 0 et 159 sur les 8 samples de chaque période de bit
// Quand la PLL est synchronisée, la transition de bit arrive quand la rampe vaut 0
static uint8_t pll_ramp = 0;

// La somme des valeurs des samples. si inférieur à 5 "1" samples dans le cycle de PLL
// le bit est déclaré comme 0, sinon à 1
static uint8_t sample_sum = 0;
static uint16_t rx_bits = 0;		// Les 16 derniers bits reçus, pour repérer l'octet de start
volatile unsigned long t_start = 0; // Temps du premier sample de chaque bit
static uint8_t rx_active = 0;		// Flag pour indiquer la bonne réception du message de start

#define YDLE_SPEED 1000					// Le débit de transfert en bits/secondes
#define YDLE_TPER 1000000/YDLE_SPEED	// La période d'un bit en microseconds
#define YDLE_FBIT YDLE_TPER/8			// La fréquence de prise de samples

static uint8_t bit_value = 0;	// La valeur du dernier bit récupéré
static uint8_t bit_count = 0;	// Le nombre de bits récupérés
static uint8_t sender = 0;		// Id sender reçue
static uint8_t receptor = 0;	// Id receptor reçue
static uint8_t type = 0;		// Info type reçue
static uint8_t parite = false;	// Info parité reçue
static uint8_t taille = 0;		// Info taille reçue
static int rx_bytes_count = 0;	// Nombre d'octets reçus
static uint8_t length_ok = 0;	// Disponibilité de la taille de trame

volatile uint8_t wait_ack = 0;
volatile uint8_t last_check = 0;
volatile uint8_t retry = 0;

static uint8_t rx_done = 0; // Si le message est complet
static uint8_t frameReadyToBeRead = false;
static uint8_t pFrame = 0;
volatile uint8_t transmission_on = false;

// Catch function for interrupt the reset button
void reset(){
#ifdef _YDLE_DEBUG
	Serial.println("Interrupt catched");
#endif
	ydle::resetNode();
}

void ydle::resetNode(){
	memset((void*)&m_Config, 0x0, sizeof(Config_t));
	eeprom_write_block((void*)&m_Config,0, sizeof(m_Config));

	m_initializedState = false;
	m_bLnkSignalReceived = false;
}

// Initialisation des IO avec des valeurs entrées par l'utilisateur
ydle::ydle(int rx, int tx, int button)
{
	m_bLnkSignalReceived = false;
	m_initializedState = false;
	this->_callback_set = false;
	readEEProm();
	pinMode(rx, INPUT);
	pinRx = rx;
	pinMode(tx, OUTPUT);
	pinTx = tx;
	//pinMode(button, INPUT);
	pinButton = button;
	pinMode(pinLed, OUTPUT);
	//attachInterrupt(1, reset, RISING);
}

// Initialisation des IO avec les valeurs par défaut
ydle::ydle()
{
	m_bLnkSignalReceived = false;
	m_initializedState = false;
	this->_callback_set = false;
	readEEProm();
	pinMode(pinRx, INPUT);
	pinMode(pinTx, OUTPUT);
	pinMode(pinButton, INPUT);
	pinMode(pinLed, OUTPUT);
	// Permet la recopie du signal sur une sortie pour vérification é l'oscilloscope
	//pinMode(pinCop, OUTPUT);
	
}

void ydle::init_timer(){
	Timer1.initialize(YDLE_FBIT); // set a timer of length 125 microseconds
	Timer1.attachInterrupt( timerInterrupt ); // attach the service routine here
}

void timerInterrupt(){
	if(!transmission_on){
		if(rx_done)
		{
			rx_done = false;
		}
		sample_value = digitalRead(pinRx);
		pll();
	}
}

void ydle::attach(ydleCallbackFunction function){
	this->callback = function;
	this->_callback_set = true;
}

// Used to read the configuration
void ydle::ReadConfig()
{
	readEEProm();
}

// lire en mémoire EEProm du signal de référence
void ydle::readEEProm()
{
	memset((void*)&m_Config,0x0,sizeof(m_Config));
	eeprom_read_block ((void*)&m_Config, 0, sizeof(Config_t));
	if((m_Config.IdMaster!=0)&&(m_Config.IdNode!=0)){
		m_initializedState = true;
#ifdef _YDLE_DEBUG
		log("Config find in eeprom");
		log("Config.IdMaster : ", m_Config.IdMaster);
		log("Config.IdNode :", m_Config.IdNode);
#endif
	}
	else{
		#ifdef _YDLE_DEBUG
		log("NODE IS NOT INIT");
		log("No config in EEprom");
		#endif
		m_initializedState = false;
		memset((void*)&m_Config,0,sizeof(m_Config));
	}
}

// Ecriture en mémoire EEProm du signal de référence
void ydle::writeEEProm()
{
	eeprom_write_block((void*)&m_Config,0, sizeof(m_Config));
	#ifdef _YDLE_DEBUG
	log("Enregistrement du signal reçu comme signal de réference");
	#endif
}

uint8_t ydle::crc8(const uint8_t* buf, uint8_t length) {
	// The inital and final constants as used in the ATM HEC.
	const uint8_t initial = 0x00;
	const uint8_t final = 0x55;
	uint8_t crc = initial;
	while (length) {
		crc = pgm_read_byte_near(_atm_crc8_table + (*buf ^ crc));
		buf++;
		length--;
	}
	return crc ^ final;
}

unsigned char ydle::computeCrc(Frame_t* frame){
	unsigned char *buf, crc;
	int a,j;

	buf = (unsigned char*)malloc(frame->taille+3);
	memset(buf, 0x0, frame->taille+3);

	buf[0] = frame->sender;
	buf[1] = frame->receptor;
	buf[2] = frame->type;
	buf[2] = buf[2] << 5;
	buf[2] |= frame->taille;

	for(a=3, j=0 ;j < frame->taille - 1;a++, j++){
		buf[a] = frame->data[j];
	}
	crc = crc8(buf,frame->taille+2);
	free(buf);
	return crc;
}

void ydle::receive(){
	unsigned char crc_p;
	
	if(frameReadyToBeRead){
		for(unsigned char i = 0; i < pFrame; i++){
			crc_p = computeCrc(&g_frameBuffer[i]);
			if(crc_p != g_frameBuffer[i].crc)
			{
#ifdef _YDLE_DEBUG
				printFrame(&g_frameBuffer[i]);
				log("crc error!!!!!!!!!");
#endif // _YDLE_DEBUG
			}
			else
			{
#ifdef _YDLE_DEBUG
				Serial.println("Frame ready to be handled");
#endif // _YDLE_DEBUG
				// We receive a CMD so trait it
				if(g_frameBuffer[i].type == YDLE_TYPE_CMD)
				{
					handleReceivedFrame(&g_frameBuffer[i]);
#ifdef _YDLE_DEBUG
					printFrame(&g_frameBuffer[i]);
#endif // _YDLE_DEBUG
				}else if(g_frameBuffer[i].type == YDLE_TYPE_ACK){
					Serial.println("ACK received");
					if(g_frameBuffer[i].sender == g_sendFrameBuffer.receptor 
						&& g_frameBuffer[i].receptor == g_sendFrameBuffer.sender){
						wait_ack = 0;
						retry = 0;
						last_check = 0;
					}
				}else{
#ifdef _YDLE_DEBUG
					printFrame(&g_frameBuffer[i]);
#endif
					// Send the frame to the callback function
					if(this->_callback_set)
						this->callback(&g_frameBuffer[i]);
				}
				// Frame handled			
			}
		}
		// Peut poser un probléme si une interruption se produit et
		// que la pll termine de traiter un paquet et la pose sur la pile exactement à ce moment là
		pFrame = 0;
		frameReadyToBeRead = false;
		if(wait_ack == 1){
			if(retry <= 3){
				uint8_t curt = millis();
				if(curt - last_check >= YDLE_ACK_TIMEOUT){
					last_check = curt;
					this->send(&g_sendFrameBuffer);
#ifdef _YDLE_DEBUG
					Serial.println("Timeout, resending frame");
#endif
				}
				retry++;
			}else{
				// Lost packet... sorry dude !
				wait_ack = 0;
				retry = 0;
				last_check = 0;
#ifdef _YDLE_DEBUG
				Serial.println("Lost packet... sorry dude !");
#endif
			}
		}
	}
}

// Do something with a received Command
void ydle::handleReceivedFrame(Frame_t *frame)
{
	int litype;
	long livalue;
	Serial.println("I'm here");
	//Special case for CMD_LINK
	if(this->extractData(frame, 0,litype, livalue)==1)
	{
		#ifdef _YDLE_DEBUG
		log("Type of  Message Received ",litype);
		#endif
		//A node ask to link us
		if(litype == YDLE_CMD_LINK)
		{
			#ifdef _YDLE_DEBUG
			log("Link received ",litype);
			#endif
			// we are not already linked
			if(!m_initializedState)
			{
				m_Config.IdNode = frame->receptor;
				m_Config.IdMaster = frame->sender;
				m_initializedState = true;
				writeEEProm();
			}
		}
		else if(litype==YDLE_CMD_RESET)
		{
			
			if (checkSignal(frame))
			{
				#ifdef _YDLE_DEBUG
				log("Reset received ",litype);
				#endif
				m_Config.IdNode = 0;
				m_Config.IdMaster = 0;
				writeEEProm();
				m_initializedState = false;
			}
		}
		
		// send ACK if frame is for us.
		if(checkSignal(frame))
		{
			delay(1000);
			#ifdef _YDLE_DEBUG
			log("**************Send ACK********************");
			#endif
			Frame_t response;
			memset(&response, 0x0, sizeof(response));
			dataToFrame(&response, YDLE_TYPE_ACK);	// Create a new ACK Frame
			#ifdef _YDLE_DEBUG
			printFrame(&response);
			Serial.println("End send ack");
			#endif
			send(&response);
		}
	}
}

// Synchronise l'AGC, envoie l'octet de start puis transmet la trame
void ydle::send(Frame_t *frame)
{
	int i = 0,j = 0;

	digitalWrite(pinLed, HIGH);   // on allume la Led pour indiquer une émission
	digitalWrite(5, LOW);
	// calcul crc
	frame->taille++; // add crc BYTE
	frame->crc = computeCrc(frame);

	if(frame->type == YDLE_TYPE_STATE_ACK){
		if(wait_ack != 1){
			memcpy(&g_sendFrameBuffer, &frame, sizeof(Frame_t));
			wait_ack = 1;
		}
	}

#ifdef _YDLE_DEBUG
	printFrame(frame);
#endif
	// From now, we are ready to transmit the frame

	if(!rx_active){
		// Wait that the current transmission finish
		delay(250);
	}
	// So, stop the interruption while sending
	Timer1.stop();
	
 	// Sequence de bits pour réglages de l'AGC
 	for (i=0; i<32; i++)
 	{
 		digitalWrite(pinTx, HIGH);
 		delayMicroseconds(YDLE_TPER);     // un bit é l'état haut
 		digitalWrite(pinTx, LOW);
 		delayMicroseconds(YDLE_TPER);     // un bit é l'état bas
	}
	
 	// Octet de start annoncant le départ du signal au recepteur 	
	for (i = 7; i>=0; i--)
	{
		digitalWrite(pinTx, (start_bit2 & 1<<i)); 
		delayMicroseconds(YDLE_TPER); 
		digitalWrite(pinTx, !(start_bit2 & 1<<i));
		delayMicroseconds(YDLE_TPER);
 	}

	for(i = 7; i>=0; i--){
		digitalWrite(pinTx, (frame->receptor & 1<<i)); 
		delayMicroseconds(YDLE_TPER);
		digitalWrite(pinTx, !(frame->receptor & 1<<i));
		delayMicroseconds(YDLE_TPER);
	}
	for(i = 7; i>=0; i--){
		digitalWrite(pinTx, (frame->sender & 1<<i));
		delayMicroseconds(YDLE_TPER);
		digitalWrite(pinTx, !(frame->sender & 1<<i));
		delayMicroseconds(YDLE_TPER);
	}
	for(i = 2; i>=0; i--){
		digitalWrite(pinTx, (frame->type & 1<<i));
		delayMicroseconds(YDLE_TPER);
		digitalWrite(pinTx, !(frame->type & 1<<i));
		delayMicroseconds(YDLE_TPER);
	}	
	for(i = 4; i>=0; i--){
		digitalWrite(pinTx, (frame->taille & 1<<i));
		delayMicroseconds(YDLE_TPER);
		digitalWrite(pinTx, !(frame->taille & 1<<i));
		delayMicroseconds(YDLE_TPER);
	}
	for(i=0;i<frame->taille-1;i++){
		for(j = 7; j>=0; j--){
			digitalWrite(pinTx, (frame->data[i] & 1<<j)); 
			delayMicroseconds(YDLE_TPER);
			digitalWrite(pinTx, !(frame->data[i] & 1<<j));
			delayMicroseconds(YDLE_TPER);
		}
	}
	for(i = 7; i>=0; i--){
		digitalWrite(pinTx, (frame->crc & 1<<i));
		delayMicroseconds(YDLE_TPER);
		digitalWrite(pinTx, !(frame->crc & 1<<i));
		delayMicroseconds(YDLE_TPER);
	}

	digitalWrite(pinTx, LOW);
	digitalWrite(pinLed, LOW);
	digitalWrite(5, HIGH);
	// Restart the timer task
	Timer1.initialize(128);
	Timer1.attachInterrupt(timerInterrupt);
}

// Comparaison du signal reçu et du signal de référence
bool ydle::checkSignal(Frame_t *frame)
{
	if(frame->sender==m_Config.IdMaster && frame->receptor==m_Config.IdNode)
		return true;
 	else
		return false;
}


void pll()
{
	sample_count ++;
	// On additionne chaque sample et on incrémente le nombre du prochain sample
	if (sample_value){
		sample_sum++;
	}
	
	// On vérifie s'il y a eu une transition de bit
	if (sample_value != last_sample_value){
		// Transition, en avance si la rampe > 40, en retard si < 40
		if(pll_ramp < 80){
			pll_ramp += 11;
		} 
		else{
			pll_ramp += 29;
		}
		last_sample_value = sample_value;
	}
	else{
		// Si pas de transition, on avance la rampe de 20 (= 80/4 samples)
		pll_ramp += 20;
	}
	
	// On vérifie si la rampe é atteint son maximum de 80
	if (pll_ramp >= 160)
	{
		//t_start = micros();
		// On ajoute aux 16 derniers bits reéus rx_bits, MSB first
		// On stock les 16 derniers bits
		rx_bits <<= 1;
		
		// On vérifie la somme des samples sur la période pour savoir combien était é l'état haut
		// S'ils étaient < 2, on déclare un 0, sinon un 1;
		if (sample_sum >= 5){
			rx_bits |= 0x1;
			bit_value = 1;
		}
		else{
			rx_bits |= 0x0;
			bit_value = 0;
		}
		pll_ramp -= 160; // On soustrait la taille maximale de la rampe é sa valeur actuelle
		sample_sum = 0; // On remet la somme des samples é 0 pour le prochain cycle
		sample_count = 1; // On ré-initialise le nombre de sample

		// Si l'on est dans le message, c'est ici qu'on traite les données
		if (rx_active){
			bit_count ++;
			// On récupére les bits et on les places dans des variables
			// 1 bit sur 2 avec Manchester
			if (bit_count % 2 == 1){
				if (bit_count < 16){
					// Les 8 premiers bits de données
					receptor <<= 1;
					receptor |= bit_value;
				}
				else if (bit_count < 32){
					// Les 8 bits suivants
					sender <<= 1;
					sender |= bit_value;
				}
				else if (bit_count < 38){
					// Les 3 bits de type
					type <<= 1;
					type |= bit_value;
				}
				else if (bit_count < 48){
					// Les 5 bits de longueur de trame
					rx_bytes_count <<= 1;
					rx_bytes_count |= bit_value;
				}
				else if ((bit_count-48) < (rx_bytes_count * 16)){
					// les données
					length_ok = 1;
					m_data[(bit_count-48)/16] <<= 1;
					m_data[(bit_count-48)/16]|= bit_value;
				}
			}
			
			// Quand on a reéu les 24 premiers bits, on connait la longueur de la trame
			// On vérifie alors que la longueur semble logique
			if (bit_count >= 48)
			{
				// Les bits 19 é 24 informent de la taille de la trame
				// On les vérifie car leur valeur ne peuvent étre < é 1 et > é 30 + 1 pour le CRC
				if (rx_bytes_count < 1 || rx_bytes_count > 30)
				{
#ifdef _YDLE_DEBUG
					Serial.println("error!");
#endif
					// Mauvaise taille de message, on ré-initialise la lecture
					rx_active = false;
					sample_count = 1;
					bit_count = 0;
					length_ok = 0;
					sender = 0;
					receptor = 0;
					type = 0;
					parite = 0;
					taille = 0;
					memset(m_data,0,sizeof(m_data));
					t_start = micros();
					return;
				}
			}

			// On vérifie si l'on a reéu tout le message
			if ((bit_count-48) >= (rx_bytes_count*16) && (length_ok == 1))
			{
#ifdef _YDLE_DEBUG
				Serial.println("complete");
#endif

				rx_active = false;
				g_m_receivedframe.sender = sender;
				g_m_receivedframe.receptor = receptor;
				g_m_receivedframe.type = type;
				g_m_receivedframe.taille = rx_bytes_count; // data + crc
				memcpy(g_m_receivedframe.data,m_data,rx_bytes_count-1); // copy data len - crc
				g_m_receivedframe.crc=m_data[rx_bytes_count-1];

				// May be an array ?
				if(pFrame == YDLE_MAX_FRAME){
					pFrame = 0;
				}
				memcpy(&g_frameBuffer[pFrame], &g_m_receivedframe, sizeof(Frame_t));
				pFrame++;
				frameReadyToBeRead = true;

				length_ok = 0;
				sender = 0;
				receptor = 0;
				type = 0;
				taille = 0;
				memset(m_data,0,sizeof(m_data));
			}

		}

		// Pas dans le message, on recherche l'octet de start
		else if (rx_bits == 0x6559)
		{
#ifdef _YDLE_DEBUG
			Serial.println("start");
#endif
			// Octet de start, on commence é collecter les données
			rx_active = true;
			bit_count = 0;
			rx_bytes_count = 0;
		}
	}
}


// Fonction qui crée une trame avec les infos fournies
void ydle::dataToFrame(Frame_t *frame, unsigned long destination, unsigned long sender, unsigned long type)
{
	frame->sender = sender;
	frame->receptor = destination;
	frame->type = type;
	frame->taille = 0;
	frame->crc = 0;
	memset(frame->data,0,sizeof(frame->data));
} 

// Fonction qui crée une trame avec un type fournie
void ydle::dataToFrame(Frame_t *frame, unsigned long type)
{
	frame->sender = m_Config.IdNode;
	frame->receptor = m_Config.IdMaster;
	frame->type = type;
	frame->taille = 0;
	frame->crc = 0;
	memset(frame->data, 0x0, sizeof(frame->data));
} 

// Fonction qui retourne "true" si la Node est initialisée
bool ydle::initialized()
{
	return m_initializedState;
}

int ydle::isSignal()
{
	return rx_active;
}

// ----------------------------------------------------------------------------
/**
	   Function: addCmd
	   Inputs:  int type type of data
				int data

	   Outputs: 

*/
// ----------------------------------------------------------------------------
void ydle::addCmd(Frame_t *frame, int type, int data)
{
	frame->data[frame->taille]=type<<4;
	frame->data[frame->taille+1]=data;
	frame->taille+=2;
}

// ----------------------------------------------------------------------------
/**
	   Function: extractData
	   Inputs:  int index: index de la value recherche (0..29)
				int itype: en retour type de la value
				int ivalue: en retour, value

	   Outputs: 1 value trouve,0 non trouve,-1 no data

*/
// ----------------------------------------------------------------------------
int ydle::extractData(Frame_t *frame, int index, int &itype, long &ivalue)
{
	uint8_t* ptr;
	bool bifValueisNegativ=false;
	int iCurrentValueIndex=0;
	bool bEndOfData=false;
	int  iModifType=0;
	int  iNbByteRest;

	ptr=frame->data;
	
	if(frame->taille <2) // Min 1 byte of data with the 1 bytes CRC always present, else there is no data
	 return -1;
	
	iNbByteRest= frame->taille-1;
	while (!bEndOfData)
	{
		itype=*ptr>>4;
		bifValueisNegativ=false;
		
		// This is a very ugly code :-( Must do something better
		if( frame->type==YDLE_TYPE_CMD)
		{
			// Cmd type if always 12 bits signed value
			iModifType=YDLE_DATA_DEGREEC;
		}
		else if(frame->type==YDLE_TYPE_STATE)
		{
			iModifType=itype;
		}
		else
		{
			iModifType=itype;
		}

		switch(iModifType)
		{
			// 4 bits no signed
			case YDLE_DATA_STATE :    
				ivalue=*ptr&0x0F;
				ptr++;
				iNbByteRest--;
			break;	

			// 12 bits signed
			case YDLE_DATA_DEGREEC:  
			case YDLE_DATA_DEGREEF : 
			case YDLE_DATA_PERCENT : 
			case YDLE_DATA_HUMIDITY: 
				if(*ptr&0x8)
					bifValueisNegativ=true;
				ivalue=*ptr&0x0F<<8;
				ptr++;
				ivalue+=*ptr;
				ptr++;
				if(bifValueisNegativ) 
					ivalue=ivalue *(-1);
				iNbByteRest-=2;	
			break;	

			// 12 bits no signed
			case YDLE_DATA_DISTANCE: 
			case YDLE_DATA_PRESSION: 
				ivalue=(*ptr&0x0F)<<8;
				ptr++;
				ivalue+=*ptr;
				ptr++;
				iNbByteRest-=2;	
			break;	

			// 20 bits no signed
			case YDLE_DATA_WATT  :   
				ivalue=(*ptr&0x0F)<<16;
				ptr++;
				ivalue+=*ptr<<8;
				ptr++;
				ivalue+=*ptr;
				ptr++;
				iNbByteRest-=3;	
			break;	
		}
		
		if (index==iCurrentValueIndex)
			return 1;
		
		iCurrentValueIndex++;
		
		if(iNbByteRest<1)
			bEndOfData =true;;
	}

	return 0;	
}


union _FP16 ydle::floatToHalf(float number){
	union _FP16 o = { 0 };
	union _FP32 f;
	f.f = number;
	// Based on ISPC reference code (with minor modifications)
	if (f.Exponent == 0) // Signed zero/denormal (which will underflow)
	o.Exponent = 0;
	else if (f.Exponent == 255) // Inf or NaN (all exponent bits set)
	{
		o.Exponent = 31;
		o.Mantissa = f.Mantissa ? 0x200 : 0; // NaN->qNaN and Inf->Inf
	}
	else // Normalized number
	{
		// Exponent unbias the single, then bias the halfp
		int newexp = f.Exponent - 127 + 15;
		if (newexp >= 31) // Overflow, return signed infinity
		o.Exponent = 31;
		else if (newexp <= 0) // Underflow
		{
			if ((14 - newexp) <= 24) // Mantissa might be non-zero
			{
				uint16_t mant = f.Mantissa | 0x800000; // Hidden 1 bit
				o.Mantissa = mant >> (14 - newexp);
				if ((mant >> (13 - newexp)) & 1) // Check for rounding
				o.u++; // Round, might overflow into exp bit, but this is OK
			}
		}
		else
		{
			o.Exponent = newexp;
			o.Mantissa = f.Mantissa >> 13;
			if (f.Mantissa & 0x1000) // Check for rounding
			o.u++; // Round, might overflow to inf, this is OK
		}
	}
	            
	o.Sign = f.Sign;
	return o;
}

void ydle::addData(Frame_t *frame, float data){
	union _FP16 h_data = this->floatToHalf(data);
	int current_index = frame->taille;
		
	frame->data[current_index] = ((0xFF00) & h_data.u) >> 8;
	++current_index;
	frame->data[current_index] = ((0xFF) & h_data.u);
	++current_index;
	frame->taille = current_index;
}

void ydle::addData(Frame_t *frame, int data){
	uint16_t data_o = (uint16_t) data;
	int current_index = frame->taille;
	frame->data[current_index] = ((0xFF00) & data) >> 8;
	++current_index;
	frame->data[current_index] = ((0xFF) & data);
	++current_index;
	frame->taille = current_index;
}
// ----------------------------------------------------------------------------
/**
	   Function: addData
	   Inputs:  int type type of data
				long data

	   Outputs: 

*/
// ----------------------------------------------------------------------------
void ydle::addData(Frame_t * frame, int type, long data)
{
	int oldindex = frame->taille;

	switch (type){
		// 4 bits no signed
		case YDLE_DATA_STATE :    
			if (frame->taille < 29)
			{
				frame->taille++;
				frame->data[oldindex]=type<<4;
				frame->data[oldindex]+=data&0x0f;
			}
#ifdef _YDLE_DEBUG
			else
				log("invalid trame len in addData");
#endif
		break;	

		// 12 bits signed
		case YDLE_DATA_DEGREEC:  
		case YDLE_DATA_DEGREEF : 
		case YDLE_DATA_PERCENT : 
		case YDLE_DATA_HUMIDITY: 
			if (frame->taille < 28)
			{
				frame->taille += 2;
				frame->data[oldindex]=type<<4;
				if (data <0)
				{
					data=data *-1;
					frame->data[oldindex]^=0x8;
				}
				frame->data[oldindex]+=(data>>8)&0x0f;
				frame->data[oldindex+1]=data;
			}
#ifdef _YDLE_DEBUG
			else
				log("invalid trame len in addData");
#endif
		break;	

		// 12 bits no signed
		case YDLE_DATA_DISTANCE: 
		case YDLE_DATA_PRESSION: 
			if (frame->taille < 28)
			{
				frame->taille += 2;
				frame->data[oldindex]=type<<4;
				frame->data[oldindex]+=(data>>8)&0x0f;
				frame->data[oldindex+1]=data;
			}
#ifdef _YDLE_DEBUG
			else
				log("invalid trame len in addData");
#endif
		break;	

		// 20 bits no signed
		case YDLE_DATA_WATT  :   
			if (frame->taille<27)
			{
				frame->taille+=3;
				frame->data[oldindex]=type<<4;
				frame->data[oldindex]+=(data>>16)&0x0f;
				frame->data[oldindex+1]=(data>>8)&0xff;
				frame->data[oldindex+2]=data;
			}
#ifdef _YDLE_DEBUG
			else
				log("invalid trame len in addData");
#endif // _YDLE_DEBUG
		break;	
	}
}

// Affiche les logs sur la console série
void ydle::log(String msg)
{
#if not defined( __AVR_ATtiny85__ ) or defined(_YDLE_DEBUG)
	Serial.println(msg);
#endif
}
// Affiche les logs sur la console série
void ydle::log(String msg,int i)
{
#if not defined( __AVR_ATtiny85__ ) or defined(_YDLE_DEBUG)
	Serial.print(msg);
	Serial.println(i);
#endif
}

// ----------------------------------------------------------------------------
/**
	   Function: printFrame
	   Inputs:  Frame_t trame  frame to log
				int data

	   Outputs: 
		Log a frame if debug activated
*/
// ----------------------------------------------------------------------------
void ydle::printFrame(Frame_t *trame)
{
#if not defined( __AVR_ATtiny85__ ) or defined (_YDLE_DEBUG)
	// if debug
		char sztmp[255];
		
		log("-----------------------------------------------");
		sprintf(sztmp,"Emetteur :%d",trame->sender);
		log(sztmp);

		sprintf(sztmp,"Recepteur :%d",trame->receptor);
		log(sztmp);

		sprintf(sztmp,"Type :%d",trame->type);
		log(sztmp);

		sprintf(sztmp,"Taille :%d",trame->taille);
		log(sztmp);

		sprintf(sztmp,"CRC :%d",trame->crc);
		log(sztmp);

		sprintf(sztmp,"Data Hex: ");
		for (int a=0;a<trame->taille-1;a++)
			sprintf(sztmp,"%s 0x%02X",sztmp,trame->data[a]);
		log(sztmp);

		sprintf(sztmp,"Data Dec: ");
		for (int a=0;a<trame->taille-1;a++)
			sprintf(sztmp,"%s %d",sztmp,trame->data[a]);
		log(sztmp);
		log("-----------------------------------------------");
#endif
}

