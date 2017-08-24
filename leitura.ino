//Obter a biblioteca DS3231 em: http://www.rinkydinkelectronics.com/library.php?id=73
////////////////////////  Inicio do RTC  ////////////////////////////////////// 
	#include <DS3231.h>

	DS3231  rtc(SDA, SCL);
///////////////////////////////////////////////////////////////////////////////

////////////////////////  Inicio dos Sensores de corrente ////////////////////////
	const uint8_t SENSOR_PIN = A3;
	const uint8_t SENS = 610;   // sensor sensitivity from datasheet in mV/A. 5A sensor=185, 20A=100, 30A=66
	int16_t sensorZeroAdj = 0;  // calculate in setup()
	#define SENSE_DC  0   // sense DC or AC - changes how the current is calculated

	const uint8_t SENSOR2_PIN = A2;   // sensor sensitivity from datasheet in mV/A. 5A sensor=185, 20A=100, 30A=66
	int16_t sensor2ZeroAdj = 0;  // calculate in setup()
	// sense DC or AC - changes how the current is calculated
///////////////////////////////////////////////////////////////////////////////

////////////////////////  Inicio do Sensor UV /////////////////////////////////
	//Connect the following ML8511 breakout board to Arduino:
 	//3.3V = 3.3V
 	//OUT = A0
 	//GND = GND
 	//EN = 3.3V
 	//3.3V = A1 Pino de referência da alimentação (jump do A1 para 3V3)
	int UVOUT = A0;
	int REF_3V3 = A1;
///////////////////////////////////////////////////////////////////////////////

////////////////////////////// SD CARD ///////////////////////////////////////
	// Ported to SdFat from the native Arduino SD library example by Bill Greiman
	// On the Ethernet Shield, CS is pin 4. SdFat handles setting SS
	const int chipSelect = 4;
	/*
 	SD card read/write
  
 	This example shows how to read and write data to and from an SD card file 	
 	The circuit:
 	* SD card attached to SPI bus as follows:
 	** MOSI - pin 11
 	** MISO - pin 12
 	** CLK - pin 13
 	** CS - pin 4
 
 	created   Nov 2010
 	by David A. Mellis
 	updated 2 Dec 2010
 	by Tom Igoe
 	modified by Bill Greiman 11 Apr 2011
 	This example code is in the public domain.
 	*/
  #include <SPI.h>
  #include <SD.h>
  File Coleta;  
 
	int LEITURA = 7;  //PINO PARA FREQUENCIA DE AMOSTRAGEM
	boolean ESTADO;


//////////////////////////////////////////////////////////////////////////////

void setup()
{
	analogReference(DEFAULT);
	delay(1000);
  	Serial.begin(9600);

//////////////////////////////////////////////////////////
  	Serial.println("Inicializando SD card...");

  if (!SD.begin(4)) 
  {
    Serial.println("Inicializacao falhou!");
    return;
  }
  /*for (int i = 0; i < 3; ++i)
    {
      digitalWrite(pronto, HIGH);
      delay(100);
      digitalWrite(pronto, LOW);
      delay(100);
    }*/
//////////////////////////////////////////////////////////

	pinMode(LEITURA, OUTPUT); 
	Serial.println("inicializando RTC");
	rtc.begin();//inicía o RTC
////////////////////////////  RTC  //////////////////////////////////////////////////////////////
	// As próximas linhas são apenas para definir hora e datas do RTC
	// (ATENÇÃO)-> Após a configuração deve-se recarregar o código sem essas linhas!	
  	//rtc.setDOW(WEDNESDAY);     // Set Day-of-Week to SUNDAY
  	//rtc.setTime(12, 0, 0);     // Set the time to 12:00:00 (24hr format)
  	//rtc.setDate(1, 1, 2014);   // Set the date to January 1st, 2014


	//Usar rtc.get... DOWStr ou DateStr ou TimeTtr para adiquirir os dados como no exemplo a baixo
 	// Send date
//////////////////////////////////////////////////////////////////////////////////////////////////////
	pinMode(UVOUT, INPUT);//( define entrada para Sensor UV )
  	pinMode(REF_3V3, INPUT);//(define entrada para Tensão de referencia do sensor UV para o Arduino)


  	/////////////////////////  Sensor de corrente /////////////////////////////////////////////////  	
  
  pinMode(SENSOR_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);

  // Read the analog input a few times to makes sure that we get
  // a good zero adjustment from nominal center value. There should be no
  // current flowing while this is happening.
  /*Serial.println("Calibrando sensores de correntes");
  for (uint8_t a=0; a<10; a++)
  {
    uint16_t error = 512 - analogRead(SENSOR_PIN);        //ATENÇÃO: EXISTEM DOIS SENSORES 
    sensorZeroAdj = ((sensorZeroAdj * (a-1)) + error)/a;
  }


  for (uint8_t b=0; b<10; b++)
  {
    uint16_t error = 512 - analogRead(SENSOR2_PIN);        //ATENÇÃO: EXISTEM DOIS SENSORES 
    sensor2ZeroAdj = ((sensor2ZeroAdj * (b-1)) + error)/b;
  }
  	Serial.println("----- Fim da calibração ------");
  	Serial.print("valor do zero sensor 1: ");
  	Serial.println(sensorZeroAdj);
  	Serial.print("Valor do zero sensor 2: ");
   	Serial.println(sensor2ZeroAdj);*/

  ///////////////////////////////////  FIM DO SETUP  ///////////////////////////////////////////	
}

void loop()
{
	Serial.println("Iniciando LOOP");
	float CORRENTE=0;
	float CORRENTE2=0;
	float UV=0;


	ESTADO=!ESTADO;
	digitalWrite(LEITURA, ESTADO);	
	
/////////////////////////////// COMEÇO DA LEITURA ////////////////////////////////////////////////////  
/////////////////////////////  SENSOR DE CORRENTE //////////////////////////////////////////////////

		Serial.println("----- Inicio da coleta -----");
		Serial.println(rtc.getTimeStr());
		CORRENTE=senseCurrent(SENSOR_PIN,sensorZeroAdj);		
		CORRENTE2=senseCurrent(SENSOR2_PIN,sensor2ZeroAdj); 
		//Serial.print("Valor do pino analogico 1: ");
		//Serial.println(analogRead(SENSOR_PIN));
		//Serial.print("Valor do pino analogico 2: ");
		//Serial.println(analogRead(SENSOR2_PIN));
		Serial.print("Valor da corrente 1: ");
		Serial.println(CORRENTE,4);
		Serial.print("Valor da corrente 2: "); 
		Serial.println(CORRENTE2,4);
		
		//delay(500);	
////////////////////////////////////////////////////////////////////////////////////////////////////

 /////////////////////////////// SENSOR UV ////////////////////////////////////////////////////////////
		Serial.println("Coletando UV sensor");
		int uvLevel = averageAnalogRead(UVOUT);
		int refLevel = averageAnalogRead(REF_3V3);
		//Use the 3.3V power pin as a reference to get a very accurate output value from sensor
		 float outputVoltage = 3.3 / refLevel * uvLevel;

     	float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV
     	UV=uvIntensity; 

 //////////////////////////////////////////////////////////////////////////////////////////////////////

     	//Serial.println(rtc.getDateStr());
     	Serial.print("Sensor UV: ");    	
     	Serial.println(UV);
     	Serial.println("----- Fim da coleta -----");
		Serial.println(" ");

 ///////////////////////////////////////////// SD CARD //////////////////////////////////////////////////////////
		// open the file for write at end like the Native SD library
  
  Serial.println("Inicializacao pronta.");
  float X=112233;
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  Coleta = SD.open("Coleta.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (Coleta) 
  	{
    	Serial.println("Gravando em Coleta.txt...");
    	Coleta.print(rtc.getDateStr());
		Coleta.print(";");
		Coleta.print(rtc.getTimeStr());
		Coleta.print(";");
		Coleta.print(CORRENTE);
		Coleta.print(";");
		Coleta.print(CORRENTE2);
		Coleta.print(";");
		Coleta.println(UV);
		Coleta.close();
    	Serial.println("Fim da gravacao.");
  	} 
  else 
  	{
    	// if the file didn't open, print an error:
    	Serial.println("Erro ao tentar abrir Coleta.txt");
  	}
    //delay (1000);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
}
////////////////////////////////////// FIM DO LOOP //////////////////////////////////////////////////////////////


/////////////////////////////////////////// SENSOR DE CORRENTE Funções ////////////////////////////////////////////

// Calculate current Vcc in mV from the 1.1V reference voltage
/*float readVcc()
{
  	float result = 5000L;

  	// Read 1.1V reference against AVcc - hardware dependent
  	// Set the reference to Vcc and the measurement to the internal 1.1V reference
	#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  		ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  		ADMUX = _BV(MUX5) | _BV(MUX0);
	#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  		ADMUX = _BV(MUX3) | _BV(MUX2);
	#else
  		ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#endif  
  
	#if defined(__AVR__)
  	delay(2);         // Wait for Vref to settle
  		ADCSRA |= _BV(ADSC);    // Convert, result is stored in ADC at end
  	while (bit_is_set(ADCSRA,ADSC)); // measuring
  		result = ADCL; // must read ADCL (low byte) first - it then locks ADCH 
  		result |= ADCH<<8; // // unlocks both

  		result = 1125300L / result; // Back-calculate AVcc in mV; 1125300 = 1.1*1023*1000
	#endif
  
  	return(result);
}
*/
// Sense the DC current
float senseCurrent(int pino, int zero)
{
  // read the value from the sensor:

  unsigned int sensorValue =0;
  for (int i = 0; i < 10; ++i)
  {
    sensorValue =sensorValue + analogRead(pino) + zero;
  }
    sensorValue=sensorValue/10;
    Serial.print("Valor do pino analogico leitura: ");
    Serial.println(sensorValue);

  delay(10);
  float Vcc = 4.6;
  
    //float convertedmA = (1000000.0 * (((Vcc*(float)sensorValue)/1023.0)-(Vcc/2.0)))/SENS;
  	float convertedmA = ((float)sensorValue*4.3/1023.0);
  //Serial.print(sensorValue);
  //Serial.print(" @Vcc ");
  //Serial.print(Vcc);
  //Serial.print (" mV: ");

  return(convertedmA);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////   SENSOR UV funções //////////////////////////////////////////////////////////////
  //Takes an average of readings on a given pin
  //Returns the average
	int averageAnalogRead(int pinToRead)
	{
  		byte numberOfReadings = 10;
  		unsigned int runningValue = 0; 

  		for(int x = 0 ; x < numberOfReadings ; x++)
    	runningValue += analogRead(pinToRead);
  		runningValue /= numberOfReadings;

  		return(runningValue);  
	}

	//The Arduino Map function but for floats
	//From: http://forum.arduino.cc/index.php?topic=3922.0
		float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
		{
  			return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);

		}//Convert the voltage to a UV intensity level
 
 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
