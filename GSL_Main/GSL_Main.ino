/* Edited by Brian Sarracino-Aguilera 9/3/2015
 * Note: I do not own this code, it is open source code
 * that I edited for my own applications. 
*/

/*Arduino Total Control para principiantes
 Recuerda desconectar el bluetooth pin TX cuando cargues el codigo
 Funciones basicas para controlar y mostras informacion con la app.
 Este codigo funciona para cualqiuer placa arduino:

 * Bluetooth Module conectado a puerto serial
   * Contola 8 relays, aplicaciones, circuitos, etc. (y es expandible)
 * Relays conectados a los pines definidos en RelayPins[MAX_RELAYS]
 * Toma muestras analogicas de los convertidoes en A0 hasta A4
 * Hay un boton conectado de tierra (GND) a A5 para explicar como apagar y prender los relays manualmente (relay 1)
 * Los estados de los relevadores son recordados en la EEPROM

 Para enviar informacion a la app usar los siguientes tags o etiquetas:
 Para controlar el estado de los botones de la app: (<ButnXX:Y\n), Donde XX va de 0 a 19 y es el numero de boton, Y es 0 o 1 es el estado del boton
 Ejemplo: Serial.println("<Butn05:1"); Pondra en estado desactivado el boton 5 de la app

 Para modificar el contenido de los textos via arduino: <TextXX:YYYY\n, Donde XX va de 0 a 19 es el numero de texto, YYYY... es la cadena de texto a mostrar
 Ejemplo: Serial.println("<Text01:A1: 253"); Mostrara el siguiente texto "A1: 253" en el texto de la app numero 1

 Para modificar el estado de las imagenes (cada imagen tiene 3 estados): <ImgsXX:Y\n, Donde XX va de 0 a 19 es el numero de imagen, Y es el estado de la imagen (0, 1 or 2)a ser mostrado
 Ejemolo: Serial.println("<Imgs02:1"); Cambiara la imagen numero 2 al estado presionado pressed state

 Para activar alarmas sonoras desde arduino: Serial.println("<Alrm00"); hara que suene una alarma en la app

 Hacer que la aplicacion hable: Texto a voz <TtoS0X:YYYY\n, donde X es 0 par idioma ingles y 1 para tu lenguage por defecto, YYYY... es cualquier cadena de texto
 Ejemplo: Serial.println("<TtoS00:Hello world"); hara que la aplicacion hable diciendo hola mundo en ingles

 Si una cadena de texto que termine en salto de linea se envia a la aplicacion, sin tener ningun tag, ese texto aparecera en la parte superior de la app.
 Ejemplo: Serial.println("Hello Word"); "Hello Word" aparecera en la parte superior de la app

 Informacion especial puede ser recivida de la aplicacion, como datos de sensor o datos de las barras deslisantes:
 * "<SkbX:SYYYYY\n", Donde X va de 0 a 7 es el numero de barra, y YYYYY es el valor de la barra de  +00000 a +00255
 * "<AccX:SYYYYY\n", Donde X puede ser "X,Y o Z" es el eje del acelerometro, y YYYYY es el valor del acelerometro em m/s^2
   multiplicados por 100, por ejemplo: 981 == 9.81m/s^2
 * El caracter "S" es el signo de la aceleracion (+ o -)

 Author: Juan Luis Gonzalez Bello
 Date: March 2015
 Get the app: https://play.google.com/store/apps/details?id=com.apps.emim.btrelaycontrol
 ** Despues de hacer copy paste, recomiendo utilizar Herramientas -> Autoformato
 */
#include <i2cmaster.h>
#include <EEPROM.h>
#include <Wire.h>
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
// Brians edits begin 
int Addr = 105;                 // I2C address of gyro
int x, y, z;
//Definition of converting raw gyro data to degrees
float SC500dps = 1.0 / 57.0; //For 500dps from Manufacturer-DataSheet
float SC250dps = 1.0 / 114.0;
float ODR = 1.0 / 200.0;
float threshhold = 75;
int NumDataPts = 100;
int zg_count = 0;
int xsum = 0;
int ysum = 0;
int zsum = 0;
int zero_rate[3] = {0, 0, 0};
int angular_rate[3] = {0, 0, 0};
int true_angle[3] = {0, 0, 0};
int gyroVal[3] = {0, 0, 0};
float tempHot = 100.0;

// Baud rate para modulo bluetooth
// (Por defecto 9600 para la mayoria de los modulos)
#define BAUD_RATE 9600

// Numero de relevadores
#define MAX_RELAYS 8

// Relevador 1 esta en pin 2, Relevaodr 2 esta en pin 3 y asi sucesivamente.
int RelayPins[MAX_RELAYS]  = {
  2, 3, 4, 5, 6, 7, 10, 12
}; //13-->12
// Relevador 1 reportara su estado a toggle button y image 1, relevaor 2 a button 2 y image 2 y asi sucesivamente.
String RelayAppId[] = {
  "00", "01", "02", "03", "04", "05", "06", "07"
};

// Lista de comandos, esto es solo para simplificar el entendimiento, puede ser cualquier comando
//#define CMD_R1_ON  'A'
//#define CMD_R1_OFF 'a'
#define CMD_R2_ON  'B'
#define CMD_R2_OFF 'b'
#define CMD_R3_ON  'C'
#define CMD_R3_OFF 'c'
#define CMD_R4_ON  'D'
#define CMD_R4_OFF 'd'
#define CMD_R5_ON  'E'
#define CMD_R5_OFF 'e'
#define CMD_R6_ON  'F'
#define CMD_R6_OFF 'f'
#define CMD_R7_ON  'G'
#define CMD_R7_OFF 'g'
#define CMD_R8_ON  'H'
#define CMD_R8_OFF 'h'

// Comandos especiales, estos tienen un valor especial en la aplicacion evitar usarlos en los data send on touch/check/untouch/uncheck
#define CMD_SPECIAL '<'
#define CMD_ALIVE   '['

// Estas variables son para mantener trackeado el estado de los relays
int RelayStatus = 0;
int STATUS_EEADR = 20;

// Esta variable es usada para hacer una especie de retardo
int Prescaler = 0;
boolean buttonLatch = false;

// Conjunto de datos para almacenar la informacion de aceleracion y de las barras despazantes
int Accel[3] = {0, 0, 0}; // 3 axis accelerometer
int SeekBarValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
  // Inicializamos el bluetooth a la velocidad deseada
  Serial.begin(BAUD_RATE);
  // Brians Edits
  i2c_init(); //Initialise the i2c bus
  PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups
  Wire.begin();
  writeI2C(CTRL_REG1, 0x1F);    // Turn on all axes, disable power down
  writeI2C(CTRL_REG3, 0x08);    // Enable control ready signal
  writeI2C(CTRL_REG4, 0x80);    // Set scale (500 deg/sec)
  delay(200);                   // Wait to synchronize
  // Zero the gyro one time
  for (int k = 0; k < 50; k++) {
    getGyroValues();
    zg_count++;
    zero_gyro(gyroVal);
  }  // end of Brian's edits for now
  
  // Inicializamos los puertos de salida
  for (int i = 0; i < MAX_RELAYS; i++) {
    pinMode(RelayPins[i], OUTPUT);
  }

  // Recuperamos el ultimo valor conocido de los relevadores leyendo la informacion de la EEPROM
  RelayStatus = EEPROM.read(STATUS_EEADR);
  for (int i = 0; i < MAX_RELAYS; i++) {
    // Dependiendo de relay status, prendemos y/o apagamos los elementos en la app
    // Ejemplo si RelayStatus = 01010001, signigica que los reles 0, 4 y 6 hay que prenderlos y los demas los apagamos
    String stringNumber = String(i);
    if ((RelayStatus & (1 << i)) == 0) {
      digitalWrite(RelayPins[i], LOW); // esto manda al pin correspondiente a cero volts
      Serial.println("<Butn" + RelayAppId[i] + ":0"); // hacemos saber a la app que ese relevador esta apagado
    }
    else {
      digitalWrite(RelayPins[i], HIGH); // esto manda al pin correspondiente a 5 volts
      Serial.println("<Butn" + RelayAppId[i] + ":1");  // hacemos saber a la app que ese relevador esta encendido
    }
  }

  // Ejemplo de como mostrar texto en la parte superior de la app
  //Serial.println("Thanks for your support!");

  // Hacemos hablar la app en ingles (Numero de lenguaje 00, usa 01 para hablar en espanol)
  Serial.println("<TtoS00: welcome to arduino total control");

}

void loop() {
  String sSample;
  String sSampleNo;
  int iSample;
  int appData;

  delay(1); //Este delay es una referencia, si queremos esparar un segundo, hay que esperar a que el ciclo se repita mil veces

  // Esto se cumple cada 1 segundo aproximadamente
  if (Prescaler++ > 200) {
    Prescaler = 0; // Reseteamos el prescaler para volver a empezar la cuenta
    // More of Brian's edits
    float temperature1 = readDevice(0x51);   // addresses from the scanner
    float temperature2 = readDevice(0x52);
    getGyroValues();
    getAngularRate(gyroVal, zero_rate, SC250dps);
    integrate4Angles(angular_rate, ODR);
    int Xx = true_angle[0];
    int Yy = true_angle[1];
    int Zz = true_angle[2];

    /*++++   Forcing variable to equal temperature   +++++++ */
    // Se envian muestras a la aplicacion
    //iSample = analogRead(A0);  // Toma la muestra analogica
    //iSample = celcius;
    sSample = String(temperature1); // Convierte el int a string
    Serial.println("<Text00:F1: " + sSample);

    sSample = String(temperature2);
    Serial.println("<Text01:F2: " + sSample);

   
    if (temperature1 >= tempHot || temperature2 >= tempHot) {
      Serial.println("<TtoS00: Warning Tire Is Hot");
    }
   

    sSample = String(Xx);
    Serial.println("<Text02:X: " + sSample);

    sSample = String(Yy);
    Serial.println("<Text03:Y: " + sSample);

    sSample = String(Zz);
    Serial.println("<Text16:Z: " + sSample);

    /*
        sSample = String(analogRead(A2));
        Serial.println("<Text02:Speed 2: " + sSample);

        sSample = String(analogRead(A3));
        Serial.println("<Text03:Photo 3: " + sSample);
        //Commenting out, temp sensor needs A4
        //sSample = String(analogRead(A4));
        //Serial.println("<Text16:Flux 4: " + sSample);
      } */
  }
  // ===========================================================
  // ESTE ES EL PUNTO donde se obtiene la informacion de la app
  appData = Serial.read();   // Obten un byte de la app, si esta disponible
  switch (appData) {
    /*
    case CMD_R1_ON:
      // Enciende relevador 1, y guarda el estado en la EEPROM, ademas manda tags de imagen y boton a app como realimentacion.
      setRelayState(0, 1);
      // Ejemplo de como hacer sonar la alarma
      Serial.println("<Alrm00");
      break;

    case CMD_R1_OFF:
      // APAGA relevador 1, y guarda el estado en la EEPROM, ademas manda tags de imagen y boton a app como realimentacion.
      setRelayState(0, 0);
      break;
    */
    case 'B':  // Esto es lo mismo que poner "case CMD_R2_ON:"
      // Lo siguiente es igual que poner setRelayState(1, 1);
      digitalWrite(RelayPins[1], HIGH);        // Enciende el puerto
      Serial.println("<Butn" + RelayAppId[1] + ":1"); // Envia realimentacion de estado de boton a app
      Serial.println("<Imgs" + RelayAppId[1] + ":1"); // Envia realimentacion de estado de imagena app
      RelayStatus |= (0x01 << 1);              // Pon el bit correspondiente en 1 logico
      EEPROM.write(STATUS_EEADR, RelayStatus); // Guarda el estado el la EEPROM
      break;

    case 'b':  // Esto es lo mismo que poner "case CMD_R2_OFF:"
      // Lo siguiente es igual que poner setRelayState(1, 0);
      digitalWrite(RelayPins[1], LOW);        // Apaga el puerto
      Serial.println("<Butn" + RelayAppId[1] + ":0"); // Envia realimentacion de estado de boton a app
      Serial.println("<Imgs" + RelayAppId[1] + ":0"); // Envia realimentacion de estado de imagena app
      RelayStatus &= ~(0x01 << 1);             // Pon el bit correspondiente en 1 logico
      EEPROM.write(STATUS_EEADR, RelayStatus); // Guarda el estado el la EEPROM
      break;

    case CMD_R3_ON:
      setRelayState(2, 1);
      break;

    case CMD_R3_OFF:
      setRelayState(2, 0);
      break;

    case CMD_R4_ON:
      setRelayState(3, 1);
      break;

    case CMD_R4_OFF:
      setRelayState(3, 0);
      break;

    case CMD_R5_ON:
      setRelayState(4, 1);
      break;

    case CMD_R5_OFF:
      setRelayState(4, 0);
      break;

    case CMD_R6_ON:
      setRelayState(5, 1);
      break;

    case CMD_R6_OFF:
      setRelayState(5, 0);
      break;

    case CMD_R7_ON:
      setRelayState(6, 1);
      break;

    case CMD_R7_OFF:
      setRelayState(6, 0);
      break;

    case CMD_R8_ON:
      setRelayState(7, 1);
      break;

    case CMD_R8_OFF:
      setRelayState(7, 0);
      break;

    case CMD_SPECIAL:
      // Se ha recivido un comando especial
      // Despues de esta funcion, Accel[] y SeekBarValues[] son actualizados
      DecodeSpecialCommand();
      break;

    case CMD_ALIVE:
      // El caracter '[' es recivido cada 2.5s,
      // Podemos usar este evento para decirle a la app que seguimos aqui
      Serial.println("Beginner's Code");

      for (int i = 0; i < MAX_RELAYS; i++) {
        // Refresca los estados de los botones e imagenes a la app(<BtnXX:Y\n)
        if (digitalRead(RelayPins[i])) {
          Serial.println("<Butn" + RelayAppId[i] + ":1");
          Serial.println("<Imgs" + RelayAppId[i] + ":1");
        }
        else {
          Serial.println("<Butn" + RelayAppId[i] + ":0");
          Serial.println("<Imgs" + RelayAppId[i] + ":0");
        }
      }
      break;
  }


  /*   A5-->A2
  // Ejemplo de como poner un boton manual
  if(!digitalRead(A2)){ // Si el boton es presionado
    // Enclavamos el motor para no cambiar el estado hasta que el boton se suelta y presiona otra vez
    if(buttonLatch){
      setRelayState(0, !digitalRead(RelayPins[0])); // Se cumple el latch, cambiamos es estado del boton leyendo su estado actual
      buttonLatch = false;
    }
  }
  else{
    // El boton se ha soltado, abilita el siguiente botonazo
    buttonLatch = true;
  } */
  // ==========================================================
}

// Sets the relay state for this example
// relay: 0 to 7 relay number
// state: 0 is off, 1 is on
void setRelayState(int relay, int state) {
  if (state == 1) {
    digitalWrite(RelayPins[relay], HIGH);           // Write ouput port
    Serial.println("<Butn" + RelayAppId[relay] + ":1"); // Feedback button state to app
    Serial.println("<Imgs" + RelayAppId[relay] + ":1"); // Set image to pressed state

    RelayStatus |= (0x01 << relay);                 // Set relay status
    EEPROM.write(STATUS_EEADR, RelayStatus);        // Save new relay status
  }
  else {
    digitalWrite(RelayPins[relay], LOW);            // Write ouput port
    Serial.println("<Butn" + RelayAppId[relay] + ":0"); // Feedback button state to app
    Serial.println("<Imgs" + RelayAppId[relay] + ":0"); // Set image to default state

    RelayStatus &= ~(0x01 << relay);                // Clear relay status
    EEPROM.write(STATUS_EEADR, RelayStatus);        // Save new relay status
  }
}

// DecodeSpecialCommand
//
// A '<' flags a special command comming from App. Use this function
// to get Accelerometer data (and other sensors in the future)
// Input:
//   None
// Output:
//   None
void DecodeSpecialCommand() {
  // Read the hole command
  String thisCommand = Readln();

  // First 5 characters will tell us the command type
  String commandType = thisCommand.substring(0, 5);

  // Next 6 characters will tell us the command data
  String commandData = thisCommand.substring(5, 11);

  if (commandType.equals("AccX:")) {
    if (commandData.charAt(0) == '-') // Negative acceleration
      Accel[0] = -commandData.substring(1, 6).toInt();
    else
      Accel[0] = commandData.substring(1, 6).toInt();
  }

  if (commandType.equals("AccY:")) {
    if (commandData.charAt(0) == '-') // Negative acceleration
      Accel[1] = -commandData.substring(1, 6).toInt();
    else
      Accel[1] = commandData.substring(1, 6).toInt();
  }

  if (commandType.equals("AccZ:")) {
    if (commandData.charAt(0) == '-') // Negative acceleration
      Accel[2] = -commandData.substring(1, 6).toInt();
    else
      Accel[2] = commandData.substring(1, 6).toInt();
  }

  if (commandType.substring(0, 3).equals("Skb")) {
    int sbNumber = commandType.charAt(3) & ~0x30;
    SeekBarValue[sbNumber] = commandData.substring(1, 6).toInt();
  }
}

// Readln
// Use this function to read a String line from Bluetooth
// returns: String message, note that this function will pause the program
//          until a hole line has been read.
String Readln() {
  char inByte = -1;
  String message = "";

  while (inByte != '\n') {
    inByte = -1;

    if (Serial.available() > 0)
      inByte = Serial.read();

    if (inByte != -1)
      message.concat(String(inByte));
  }

  return message;
}

float readDevice(int address) {
  int dev = address << 1;
  int data_low = 0;
  int data_high = 0;
  int pec = 0;

  // RAW READ
  i2c_start_wait(dev + I2C_WRITE);

  i2c_write(0x07);

  i2c_rep_start(dev + I2C_READ);

  data_low = i2c_readAck(); //Read 1 byte and then send ack
  data_high = i2c_readAck(); //Read 1 byte and then send ack
  pec = i2c_readNak();
  i2c_stop();

  //This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
  double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
  double tempData = 0x0000; // zero out the data
  int frac; // data past the decimal point

  // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
  tempData = (double)(((data_high & 0x007F) << 8) + data_low);
  tempData = (tempData * tempFactor) - 0.01;

  //Process tempData
  float celcius = tempData - 273.15;
  float objTemp = celcius * 1.8 + 32;
  //delay(200);
  return objTemp;

}//readDevice()


void getGyroValues() {
  //Direct measurement from gyroscope in "signed integer LSB"
  for (int i = 1; i < NumDataPts + 1; i++) {
    byte MSB, LSB;

    MSB = readI2C(0x29);
    LSB = readI2C(0x28);
    x = ((MSB << 8) | LSB);

    MSB = readI2C(0x2B);
    LSB = readI2C(0x2A);
    y = ((MSB << 8) | LSB);

    MSB = readI2C(0x2D);
    LSB = readI2C(0x2C);
    z = ((MSB << 8) | LSB);

    gyroVal[0] = x;
    gyroVal[1] = y;
    gyroVal[2] = z;
  }
}

int readI2C (byte regAddr) {
  //Related to gyro
  Wire.beginTransmission(Addr);
  Wire.write(regAddr);                // Register address to read
  Wire.endTransmission();             // Terminate request
  Wire.requestFrom(Addr, 1);          // Read a byte
  while (!Wire.available()) { };      // Wait for receipt
  return (Wire.read());               // Get result
}

void writeI2C (byte regAddr, byte val) {
  //Related to gyro
  Wire.beginTransmission(Addr);
  Wire.write(regAddr);
  Wire.write(val);
  Wire.endTransmission();
}


int zero_gyro(int gyroVal[3]) {
  // counter must be incremented in a loop that calls zero_gyro(x)
  int sampleSize = 50; // Lower reset angle sooner
  if (zg_count > sampleSize - 1) {
    zero_rate[0] = xsum / sampleSize; //take average of gyro data x,y,z
    zero_rate[1] = ysum / sampleSize;
    zero_rate[2] = zsum / sampleSize;
    ///////////////////////////////////////////////////////////////////
    gyroVal[0] = 0; //!!! RESET x for redundancy I noticed in output!!!
    gyroVal[1] = 0;
    gyroVal[2] = 0;
    ///////////////////////////////////////////////////////////////////
    xsum = 0; //reset xsum
    ysum = 0;
    zsum = 0;
    /*
    Serial.println("");
    Serial.print("++++++++++^^^^^^^^^   Zero Rate Level = ");
    Serial.print(zero_rate[0]);
    Serial.print("  ");
    Serial.print(zero_rate[1]);
    Serial.print("  ");
    Serial.println(zero_rate[2]);
    Serial.println("");
    */
    zg_count = 0;
    int temp[2];
    temp[0] = max(zero_rate[0], zero_rate[1]);
    temp[1] = max(zero_rate[1], zero_rate[2]);
    threshhold = max(temp[0], temp[1]);

  } else {
    xsum += x;
    ysum += y;
    zsum += z;
    /*
    Serial.print("zg_count= ");
    Serial.println(zg_count);
    Serial.print("x=");
    Serial.print(x);
    Serial.print(" xsum=");
    Serial.print(xsum);
    Serial.print("  ");
    Serial.print("y=");
    Serial.print(y);
    Serial.print(" ysum=");
    Serial.print(ysum);
    Serial.print("  ");
    Serial.print("z=");
    Serial.print(z);
    Serial.print(" zsum=");
    Serial.println(zsum);
    */
  }
}

int getAngularRate(int gyroVal[3], int zero_rate[3], int SC) {
  /* Using the maximum zero_rate value as threshhold for neglecting gyro-noise
   *  Future imrovement is to use the VARIANCE instead of the average of first
   *  100 gyrovals to calculate the zero_rate. Although the average seems to work
   *  pretty decent - not perfect though.
   */
  //for (int k=0; k < (sizeof(zero_rate)-1); k++ ) {
  // int temp[2];
  // temp[0] = max(zero_rate[0],zero_rate[1]);
  // temp[1] = max(zero_rate[1],zero_rate[2]);
  // threshhold = max(temp[0],temp[1]);
  //}
  if ( abs(gyroVal[0] - zero_rate[0]) < threshhold ) {
    //Serial.println("X Under Thresh hold");
    angular_rate[0] = 0; //units LBS
    ///////////////////////////////////////////////////////////////////
    gyroVal[0] = 0;    // !!!!! RESET x for redundancy I noticed in output !!!!!
    true_angle[0] = 0;                                              ///
    ///////////////////////////////////////////////////////////////////
  } else {
    //Serial.println("X Above threshold");
    angular_rate[0] = ( gyroVal[0] - zero_rate[0] );
    //Serial.print("Angular Rate X = ");
    //Serial.println(angular_rate[0]);
  }

  if ( abs(gyroVal[1] - zero_rate[1]) < threshhold ) {
    //Serial.println("Y Under Thresh hold");
    angular_rate[1] = 0; //units LBS
    ///////////////////////////////////////////////////////////////////
    gyroVal[1] = 0;    // !!!!! RESET x for redundancy I noticed in output !!!!!
    true_angle[1] = 0;                                              ///
    ///////////////////////////////////////////////////////////////////
  } else {
    //Serial.println("Y Above threshold");
    angular_rate[1] = ( gyroVal[1] - zero_rate[1] );
    //Serial.print("Angular Rate Y = ");
    //Serial.println(angular_rate[1]);
  }

  if ( abs(gyroVal[2] - zero_rate[2]) < threshhold ) {
    //Serial.println("Z Under Thresh hold");
    angular_rate[2] = 0; //units LBS
    ///////////////////////////////////////////////////////////////////
    gyroVal[2] = 0;    // !!!!! RESET x for redundancy I noticed in output !!!!!
    true_angle[2] = 0;                                              ///
    ///////////////////////////////////////////////////////////////////
  } else {
    //Serial.println("Z Above threshold");
    angular_rate[2] = ( gyroVal[2] - zero_rate[2] );
    //Serial.print("Angular Rate Z = ");
    //Serial.println(angular_rate[2]);
  }

}

int integrate4Angles(int angular_rate[3], float ODR) {
  for (int k = 0; k < 3; k++) {
    //Serial.print("Omega*Time= ");
    //Serial.println(angular_rate[k] * ODR);
    true_angle[k] = true_angle[k] + angular_rate[k] * ODR ;
  }

}

