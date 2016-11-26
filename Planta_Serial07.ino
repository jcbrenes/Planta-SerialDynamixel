/* Programa que monitorea 1 sensor SHARP de proximidad y 4 sensores de presión.
 * Cuando recibe un dato serial, revisa que tenga las tramas de un mensaje Dynamixel.
 * Si además la dirección y el tipo de mensaje es correcto, devuelve en formato Dynamixel
 * el valor del sensor que se pidió o el valor del Centro de Masa
 */

const byte idPlanta= 51;
const long velTransmision = 115200;

const int pinControl = 2;
const bool enviar = HIGH; 
const bool recibir = LOW; 
const int delayRespuesta= 20;

byte firstByte = 0;
byte secondByte = 0;

//word valorSensorP1 = 500;
//word valorSensorP2 = 0;
//word valorSensorP3 = 0;
//word valorSensorP4 = 0;
//word valorCentroMasa = 5;

// Dimensiones Planta
float X = 6000;//mm
float Y = 10000;//mm
float x1,x2,x3,x4;
float y1,y2,y3,y4;
word CDM_X, CDM_Y;

// Sensor SHARP
word voltSharp = 0;
word cmSharp = 0;
word mmSharp = 0;

// Valores Filtrados de FSR1, FSR2, FSR3 y FSR4
word fsr11, fsr12, fsr13;
word fsr21, fsr22, fsr23;
word fsr31, fsr32, fsr33;
word fsr41, fsr42, fsr43;

// Medidas Sensores FSR
word filtradofsr1;
word filtradofsr2;
word filtradofsr3;
word filtradofsr4;

// Valores FSR en mV
word fsr1V, fsr2V,fsr3V,fsr4V;

// Entradas analogicas FSR
int pinfsr1 = A0;
int pinfsr2 = A1;
int pinfsr3 = A2;
int pinfsr4 = A3;
// Entrada analogica SHARP
int pinsharp = A5;

const int mVmaxFSR= 4000;
const int anmaxFSR= 818;

void setup()
{
  // start serial port at 9600 bps:
  Serial.begin(velTransmision);
  pinMode(pinControl, OUTPUT);   // pin será salida, que es la señal de control de enviar
  digitalWrite(pinControl, recibir); //Inicializa la comunicación en recibir
  //establishContact();  // send a byte to establish contact until receiver responds
  //pinMode(13, OUTPUT); 
}

void loop()
{
  //***************************
  //Leer acá los sensores y guardarlos en las variables globales declaradas al inicio

  // Lecturas FSR
 word valorfsr1 = analogRead(pinfsr1);
 word valorfsr2 = analogRead(pinfsr2); 
 word valorfsr3 = analogRead(pinfsr3);
 word valorfsr4 = analogRead(pinfsr4);
 
  // Lecturas y conversion (cm) SHARP
  voltSharp = analogRead (pinsharp);
  cmSharp = round(pow(3027.4 /voltSharp, 1.2134));
  mmSharp = round(pow(3027.4 /voltSharp, 1.2134)*10);
 
  // Valores Analogicos Filtrados//
  // Filtrado FSR1
  fsr13 = fsr12;
  fsr12 = fsr11;
  fsr11 = valorfsr1;
  filtradofsr1 = round((fsr11 + fsr12 + fsr13) / 3);
  
    // Filtrado FSR2
  fsr23 = fsr22;
  fsr22 = fsr21;
  fsr21 = valorfsr2;
  filtradofsr2 = round( (fsr21 + fsr22 + fsr23) / 3);
  
    // Filtrado FSR3
  fsr33 = fsr32;
  fsr32 = fsr31;
  fsr31 = valorfsr3; 
  filtradofsr3 = round( (fsr31 + fsr32 + fsr33) / 3);
  
    // Filtrado FSR4
  fsr43 = fsr42;
  fsr42 = fsr41;
  fsr41 = valorfsr4;
  filtradofsr4 = round( (fsr41 + fsr42 + fsr43) / 3);
  
  // Valores en mV
  //fsr1V = map(filtradofsr1, 0, 1023, 0, 5000);
  fsr1V = map(filtradofsr1, 0, anmaxFSR, 0, mVmaxFSR);
  fsr2V = map(filtradofsr2, 0, anmaxFSR, 0, mVmaxFSR);
  fsr3V = map(filtradofsr3, 0, anmaxFSR, 0, mVmaxFSR);
  fsr4V = map(filtradofsr4, 0, anmaxFSR, 0, mVmaxFSR);

  
  // Centro de Masas
  x1 = (float)fsr1V/mVmaxFSR * X/2;
  y1 = (float)fsr1V/mVmaxFSR * -Y/2;
  x2 = (float)fsr2V/mVmaxFSR * -X/2;
  y2 = (float)fsr2V/mVmaxFSR * -Y/2;
  x3 = (float)fsr3V/mVmaxFSR * -X/2;
  y3 = (float)fsr3V/mVmaxFSR * Y/2;
  x4 = (float)fsr4V/mVmaxFSR * X/2;
  y4 = (float)fsr4V/mVmaxFSR * Y/2;
  
  CDM_X = round( min( max( x1 + x2 + x3 + x4 + (X/2), 0 ), X) );
  CDM_Y = round( min( max( y1 + y2 + y3 + y4 + (Y/2), 0 ), Y) );

  //********************** 
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    // get incoming byte:
    //inByte = Serial.read(); 
    firstByte = Serial.read();
    delay(10);
    secondByte = Serial.read();

    //Reacciona si lee en el puerto serie 2 bytes FF
    if (firstByte==0xFF && secondByte==0xFF){
      //Serial.print('Header');
      byte id=Serial.read();
      byte len=Serial.read();
      byte inst=Serial.read();
      byte address=Serial.read();
      byte datalen=Serial.read();
      byte chksum=Serial.read();


      //Si el addreess coincide con el de Planta va a dar una respuesta
      if (id==idPlanta){
        //Serial.print('ID');
        //****acá activo el buffer para enviar
        digitalWrite(pinControl, enviar);
        //inserto un delay antes de responder, para darle chance al controlador de escuchar
        delay(delayRespuesta);
        
        //Sólo responde en instrucciones de lectura específicas 
        //Lectura del sensor SHARP
        if (len==0x04 && inst==0x02 && address==0x00 && datalen==0x02){
          //Address 0: Devolver sensor SHARP
          //Serial.print('S');
          datalen= 4;
          byte valorAlto = round(mmSharp / 256);
          byte valorBajo = mmSharp % 256;  
          chksum= getCheckSum(id,datalen,0,valorBajo, valorAlto);
          char cadena[8] = {0xFF, 0xFF, id, datalen, 0x00, valorBajo, valorAlto, chksum}; 
          Serial.write(cadena,8);

        //Lectura del valor del Sensor de presión 1  
        }else if (len==0x04 && inst==0x02 && address==0x02 && datalen==0x02){
          //Address 2: Devolver Dato de 2 Sensor de presion
          //Serial.print('SR');
          datalen= 4;
          byte valorAlto = round(fsr1V / 256);
          byte valorBajo = fsr1V % 256;  
          chksum= getCheckSum(id,datalen,0,valorBajo, valorAlto);
          char cadena[8] = {0xFF, 0xFF, id, datalen, 0x00, valorBajo, valorAlto, chksum}; 
          Serial.write(cadena,8);

        //Lectura del valor del Sensor de presión 2  
        }else if (len==0x04 && inst==0x02 && address==0x04 && datalen==0x02){
          //Address 4: Devolver Dato de 1 Sensor de presion
          //Serial.print('SR');
          datalen= 4;
          byte valorAlto = round(fsr2V / 256);
          byte valorBajo = fsr2V % 256;  
          chksum= getCheckSum(id,datalen,0,valorBajo, valorAlto);
          char cadena[8] = {0xFF, 0xFF, id, datalen, 0x00, valorBajo, valorAlto, chksum}; 
          Serial.write(cadena,8);

        //Lectura del valor del Sensor de presión 3  
        }else if (len==0x04 && inst==0x02 && address==0x06 && datalen==0x02){
          //Address 6: Devolver Dato de 1 Sensor de presion
          //Serial.print('SR');
          datalen= 4;
          byte valorAlto = round(fsr3V / 256);
          byte valorBajo = fsr3V % 256;  
          chksum= getCheckSum(id,datalen,0,valorBajo, valorAlto);
          char cadena[8] = {0xFF, 0xFF, id, datalen, 0x00, valorBajo, valorAlto, chksum}; 
          Serial.write(cadena,8);

        //Lectura del valor del Sensor de presión 4  
        }else if (len==0x04 && inst==0x02 && address==0x08 && datalen==0x02){
          //Address 8: Devolver Dato de 1 Sensor de presion
          //Serial.print('SR');
          datalen= 4;
          byte valorAlto = round(fsr4V / 256);
          byte valorBajo = fsr4V % 256;  
          chksum= getCheckSum(id,datalen,0,valorBajo, valorAlto);
          char cadena[8] = {0xFF, 0xFF, id, datalen, 0x00, valorBajo, valorAlto, chksum}; 
          Serial.write(cadena,8);
                    
        //Lectura del Centro de Masa en X  
        }else if (len==0x04 && inst==0x02 && address==0x0A && datalen==0x02){
          //Address 10: Devolver Centro MASA en X
          //Serial.print('CM');
          datalen= 4;
          byte valorAlto = round(CDM_X / 256);
          byte valorBajo = CDM_X % 256;  
          chksum= getCheckSum(id,datalen,0,valorBajo, valorAlto);
          char cadena[8] = {0xFF, 0xFF, id, datalen, 0x00, valorBajo, valorAlto, chksum}; 
          Serial.write(cadena,8);

        //Lectura del Centro de Masa en Y  
        }else if (len==0x04 && inst==0x02 && address==0x0C && datalen==0x02){
          //Address 12: Devolver Centro MASA en Y
          //Serial.print('CM');
          datalen= 4;
          byte valorAlto = round(CDM_Y / 256);
          byte valorBajo = CDM_Y % 256;  
          chksum= getCheckSum(id,datalen,0,valorBajo, valorAlto);
          char cadena[8] = {0xFF, 0xFF, id, datalen, 0x00, valorBajo, valorAlto, chksum}; 
          Serial.write(cadena,8);

        //Lectura del Centro de Masa (Ambos valores - 4 bytes)  
        }else if (len==0x04 && inst==0x02 && address==0x0A && datalen==0x04){
          //Address 10: Devolver Centro MASA (4 bytes)
          //Serial.print('CM');
          datalen= 6;
          byte valorAltoX = round(CDM_X / 256);
          byte valorBajoX = CDM_X % 256; 
          byte valorAltoY = round(CDM_Y / 256);
          byte valorBajoY = CDM_Y % 256;   
          chksum= getCheckSum4B(id,datalen,0,valorBajoX, valorAltoX, valorBajoY, valorAltoY);
          char cadena[10] = {0xFF, 0xFF, id, datalen, 0x00, valorBajoX, valorAltoX, valorBajoY, valorAltoY, chksum}; 
          Serial.write(cadena,10);
          
        }else {
          //Si no coincide con los casos que tenemos, devuelve un error
          datalen= 2;
          chksum= getCheckSum(id,datalen,0x40, 0x00, 0x00);
          char cadena[8] = {0xFF, 0xFF, id, datalen, 0x40, chksum};
          Serial.write(cadena,6);

          //tal vez
          //En caso de que el checksum falle, devuelva error
          //if (chksum<>getCheckSum(id,len,inst,address,datalen){;
        }
        delay (20); // da chance de que se envíe todo
        //desactivar acá el buffer de la salida
        digitalWrite(pinControl, recibir);
      }
    }
  }
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
  }
}

byte getCheckSum(byte id, byte lenght, byte error, byte param1, byte param2){
  
  byte cksum = ~((id+lenght+error+param1+param2)%256);
  
  return cksum;
}

byte getCheckSum4B(byte id, byte lenght, byte error, byte param1, byte param2, byte param3, byte param4){
  
  byte cksum = ~((id+lenght+error+param1+param2+param3+param4)%256);
  
  return cksum;
}

