/********************************************************************************************
 * 	    	                              HYPERION                                            *
 *		                                  USTEPPER                       						          *
 ********************************************************************************************/

// CAN Receive Example
//

#include <mcp_can.h>
#include <SPI.h>
#include <UstepperS32.h>

#define STEPSPERREV 200 //Number of steps pr revolution. 200 for a 1.8deg motor, 400 for a 0.9deg motor
#define RES (STEPSPERREV *256)/360.0//calculate microstep pr. degree
#define STEPPRMM 200//full step pr. mm for the rail used in the demo
#define MMPRSTEP 1/(STEPPRMM*256)//mm pr. microstep
#define MMPRDEG MMPRSTEP*RES//mm pr. degree
#define STALLSENSITIVITY 2//sensitivity of the stall detection, between -64 and 63 - higher number is less sensitive

unsigned long rxId;
unsigned char len = 0;
unsigned char rxBuf[8];//                                                
char msgString[128]; // Array to store serial string

#define CAN0_INT 1 // Set INT to pin 1
MCP_CAN CAN0(9);   // Set CS to pin 9
UstepperS32 stepper;

int uStepper_ID = 2;

int home_rpm = 50;

bool controle_verin = 0;
int vitesse_max_moteur = 5000;

float Angle_Demande = 0;
float Angle_Demande_Moteur = 0;
float Angle_Reel = 0;

bool Home_Demande = 0;
bool Home_Fait = 0;

bool Verin_Demande = 0;
bool Verin_Fait = 0;

unsigned long Adresse_Angle_Demande = 100 + (10 * uStepper_ID) + 1; // Génération des adresses en fonction de l'ID du uStepper
unsigned long Adresse_Angle_Reel = 100 + (10 * uStepper_ID) + 2;
unsigned long Adresse_Home_Demande = 100 + (10 * uStepper_ID) + 3;
unsigned long Adresse_Home_Fait = 100 + (10 * uStepper_ID) + 4;
unsigned long Adresse_Verin_Demande = 100 + (10 * uStepper_ID) + 5;
unsigned long Adresse_Verin_Fait = 100 + (10 * uStepper_ID) + 6;

unsigned long previousMillis1 = 0;  // Variable pour stocker le temps précédent
const long interval1 = 100;
unsigned long previousMillis2 = 0;  // Variable pour stocker le temps précédent
const long interval2 = 1000;
unsigned long previousMillis3 = 0;  // Variable pour stocker le temps précédent
const long interval3 = 50;

float PrecedenteValeurAngle = 0;
float PrecedenteValeurHome = 0;
float PrecedenteValeurVerin = 0;

String Initialisation_BUS_CAN()
{
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) // init can bus : baudrate = 500k et frequence du quartz = 8MHz
  {
    CAN0.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.
    pinMode(CAN0_INT, INPUT); // Configuring pin for /INT input
    return "Initialiation du BUS CAN réussie";
  }
  else
  {
    return "Erreur d'initialisation du BUS CAN, est t'il branché ?";
  }
}

void Initialisation_Moteur()
{
  stepper.setup(NORMAL, STEPSPERREV);         // Initialisation of the uStepper S
  stepper.checkOrientation(10.0);             // Check the orientation of the motor
  stepper.setMaxAcceleration(2000);           // Acceleration max de 2000 fullsteps/s^2
  stepper.setMaxDeceleration(2000);           // Acceleration max de 2000 fullsteps/s^2
  stepper.setMaxVelocity(vitesse_max_moteur); // Vitesse max de 500 fullsteps/s
}

bool SetHome()
{
  Home_Fait = 0; // On indique que le home n'est pas fait au bus CAN*
  Serial.println("Home en cours");
  stepper.moveToEnd(CW, home_rpm, STALLSENSITIVITY); // Set the current position as home
  // stepper.moveToEnd(CW, home_rpm, STALLSENSITIVITY);         // Set the current position as home
  stepper.setup(CLOSEDLOOP, STEPSPERREV, 1, 1, 1, 16, true); // ustepperS32Instance.setup(NORMAL, 200, 10.0, 0.0, 0.0, 16, true, 0, 60);
  stepper.setControlThreshold(15);                           // Control threshold de 15 fullsteps7
  Home_Demande = 0;                                          // On remet la demande de home à 0 pour stopper la boucle
  Home_Fait = 1;                                             // On indique que le home est fait au bus CAN
  Serial.println("Home fait");
  return true;
}

float hexStringToFloat(String hexString)
{
  hexString.substring(0);
  int values[4];
  union
  {
    char c[4];
    float f;
  } u;
  for (int i = 0; i < 4; i++)
  {
    String hexValue = hexString.substring(i * 2, (i * 2) + 2);
    values[i] = strtol(hexValue.c_str(), NULL, 16);
    u.c[i] = values[i];
  }
  return u.f;
}

void ReadCan()
{
  if (!digitalRead(CAN0_INT)) // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);   // Read data: len = data length, buf = data byte(s)
    if ((rxId & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
  }
  String data = "";
  if ((rxId & 0x40000000) == 0x40000000)
  { // Determine if message is a remote request frame.
    // sprintf(msgString, " REMOTE REQUEST FRAME");
    Serial.print(msgString);
  }
  else
  {
    String hexString = "";
    for (byte i = 0; i < len; i++)
    {
      sprintf(msgString, "%.2X", rxBuf[i]);
      hexString += String(msgString);
    }
    //Serial.println("Data Read : " + hexString + " | Adresse : 0x" + String(rxId, HEX) + " (HEX) | Valeur en Float : " + String(hexStringToFloat(hexString)));
    if (rxId == Adresse_Angle_Demande)
    {
      Angle_Demande = hexStringToFloat(hexString);
    }
    else if (rxId == Adresse_Home_Demande)
    {
      Home_Demande = hexStringToFloat(hexString);
    }
    else if (rxId == Adresse_Verin_Demande)
    {
      Verin_Demande = hexStringToFloat(hexString);
    }
  }
}

union FloatToBytes
{
  float value;
  byte bytes[4];
};

bool WriteCan(unsigned long Adresse, float ValeurFloat)
{
  FloatToBytes converter;
  converter.value = ValeurFloat;
  if (CAN0.sendMsgBuf(Adresse, 0, 4, converter.bytes) == CAN_OK)
  {
    // Serial.println("Message Sent Successfully!");
    return true;
  }
  else
  {
    // Serial.println("Error Sending Message...");
    return false;
  }
}

void setup()
{
  Serial.begin(115200);                     // Initialisation du port série
  delay(100);                               // Delai pour permettre au port série de s'initialiser et de pouvoir afficher les messages
  Serial.println(Initialisation_BUS_CAN()); // Initialisation du bus CAN
  Initialisation_Moteur();
  // Home_Demande = 1;
}

void loop()
{
  ReadCan();
  Angle_Reel = stepper.encoder.getAngleMoved();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis1 >= interval1) {
    previousMillis1 = currentMillis;
    WriteCan(Adresse_Angle_Reel, Angle_Reel);
  }
  if (currentMillis - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis;
    WriteCan(Adresse_Home_Fait, Home_Fait);
  }

  if (Home_Demande)
  {
    SetHome();
  }
  else
  {
    if (currentMillis - previousMillis3 >= interval3) {
      previousMillis3 = currentMillis;
      Angle_Demande_Moteur = -Angle_Demande*(180/3.14159265359)*(10000);
    }
    //mm to angle
    //if (!stepper.getMotorState())
    //{
      //Serial.println("Angle : " + String(Angle_Demande) + " | Etat_Home : " + String(Home_Demande) + " | Etat_Verin : " + String(Verin_Demande));
      stepper.moveToAngle(Angle_Demande_Moteur); // (ratio_reducteur * STEPSPERREV * (Angle_Demande/2)
    //}
    //Serial.println("Angle: " + String(Angle_Demande));
  }

  Serial.println("Valeur demandée : " + String(Angle_Demande) + " | Angle demandé : " + String(Angle_Demande_Moteur) + " | Home_Demande : " + String(Home_Demande) + " | Verin_Demande : " + String(Verin_Demande) + " | Angle_Reel : " + String(Angle_Reel));
  delay(10);
}