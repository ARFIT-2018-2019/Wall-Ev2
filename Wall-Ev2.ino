///////////////////////////////////////////////////////////////////////////////////////////////
//  Programme Asservissement de la trajectoire et des mouvements du petit robot ARFIT
//  ATOMARFIT 2019  
///////////////////////////////////////////////////////////////////////////////////////////////

//
#include <math.h>


//Bibliothèque qui gère le timer des asservissements
# include <DueTimer.h>

 #include <SPI.h>
 #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// #define OLED_RESET 4
// Adafruit_SSD1306 display(128, 32, &Wire, OLED_RESET); //width*height screen size

//Bibliothèque de l'application de commande des moteurs (Création d'une PWM)

// Définition des intructions de précompilation
//# define BAUDRATE 115200     // 115200 bauds
# define TEDATA 500           // Période transmission des données (ms)



// Déclarations pour les moteurs gauche et droit.
# define PWM_MG 6
# define MG_IN1 7
# define MG_IN2 8
# define PWM_MD 9
# define MD_IN1 11
# define MD_IN2 10

// Déclarations des pins pour les encodeurs des moteurs
# define Enc_RMG_VA 22
# define Enc_RMG_VB 23
# define Enc_RMD_VA 26
# define Enc_RMD_VB 27

// Déclarations des pins pour les encodeurs des roues codeuses
# define Enc_RCG_VA 24
# define Enc_RCG_VB 25
# define Enc_RCD_VA 28
# define Enc_RCD_VB 29

// Pour le calcul de la vitesse via moyenne mobile de longueur NmoyenneMobile
# define NmoyenneMobile 10


#define tirette 49
#define interCouleur 47

#define captAvDroit A3
#define captArDroit A4
#define captAvGauche A6
#define captArGauche A5


boolean RAZ = false;


boolean etatTirette = false;
boolean etatInterCouleur = false;

//Définition des compteurs d'impulsion des encodeurs moteurs et des roues encodeuses
volatile long CountEncodeurMG = 0;
volatile long CountEncodeurMD = 0;
volatile long CountEncodeurCG = 0;
volatile long CountEncodeurCD = 0;
volatile long CountEncodeurCGodo = 0;
volatile long CountEncodeurCDodo = 0;

//Paramètre physique du robot en cm
volatile const float DRoueEnc = 6.2200;
//volatile const float DRoueMot = 7.3024;
volatile float EntraxeRoueEnc = 6.4000;
//volatile const float EntraxeRoueMot = 12.4000;

// Gestion du temps pour l'envoi des données.
unsigned long TempsCourant        = 0;
unsigned long TempsDernierEnvoi   = 0;

volatile int indiceTicksCodeurMG = 0;
volatile int indiceTicksCodeurMD = 0;
volatile int TabCountCodeurMG[NmoyenneMobile];
volatile int TabCountCodeurMD[NmoyenneMobile];
volatile int DeltaPositionCodeurMG;
volatile int DeltaPositionCodeurMD;
volatile int TabCountCodeurCD[NmoyenneMobile];
volatile int TabCountCodeurCG[NmoyenneMobile];

// Variables globales pour la mise en place de l'asservissement.
volatile float omegaref    = 0.0;   // la consigne de vitesse (rad/s)
volatile float omegaMG       = 0.0;   // La vitesse de rotation (rad/s) du moteur gauche
volatile float omegaMD       = 0.0;   // La vitesse de rotation (rad/s) du moteur droit

const float TECOMMANDE = 10.0  ;   // ms
volatile float dt = TECOMMANDE / 1000.0; // s

volatile float CommandeMG;
volatile float CommandeMD;

volatile float temps = 0.0;  // temps courant en sec

int DataSize  =  100 ;  // Nombre d'écriture à  effectuer sur le port série dès que la commande U est modifiée.
int ChangeCmd = 0 ;     // flag pour indication changement de commande.
long unsigned  i = 0 ;     // Compteur d'écriture effectuée;
double uMG, u1MG, uMD, u1MD;  // création des nouvelles variables necessaire pour la commande 
double eMG, e1MG, eMD, e1MD;
float V,R,Ktheta;    // Création de la variable consigne 

float VitesseTransRot[2]; //Vitesse en translation (cm.s-1) et en rotation (rad.s-1) du robot
float ConsigneVitesseRotMG;  //Vitesse angulaire de chaque roue du robot (en rad.s-1)
float ConsigneVitesseRotMD;
int NbImpulsionInitCD;
int NbImpulsionInitCG;
int NbImpulsionObj;
int EcartImpulsion;
int TransErreurTheta;

//odometrie
volatile float xt = 0.0000;
volatile float yt = 0.0000;
volatile float thetat = 0.0000;
volatile float xtETdt, ytETdt, thetatETdt, DRD, DRG, DC;
volatile int CountEncodeurCGTempsCourant, CountEncodeurCDTempsCourant;
volatile int CountEncodeurCGTempsPrecedent = 0;
volatile int CountEncodeurCDTempsPrecedent = 0;

////////////////////////TRANSLATIONS////////////////
float erreurTransMD = 0;
float erreurTransMG = 0;

//float GainDroit=0.2;
//float GainGauche=0.2;  //0.13

float GainTransDroit=0.03;
float GainTransGauche=0.03;

float erreurTransOrientation = 0;
float SPpulse = 0;

//avancer
float distanceA = 0;
float dprevA = 0;

float gainOrientationTransDroitA = 0.21;
float gainOrientationTransGaucheA = 0.21;

/*float gainOrientationTransDroitA = 0.3;
float gainOrientationTransGaucheA = 0.3;*/

int changementAvancer = 1;

//reculer
float distanceR = 0;
float dprevR = 0;

/*float gainOrientationTransDroitR = 0.3;
float gainOrientationTransGaucheR = 0.3;*/
//0.21
float gainOrientationTransDroitR = 0.25;
float gainOrientationTransGaucheR = 0.25;

int changementReculer = 1;

////////////////////////////ROTATIONS//////////////////
float SPpulseD = 0;
float SPpulseG = 0;

float angle = 0;
float aprev = 0;

float erreurRotMD = 0;
float erreurRotMG = 0;

//float gainRotDroit = 0.3;
//float gainRotGauche = 0.3;
//0.27
float gainRotDroit = 0.32;
float gainRotGauche = 0.32;

const int Precision = 50; //Précision en nombre d'impulsions sur l'encodeur

int changementRotation = 1;

////////////////////////////Aller A//////////////////////
float consigneOrientation = 0;
float distanceCible = 0;
int signe = 0;

float XCIBLE = 0;
float YCIBLE = 0;

int changementAllerA = 0;
char etatAllerA;
char etatReculerVers;




float pi = 3.141592654;
float c = 0;
float s = 0;
float CapObjectif = 0;


String couleur = "";

unsigned long startMillis=millis();    // Top départ  pour calcul de la durée du RUN
unsigned long tip, top;












////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(interCouleur, INPUT_PULLUP);
  pinMode(tirette, INPUT_PULLUP);

  pinMode(Enc_RCG_VA, INPUT_PULLUP);
  pinMode(Enc_RCD_VA, INPUT_PULLUP);

    //Moteurs gauche et droit
  pinMode(PWM_MG, OUTPUT);
  pinMode(MG_IN1, OUTPUT);
  pinMode(MG_IN2, OUTPUT);
  pinMode(PWM_MD, OUTPUT);
  pinMode(MD_IN1, OUTPUT);
  pinMode(MD_IN2, OUTPUT);

  
// display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
// display.display();
// delay(2000);

  // Initialisation de la  com série.
  Serial.begin(9600);
  Serial.println("Dans le setup");

  

  
 

  //Capteurs de collision
  pinMode(captAvDroit, INPUT);
  pinMode(captArDroit, INPUT);
  pinMode(captAvGauche, INPUT);
  pinMode(captArGauche, INPUT);

  // Définition d'interruptions sur fronts montants des 2 voies A&B de chaque encodeur (moteurs et roues encodeuses).

  attachInterrupt(Enc_RCG_VA, fmCG_A, RISING);
  attachInterrupt(Enc_RCD_VA, fmCD_A, RISING);


top=millis();
  
  // Paramètrage et lancement de la tâche périodique: basé sur la librairie "DueTimer.h"
  // Timer.getAvailable().attachInterrupt(asservissementMD).setFrequency(1000.0/TECOMMANDE).start();
  //Timer1.getAvailable().attachInterrupt(asservissementMG).setFrequency(1000.0/TECOMMANDE).start();
//    Timer2.getAvailable().attachInterrupt(TranslaterDe).setFrequency(1000/TECOMMANDE);
//  Timer3.getAvailable().attachInterrupt(asservissement_rotation).setFrequency(1000/TECOMMANDE);
//  Timer4.getAvailable().attachInterrupt(asservissement_translaterR).setFrequency(1000/TECOMMANDE);
 // Timer5.getAvailable().attachInterrupt(odometrie).setFrequency(1000.0/TECOMMANDE).start();
  Timer5.getAvailable().attachInterrupt(odometrie).start(10000); //  toutes les 10 ms.




//  while(digitalRead(tirette)==0)
//  {
//
//    if(digitalRead(interCouleur))
//    {
//      display.setTextSize(3);
//      display.setTextColor(WHITE);
//      display.setCursor(0, 0);
//      display.println("ORANGE");
//      display.display();
//      display.clearDisplay();
//      couleur = "ORANGE";
//      // Initialisation de l'état en fonction de la couleur.
//      xt = 48.0000;
//      yt = 28.0000;
//      thetat=3.14;
//    }
//    else
//    {
//      display.setTextSize(3);
//      display.setTextColor(WHITE);
//      display.setCursor(0, 0);
//      display.println("VIOLET");
//      display.display();
//      display.clearDisplay();
//      couleur = "VIOLET";
//      // Initialisation de l'état en fonction de la couleur.
//      xt = 48.0000;
//      yt = 273.0000;
//      thetat=3.14;
//    } 
//  } // Tant que (La tirette n'est pas tirée) --> Boucle infine
  
  startMillis = millis();    //  On prend le top debut partie au tirage tirette.

  
}
///fin setup()///////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{

  
//  while(millis()<=(startMillis+99000))
//  {

//  /*
if(millis()-top>TEDATA){
  Serial.print(xt);Serial.print('\t');Serial.print(yt);Serial.print('\t');Serial.println(thetat);
top=millis();
}
//  */   
   
//    display.setTextSize(4);
//    display.setTextColor(WHITE);
//    display.setCursor(0, 0);
//    display.println("FIN");
//    display.display();
//    display.clearDisplay();
//
//    Timer.stop();
//    Timer1.stop();
//    Timer2.stop();
//    Timer3.stop();
//    Timer4.stop();
//   Timer5.stop();
    
  
  

  //-----  Traitement des trames en réception ------------------
  while (Serial.available()) {
    char c    = Serial.read();  // Consomme un octet sur le buffer de réception et l'affecte à  c.
    float val = Serial.parseFloat();

    switch (c) {
      case 'T' :    //translation (cm)
       Serial.print("Reception msg T,  val= ");Serial.println(val);
       TranslaterDe(val);
       
      break;
      
      case 'R' :    // rotation(deg)
      break;

      case 'E' :    
      Applique_Commande(8,8);
      delay(1000);
      Applique_Commande(0,0); 
      break;
      
      
      default:;
    }
  }
  // ----Fin du traitement des réceptions rx.

//  }
  
}
/////fin void loop()//////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////Fonction gérant les servos dynamixel du robot///////////////////////////////////////////

//////////////////////////////////////////////////////////Envoie de données pour Scilab afin d'asservir le robot//////////////////////////////////
//ATTENTION si EcritureData est moifié, le script Scilab permettant l'acquisition des données doit être repris
void EcritureData(void) {
  // Ecriture des données sur le port série toutes le TEDATA millisecondes.
  if (ChangeCmd == 1) {
    TempsCourant  =  millis();

    if ( TempsCourant - TempsDernierEnvoi > TEDATA) {

      Serial.print(temps);
      Serial.print(",");
      Serial.print(CommandeMG);
      Serial.print(",");
      Serial.print(CommandeMD);
      Serial.print(",");
      Serial.print(omegaMG);
      Serial.print(",");
      Serial.print(omegaMD);
      Serial.print(",");
      Serial.print(ConsigneVitesseRotMD);
      Serial.print(",");
      Serial.print(ConsigneVitesseRotMG);
      Serial.print(";");
      Serial.print("\r"); Serial.print("\n");
      TempsDernierEnvoi = TempsCourant;
      i++;
      if (i == DataSize) {ChangeCmd = 0;
      i=0;}     
  }
 }
}
//--------------------------------------------------------------------------------------------------------------//


