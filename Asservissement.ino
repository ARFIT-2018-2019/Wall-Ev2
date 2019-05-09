

/////////////////////////////////////////////////////////////////////////////////////////////////////
boolean TranslaterDe( float distance_cm){
// Asservissement en translation.   distance_cm peut être négatif.
// On calcule à chaque interruption  100ms, 200ms (...?...)
// 1°) L'erreur de boucle i.e.: La distance à parcourir souhaitée - la distance parcourue depuis le lancement de la fonction.
// 2°) On génére la vitesse souhaitée en conséquence ( Comprise entre -127  et 127 ...)   127 = Vitesse Max.
//      A priori, un asservissement PD   avec éventuelle saturation vitesse max ou une limitation en accélération et/ou décéleration.
// 3°) On applqiue roboclaw.ForwardMixed() ou roboclaw.BackwardMixed() selon que la vitesse soit positive ou négative.
// 4°) La fonction s'arrête quand immobile (retourne TRUE) ou bien quand obstacle détecté (retourne FALSE).


float x_init = xt;
float y_init = yt;
float theta_init = thetat;
float err_dist = 0;
float err_cap = 0;

float dp=0;   // pour le calcul de la sortie de boucle après mise en place d'un filtre de consigne.
float SP;
float SP_m1=0
;

float err_dist_m1 =0.0;
float err_cap_m1  =0.0;
float err_dist_m2 =0.0;
float err_cap_m2  =0.0;
float S,D;
float S_m1  =0.0;
float D_m1 =0.0;
float S_m2  =0.0;
float D_m2 =0.0;


float Cmd_MD, Cmd_MG;

float Kp;

tip = millis();
top = tip;

int i=0;

Kp = 12/distance_cm;



do{

if(millis()-tip>10){     // toutes les 10ms

            // filtrage de la consigne
             SP = 0.97*SP_m1 + 0.03*distance_cm;
           
            dp = distance_parcourue_cm(x_init,y_init);
            
            err_dist =SP-dp;
            err_cap = theta_init-thetat;


           
            // S = 1.4647 * err_dist - 1.4470 * err_dist_m1 + 0.875 * S_m1;  // Kp=12/50  Td=0.5 tau=0.5
             
             S = Kp*(6.0167 * err_dist - 10.9833 * err_dist_m1 + 5 * err_dist_m2) + 1 * S_m1; //-0.3333 * S_m2;
                                                                      // PID: Kp=12/85  Td=0.75    Ti=0.5

            
            D = 10.5592 * err_cap -19.1114 * err_cap_m1 + 8.7267 * err_cap_m2 + 1 * D_m1 ;//- 0.8181 * D_m2;
            


            // Préparation de l'itération suivante
            err_dist_m2   = err_dist_m1;
            err_cap_m2    = err_cap_m1;
            S_m2          = S_m1;
            D_m2          = D_m1;

            
            err_dist_m1   = err_dist;
            err_cap_m1    = err_cap; 
            S_m1          = S;
            D_m1          = D;
            SP_m1         = SP;


            // Calcul et application des commandes moteurs.
            Cmd_MD = S + D;
            Cmd_MG = S - D;

            Applique_Commande(Cmd_MD,Cmd_MG);
            
            Serial.print(i);
            Serial.print('\t');
            //Serial.print(SP);
            //Serial.print('\t');
            Serial.print(thetat*180/3.1416);
            Serial.print('\t');
            //Serial.print(S);
            //Serial.print('\t');
            Serial.println(D);

            i++;
            tip=millis();
            if((tip-top)>10000){RAZ=true;}  // timeout de 4s
}  
  
  
  } while(abs(distance_cm - dp)>1.0 && !RAZ);    // tant que l'erreur est > 1cm
  Serial.println(distance_cm);
  Serial.println(dp);
Applique_Commande(0,0);          // Arrêt des moteurs
Serial.println(RAZ);
RAZ = false;
Serial.println("Done");
  }   // fin TranslaterDe()
/////////////////////////////////////////////////////////////////////////////////////////////////////




















////////////////////////////////////////////////////////////////////////////////////////
// Applique les commandes (-100% to 100%) *Tension Alimentation sur les 2 sorties PWM
// des moteurs droit et gauche.
// Les commandes sont normalisÃ©es 0-100% vers 0-255 (PWM codÃ©es sur 8bits) + Direction.
////////////////////////////////////////////////////////////////////////////////////////
void Applique_Commande(float Commande_MoteurD_enV, float Commande_MoteurG_enV)  {
  // Gamme: -100 ->  +100% de la TensionAlim
  // Commandes remappees ulterieurement : 0->255 sur les sorties analogiques.


// Traitement pour moteur droit
  int Cmd_MD = (int)(255 * (Commande_MoteurD_enV / 12.0));    // 12V = Tension Alim

  // Saturation
  if (Cmd_MD > 255) {
    Cmd_MD = 255;
  }
  if (Cmd_MD < -255) {
    Cmd_MD = -255;
  }

  // Ecriture effective de la commande
  if (Cmd_MD >= 0)
  { digitalWrite(MD_IN1, HIGH);
    digitalWrite(MD_IN2, LOW);
    analogWrite(PWM_MD, Cmd_MD);
  }
  else {
    digitalWrite(MD_IN1, LOW);
    digitalWrite(MD_IN2, HIGH);
    analogWrite(PWM_MD, -Cmd_MD);
  }

// Traitement pour moteur gauche
int Cmd_MG = (int)(255 * (Commande_MoteurG_enV / 12.0));    // 12.0 V = Tension Alim

  // Saturation
  if (Cmd_MG > 255) {
    Cmd_MG = 255;
  }
  if (Cmd_MG < -255) {
    Cmd_MG = -255;
  }

  // Ecriture effective de la commande
  if (Cmd_MG >= 0)
  { digitalWrite(MG_IN1, HIGH);
    digitalWrite(MG_IN2, LOW);
    analogWrite(PWM_MG, Cmd_MG);
  }
  else {
    digitalWrite(MG_IN1, LOW);
    digitalWrite(MG_IN2, HIGH);
    analogWrite(PWM_MG, -Cmd_MG);
  }
 
 //Serial.print("Dans Applique Commande :");Serial.print(Cmd_MD);Serial.print('\t');Serial.println(Cmd_MG);
}  // Fin Applique_Commande()
/////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////Calcul de la position du robot/////////////////////////////////////////////////////
void odometrie() {
  // Fonction executée toutes les TECOMMANDE ms sur interruption.
  // Afin de faciliter l'odométrie, veuillez mettre en commentaire la commande des moteurs dans la boucle asservissement
  
  CountEncodeurCGTempsCourant = CountEncodeurCGodo - CountEncodeurCGTempsPrecedent;
  CountEncodeurCDTempsCourant = CountEncodeurCDodo - CountEncodeurCDTempsPrecedent;
  DRG = (3.1416*DRoueEnc*CountEncodeurCGTempsCourant)/1024.0000;     // distance linéaire roue codeuse gauche
  DRD = (3.1416*DRoueEnc*CountEncodeurCDTempsCourant)/1024.0000;     // distance linéaire roue codeuse droite
  DC = (DRG+DRD)*0.5;                                                // distance linéaire du centre des mouvements. 
  xtETdt = xt + DC*cos(thetat);
  ytETdt = yt + DC*sin(thetat);
  thetatETdt = thetat + ((DRD-DRG)/EntraxeRoueEnc);

  xt=xtETdt;
  yt = ytETdt;
  thetat = thetatETdt;
  CountEncodeurCGTempsPrecedent = CountEncodeurCGodo;
  CountEncodeurCDTempsPrecedent = CountEncodeurCDodo;
}   // Fin odometrie()
/////////////////////////////////////////////////////////////////////////////////////////









//////////////////////////////////////////////////////////////////////////////////////
//Définition de 4 fonctions permettant de calculer le nombre de front montant de chaque encodeur
//////////////////////////////////////////////////////////////////////////////////////
void fmMG_B() {
  // Sur interruption sur front montant et descendant d'EncodeurB
  if (digitalRead(Enc_RMG_VA) == digitalRead(Enc_RMG_VB)) {
    CountEncodeurMG++;
  }
  else {
    CountEncodeurMG--;
  }
}
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
void fmMG_A() {
  // Sur interruption sur front montant et descendant d'EncodeurA
  if (digitalRead(Enc_RMG_VA) == digitalRead(Enc_RMG_VB)) {
    CountEncodeurMG--;
  }
  else {
    CountEncodeurMG++;
  }
}
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
void fmMD_B() {
  // Sur interruption sur front montant et descendant d'EncodeurB
  if (digitalRead(Enc_RMD_VA) == digitalRead(Enc_RMD_VB)) {
    CountEncodeurMD++;
  }
  else {
    CountEncodeurMD--;
  }
}
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
void fmMD_A() {
  // Sur interruption sur front montant et descendant d'EncodeurA
  if (digitalRead(Enc_RMD_VA) == digitalRead(Enc_RMD_VB)) {
    CountEncodeurMD--;
  }
  else {
    CountEncodeurMD++;
  }
}
//////////////////////////////////////////////////////////////////////////////////////






//////////////////////////////////////////////////////////////////////////////////////
void fmCG_A() {
  // Sur interruption sur front montant et descendant d'EncodeurA
  if (digitalRead(Enc_RCG_VA) == digitalRead(Enc_RCG_VB)) {
    CountEncodeurCG++;
    CountEncodeurCGodo++;
  }
  else {
    CountEncodeurCG--;
    CountEncodeurCGodo--;
  }
}
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
void fmCD_A() {
  // Sur interruption sur front montant et descendant d'EncodeurA
  if (digitalRead(Enc_RCD_VA) == digitalRead(Enc_RCD_VB)) {
    CountEncodeurCD++;
    CountEncodeurCDodo++;
  }
  else {
    CountEncodeurCD--;
    CountEncodeurCDodo--;
  }
}
//////////////////////////////////////////////////////////////////////////////////////
