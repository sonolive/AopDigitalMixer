

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h> 
#include "EEPROM.h"
#include <Wire.h>


  int cat ; // catégorie : shape / color ...
  String catstring [2] = { "Drive / Shape" , "Recovery / Color" } ;
  int eepadd ; // adresse eeprom
  int ii ;
  int jj ;
  int cc ;
  int dd ;

// ------------------------ Control Values biquad
  float Fs = 48000 ;
  float gain = 0 ; // gain du signal

//--------------------- biquad coaefs
  float b0 ;
  float b1 ;
  float b2 ;
  float a1 ;
  float a2 ;

// Définitions des adresses des paramètres DSP
//                                  { CH1 , CH2 , CH3 , CH4 , RVB , AUX1 , AUX2 , FX2 ,... , MASTER }
  unsigned int ADDR_MIX [10] =      { 691 , 690 , 689 , 688 , 760 , 000 ,  000 , 000 , 000 , 770  } ; 
  unsigned int ADDR_PANleft [10] =  { 000 , 000 , 000 , 000 , 000 ,  000 , 000 , 000 , 000 , 000  } ;
  unsigned int ADDR_PANright [10] = { 000 , 000 , 000 , 000 , 000 ,  000 , 000 , 000 , 000 , 000  } ;
  
  unsigned int ADDR_AUX1 [10] =     { 731 , 726 , 723 , 718 , 000 , 000 ,  000 , 000 , 000 , 000  } ; 
  unsigned int ADDR_AUX2 [10] =     { 730 , 725 , 720 , 717 , 000 , 000 ,  000 , 000 , 000 , 000  } ; 
  unsigned int ADDR_RVB [10] =      { 729 , 724 , 721 , 719 , 000 , 000 ,  000 , 000 , 000 , 000  } ; 
  unsigned int ADDR_FX2 [10] =      { 728 , 727 , 722 , 716 , 000 , 000 ,  000 , 000 , 000 , 000  } ; 
  
  unsigned int ADDR_EQ1_B2 [10] =   { 181 , 166 , 151 , 136 , 000 , 000 ,  000 , 000 , 000 , 000  } ; 
  unsigned int ADDR_EQ2_B2 [10] =   { 186 , 171 , 156 , 141 , 000 , 000 ,  000 , 000 , 000 , 000  } ; 
  unsigned int ADDR_EQ3_B2 [10] =   { 191 , 176 , 161 , 146 , 000 , 000 ,  000 , 000 , 000 , 000  } ; 

  unsigned int ADDR_MIX1 = 0x02BB ;
  unsigned int ADDR_MIX2 = 0x02BB ;
  unsigned int ADDR_MIX3 = 0x02BB ;
  unsigned int ADDR_MIX4 = 0x02BB ;
  unsigned int ADDR_MIXrvb = 0x02BB ;
  unsigned int ADDR_MIXfx2 = 0x02BB ;
  unsigned int ADDR_MIXmaster = 0x02BB ;
  
  unsigned int ADDR_PAN1 [2] = { 0x02C2 , 0x02C3 } ;


// ------------------------------------------ SHAPE1 : PROCESS + EQ

  int processSHP1 = 2 ; // paramandroid 10 : 0=OFF 1=ON
  float eqSHP1 = 2 ; // paramandroid 20 : 0=OFF 1=ON

// ------------------------------------------ SHAPE1 : 2 HIPASS FILTERS

  int hpSHP1 = 2 ; // HI PASS on (2) / off (1)

// --- hi pass 1
  // -- COEFS ADDRESSES
  unsigned int ADDR_SHP1_HP1_B0 = 0x0041 ;
  unsigned int ADDR_SHP1_HP1_B1 = 0x0042 ;
  unsigned int ADDR_SHP1_HP1_B2 = 0x0043 ;
  unsigned int ADDR_SHP1_HP1_A1 = 0x0044 ;
  unsigned int ADDR_SHP1_HP1_A2 = 0x0045 ; 

// --- hi pass 2
  // -- COEFS ADDRESSES
  unsigned int ADDR_SHP1_HP2_B0 = 0x0046 ;
  unsigned int ADDR_SHP1_HP2_B1 = 0x0047 ;
  unsigned int ADDR_SHP1_HP2_B2 = 0x0048 ;
  unsigned int ADDR_SHP1_HP2_A1 = 0x0049 ;
  unsigned int ADDR_SHP1_HP2_A2 = 0x004A ; 

// -- coefs hipass on
  float SHP1hp1ONb0 = 0.970455765724182 ;
  float SHP1hp1ONb1 = -1.94091153144836 ;
  float SHP1hp1ONb2 = 0.970455765724182 ;
  float SHP1hp1ONa1 = 1.98606014251709 ;
  float SHP1hp1ONa2 = -0.9861820936203 ;

// -- coefs hipass off
  float SHP1hp1OFFb0 = 1.0 ;
  float SHP1hp1OFFb1 = 0 ;
  float SHP1hp1OFFb2 = 0 ;
  float SHP1hp1OFFa1 = 0 ;
  float SHP1hp1OFFa2 = 0 ;

// ------------------------------------------ COL1 : 1 HIPASS FILTERS

  int hpCOL1 = 2 ; // HI PASS on (2) / off (1)

// --- hi pass 1
  // -- COEFS ADDRESSES
  unsigned int ADDR_COL1_HP1_B0 = 0x001E ;
  unsigned int ADDR_COL1_HP1_B1 = 0x001F ;
  unsigned int ADDR_COL1_HP1_B2 = 0x0020 ;
  unsigned int ADDR_COL1_HP1_A1 = 0x0021 ;
  unsigned int ADDR_COL1_HP1_A2 = 0x0022 ; 

// -- coefs hipass on
  float COL1hp1ONb0 = 0.970455765724182 ;
  float COL1hp1ONb1 = -1.94091153144836 ;
  float COL1hp1ONb2 = 0.970455765724182 ;
  float COL1hp1ONa1 = 1.98606014251709 ;
  float COL1hp1ONa2 = -0.9861820936203 ;

// -- coefs hipass off
  float COL1hp1OFFb0 = 1.0 ;
  float COL1hp1OFFb1 = 0 ;
  float COL1hp1OFFb2 = 0 ;
  float COL1hp1OFFa1 = 0 ;
  float COL1hp1OFFa2 = 0 ;
    

// ------------------------------------------ SHAPE1 : 3 PARAMETRIC 

// --- parametric 1
  // -- COEFS ADDRESSES
  unsigned int ADDR_SHP1_1_B2 = 0x00C4 ;
    // -- Control Values
  float f_SHP1_1_udp = 123 ;    // Android parameter 11 = 145 hz
  float Q_SHP1_1_udp = 85 ;      // Android parameter 12 = 1.42
  float boost_SHP1_1_udp = 512 ;  // Android parameter 13 = 0 db

// --- parametric 2
  // -- COEFS ADDRESSES
  unsigned int ADDR_SHP1_2_B2 = 0x0050;
    // -- Control Values
  float f_SHP1_2_udp = 350 ;   // Android parameter 14 = 1200 hz
  float Q_SHP1_2_udp = 85 ;      // Android parameter 15 = 1.42
  float boost_SHP1_2_udp = 512 ;  // Android parameter 16  = 0db

// --- parametric 3
  // -- COEFS ADDRESSES
  unsigned int ADDR_SHP1_3_B2 = 0x0055;
    // -- Control Values
  float f_SHP1_3_udp = 552 ;    // Android parameter 17  = 3000 hz
  float Q_SHP1_3_udp = 85 ;    // Android parameter 18 = 1.42
  float boost_SHP1_3_udp = 256 ;   // Android parameter 19  = -10 db    

// ------------------------------------------ COLOR1 : 3 PARAMETRIC 

// --- parametric 1
  // -- COEFS ADDRESSES
  unsigned int ADDR_COL1_1_B2 = 0x0021 ;

    // -- Control Values
  float f_COL1_1_udp = 123 ;    // Android parameter 21
  float Q_COL1_1_udp = 85 ;      // Android parameter 22
  float boost_COL1_1_udp = 512 ;  // Android parameter 23  

// --- parametric 2
  // -- COEFS ADDRESSES
  unsigned int ADDR_COL1_2_B0 = 0x002D ;
  unsigned int ADDR_COL1_2_B1 = 0x002E ;
  unsigned int ADDR_COL1_2_B2 = 0x002F ;
  unsigned int ADDR_COL1_2_A1 = 0x0030 ;
  unsigned int ADDR_COL1_2_A2 = 0x0031 ;  
    // -- Control Values
  float f_COL1_2_udp = 350 ;   // Android parameter 24
  float Q_COL1_2_udp = 85 ;    // Android parameter 25
  float boost_COL1_2_udp = 512 ;  // Android parameter 26  

// --- parametric 3
  // -- COEFS ADDRESSES
  unsigned int ADDR_COL1_3_B0 = 0x0032 ;
  unsigned int ADDR_COL1_3_B1 = 0x0033 ;
  unsigned int ADDR_COL1_3_B2 = 0x0034 ;
  unsigned int ADDR_COL1_3_A1 = 0x0035 ;
  unsigned int ADDR_COL1_3_A2 = 0x0036 ;    
    // -- Control Values
  float f_COL1_3_udp = 552 ;   // Android parameter 27
  float Q_COL1_3_udp = 85 ;      // Android parameter 28
  float boost_COL1_3_udp = 512 ;  // Android parameter 29  
  

    //const int bout1 = 25; //constante du pin bouton 1
    //const int bout2 = 26; //constante du pin bouton 2

    //boolean etatBout1 = true;
    //boolean etatBout2 = true;
    //boolean etatbouton11 ;
    //boolean etatbouton22 ;



    int udpupdateremote = 0 ;

    int pageudptr = 1; 
    int paramudptr = 1; 
    int valueudptr = 120;

    int motor = 1 ; 
    int pageandroid ;
    int paramandroid ;
    int param ;
    int channel ;
    int valueandroid ;

    //int storedsize2 = 0 ;
    //int storeddelay2 = 0 ;
    //int storedshape2 = 0 ;
    //int storedcolor2 = 0 ;
                            
// ---------------------------------------------menu

//int pageoled1 = 0; //variable de position dans menu principal
//int pageoled2 = 2; //variable de position dans menu principal

//String ligne0 [3] = {"", "-A-", "-B-"} ;
//String ligne1[8] = {"   MIX" ,"   PAN" , "  SEND", "  SIZE", " DELAY", " SHAPE"," COLOR", "EQ1"}; //tableau de titre du menu principal
//String ligne2 = "      ";//chaîne pour la ligne 2 (dépend du menu 1)
//String ligne3 = " ";
//String ligne4 = "OFF";
// ---------------------------------------------fin menu

// ----------------------------------------readback vumeter
float vol2 = 0 ;
float vol2db = 0 ;
unsigned int rb1 = 0x092A; // data a ercrire pour readback 1
//-------------------------------------------------------------
// ------------------------------------------------------------valeur & adresse de l'encodeur
unsigned int address = 0x0000; //adresse initiale
//-------------------------------------------------------------

// ---------------------------------------------encoder
int intDebounce ;

//int encoderPin1 = 12;
//int encoderPin2 = 13;

//volatile int lastEncoded = 0;
//long lastencoderValue = 0;
//int encoderValue = 1;


  float fMIX_udp [7] = { 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 } ;
  float fPAN_udp [7] = { 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 } ;
  float fAUX1_udp [7] = { 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 } ;
  float fAUX2_udp [7] = { 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 } ;
  float fRVB_udp [7] = { 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 } ;
  float fFX2_udp [7] = { 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 } ;
  float fFREQ1_udp [10] = { 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 } ;
  float fQ1_udp [10] = { 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 } ;
  float fBOOST1_udp [10] = { 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 } ;
  float fFREQ2_udp [10] = { 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 } ;
  float fQ2_udp [10] = { 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 } ;
  float fBOOST2_udp [10] = { 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 } ;
  float fFREQ3_udp [10] = { 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 } ;
  float fQ3_udp [10] = { 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 } ;
  float fBOOST3_udp [10] = { 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 , 0.5 } ; ; 

  int MIX1_udp = 512;
  int MIXrvb_udp = 512;
  int MIXfx2_udp = 512;
  int MIXmaster_udp = 512;
  float fMIX1_udp ;
  
  int PAN1_udp = 512;
  float fPAN1_udp ;
  float fPAN2_udp ;
  float fPAN3_udp ;
  float fPAN4_udp ;
  float fPANrvb_udp ;
  float fPANfx2_udp ;
  float fPAN1master_udp ;
  
  float volumetoleft = 1 ;
  float volumetoright = 1 ;

  int LEVEL1_udp = 512.0;
  float fLEVEL1_udp ;
  int SIZE1_udp = 2 ;
  int relay11; // Size1 Relai 1 du moteur 1
  int relay21; // Size1 Relai 1 du moteur 1
  int DELAY1_udp = 1 ;
  int SHAPE1_udp = 0 ;
  int COLOR1_udp = 0 ;  

//int encoderValuemix2 = 200;
//int encoderValuepan2 = 127;
//int encoderValuesend2 = 127;


  

//int fadeAmount = 4;

//-------------------------------------------------------------fin encoder
// --------------------------------------------- serveur udp
const char* ssid     = "wemos2";
const char* password = "sonolive2";

WiFiUDP UDPServer;
unsigned int UDPPort = 10023;

char packetBuffer2[16];

//------------------------------------------------- client udp
const char * udpAddresstx = "192.168.4.2";
const int udpporttx = 55000;
/*
const char * udpAddresstx2 = "192.168.4.3";
const int udpporttx2 = 55055;

WiFiUDP UDPServer;
*/
// --------------------------------------------- fin client udp

//-------------------------------------------------------------fin déclarations

void receiveudp() {

    String  readString="";
  // if there's data available, read a packet
  int packetSize2 = UDPServer.parsePacket();
  if (packetSize2) {
    // read the packet into packetBufffer
    int len = UDPServer.read(packetBuffer2, 16);

    if (len > 0) {
      packetBuffer2[len] = 0;
                 }
//Serial.println(packetSize2);
//Serial.println(len);

    for(int i = 0; i < 16; i++) { // 3 octets pour la val
      readString += packetBuffer2[i];
                                 }
 //Serial.println("-------------------------");                                 
//Serial.println(readString);

    motor = readString.substring(0,3).toInt();
    channel = readString.substring(6,8).toInt() -1 ;
    param = readString.substring(9,11).toInt();
    valueandroid = readString.substring(11,15).toInt();
    delay (1);
    Serial.println("-------------------------");
    Serial.print("channel = ");Serial.print(channel + 1 );
    Serial.print(" - param = ");Serial.print(param);
    Serial.print(" - value = ");Serial.println(valueandroid);
    
// MIX
    if ( param == 1 ) {
                              test ( valueandroid / 1023.0 , ADDR_MIX[channel] ) ;
                             }
//EQ1                             
    if ( param == 2 ) {
                              fFREQ1_udp [channel] = valueandroid;
                              parametric ( fFREQ1_udp [channel] ,
                              fQ1_udp [channel] ,
                              fBOOST1_udp [channel] ,
                              ADDR_EQ1_B2 [channel] ) ;
                             }     
    if ( param == 3 ) {
                              fQ1_udp [channel] = valueandroid ;
                              parametric ( fFREQ1_udp [channel] ,
                              fQ1_udp [channel] ,
                              fBOOST1_udp [channel] ,
                              ADDR_EQ1_B2 [channel] ) ;
                             }
    if ( param == 4 ) {
                              fBOOST1_udp [channel] = valueandroid ;
                              parametric ( fFREQ1_udp [channel] ,
                              fQ1_udp [channel] ,
                              fBOOST1_udp [channel] ,
                              ADDR_EQ1_B2 [channel] ) ;
                             } 
// EQ2
    if ( param == 5 ) {
                              fFREQ2_udp [channel] = valueandroid;
                              parametric ( fFREQ2_udp [channel] ,
                              fQ2_udp [channel] ,
                              fBOOST2_udp [channel] ,
                              ADDR_EQ2_B2 [channel] ) ;
                             }     
    if ( param == 6 ) {
                              fQ2_udp [channel] = valueandroid ;
                              parametric ( fFREQ2_udp [channel] ,
                              fQ2_udp [channel] ,
                              fBOOST2_udp [channel] ,
                              ADDR_EQ2_B2 [channel] ) ;
                             }
    if ( param == 7 ) {
                              fBOOST2_udp [channel] = valueandroid ;
                              parametric ( fFREQ2_udp [channel] ,
                              fQ2_udp [channel] ,
                              fBOOST2_udp [channel] ,
                              ADDR_EQ2_B2 [channel] ) ;
                             }   
// EQ3
    if ( param == 8 ) {
                              fFREQ3_udp [channel] = valueandroid;
                              parametric ( fFREQ3_udp [channel] ,
                              fQ3_udp [channel] ,
                              fBOOST3_udp [channel] ,
                              ADDR_EQ3_B2 [channel] ) ;
                             }     
    if ( param == 9 ) {
                              fQ3_udp [channel] = valueandroid ;
                              parametric ( fFREQ3_udp [channel] ,
                              fQ3_udp [channel] ,
                              fBOOST3_udp [channel] ,
                              ADDR_EQ3_B2 [channel] ) ;
                             }
    if ( param == 10 ) {
                              fBOOST3_udp [channel] = valueandroid ;
                              parametric ( fFREQ3_udp [channel] ,
                              fQ3_udp [channel] ,
                              fBOOST3_udp [channel] ,
                              ADDR_EQ3_B2 [channel] ) ;
                             }
    if ( param == 11 ) { //AUX1
                              test ( valueandroid / 1023.0 , ADDR_AUX1[channel] ) ;
                             } 
    if ( param == 12 ) { //AUX2
                              test ( valueandroid / 1023.0 , ADDR_AUX2[channel] ) ;
                             }                                                                  
    if ( param == 13 ) { // RVB
                              test ( valueandroid / 1023.0 , ADDR_RVB[channel] ) ;
                             }                          
}// packet size
}

void pan1 (float PAN) 
{
    volumetoleft = 1.0 - (PAN / 1023.0) ;      
    volumetoright = PAN / 1023.0;

    test ( volumetoleft , ADDR_PANleft [channel] );
    test ( volumetoright , ADDR_PANright [channel] );   
}

//--------------------------------------- PARAMETRIC
void parametric ( float freq , float fQ , float boost , unsigned int ADDR_B2 )
  {
    // CALCULATE
    // Conversion des values en 0<>1023 vers Hz, q et db
      float f01 = (freq/10) * (freq/10) / 1.046529 ;
      float Q1 = map( fQ , 0 , 1023 , 20 , 1500 ) / 100.0 ;
      float boost1 = map( boost , 0 , 1023 , -2000 , 2000 ) / 100.0 ; 
      
   Serial.print ( f01, 4 ) ; Serial.println ( " Hz" ) ; // en hz
   Serial.println ( Q1, 4 ) ;
   Serial.print ( boost1, 4 ) ; Serial.println ( " dB" ) ; // en db
   Serial.println ("--------------------------------------------------") ;

  // -------------------------------- Intermediate Values 
  double Ax = pow (10 , ( boost1 / 40 ) ) ;
  double omega = 2 * PI * f01 / Fs ;
  double sn = sin ( omega ) ;
  double cs = cos ( omega ) ;
  double alpha = sn / ( 2 * Q1 ) ;
  double a0 = 1 + ( alpha / Ax ) ;
  double gainlinear = pow ( 10 , ( gain / 20 )) / a0 ;

  b2 = ( 1 - ( alpha * Ax ) ) * gainlinear ;
  b1 = - ( 2 * cs ) * gainlinear ;
  b0 = ( 1 + ( alpha * Ax ) ) * gainlinear ;
  a2 = - ( 1 - ( alpha / Ax ) ) / a0 ;
  a1 = (2 * cs ) / a0  ;

  // TRANSMIT
  safeloadi2c ( b2 , b1 , b0 , a2 , a1 , ADDR_B2);
  }

  // ----------------------------------------- SAFELOAD I2C-------------------------
void safeloadi2c ( float sfld0 , float sfld1 , float sfld2 , float sfld3 , float sfld4 , unsigned int addparam0 )
  {
    
  int32_t fixpoint0 = (16777216.0f * sfld0);
  uint8_t bytes0[4];
  bytes0[0] = ((uint8_t*)(&fixpoint0))[3];
  bytes0[1] = ((uint8_t*)(&fixpoint0))[2];
  bytes0[2] = ((uint8_t*)(&fixpoint0))[1];
  bytes0[3] = ((uint8_t*)(&fixpoint0))[0];

  int32_t fixpoint1 = (16777216.0f * sfld1);
  uint8_t bytes1[4];
  bytes1[0] = ((uint8_t*)(&fixpoint1))[3];
  bytes1[1] = ((uint8_t*)(&fixpoint1))[2];
  bytes1[2] = ((uint8_t*)(&fixpoint1))[1];
  bytes1[3] = ((uint8_t*)(&fixpoint1))[0];

  int32_t fixpoint2 = (16777216.0f * sfld2);
  uint8_t bytes2[4];
  bytes2[0] = ((uint8_t*)(&fixpoint2))[3];
  bytes2[1] = ((uint8_t*)(&fixpoint2))[2];
  bytes2[2] = ((uint8_t*)(&fixpoint2))[1];
  bytes2[3] = ((uint8_t*)(&fixpoint2))[0];  

  int32_t fixpoint3 = (16777216.0f * sfld3);
  uint8_t bytes3[4];
  bytes3[0] = ((uint8_t*)(&fixpoint3))[3];
  bytes3[1] = ((uint8_t*)(&fixpoint3))[2];
  bytes3[2] = ((uint8_t*)(&fixpoint3))[1];
  bytes3[3] = ((uint8_t*)(&fixpoint3))[0]; 

  int32_t fixpoint4 = (16777216.0f * sfld4);
  uint8_t bytes4[4];
  bytes4[0] = ((uint8_t*)(&fixpoint4))[3];
  bytes4[1] = ((uint8_t*)(&fixpoint4))[2];
  bytes4[2] = ((uint8_t*)(&fixpoint4))[1];
  bytes4[3] = ((uint8_t*)(&fixpoint4))[0]; 

  // adresse de b2 du biquad
  uint8_t ADDR_PARAM0 [2] = {addparam0 >> 8, addparam0 & 0xff}; 
  
delay(1);
  // adresse du registre 1 du safeload
  Wire.beginTransmission(0x3B);
  Wire.write(0x60);
  Wire.write(0x00);
  // b2
  Wire.write(bytes0[0]);
  Wire.write(bytes0[1]);
  Wire.write(bytes0[2]);
  Wire.write(bytes0[3]);
  Wire.endTransmission();
delay(1);
  
  Wire.beginTransmission(0x3B);
  // adresse du registre 2 du safeload
  Wire.write(0x60);
  Wire.write(0x01);
  // b1
  Wire.write(bytes1[0]);
  Wire.write(bytes1[1]);
  Wire.write(bytes1[2]);
  Wire.write(bytes1[3]);
  Wire.endTransmission();
delay(1);
  
  Wire.beginTransmission(0x3B);
  // adresse du registre 3 du safeload
  Wire.write(0x60);
  Wire.write(0x02);
  // b0
  Wire.write(bytes2[0]);
  Wire.write(bytes2[1]);
  Wire.write(bytes2[2]);
  Wire.write(bytes2[3]);
  Wire.endTransmission();
delay(1);
  
  Wire.beginTransmission(0x3B);
  // adresse du registre 4 du safeload
  Wire.write(0x60);
  Wire.write(0x03);
  // a1
  Wire.write(bytes3[0]);
  Wire.write(bytes3[1]);
  Wire.write(bytes3[2]);
  Wire.write(bytes3[3]);
  Wire.endTransmission();
delay(1);
  
  Wire.beginTransmission(0x3B);
  // adresse du registre 5 du safeload
  Wire.write(0x60);
  Wire.write(0x04);
  // a0
  Wire.write(bytes4[0]);
  Wire.write(bytes4[1]);
  Wire.write(bytes4[2]);
  Wire.write(bytes4[3]);
  Wire.endTransmission();
delay(1);

  Wire.beginTransmission(0x3B);
  Wire.write(0x60);
  Wire.write(0x05);
  Wire.write(0x00);
  Wire.write(0x00);
  // adresse du 1er parametre  (incrémentation automatique pour plusieurs parametres - biquad )
  Wire.write(ADDR_PARAM0[0]);
  Wire.write(ADDR_PARAM0[1]);
  Wire.endTransmission();
delay(1);

  Wire.beginTransmission(0x3B);
  Wire.write(0x60);
  Wire.write(0x06);
  // nonbre de parametres a envoyer
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x05);
  Wire.endTransmission();
delay(1);
  

}
// ----------------------------------------- END BIQUAD I2C----------------------

  // ----------------------------------------- COEFS TO I2C-------------------------
void coefstoi2c ( float bb0 , float bb1 , float bb2 , float ba1 , float ba2 , 
                        unsigned int add0 , unsigned int add1 , unsigned int add2 , unsigned int add3 , unsigned int add4 )
  {
/*
  Serial.println ( "-----------------------------" ) ;
  Serial.println ( b0 , 14 ) ;
  Serial.println ( b1 , 14 ) ;
  Serial.println ( b2 , 14 ) ;
  Serial.println ( a1 , 14 ) ;
  Serial.println ( a2 , 14 ) ;
  Serial.println ( "-----------------------------" ) ;
*/
  int32_t fixpoint0 = (int32_t)(16777216.0f * bb0);
  uint8_t bytes0[4];
  bytes0[0] = ((uint8_t*)(&fixpoint0))[3];
  bytes0[1] = ((uint8_t*)(&fixpoint0))[2];
  bytes0[2] = ((uint8_t*)(&fixpoint0))[1];
  bytes0[3] = ((uint8_t*)(&fixpoint0))[0];

  int32_t fixpoint1 = (int32_t)(16777216.0f * bb1);
  uint8_t bytes1[4];
  bytes1[0] = ((uint8_t*)(&fixpoint1))[3];
  bytes1[1] = ((uint8_t*)(&fixpoint1))[2];
  bytes1[2] = ((uint8_t*)(&fixpoint1))[1];
  bytes1[3] = ((uint8_t*)(&fixpoint1))[0];

  int32_t fixpoint2 = (int32_t)(16777216.0f * bb2);
  uint8_t bytes2[4];
  bytes2[0] = ((uint8_t*)(&fixpoint2))[3];
  bytes2[1] = ((uint8_t*)(&fixpoint2))[2];
  bytes2[2] = ((uint8_t*)(&fixpoint2))[1];
  bytes2[3] = ((uint8_t*)(&fixpoint2))[0];  

  int32_t fixpoint3 = (int32_t)(16777216.0f * ba1);
  uint8_t bytes3[4];
  bytes3[0] = ((uint8_t*)(&fixpoint3))[3];
  bytes3[1] = ((uint8_t*)(&fixpoint3))[2];
  bytes3[2] = ((uint8_t*)(&fixpoint3))[1];
  bytes3[3] = ((uint8_t*)(&fixpoint3))[0]; 

  int32_t fixpoint4 = (int32_t)(16777216.0f * ba2);
  uint8_t bytes4[4];
  bytes4[0] = ((uint8_t*)(&fixpoint4))[3];
  bytes4[1] = ((uint8_t*)(&fixpoint4))[2];
  bytes4[2] = ((uint8_t*)(&fixpoint4))[1];
  bytes4[3] = ((uint8_t*)(&fixpoint4))[0]; 

  uint8_t addr0[] = {add0>>8, add0&0xff}; // Adresse DSP du paramètre0
  uint8_t addr1[] = {add1>>8, add1&0xff}; // Adresse DSP du paramètre1
  uint8_t addr2[] = {add2>>8, add2&0xff}; // Adresse DSP du paramètre2
  uint8_t addr3[] = {add3>>8, add3&0xff}; // Adresse DSP du paramètre1
  uint8_t addr4[] = {add4>>8, add4&0xff}; // Adresse DSP du paramètre1

  
  Wire.beginTransmission(0x34);
  //write data of parameter to safeload address 0810
  Wire.write(0x08);// on écrit dans l'adresse 0815 Adresse 0 du safeload ...
  Wire.write(0x10);
  Wire.write(0x00); // ... 5 bytes pour le Parametre
  Wire.write(bytes0[0]);
  Wire.write(bytes0[1]);
  Wire.write(bytes0[2]);
  Wire.write(bytes0[3]);
  Wire.endTransmission();

  Wire.beginTransmission(0x34);
  // write address of parameter to safeload address 0816  
  Wire.write(0x08);// on écrit dans l'adresse 0816 Adresse 1 du safeload ...
  Wire.write(0x16);
  Wire.write(addr1, 2);// ... L'adresse DSP du parametre
  Wire.endTransmission();
  
  Wire.beginTransmission(0x34);
  // write data of parameter to safeload address 0811
  Wire.write(0x08);// on écrit dans l'adresse 0811 Adresse 1 du safeload ...
  Wire.write(0x11);
  Wire.write(0x00); // ... 5 bytes pour le Parametre
  Wire.write(bytes1[0]);
  Wire.write(bytes1[1]);
  Wire.write(bytes1[2]);
  Wire.write(bytes1[3]);
  Wire.endTransmission();

  Wire.beginTransmission(0x34);
  // write address of parameter to safeload address 0817  
  Wire.write(0x08);// on écrit dans l'adresse 0817 Adresse 2 du safeload ...
  Wire.write(0x17);
  Wire.write(addr2, 2);// ... L'adresse DSP du parametre
  Wire.endTransmission();
  
  Wire.beginTransmission(0x34);
  // write data of parameter to safeload address 0812
  Wire.write(0x08);// on écrit dans l'adresse 0812 Adresse 2 du safeload ...
  Wire.write(0x12);
  Wire.write(0x00); // ... 5 bytes pour le Parametre
  Wire.write(bytes2[0]);
  Wire.write(bytes2[1]);
  Wire.write(bytes2[2]);
  Wire.write(bytes2[3]);
  Wire.endTransmission();

  Wire.beginTransmission(0x34);
  // write address of parameter to safeload address 0818  
  Wire.write(0x08);// on écrit dans l'adresse 0818 Adresse 3 du safeload ...
  Wire.write(0x18);
  Wire.write(addr3, 2);// ... L'adresse DSP du parametre
  Wire.endTransmission();
  
  Wire.beginTransmission(0x34);
  // write data of parameter to safeload address 0813
  Wire.write(0x08);// on écrit dans l'adresse 0813 Adresse 3 du safeload ...
  Wire.write(0x13);
  Wire.write(0x00); // ... 5 bytes pour le Parametre
  Wire.write(bytes3[0]);
  Wire.write(bytes3[1]);
  Wire.write(bytes3[2]);
  Wire.write(bytes3[3]);
  Wire.endTransmission();

  Wire.beginTransmission(0x34);
  // write address of parameter to safeload address 0819  
  Wire.write(0x08);// on écrit dans l'adresse 0819 Adresse 4 du safeload ...
  Wire.write(0x119);
  Wire.write(addr4, 2);// ... L'adresse DSP du parametre
  Wire.endTransmission();
  
  Wire.beginTransmission(0x34);
  // write data of parameter to safeload address 0814
  Wire.write(0x08);// on écrit dans l'adresse 0814 Adresse 4 du safeload ...
  Wire.write(0x14);
  Wire.write(0x00); // ... 5 bytes pour le Parametre
  Wire.write(bytes4[0]);
  Wire.write(bytes4[1]);
  Wire.write(bytes4[2]);
  Wire.write(bytes4[3]);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x34);
  Wire.write(0x08); // Initiate safeload transfer bit, address 0x081C
  Wire.write(0x1C); 
  Wire.write(0x00); // On transmet à chaque cycle d'horloge
  Wire.write(0x3C);
  Wire.endTransmission();
}
// ----------------------------------------- END COEFS TO I2C----------------------

/*
//****************************************************************************** udp : ESP TO ANDROID ****

void transmitudp () {
//Serial.print ("Core"); Serial.println (xPortGetCoreID()); 

     //UDPServer.beginPacket(UDPServer.remoteIP(), UDPServer.remotePort());
    //Serial.println (UDPServer.remoteIP());
    //Serial.println (UDPServer.remotePort());
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    //uint8_t buffer1[10] = "Connected" ;
    //UDPServer.write( buffer1 , 10 );
    UDPServer.printf("m%03u", 1 );
    UDPServer.printf("pg%05u", pageudptr );
    UDPServer.printf("p%03u", paramudptr );
    UDPServer.printf("v%04u", valueudptr );
    //Serial.print("udpmessage = ") ; Serial.print (1) ; Serial.print (pageudptr) ; Serial.print (paramudptr) ; Serial.println (valueudptr) ;
    UDPServer.endPacket();
    delay(5); // pause de 10ms

}
*/

void updateremoteappudp () {
  
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 1 );
    UDPServer.printf("pg%05u", 1 );
    UDPServer.printf("p%03u", 1 );
    UDPServer.printf("v%04u", MIX1_udp );
    UDPServer.endPacket();
    delay(100); // pause de 10ms
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 1 );
    UDPServer.printf("pg%05u", 1 );
    UDPServer.printf("p%03u", 2 );
    UDPServer.printf("v%04u", PAN1_udp );
    UDPServer.endPacket();
    delay(100); // pause de 10ms
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 1 );
    UDPServer.printf("pg%05u", 1 );
    UDPServer.printf("p%03u", 3 );
    UDPServer.printf("v%04u", LEVEL1_udp );
    UDPServer.endPacket();
    delay(100);; // pause de 10ms
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 1 );
    UDPServer.printf("pg%05u", 2 );
    UDPServer.printf("p%03u", 6 );
    UDPServer.printf("v%04u", SIZE1_udp + 1 );
    UDPServer.endPacket();
    delay(100); // pause de 10ms
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 1 );
    UDPServer.printf("pg%05u", 2 );
    UDPServer.printf("p%03u", 7 );
    UDPServer.printf("v%04u", DELAY1_udp + 1 );
    UDPServer.endPacket();
    delay(100); // pause de 10ms
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 1 );
    UDPServer.printf("pg%05u", 3 );
    UDPServer.printf("p%03u", 4 );
    UDPServer.printf("v%04u", SHAPE1_udp + 1 );
    UDPServer.endPacket();
    delay(100); // pause de 10ms
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 1 );
    UDPServer.printf("pg%05u", 3 );
    UDPServer.printf("p%03u", 5 );
    UDPServer.printf("v%04u", COLOR1_udp + 1 );
    UDPServer.endPacket();
    delay(100); // pause de 10ms
/*    
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 2 );
    UDPServer.printf("pg%05u", 1 );
    UDPServer.printf("p%03u", 1 );
    UDPServer.printf("v%04u", encoderValuemix2 );
    UDPServer.endPacket();
    delay(300);// pause de 10ms
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 2 );
    UDPServer.printf("pg%05u", 1 );
    UDPServer.printf("p%03u", 2 );
    UDPServer.printf("v%04u", encoderValuepan2 );
    UDPServer.endPacket();
    delay(300); // pause de 10ms
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 2 );
    UDPServer.printf("pg%05u", 1 );
    UDPServer.printf("p%03u", 3 );
    UDPServer.printf("v%04u", encoderValuesend2 );
    UDPServer.endPacket();
    delay(300); // pause de 10ms
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 2 );
    UDPServer.printf("pg%05u", 2 );
    UDPServer.printf("p%03u", 6 );
    UDPServer.printf("v%04u", storedsize2 + 4 );
    UDPServer.endPacket();
    delay(300); // pause de 10ms
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 2 );
    UDPServer.printf("pg%05u", 2 );
    UDPServer.printf("p%03u", 7 );
    UDPServer.printf("v%04u", storeddelay2 + 4 );
    UDPServer.endPacket();
    delay(300); // pause de 10ms
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 2 );
    UDPServer.printf("pg%05u", 3 );
    UDPServer.printf("p%03u", 4 );
    UDPServer.printf("v%04u", storedshape2 + 6 );
    UDPServer.endPacket();
    delay(300); // pause de 10ms
    UDPServer.beginPacket( udpAddresstx , udpporttx );
    UDPServer.printf("m%03u", 2 );
    UDPServer.printf("pg%05u", 3 );
    UDPServer.printf("p%03u", 5 );
    UDPServer.printf("v%04u", storedcolor2 + 6 );
    UDPServer.endPacket();
    delay(300); // pause de 10ms
*/
    
    udpupdateremote = 0 ;

}
/*
//---------------------------------------------------------------- ENCODER
void updateEncoder(){
int MSB = digitalRead(encoderPin1); //MSB = most significant bit
int LSB = digitalRead(encoderPin2); //LSB = least significant bit
int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

switch (motor) { // en fonction du motor
  
case 1:

        switch (pageoled1) { // en fonction du menu 1
                  case 0: // si menu VOL   
                  encoderValue = MIX1_udp;               
                  break;
                  case 1 : 
                  encoderValue = PAN1_udp;
                  break;
                  case 2 : 
                  encoderValue = LEVEL1_udp;
                  break;
                  case 3: // si menu size
                   break;
                  case 4: // si menu delay
                  break;
                  case 5: // si menu shape
                  break;    
                  case 6: // si menu color
                  break;  
                           }

          //MSB = digitalRead(encoderPin1); //MSB = most significant bit
          //LSB = digitalRead(encoderPin2); //LSB = least significant bit
          //encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
          //sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
          
          if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011){
          if(encoderValue + fadeAmount <= 255) encoderValue += fadeAmount;}
          if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
          if(encoderValue + fadeAmount >= 10) encoderValue -= fadeAmount;}
          lastEncoded = encoded; //store this value for next time

        switch (pageoled1) { // en fonction du menu 1

                  case 0 : 
                  MIX1_udp = encoderValue;       
                  break;
                  case 1 : 
                  PAN1_udp = encoderValue;                  
                  break;
                  case 2 : 
                  LEVEL1_udp = encoderValue;                  
                  break;
                  case 3: // si menu size
                  break;
                  case 4: // si menu delay
                  break;    
                  case 5: // si menu shape
                  break;    
                  case 6: // si menu color
                  break;     
                          }

break; 

case 2:

        switch (pageoled2) { // en fonction du menu 1
                  case 0: // si menu VOL   
                  encoderValue = encoderValuemix2;               
                  break;
                  case 1 : 
                  encoderValue = encoderValuepan2;
                  break;
                  case 2 : 
                  encoderValue = encoderValuesend2;
                  break;
                  case 3: // si menu size
                   break;
                  case 4: // si menu delay
                  break;
                  case 5: // si menu shape
                  break;    
                  case 6: // si menu color
                  break;  
                           }

          //MSB = digitalRead(encoderPin1); //MSB = most significant bit
          //LSB = digitalRead(encoderPin2); //LSB = least significant bit
          //encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
          //sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
          
          if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011){
          if(encoderValue + fadeAmount <= 255) encoderValue += fadeAmount;}
          if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
          if(encoderValue + fadeAmount >= 10) encoderValue -= fadeAmount;}
          lastEncoded = encoded; //store this value for next time

        switch (pageoled2) { // en fonction du menu 1

                  case 0 : 
                  encoderValuemix2 = encoderValue;       
                  break;
                  case 1 : 
                  encoderValuepan2 = encoderValue;                  
                  break;
                  case 2 : 
                  encoderValuesend2 = encoderValue;                  
                  break;
                  case 3: // si menu size
                  break;
                  case 4: // si menu delay
                  break;    
                  case 5: // si menu shape
                  break;    
                  case 6: // si menu color
                  break;     
                          }

break;

}
}
// --------------------------------------------- fin encoder
*/
/*
//----------------------------------------------------- LECTURE read back from dsp + vumetre
void readbacki2c (unsigned int readback)
{
  unsigned int address2 = 0x081A;// ADRESSE DU PARAMETRE DANS LE REGISTRE  DSP
  uint8_t addr2[] = {address2>>8, address2&0xff };//{takeHibyte=08,takelowbyte=1A}
  unsigned int address3 = readback;
  uint8_t addr3[] = {address3>>8, address3&0xff };//{takeHibyte=08,takelowbyte=1A}
  Wire.beginTransmission(0x34); //transmit to device 0x34, address is specified in datasheet 
  Wire.write(addr2, 2); // addr is array type, send 2 bytes of data.. MSB(08) then LSB(1A)
  Wire.write(addr3, 2); // addr is array type, send 2 bytes of data.. MSB(08) then LSB(1A)
  Wire.endTransmission(); // send a restart, keeping the connection active
  Wire.beginTransmission(0x34); //transmit to device 0x34, address is specified in datasheet 
  Wire.write(addr2, 2); // addr is array type, send 2 bytes of data.. MSB(08) then LSB(1A)
 Wire.endTransmission(false); // send a restart, keeping the connection active
 Wire.requestFrom(0x34, 3, true);//true:send stop message, release bus > 0x34 = dsp , 4 = 4 bytes, 
  // boucle pour la lecture des 4 bytes
  while(Wire.available())    // Wire.available(), returns the number of bytes available for reading
  {
    long vol = 0;
    byte volbyte[3];
for(int i = 0; i < 3; i++)
 {
 volbyte[i] = Wire.read();
 vol = (vol << 8) | volbyte[i];
 }
float vol2 = (float) vol / 524288.0f; // x / (1 << 19)
float vol2db = 20 * log10(vol2);
  int r = 100.00 + vol2db ;
  if (r < 10 ) r = 0 ;
  //Serial.println(r );

      display.drawRect(14, 36, 100, 12);
     //display.setColor(BLACK);
      display.drawLine(14+r,36, 14+r,47);
      display.drawLine(15+r,36, 15+r,47);
      display.fillRect(14, 39, r, 6);
      display.display();

  }
  delay(2);
}
//-------------------------------------------------------FIN LECTURE read back from dsp  + vumetre     
*/
void setup() {
  Serial.begin(115200);
    Wire.begin( 5 , 4 );
    EEPROM.begin(512);
    //------------------------------------------------------------ esp32 as acces point
    //You can remove the password parameter if you want the AP to be open.
    Serial.println("Configuring access point...");
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    Serial.println("access point started");
    //------------------------------------------------------------ esp32 as udp server
    Serial.println("Configuring udp server...");
    UDPServer.begin(UDPPort);
    Serial.println("udp server started");
/*

  display.flipScreenVertically();

//------------------------------------------------------------ esp32 as access point
    display.setColor(WHITE);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0,0,"Reverb Controls");
    display.setFont(ArialMT_Plain_10);
    display.setColor(WHITE);
    display.drawString(0,16,"Config. access point");
    display.display();
    delay (500);
    display.drawString(0,32,"Acces Point IP address: ");
    display.setFont(ArialMT_Plain_16);
    display.drawString(0,48,"192.168.4.1");
    display.display();
    delay (1000);
    display.clear();
    display.setColor(WHITE);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0,0,"Reverb Controls");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0,16,"UDP server started");
    display.display();
    delay (500);
    display.drawString(0, 32, "UDP Server Port is :");  
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 48, "10023");  
    display.display();
    delay (1000);

  //-----------------------------------------------------------  Message acceuil oled
  display.clear();
  delay(100);
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setColor(WHITE);
  display.drawString(0,0,"TubeWorks");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0,16,"Reverb Controls");
  display.display();
  delay(500);
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setColor(WHITE);
  display.drawString(0,16,"USE BUTTON 1 FOR");
  display.drawString(0,32,"MAIN MENUES");
  display.display();
  delay(500);
  display.clear();
  display.drawString(0,16,"USE BUTTON 2 FOR");
  display.drawString(0,32,"SUB MENUES");
  display.display();
  delay(500);
  display.clear();
  //--------------------------------------------------------- fin Message acceuil oled

    //---------------------------------------------------- configuration des pins push buttons 
  pinMode(bout1, INPUT_PULLUP);
  pinMode(bout2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt (bout1), navigation, CHANGE); 
  attachInterrupt(digitalPinToInterrupt (bout2), navigation, CHANGE); 
  //attachInterrupt(digitalPinToInterrupt (bout1), affichage, CHANGE); 
  //attachInterrupt(digitalPinToInterrupt (bout2), affichage, CHANGE);   

  //----------------------------------------------------- fin configuration des pins push buttons 
  // ---------------------------------------------setup encoder
  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);
  //digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  //digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
  attachInterrupt( digitalPinToInterrupt (encoderPin1) , updateEncoder, CHANGE);
  //attachInterrupt(12, transmitudp , CHANGE); 
  attachInterrupt(digitalPinToInterrupt (encoderPin2) , updateEncoder, CHANGE);
  //attachInterrupt(13, transmitudp, CHANGE);
  // --------------------------------------------- fin setup encoder
  */
}
// ----------------------------------------------------------------------- fin setup
// ----------------------------------------------------------------------- loop
void loop() {

if ( udpupdateremote == 1 ) 
     {
      updateremoteappudp ();
     }

receiveudp();
//navigation();
//affichage();

/*
 //--------------------------------------------i2c scanner
 byte error , addressscan ;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(addressscan = 1; addressscan < 127; addressscan++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(addressscan);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (addressscan<16)
        Serial.print("0");
      Serial.print(addressscan,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (addressscan<16)
        Serial.print("0");
      Serial.println(addressscan,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  delay(200); 
*/

    }

    
    void test ( float testaa , unsigned int addresssladdra ) 
    {
  
  
  int32_t fixpointa = (16777216.0 * testaa );
  uint8_t bytesa[4];
  bytesa[0] = ((uint8_t*)(&fixpointa))[3];
  bytesa[1] = ((uint8_t*)(&fixpointa))[2];
  bytesa[2] = ((uint8_t*)(&fixpointa))[1];
  bytesa[3] = ((uint8_t*)(&fixpointa))[0];
  uint8_t addrsldataa[] = {addresssladdra >> 8, addresssladdra & 0xff};

  
  Wire.beginTransmission(0x3B);
 Wire.write(addrsldataa, 2);// addr is array type, send 2 bytes of data.. MSB(00) then LSB(01)
  Wire.write(bytesa[0]);
  Wire.write(bytesa[1]);
  Wire.write(bytesa[2]);
  Wire.write(bytesa[3]);
  Wire.endTransmission();


      
    }

/*
void affichage() {
  //Serial.print ("Affichage running on Core "); Serial.println (xPortGetCoreID()); 

  String mVOL1[3] = {
    "             ",
    "      ",
    "      "
  };
    String msize1[3] = {
    "   SHORT    ",
    "   BOTH    ",
    "   LONG   "
  };
   String mDELAY1[3] = {
    "   0 MS    ",
    "   15 MS   ",
    "   30 MS   "
  };
    String mshape1[5] = {
    "   DARK    ",
    " STANDARD  ",
    "   CLEAR   ",
    "  BRIGHT   ",
    "   FLAT    "
  };
    String mCOLOR1[5] = {
    "   DARK    ",
    " STANDARD  ",
    "   CLEAR   ",
    "  BRIGHT   ",
    "   FLAT    "
  }; 
      
  display.setFont(ArialMT_Plain_16);
  display.setColor(WHITE);
    display.drawString( 0 , 0 , "MOTOR" ) ;
    display.drawString( 100 , 0 , ligne0[motor]) ;



switch (motor) { // en fonction du menu 1
case 1:
  display.setFont(ArialMT_Plain_16);
  display.setColor(WHITE);
  display.drawString(64,20,ligne1[pageoled1]);
 
  switch (pageoled1) { // en fonction du menu 1
                         case 0 : 
                         ligne2 = mVOL1[0];
                         ligne2 += MIX1_udp  ;
                             delay(1); 
                         pageudptr = 1; 
                         paramudptr = 1 ;
                         valueudptr = MIX1_udp  ;
                         break;
                         
                         case 1 : 
                         ligne2 = mVOL1[0];
                         ligne2 += PAN1_udp  ;
                             delay(1); 
                         pageudptr = 1; 
                         paramudptr = 2 ;
                         valueudptr = PAN1_udp  ;
                         break;
                         
                         case 2 : 
                         ligne2 = mVOL1[0];
                         ligne2 += LEVEL1_udp  ;
                             delay(5); 
                         pageudptr = 1;      
                         paramudptr = 3  ;
                         valueudptr = LEVEL1_udp  ;
                         break;
                         
                         case 3: // si menu size
                         ligne2 = msize1[SIZE1_udp]; //titre pris dans tableau msize  
                             delay(5); 
                         pageudptr = 2;
                         paramudptr = 6 ; 
                         valueudptr = SIZE1_udp + 1 ; 
                         break;

                         case 4: // si menu DELAY
                         ligne2 = mDELAY1[DELAY1_udp]; //titre pris dans tableau msize   
                             delay(5); 
                         pageudptr = 2;  
                         paramudptr = 7;
                         valueudptr = DELAY1_udp + 1 ; 
                         break;      
      
                         case 5: // si menu shape
                         ligne2 = mshape1[SHAPE1_udp]; //titre pris dans tableau mshape
                             delay(5); 
                         pageudptr = 3;
                         paramudptr = 4;
                         valueudptr = SHAPE1_udp + 1 ; 
                         break;

                         case 6: // si menu color
                         ligne2 = mCOLOR1[COLOR1_udp]; //titre pris dans tableau mcolor
                             delay(5); 
                         pageudptr = 3;
                         paramudptr = 5;
                         valueudptr = COLOR1_udp + 1 ; 
                         break;    

                         case 7: // si menu eq1
                         ligne2 = "EQ1"; //titre pris dans tableau mcolor
                             delay(5); 
                         
                         break;   
                       }

      display.drawString(20,44,ligne2);
      //display.setFont(ArialMT_Plain_10);
     display.display();
     display.clear();
     delay(1); 

break;

case 2:

  display.setFont(ArialMT_Plain_16);
  display.setColor(WHITE);
  display.drawString(64,20,ligne1[pageoled2]);


  switch (pageoled2) { // en fonction du menu 1
                         case 0 : 
                         ligne2 = mVOL1[0];
                         ligne2 += encoderValuemix2  ;
                             delay(5); 
                         pageudptr = 1; 
                         paramudptr = 1 ;
                         valueudptr = encoderValuemix2  ;
                         break;
                         
                         case 1 : 
                         ligne2 = mVOL1[0];
                         ligne2 += encoderValuepan2  ;
                             delay(5); 
                         pageudptr = 1; 
                         paramudptr = 2 ;
                         valueudptr = encoderValuepan2  ;
                         break;
                         
                         case 2 : 
                         ligne2 = mVOL1[0];
                         ligne2 += encoderValuesend2  ;
                             delay(5); 
                         pageudptr = 1;      
                         paramudptr = 3  ;
                         valueudptr = encoderValuesend2  ;
                         break;
                         
                         case 3: // si menu size
                         ligne2 = msize1[storedsize2]; //titre pris dans tableau msize  
                             delay(5); 
                         pageudptr = 2;
                         paramudptr = 6 ; 
                         valueudptr = storedsize2 + 1 ; 
                         break;

                         case 4: // si menu DELAY
                         ligne2 = mDELAY1[storeddelay2]; //titre pris dans tableau msize 
                             delay(5);   
                         pageudptr = 2;  
                         paramudptr = 7;
                         valueudptr = storeddelay2 + 1 ; 
                         break;      
      
                         case 5: // si menu shape
                         ligne2 = mshape1[storedshape2]; //titre pris dans tableau mshape
                             delay(5); 
                         pageudptr = 3;
                         paramudptr = 4;
                         valueudptr = storedshape2 + 1 ; 
                         break;

                         case 6: // si menu color
                         ligne2 = mCOLOR1[storedcolor2]; //titre pris dans tableau mcolor
                             delay(5); 
                         pageudptr = 3;
                         paramudptr = 5;
                         valueudptr = storedcolor2 + 1 ; 
                         break;      
  }

      display.drawString(20,44,ligne2);
      //display.setFont(ArialMT_Plain_10); 
     display.display();
     display.clear();
     delay(1); 
     break;     
      }
}
*/

 /*   
//fonction de navigation dans le menu
void navigation() {
    //Serial.print (" navigation running on Core "); Serial.println (xPortGetCoreID()); 


  //création de variables d'état pour les boutons
   etatBout1 = digitalRead(bout1);
   etatBout2 = digitalRead(bout2);
 // etatbouton11;
  //etatbouton22;



switch (motor){
case 1:  
      if (intDebounce>millis()) return;
      
  if (etatBout1) {etatbouton11 = LOW;}
  else {etatbouton11 = HIGH;}
  if (etatBout2) {etatbouton22 = LOW;}
  else {etatbouton22 = HIGH;}
  //Boucle pour naviguer dans les menus
  if (etatBout1 || etatBout2) { //si l'un des deux boutons est appuyé
    

    
    if (etatbouton11) { // si bouton 1
      pageoled1 = (pageoled1 + 1) % 7; //on change le menu principal (2 positions)
    }
    if (etatbouton22) { // si bouton 2  

      switch (pageoled1) { //l'action dépend du menu 1
        case 0: //si menu volume
        break;
        case 1: //si menu volume
        break;
        case 2: //si menu volume
        break;
        case 3: //si menu size
        SIZE1_udp = (SIZE1_udp + 1) % 3; //on change le mode de size
        break;
        case 4: //si menu delay
        DELAY1_udp = (DELAY1_udp + 1) % 3; //on change le mode de size
        break;       
        case 5 : //si menu shape
        SHAPE1_udp = (SHAPE1_udp + 1) % 5; //on change le mode de shape
        break;
        case 6 : //si menu color
        COLOR1_udp = (COLOR1_udp + 1) % 5; //on change le mode de color
        break;        
                          }
                      } 

  }
    intDebounce=millis()+300;  

//Serial.println (udptransmit) ;
     break;
     
    case 2:  
  if (intDebounce>millis()) return;
    
  if (etatBout1) {etatbouton11 = LOW;}
  else {etatbouton11 = HIGH;}
  if (etatBout2) {etatbouton22 = LOW;}
  else {etatbouton22 = HIGH;}
  
  //Boucle pour naviguer dans les menus
  if (etatBout1 || etatBout2) { //si l'un des deux boutons est appuyé
    if (etatbouton11) { // si bouton 1
      pageoled2 = (pageoled2 + 1) % 7; //on change le menu principal (2 positions)
    }
    if (etatbouton22) { // si bouton 2  
 
      switch (pageoled2) { //l'action dépend du menu 1
        case 0: //si menu volume
        break;
        case 1: //si menu volume
        break;
        case 2: //si menu volume
        break;
        case 3: //si menu size
        storedsize2 = (storedsize2 + 1) % 3; //on change le mode de size
        break;
        case 4: //si menu delay
        storeddelay2 = (storeddelay2 + 1) % 3; //on change le mode de size
        break;       
        case 5 : //si menu shape
        storedshape2 = (storedshape2 + 1) % 5; //on change le mode de shape
        break;
        case 6 : //si menu color
        storedcolor2 = (storedcolor2 + 1) % 5; //on change le mode de color
        break;          
    }


    } 
  }

    intDebounce=millis()+300; 
     break;
    }


}*/

