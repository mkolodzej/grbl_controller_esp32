




// to do

// allow access to SD file content only if status is print from SD paused.

// plan to be able to do a "continue" when you have a break while sending CMDs or STRINGs to GRBL
// no doubt authorize movements in jog (with the nunchuk in particular) if the status is in HOLD or in DOOR? (currently only Jog and Idle statuses are allowed for nunchuk
// move the tests relating to the place remaining in the serial2 bufferwrite in the function which groups the transmissions to grbl - see several times (Serial2.availableForWrite() != 0x7F )
// display a message on the tft if the telnet or BT link to grbl is requested but does not activate
// display a message on the tft if the telnet or BT link to grbl is requested but is lost (? how to detect a lost telnet link?)
// which status to apply while executing a print via sd grbl? Which blocking to implement. How to recognize the end. See if the % given in the status line is valid
// add error code labels in language. Warning: there are more errors and a lot of holes. It may be necessary to change the system, for example by storing them in preference.
// see the modifications made by bradley to avoid certain warnings; see what he did to avoid the loss of telnet at startup following the reset
// merge telnet and Bt in a single file to reduce the number of tabs
// move the telnet character in the COM screen and say there if it has wifi or not: Wifi mode: None, STA , AP; Wifi connected/Wifi not connected
// check that messages from grbl are stored in LastgrblMsg and not in Msg.
// measure the delay between the sending of a command and the OK (via telnet it seems to be ok, to see via serial)
// check if the soft reset causes the loss of the bluetooth or telnet connection (because how to cancel a job launch from GRBL SD card otherwise); change fCancel in the actions tab
// why sometimes get "error 120" messages
// display the file name if machining via the GRBL SD card
// the keyboard move does not always work: grbl returns an error 8
// with printing via grbl SD, when you pause, the screen must be adapted (not showing the show gcode icon) and some functions too (probably Cancel and Resume)
// during printing, display the nbr of min since the beginning (do not count when it is paused.
// why can't we make a move when we are paused (the icon is present but does not seem to work)
// when the PC is connected via serial to the grbl card, it seems that the $$ commands from the pc do not pass. (while the pc does receive the responses to the ? sent by the TFT
// check if the pause causes the motor to stop (according to the grbl doc this is not the case; maybe this was added in the STM32 version)
/*
R-cnc management with touch screen and esp32 with sd card.

- si pas fait récemment, voir si une touche est enfoncée
- si une touche est enfoncée, met à jour le menu à afficher
- traite le nunchuck (lit et active des données)
- reçoit de grbl et forward au pc si nécessaire
- envoie à grbl (soit de la carte sd, soit en passthrough, soit du menu)
- affiche l'écran (si nécessaire)



Tasks
- examine if the screen is pressed, if yes, send a message to the task which processes the screen
- display the selected screen
- send and receive from grbl
*/


/*
  Program structure for a GRBL controller on ESP32
Uses a screen with touch screen and SD card
The screen is 320 X 240
Use an esp32 with usb socket
Provide for the use of a nunchuck (I2C)
Provide file reception on the SD card by wifi

On the basic screen, provide for the display of information
// Screen Info
// USB->Grbl Idle (or Run, Alarm, ... grbl status)
// So, printing status (blanco, ,SD-->Grbl xxx%, USB<-->Grbl , Pause, Cmd) = printing status
// and GRBL status (or Run, Alarm, ... grbl status)
// Last message (ex: card inserted, card removed, card error, Error: 2;
// T T = telnet connected or # = telnet not connected
// Wpos Mpos
// X xxxxxpos xxxxxpos
// Y yyyyypos yyyyypos
// Z zzzzzpos zzzzzpos
// F 100 S 10000
  */
#include "config.h"
#include "TFT_eSPI_ms/TFT_eSPI.cpp"   // setup file has to be edited for some parameters like screen device, pins
#include "language.h"
#include "draw.h"
#include "setupTxt.h"
#include "FS.h"
#include "nunchuk.h"
#include "com.h"
//#include "SD.h"
#include "SdFat.h"
#include "menu_file.h"
#include "browser.h"
#include "telnet.h"
#include "cmd.h"
#include "log.h"
#include <Preferences.h>
#include "soc/uart_reg.h"
#include "soc/uart_struct.h"
#include "touch.h"
#include "telnetgrbl.h"
#include "bt.h"


//uart_dev_t * dev = (volatile uart_dev_t *)(DR_REG_UART_BASE) ;
//
//  uart_t _uart_bus_array[3] = {
//    {(volatile uart_dev_t *)(DR_REG_UART_BASE), 0, NULL, NULL},
//    {(volatile uart_dev_t *)(DR_REG_UART1_BASE), 1, NULL, NULL},
//    {(volatile uart_dev_t *)(DR_REG_UART2_BASE), 2, NULL, NULL}
//  };


Preferences preferences ; // object from ESP32 lib used to save/get data in flash 

extern TFT_eSPI tft ;       // Invoke custom library
extern TOUCH touchscreen; 


//       tft and touchscreen variables 
uint8_t prevPage , currentPage = 0 ;
boolean waitReleased = false ;
boolean updateFullPage = true ;
boolean updatePartPage = true ;
uint8_t justPressedBtn , justReleasedBtn, longPressedBtn ,currentBtn = 0 ; // 0 = nihil, 1 à 9 = position sur l'écran du bouton (donc pas la fonction du bouton)
uint32_t beginChangeBtnMillis ;
char lastMsg[80] = { 0} ;        // last message to display
uint16_t lastMsgColor ;          // color of last message
boolean lastMsgChanged = false ;
char grblLastMessage[STR_GRBL_BUF_MAX_SIZE] ; //= "1234567890123456789012345678901234567890"         ;
boolean grblLastMessageChanged;


//         SD variables  
int8_t dirLevel ; //-1 means that card has to be (re)loaded
char fileNames[MAX_FILES][23] ; // 22 car per line + "\0"; utilisé dans la construction de la liste des fichiers

//File root ;
//File workDir ;
//File fileToRead ; // file being printed
//File workDirParents[DIR_LEVEL_MAX] ;
//File aDir[DIR_LEVEL_MAX] ;

SdFat32 sd;
SdBaseFile aDir[DIR_LEVEL_MAX] ; 
//SdBaseFile fileToRead ; // file being printed

uint16_t sdFileDirCnt = 0 ;
uint16_t firstFileToDisplay ;   // 0 = first file in the directory
uint32_t sdFileSize ;
uint32_t sdNumberOfCharSent ;


//         Commande à exécuter
char cmdName[11][17] ;     // store the names of the commands to be displayed on the button
uint8_t cmdIcons[11][1300] ;    // store the icons of the commands buttons (if any) 1300 = an icon of 100X100
boolean cmdIconExist[11];       // store a flag to say if an icon exist or not for a cmd
uint8_t cmdToSend = 0 ;   //store the cmd to be send to grbl

//         printing status
uint8_t statusPrinting = PRINTING_STOPPED ;


// grbl data
boolean newGrblStatusReceived = false ;
char machineStatus[9];           // Iddle, Run, Alarm, ...
volatile boolean waitOk = false ;

// Nunchuk data.
extern boolean nunchukOK ;  // keep flag to detect a nunchuk at startup

// type of wifi being used
uint8_t wifiType ; // can be NO_WIFI(= 0), ESP32_ACT_AS_STATION(= 1), ESP32_ACT_AS_AP(= 2)
extern uint8_t grblLink ;

// status pour telnet
boolean statusTelnetIsConnected = false ; 

// for grbl file system
extern uint8_t grblFileReadingStatus;
extern uint8_t parseGrblFilesStatus ;  // status to know if we are reading [FILES: lines from GRBL or if it is just done 

extern float runningPercent ; // contains the percentage of char being sent to GRBL from SD card on GRBL_ESP32; to check if it is valid
extern boolean runningFromGrblSd  ; // indicator saying that we are running a job from the GRBL Sd card ; is set to true when status line contains SD:

extern M_Button mButton[] ;

/***************   Prototypes of function to avoid forward references*********************************************************************/
//uint16_t fileCnt( level ) ;  // prototype
void initMenuOptions( void) ;     //prototype
//void sendGrblMove( int8_t dir , struct menustate* ms) ; 
/**************************************************************************************************/
 
void setup() {
// initialiser le serial vers USB
// initialiser le UART vers GRBL
// initialiser l'écran et le touchscreen
// initialiser spiffs (pour relire la config)
// détecter SD card (en principe rien à faire; SD.begin est appelé avec l'écran SD)
// initialiser Wifi server
// teste la présence et initialise le nunchuck
// initialiser les status (notamment affichage de l'écran)
  logBufferInit() ; // initialise the log buffer
  Serial.begin(115200); // init UART for debug and for Gcode passthrough via USB PC
  Serial.setRxBufferSize(1024);
  
  uart_dev_t * dev = (volatile uart_dev_t *)(DR_REG_UART_BASE) ;
  dev->conf1.rxfifo_full_thrhd = 1 ;  // set the number of char received on Serial to 1 before generating an interrupt (original value is 112 and is set by esp32-hal-uart.c)
                                      // this increase the number of interrupts but it allows to forward the char to Serial2 faster
  //Serial.print(" setup: rxfifo size before interrupt="); Serial.println(dev->conf1.rxfifo_full_thrhd) ;
  
    // initialise le port série vers grbl
  //Serial2.begin(115200, SERIAL_8N1, SERIAL2_RXPIN, SERIAL2_TXPIN); // initialise le port série vers grbl
  //Serial2.setRxBufferSize(1024);
  //pinMode (SERIAL2_RXPIN, INPUT_PULLUP ); // added to force a level when serial wire is not connected
  //pinMode(TFT_LED_PIN , OUTPUT) ;
  //digitalWrite(TFT_LED_PIN , HIGH) ;
  
  if (! spiffsInit() ) {   // try to load the cmd in memory when the files exist in spiffs (the names and the icons)
    fillMsg(_SPIFFS_FORMATTED , SCREEN_NORMAL_TEXT ) ;
  } else {
    if (! cmdNameInit() ) {
      fillMsg(_CMD_NOT_LOADED ) ;
    }
  }
  initButtons() ; //initialise les noms des boutons et les boutons pour chaque page.
  tftInit() ; // init screen and touchscreen, set rotation and calibrate
  
//  listSpiffsDir( "/", 0 );   // uncomment to see the SPIFFS content
  preferences.begin("savedData") ; //define the namespace for saving preferences (used for saving WIFI parameters, and z coord for change tool)
  grblLink = preferences.getChar("grblLink", GRBL_LINK_SERIAL) ; // retrieve the last used way of communication with GRBL
  dirLevel = -1 ;   // negative value means that SD card has to be uploaded

  nunchuk_init() ; 
  prevPage = _P_NULL ;     
  currentPage = _P_INFO ;
  updateFullPage = true ;
  // en principe les données pour les buttons sont initialisés automatiquement à 0
  //drawFullPage( ) ;
  initWifi() ;
  if ( (wifiType == ESP32_ACT_AS_STATION ) || (wifiType == ESP32_ACT_AS_AP ) ) {
    telnetInit() ;
  }  
  // to debug
//  grblLastMessage[0]= 0x80 ;
//  grblLastMessage[1]= 0x81 ;
//  grblLastMessage[2]= 0x82 ;
//  grblLastMessage[3]= 0x83 ;
  clearScreen() ;
  startGrblCom(grblLink, true);
  /*
  if (grblLink == GRBL_LINK_SERIAL) {
      while (Serial2.available()>0) Serial2.read() ; // clear the serial2 buffer
      toGrbl( (char) 0x18 ) ;  // send a soft reset
      delay(100);
  } else if (grblLink == GRBL_LINK_TELNET) {
      telnetGrblInit(); // this start the connection with grbl over telnet.
  }
  if (grblLink == GRBL_LINK_BT) {
      btGrblInit();  // this start the connection over Bluetooth
  }
  // Configure Grbl
   */
  if (grblLink == GRBL_LINK_SERIAL ) {
    toGrbl( (char) 0x18 ) ;  // send a soft reset at start up when in serial mode (not sure it can be done in other mode)
    delay(100); 
  }
  toGrbl("$10=3\n\r");
  //Serial2.println("$10=3");   // $10=3 is used in order to get available space in GRBL buffer in GRBL status messages; il also means we are asking GRBL to sent always MPos.
  delay(200);    // wait that all char are sent
  //while (Serial2.availableForWrite() != 0x7F ) ;                        // wait that all char are sent
   
}

//******************************** Main loop ***************************************
void loop() {
// Lit le touchscreen et détermine si une touche est enfoncée/relachée
// Si une touche est juste enfoncée/relachée, change la couleur du bord du(des boutons)
// Déternine l'action prioritaire à écuter et l'exécute (ne change pas encore l'affichage)
// Lit nunchuck et envoie les commandes éventuelles à GRBL
// Lit les char venant de GRL (et demande un réaffichage et transmet au pc)
// Lit les char venant du pc ou de la carte sd et les transmet à grbl
// Si on a reçu de nouvelles données de GRBL, active un flag pour faire un réaffichage partiel de l'écran
// réaffiche l'écran (complètement ou partiellement) et fait un reset de flags
//#if defined ( ESP32_ACT_AS_STATION ) || defined (ESP32_ACT_AS_AP)
  static char prevMachine = '?';
  if ( wifiType > 0) {   // handle the wifi if foreseen
    processWifi();
    checkTelnetConnection();
    boolean tempTelnetIsConnected = telnetIsConnected() ;
    if ( statusPrinting == PRINTING_FROM_TELNET && !tempTelnetIsConnected ){
      statusPrinting = PRINTING_STOPPED ;
      fillMsg(_TELENET_DISCONNECTED );
    }
    statusTelnetIsConnected = tempTelnetIsConnected ;
  } 
 updateBtnState();  // check touch screen and update justPressedBtn ,justReleasedBtn , longPressedBtn and beginChangeBtnMillis
 drawUpdatedBtn() ;        // update color of button if pressed/released, apply actions foreseen for buttons (e.g. change currentPage) 
 if ( waitReleased == true && justReleasedBtn ) {
  waitReleased = false ;
 } else if ( waitReleased == false) {
  executeMainActionBtn () ;  
 }

 // handle nunchuk if implemented
  if ( nunchukOK && statusPrinting == PRINTING_STOPPED && ( machineStatus[0] == 'I' || machineStatus[0] == 'J' || machineStatus[0] == '?') )  {  //read only if the GRBL status is Idle or Jog or ?? (this last is only for testing without GRBL
    handleNunchuk() ;
  }

  getFromGrblAndForward() ; // get char from serial GRBL and always decode them (check for OK, update machineStatus and positions),
                            // if statusprinting = PRINTING_FROM_USB or if telnet is active, then forward received char from GRBL to PC (via Serial)
                          
  sendToGrbl() ;           // s'il y de la place libre dans le Tx buffer, le rempli avec le fichier de SD, une CMD ou le flux du PC; envoie périodiquement "?" pour demander le statut
//  if (newGrblStatusReceived) Serial.println( "newStatus");

  if (parseGrblFilesStatus == PARSING_FILE_NAMES_DONE) {
    parseGrblFilesStatus = PARSING_FILE_NAMES_BLOCKED ; // avoid further execute
    executeGrblEndOfFileReading(); // Change the display based on the files being read 
  }

  if (newGrblStatusReceived == true) {
    if ( prevMachine != machineStatus[0] ) {  // when machinestatus changes, it happens that we have to change the statusprinting
                                                  // it is important to check that there is a change because it can be that we receive a (late) status in reply to an ? send before last change of statusprinting
        if ( ( runningFromGrblSd ) && (machineStatus[0] == 'H' ) && ( statusPrinting == PRINTING_STOPPED || statusPrinting == PRINTING_FROM_GRBL ) ){
          statusPrinting = PRINTING_FROM_GRBL_PAUSED ;
          updateFullPage = true ; // We want to update the buttons
        } else if ( ( runningFromGrblSd ) && (machineStatus[0] != 'H' ) && ( statusPrinting == PRINTING_STOPPED || statusPrinting == PRINTING_FROM_GRBL_PAUSED ) ){
          statusPrinting = PRINTING_FROM_GRBL ;
          updateFullPage = true ; // We want to update the buttons
        } else if ( ( runningFromGrblSd == false ) && ( statusPrinting == PRINTING_FROM_GRBL || statusPrinting == PRINTING_FROM_GRBL_PAUSED)){
         statusPrinting = PRINTING_STOPPED ;
         updateFullPage = true ; // We want to update the buttons 
        } else if( statusPrinting == PRINTING_FROM_SD  && machineStatus[0] == 'H' ) { // If printing from SD and GRBL is paused
          // set PRINTING_PAUSED
          statusPrinting = PRINTING_PAUSED ;
          updateFullPage = true ; // We want to get the resume button 
        } else if( statusPrinting == PRINTING_PAUSED  && machineStatus[0] != 'H' ) { // If pause et not hold
          // set PRINTING_PAUSED
          statusPrinting = PRINTING_FROM_SD ;
          updateFullPage = true ; // We want to get the resume button
        }
    }       
    if ( currentPage == _P_INFO || currentPage == _P_MOVE || currentPage == _P_SETXYZ || currentPage == _P_SETUP || currentPage == _P_TOOL 
              || currentPage == _P_OVERWRITE || currentPage == _P_COMMUNICATION) { //force a refresh if a message has been received from GRBL and we are in a info screen or in a info screen
        updatePartPage = true ;
    } 
  }
      
  newGrblStatusReceived = false ;
  if (lastMsgChanged == true && ( currentPage == _P_INFO || currentPage == _P_MOVE || currentPage == _P_SETXYZ || currentPage == _P_SETUP 
                                || currentPage == _P_TOOL || currentPage == _P_COMMUNICATION ) ) { //force a refresh if a message has been filled
    updatePartPage = true ;
  }
  
  if (  ( updateFullPage ) ) {
    drawFullPage() ; 
  } else if ( updatePartPage ) {   
    drawPartPage() ;                           // si l'écran doit être mis à jour, exécute une fonction plus limitée qui ne redessine pas les boutons
  }    
  lastMsgChanged = false ; // lastMsgChanged is used in drawPartPage; so, it can not be set on false before
  updateFullPage = false ;
  updatePartPage = false ;
  
  //if ( prevMachine != machineStatus[0] ) {
    prevMachine = machineStatus[0] ;
  //  Serial.println( machineStatus ) ;
  //}
  yield();
}


