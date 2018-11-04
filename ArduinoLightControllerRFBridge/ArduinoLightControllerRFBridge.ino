/*  Code Created by Keith Woodard and altered to work with MAH Wireless Light System with permission.

	Code revised to work with MAH Wireless Light System by David "DaVaR" Sargent.
	https://www.MyArduinoHome.com/
	
	MAH Arduino Light Controller RF Bridge v1

    The Arduino Light Controller RF Controller Sketch is free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    The Arduino Light Controller RF Controller Sketch is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with The Arduino Light Controller RF Controller Sketch.  If not, see
    <http://www.gnu.org/licenses/>. */
    
#include <SPI.h>
#include <RF24.h>    /* Included for RF support using nRF24L01+ */
#include <EEPROM.h>  /* EEPROM library used to read/write configuration settings to EEPROM */
#include "printf.h"

const char versionId[] = "1";

#undef DEBUG_PRINT_MSGS      

#define BAUD_RATE  57600
#define MAX_NUMBER_OF_CHANNELS  256  /* Not Tested Beyond 256 channels */

#define _RF_CS_PIN  8                /* Arduino pin connected to the nRF24L01+ Chip Select signals */
#define _RF_CE_PIN  10               /* Arduino pin connected to the nRF24L01+ Chip Enable signals */

/* State/Mode definitions. */
typedef enum 
{
  BOOT_MODE,              /* Default mode upon power up */
  RUN_MODE,               /* Operational mode - processing data from Vixen */
  CONFIG_MODE,            /* Configuration mode - setting EEPROM data */
  INTERACTIVE_MODE,       /* Interactive mode - user interacting with menu system */
  TEST_MODE_ALL_FADE,     /* Test mode - fade all channels from 0% to 100% and back to 0% */
  TEST_MODE_WALK_CHANNELS /* Test mode - turn all channels on for 2 seconds and then turn on each channel individually for 1 second */
} modes_t;

// Global Data
uint8_t data[MAX_NUMBER_OF_CHANNELS];                         /* Array to store channel data */
unsigned long startTimeMs;                                   /* The start time of the application.  Used to transition from boot mode to run mode automatically */ 
modes_t mode;                                                /* Current mode */
boolean hardwareAvailable = false;                           /* Is a nRF24L01+ device found? */

// RF24 Setup
RF24  radio(_RF_CE_PIN,_RF_CS_PIN);                                     /* RF24 Device Class */
const uint64_t rf_pipe = 0xf0f0f0f0e1LL;

// Configuration data
unsigned int interface_type;  /* Interface type to Vixen */
unsigned int rf_channel;      /* For Future Use */
unsigned int number_of_channels; /* Number of channels to expected from Vixen's interface and to write to the RF Interface */

/*
 * Function: setup
 * Description: Power-on configuration of the Arduino.  Initialize the
 * correct pin modes for Arduino pins used.  Initialize serial port
 * communications, the TLC5940, and all global data. 
 */
void setup() {
  // Start the serial port
  Serial.begin(BAUD_RATE);

  pinMode(4, OUTPUT); /* Use for operational mode LED */
  pinMode(3, OUTPUT); /* Use for Serial Data received LED */

  printf_begin();
  radio.begin();
  radio.setRetries(0,0);
  radio.setPayloadSize(32);
  radio.setDataRate(RF24_1MBPS);
  radio.powerUp();
  
  /* Read configuration data from EEPROM */
  ReadConfigData();

  /* Set startup mode and store off start time */
  mode = BOOT_MODE;
  startTimeMs = millis();
  hardwareAvailable = radio.isPVariant();
  if (hardwareAvailable == false)
  {
    Serial.write(" nRF24L01+ module not found\n Visit www.MyArduinoHome.com for more information.\n");
  }
}

/* Function: ClearOutput
 * Description: This function initializes the local data array for the 
 *  TLC5940 channels to a default state of off.  NOTE: This function
 *  does not actually send the data to the devices.
 */
void ClearOutput()
{
  for (unsigned int i=0; i < MAX_NUMBER_OF_CHANNELS; i++)
  {
    data[i] = 0;
  }
}

/* Function: WriteRFData
 * Description: Writes data to the RF interface.
 */
void WriteRFData()
{
  boolean allDataSent = false;
  uint8_t  buffer[32];
  uint8_t currentAddress = 0;
  
  radio.stopListening();
  while (allDataSent == false)
  {
    buffer[0] = 0x7e;
    buffer[1] = currentAddress;
    for (uint8_t i=0; i<30; i++)
    {
      buffer[2+i] = data[currentAddress++];
    }
    
    int idx = 0;
    bool ok = radio.write(buffer, 32); /* Write a 32-byte buffer to the RF chip */
#ifdef DEBUG_PRINT_MSGS      
    if (ok)
    {
        printf("Buffer = 0x");
        for (idx = 0; idx < 32; idx++)
        {
          printf("%02X", buffer[idx]);
        }
        printf("\n");

    } 
    else
    {
       printf("Error writing buffer: %d\n", ok);
    }
#endif

    if (currentAddress >= number_of_channels)
    {
      allDataSent = true;
    }
    else
    {
      delay(2);
    }
  }    
  radio.startListening();
}

/* Function: ReadSerialCommand
 * Description: Reads data from the Vixen interface.  Serial data
 * must be proceeded by a header of "A1".
 */
void ReadSerialCommand()
{
  uint32_t index = 0;
  
  digitalWrite(3, LOW);
  // Wait for start byte which indicates board revision
  int model = 0;
  while (model != 'A')
  {
    if (Serial.available() > 0)
    {
      model = Serial.read();
    }
  }
  
  while (Serial.available() == 0) {} 
  int revision = Serial.read();
  
  if (revision == '1')
  {
    while (index < number_of_channels)
    {
      if (Serial.available() > 0)
      {
        /* Read the next byte from the serial port.  If the index exceeds
         * the starting address for this board, store off the value into the 
         * data array.
         */
          data[index++] = Serial.read();
      } 
    }
    digitalWrite(3, HIGH);
  }
}

/* Function: loop
 * Description: Main processing loop of the application. This loop
 * executes the main state machine for the board.
 */
 void loop() {
  /* Automatically transition to run mode after 5 seconds */
  unsigned long elapsedTime = millis() - startTimeMs;

  if (hardwareAvailable == true)
  {
    switch (mode)
    {
      case BOOT_MODE:
        /* If still in BOOT_MODE, automatically transition to RUN_MODE
         * after 5 seconds if the user hasn't put the system in test mode
         */      
        if ( elapsedTime > 5000l)
        {
          mode = RUN_MODE;
        } else {
          RunBootMode();
        }
        break;
      case RUN_MODE:
        digitalWrite(4, HIGH);
        RunOperationalMode();
        break;
      case INTERACTIVE_MODE:
        RunInteractiveMode();
        break;
      case TEST_MODE_ALL_FADE:
        RunTestModeFade();
        break;
      case TEST_MODE_WALK_CHANNELS:
        RunTestModeWalkChannels();
        break;
      case CONFIG_MODE:
        RunConfigMode();
        break;
      default:
        break;
    }
  }
}

/* Function: PrintMenu
 * Description: Prints user menu via the serial port.  First,
 * reads any extraneous data that might be present on the port.
 */
void PrintMenu()
{
  while (Serial.available() > 0)
  {
    Serial.read();
  }
  
  Serial.println("My Arduino Home Arduino Light Controller RF Bridge Main Menu");
  Serial.println("www.MyArduinoHome.com");
  printf("Version: %s\n", versionId);
  Serial.println(" (t) Enter Test Mode - All channels fade");
  Serial.println(" (w) Enter Test Mode - Walk channels");
  Serial.println(" (c) Enter Configuration Mode");
  Serial.println(" (r) Enter Operational Mode");
  Serial.println(" (p) Print Configuration");
  Serial.println("  > ");
}

/* Function: RunBootMode
 * Description: If in boot mode, look for any data on the serial port.  If any data
 *    is available, clear the serial port buffer and transition to interactive
 *    mode.
 */
void RunBootMode()
{
   if (Serial.available() > 0)
   {
     Serial.read();
     mode = INTERACTIVE_MODE;
   } else {
     delay(100);
   }
}

/* Function: RunInteractiveMode
 * Description: Print menu to user, read response, and transition
 *   to new mode as required by user input.
 */
void RunInteractiveMode()
{
  radio.printDetails();
  radio.setChannel(rf_channel);
  radio.openWritingPipe(rf_pipe);
  radio.openReadingPipe(1, rf_pipe+1);
  /* Print out menu */
    PrintMenu();
    
    /* Wait indefinitely for an input */
    while (Serial.available() == 0)
    {
      delay(50);
    }
    
    /* Read serial buffer */
    char command = Serial.read();
    /* Process key press */
    switch (command)
    {
      case 'r':
      case 'R':
        mode = RUN_MODE;
        Serial.println("Transitioning to operational mode");
        break;
      case 'c':
      case 'C':
        mode = CONFIG_MODE;
        Serial.println("Transitioning to configuration mode");
        break;
      case 't':
      case 'T':
        mode = TEST_MODE_ALL_FADE;
        Serial.println("Running test pattern (fade) on all channels.");
        Serial.println("Press any key to exit test pattern and return to menu.");
        break;
      case 'w':
      case 'W':
        mode = TEST_MODE_WALK_CHANNELS;
        Serial.println("Running test pattern (walk channels) on all channels.");
        Serial.println("Press any key to exit test pattern and return to menu.");
        break;
      case 'p':
      case 'P':
        PrintConfiguration();
        break; 
      default:
        Serial.println("Invalid key pressed.");
        Serial.println("");
        break;
    }
    
    /* Remove any other data from serial buffer */
    while (Serial.available() > 0)
    {
      Serial.read();
    }
}


/* Function: RunOperationalMode
 * Description: Run the operational mode and read data from the
 *   Vixen interface and send it out the RF interface.
 */
void RunOperationalMode()
{
  radio.setChannel(rf_channel);
  radio.openWritingPipe(rf_pipe);
  radio.openReadingPipe(1, rf_pipe+1);
  
  while (1)
  {
    ReadSerialCommand();
    WriteRFData();
  } /* Loop forever */
}

/* Function: RunTestModeWalkChannels
 * Description: Run the walk channels test mode.  In this test mode, 
 *   turn on all channels for 2 seconds and then walk through each channel,
 *   leaving each channel on for 1 second.  Repeat continuously.  Continue
 *   until the user presses a key.
 */
void RunTestModeWalkChannels()
{
  static boolean allOn = true;
  static uint32_t channelNumber = 0;
  static unsigned long lastTime = millis();
  
  /* Run Test Mode until user presses another key, then return to interactive mode */  
   if (Serial.available() > 0)
   {
     Serial.read();
     mode = INTERACTIVE_MODE;
   } else {
     
     if (allOn == true)
     {
       /* Set all the channels on */
       for (uint32_t channel = 0; channel < number_of_channels; channel++)
       {
         data[channel] = 255;
       }   
       
       /* After the channels have been on for two seconds, turn off all the channels
        * and transition to walking through the channels.
        */
       if ( (millis() - lastTime) > 2000l)
       {
         /* Turn off all channels */
         for (uint32_t channel = 0; channel < number_of_channels; channel++)
         {
           data[channel] = 0;
         }  
         data[0] = 255;
         lastTime = millis();       
         allOn = false;
       }
     } else {
       if ( (millis() - lastTime) > 1000ul)
       {
         lastTime = millis();
         /* Turn off the channel that is already on */
         data[channelNumber] = 0;
         /* Then, move to the next channel */
         channelNumber++;
         /* If reached the last channel, switch back to "all on" 
          * and begin repeating the loop */
         if (channelNumber == number_of_channels)
         {
           channelNumber = 0;
           allOn = true;
         } else {
           data[channelNumber] = 255;
         }
       }      
     }

     /* Send new data to the RF transmitter */
      WriteRFData();
      delay(100);
  }
}

/* Function: RunTestModeFade
 * Description: Run the fade channels test mode.  In this test mode, 
 *   continuously fade the channels from 0% to 100% and back to 0%.  Continue
 *   until user presses a key.
 */
void RunTestModeFade()
{
   static int direction = 1;
   static int value = 0;
   
   /* Run Test Mode until user presses another key, then return to interactive mode */  
   if (Serial.available() > 0)
   {
     Serial.read();
     mode = INTERACTIVE_MODE;
   } else {

    value += (1*direction);

    if (value > 255)
    {
      value = 255;
      direction = -1;
    }
    if (value < 0)
    {  
      value = 0;
      direction = 1;
    }
 
    for (uint32_t channel = 0; channel < number_of_channels; channel++)
    {
      data[channel] = value;
    } 
     WriteRFData();
     delay(50);
  }
}

/* Function: ReadNumberFromSerial
 * Description: Read an input from the serial port and interpret it as a number.
 */
int ReadNumberFromSerial()
{
  int returnValue = -1;
  
  while (Serial.available() == 0)
  {
    delay (50);
  }
  char value[8];
  int status = Serial.readBytesUntil('\n',value, 8);
  if (status > 0)
  {
    returnValue = atoi(value); 
  } else {
    returnValue = status;
  }
  return returnValue;
}

/* Function: RunConfigMode
 * Description: Run the configuration mode to all user configurable
 *   parameters to be saved to EEPROM.
 */
void RunConfigMode()
{
  /* local variables for new values prior to writing to EEPROM */
  int newNumberOfChannels;
  int newRFChannel;
  int newValue;
  
  Serial.println("Arduino Light Controller RF Bridge configuration:");

  /* Ask user for RF Channel setting */  
  Serial.print("  RF Channel ("); Serial.print(rf_channel); Serial.print("): ");
  newValue = ReadNumberFromSerial();
  Serial.println("");
  if ((newValue >= 2) && (newValue <= 80))
  {
    newRFChannel = newValue;
  } else {
    Serial.print("Invalid setting for RF Channel: "); Serial.print(newValue); Serial.println("");
    newRFChannel = rf_channel;
  }  

  /* Ask user for RF Channel setting */  
  Serial.print("  Number Of Channels ("); Serial.print(number_of_channels); Serial.print("): ");
  newValue = ReadNumberFromSerial();
  Serial.println("");
  if ((newValue >= 0) && (newValue <= 256))
  {
    newNumberOfChannels = newValue;
  } else {
    Serial.print("Invalid setting for Number of Channels: "); Serial.print(newValue); Serial.println("");
    newNumberOfChannels = number_of_channels;
  }  
  
  while (Serial.available() > 0)
  {
    Serial.read();
  }
 
  Serial.println(""); 
  Serial.println("New settings: ");
  Serial.print("RF Channel: "); Serial.print(newRFChannel); Serial.println("");
  Serial.print("Number of Channels: "); Serial.print(newNumberOfChannels); Serial.println("");
  Serial.println("Press (s) to Save settings or any other key to return to main menu without saving");
  
  while (Serial.available() == 0)
  {
    delay (50);
  }
  char cmd = Serial.read();
  
  switch (cmd)
  {
    case 'S':
    case 's':
      rf_channel = newRFChannel;
      number_of_channels = newNumberOfChannels;
 
      EEPROM.write(0, rf_channel);
      EEPROM.write(1, number_of_channels);
      EEPROM.write(2, Checksum(0,1));
      radio.setChannel(rf_channel);
      mode = INTERACTIVE_MODE;
      break;
    default:
      mode = INTERACTIVE_MODE;
      break;
  }
}

void PrintConfiguration()
{
  Serial.println(""); 
  Serial.println("Settings: ");
  Serial.print("RF Channel: "); Serial.print(rf_channel); Serial.println("");
  Serial.print("Number of Channels: "); Serial.print(number_of_channels); Serial.println("");
  Serial.println("");
}

/* Function: ReadConfigData
 * Description: Read configuration data from EEPROM.  If checksum is valid,
 *   use the configuration data.  If the checksum is invalid, use default values.
 */
void ReadConfigData()
{
  /* Check for valid checksum.  If valid, read  config data from EEPROM. */
  if ( Checksum(0, 2) == 0) 
  {
    //RKW Serial.println("Reading configuration data from EEPROM.");
    rf_channel = EEPROM.read(0);
    number_of_channels = EEPROM.read(1);
    
    //RKW Serial.print("RF Channel: "); Serial.println(rf_channel);
    //RKW Serial.print("Number of Channels: "); Serial.println(number_of_channels);
  } else {
    Serial.print("Using default configuration data. Checksum: ");
    Serial.println(Checksum(0,2), HEX);
    rf_channel = 0x4c;
    number_of_channels = 30;
  }
}
  
/* Function: Checksum
 * Description: Calculate a simple 8-bit checksum over the EEPROM range identified.
 */
byte Checksum(int startIndex, int endIndex)
{
  byte sum = 0;
  
  for (int i = startIndex; i <= endIndex; i++)
  {
    sum += EEPROM.read(i);
  }
  
  return (0xFFu - sum);
}

