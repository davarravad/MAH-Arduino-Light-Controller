/*  Copyright (c) 2014 by Keith Woodard <rkwoodard ~AT~ gmail.com>

    The Arduino Light Controller Sketch is free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    The Arduino Light Controller Sketch is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with The Arduino Light Controller Sketch.  If not, see
    <http://www.gnu.org/licenses/>. */
    
#include <Tlc5940.h> /* Include modified version to TLC5940 library */
#include <SPI.h>
#include <RF24.h>   /* Included for RF support using nRF24L01+ */
#include <EEPROM.h>  /* EEPROM library used to read/write configuration settings to EEPROM */
#include "printf.h"

const char versionId[] = "0.3\0";

#define BAUD_RATE  57600

#define NUMBER_OF_TLCS_TOTAL    2    /* Each Shield has 2 TLC5940s on it */
#define NUMBER_OF_CHANNELS     16*NUMBER_OF_TLCS_TOTAL /* Number of channels per shield (32) */
#define MAX_NUMBER_OF_SHIELDS  4    /* Maximum number of shields that can be stacked on one Arduino */

#define INTERRUPT  0                /* Interrupt used for the zero cross signal */
#define _BLANK_PIN  9               /* Arduino pin connected to the TLC5940 BLANK signals */
#define _LATCH_PIN  7               /* Default latch pin if configuration data is not valid */

/* Some definitions for light values */
/* Board design has inverted logic, so 0 = on, 4095 = off */
#define FULL_ON 0u
#define FULL_OFF 4095u

/* Data interface enumeration.
 */
typedef enum
{
  GENERIC_SERIAL,
  RF_NRF24L01
} interface_t;

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
static boolean interface_rf_initialized = false;
uint16_t data[NUMBER_OF_CHANNELS*MAX_NUMBER_OF_SHIELDS] = {0}; /* Array to store channel data */
volatile int boardToLatch = -1;          /* Board to latch during next interrupt */
boolean dataReceived;                                        /* Flag to indicate data has been received from vixen interface */
unsigned long startTimeMs;                                   /* The start time of the application.  Used to transition from boot mode to run mode automatically */ 
modes_t mode;                                                /* Current mode */

// RF24 Setup
RF24  radio(10,8);                                     /* RF24 Device Class */
const uint64_t rf_pipe = 0xF0F0F0F0E1LL;               /* RF Pipe for receiving broadcast data from bridge. */

// Configuration data
unsigned int interface_type;  /* Interface type to Vixen */
unsigned int latch_pin[MAX_NUMBER_OF_SHIELDS];  /* Store the latch pin for each possible board */
unsigned int rf_channel;      /* For Future Use */
unsigned int starting_address; /* For Future Use */
unsigned int number_of_boards; /* Number of shields connected to the Arduino */
uint16_t  zeroCrossDelay;      /* Configurable delay for zero cross delay to tweak for phase shift of zero cross transformer */

uint16_t boardAddress;  /* Board Address - 16 bits.  Used for command/response protocol to send status data back to controller */ 

/*********************************************************************
 * FORWARD DECLERATIONS
 ********************************************************************/
void PrintConfiguration();

/*
 * Function: setup
 * Description: Power-on configuration of the arduino.  Initialize the
 * correct pin modes for Arduino pins used.  Initialize serial port
 * communications, the TLC5940, and all global data. 
 */
void setup() {
  // Start the serial port
  Serial.begin(BAUD_RATE);
  printf_begin();

  /* Read configuration data from EEPROM */
  ReadConfigData();
   
  /* Setup and configure rf radio */ 
  radio.begin();
  radio.setRetries(0,0);
  radio.setPayloadSize(32);
  radio.setDataRate(RF24_1MBPS);

  /* Setup pins as needed */
  pinMode(2, INPUT_PULLUP);
  pinMode(_BLANK_PIN, OUTPUT);
  /* TODO - Figure out how to set the pin modes only for the latch pins configured for use */
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
      
  // Initialize data arrays
  ClearOutput();
  dataReceived = false;
  
  boardToLatch = -1;

  // Initialize TLC5940 Library
  Tlc.init();   
  
  // Attach interrupt to handle zero-crossing
  attachInterrupt(INT0, ZeroCross, RISING);
 
  /* Set startup mode and store off start time */
  mode = BOOT_MODE;
  
  startTimeMs = millis();
}

/* Function: ClearOutput
 * Description: This function initializes the local data array for the 
 *  TLC5940 channels to a default state of off.  NOTE: This function
 *  does not actually send the data to the devices.
 */
void ClearOutput()
{
  for (unsigned int i=0; i < NUMBER_OF_CHANNELS*number_of_boards; i++)
  {
    data[i] = FULL_OFF;
  }
}

/* Function: ZeroCross
 * Description: This is the main interrupt handler for zero cross detection.
 *   This function toggles the blank pin (to signal the TLC5940's) to begin
 *   the next PWM cycle.  It also checks to see if new data needs to be 
 *   latched into the TLC5940s.  If new data is ready to be latched, it
 *   toggles the latch pin as well.
 */
void ZeroCross()
{
  static unsigned long time = micros();
  
  /* For some reason, 4 interrupts are created for every zero cross.  This
   * logic filters out all of them except the first one.
   */
  if (micros() - time > 7000)
  {
    time = micros();
    delayMicroseconds(zeroCrossDelay);
    digitalWrite(_BLANK_PIN, HIGH);
    if ( (boardToLatch != -1) )
    {
        // Pulse the latch to latch in the new data
        // pulse must be 16 nS.  This should work fine.
        digitalWrite(latch_pin[boardToLatch], HIGH);
        digitalWrite(latch_pin[boardToLatch], LOW);
    }
    digitalWrite(_BLANK_PIN, LOW);
    boardToLatch = -1;
  }
}

/* Function: ReadRFCommand
 * Description: Reads data from the RF interface.
 */
void ReadRFCommand()
{
  boolean allDataReceived = false;
  uint8_t buffer[32];

  /* TODO - Move these calculations to somewhere else */
  int boardStartChannel = starting_address;
  int boardEndChannel = starting_address + (NUMBER_OF_CHANNELS * number_of_boards) - 1;
     
  while (allDataReceived == false)
  {
    /* Wait for data to be available */
    while (radio.available() == 0) {}

    boolean ok = radio.read(buffer, 32);
    
    if (ok)
    {
      printf(".");
      int firstByte = buffer[0];
      int startChannelIndex = buffer[1];
      
      if (firstByte == 0x7e)
      {
        uint8_t channelNumber = 0;
        for (int i=0; i<30; i++)
        {
          channelNumber = startChannelIndex+ i;
                
          int channelIndex = channelNumber - boardStartChannel;
          if ( ( channelNumber >= boardStartChannel ) && (channelNumber < boardEndChannel) )
          {
            data[channelIndex] = (unsigned int)((255 - buffer[2+i]) * 16 + 15); /* Convert from 0-255 to 0-4095 */
          }
        }

        if ( (channelNumber >= boardEndChannel) && (startChannelIndex < boardEndChannel))
        {
          allDataReceived = true;
          printf("!\n");
        }

      }    
    }
  }
}

/* Function: ReadSerialCommand
 * Description: Reads data from the Vixen interface.  Serial data
 * must be proceeded by a header of "A1".
 */
void ReadSerialCommand()
{
  uint32_t index = 0;
  uint32_t numberOfChannelsToRead = NUMBER_OF_CHANNELS * number_of_boards;
  
  // Wait for start byte which indicates board revision
  while (Serial.read() != 'A')
  {
    delay(1);
  }
  
  int revision = Serial.read();
  
  if (revision == '1')
  {
    while (index < (numberOfChannelsToRead+starting_address))
    {
      if (Serial.available() > 0)
      {
        /* Read the next byte from the serial port.  If the index exceeds
         * the starting address for this board, store off the value into the 
         * data array.
         */
        unsigned int valueRead = Serial.read();
        if (index >= starting_address)
        {
          /* Do same safety checking on the index before writing to the array */
          uint32_t arrayIndex = index - starting_address;
          if ( (arrayIndex >= 0) && (arrayIndex < NUMBER_OF_CHANNELS*number_of_boards) )
          {
            data[arrayIndex] = (unsigned int)((255 - valueRead) * 16 + 15); /* Convert from 0-255 to 0-4095 */
          } 
        }
        index++;
      } else
      {  
        delay(1);
      }
    }
  }
}

/* Function: InitializeRF
 * Description: Initialize the RF device if not already initialized.
 */
void InitializeRF()
{
    /* If configured for RF mode, initialize reading pipe */
  if ( (interface_type == RF_NRF24L01) && (interface_rf_initialized == false) )
  {
    radio.setChannel(rf_channel);
    radio.openReadingPipe(1, rf_pipe);
    radio.startListening();
    interface_rf_initialized = true;
    
    if (radio.isPVariant() == false)
    {
      Serial.write("Unable to find nRF24L01+ module\n");
    }
  }
}

/* Function: loop
 * Description: Main processing loop of the application. This loop
 * executes the main state machine for the board.
 */
 void loop() {
  /* Automatically transition to run mode after 10 seconds */
  unsigned long elapsedTime = millis() - startTimeMs;

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
      InitializeRF();
      RunOperationalMode();
      break;
    case INTERACTIVE_MODE:
      InitializeRF();
      RunInteractiveMode();
      break;
    case TEST_MODE_ALL_FADE:
      InitializeRF();
      RunTestModeFade();
      break;
    case TEST_MODE_WALK_CHANNELS:
      InitializeRF();
      RunTestModeWalkChannels();
      break;
    case CONFIG_MODE:
      RunConfigMode();
      break;
    default:
      break;
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
  
  Serial.println("Arduino Light Controller Main Menu");
  printf("Version: %s\n\n", versionId);
  Serial.println(" (t) Enter Test Mode - All channels fade");
  Serial.println(" (w) Enter Test Mode - Walk channels");
  Serial.println(" (c) Enter Configuration Mode");
  Serial.println(" (r) Enter Operational Mode");
  Serial.println(" {p} Print Configuration");
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

/* Function: WriteDataToShields
 * Description: Write channel data to each shield.  If more than one
 *   shield is used, data is written to only one shield at a time by
 *   writing out the data to one shield and then waiting until that 
 *   data has been latched (as part of Zero Cross Interrupt) before
 *   writing data to the next shield.
 */
void WriteDataToShields()
{
  for (uint32_t board = 0; board < number_of_boards; board++)
  {
    for (uint32_t i=0; i < NUMBER_OF_CHANNELS; i++)
    {
      Tlc.set(i, data[board*NUMBER_OF_CHANNELS + i]);
    }
    Tlc.update();
    boardToLatch = board;

    /* busy loop until interrupt handler clears this flag */
    while (boardToLatch != -1) {}
  }
}

/* Function: RunOperationalMode
 * Description: Run the operational mode and read data from the
 *   Vixen interface and update data to the TLC5940s.  Prior to
 *   receiving data on the Vixen interface, ensure all channels
 *   are turned off.
 */
void RunOperationalMode()
{
  while (1)
  {
    /* Once data has been received over the selected interface,
     * begin listening to the data feed.  Until then, make
     * sure all channels are turned off.
     */
    if (dataReceived == true)
    {
      // Read settings from serial port
      if (interface_type == RF_NRF24L01)
      {
        radio.startListening();
        ReadRFCommand();      
        radio.stopListening();
      } else {
        ReadSerialCommand();
      }  
      /* This will not return until all data has been written
         to the TLC5940 shields.
        */
      WriteDataToShields();
    }
    else
    {
      // This is a startup mode.  Turn lights off until data is received */
      if ( (interface_type == GENERIC_SERIAL) && (Serial.available() > 0) )
      {
        dataReceived = true;
      }

      if (interface_type == RF_NRF24L01) 
      {
        radio.startListening();
        delay(100);
        if (radio.available() > 0)
        { 
          dataReceived = true;
        }      
        radio.stopListening();
      }
      
      for (uint32_t channel = 0; channel < NUMBER_OF_CHANNELS*number_of_boards; channel++)
      {
        data[channel] = FULL_OFF;
      } 
      WriteDataToShields();
      delay(100);
    }
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
       for (uint32_t channel = 0; channel < NUMBER_OF_CHANNELS*number_of_boards; channel++)
       {
         data[channel] = FULL_ON;
       }   
       
       /* After the channels have been on for two seconds, turn off all the channels
        * and transition to walking through the channels.
        */
       if ( (millis() - lastTime) > 2000l)
       {
         /* Turn off all channels */
         for (uint32_t channel = 0; channel < NUMBER_OF_CHANNELS*number_of_boards; channel++)
         {
           data[channel] = FULL_OFF;
         }  
         data[0] = FULL_ON;
         lastTime = millis();       
         allOn = false;
       }
     } else {
       if ( (millis() - lastTime) > 1000ul)
       {
         lastTime = millis();
         /* Turn off the channel that is already on */
         data[channelNumber] = FULL_OFF;
         /* Then, move to the next channel */
         channelNumber++;
         /* If reached the last channel, switch back to "all on" 
          * and begin repeating the loop */
         if (channelNumber == NUMBER_OF_CHANNELS*number_of_boards)
         {
           channelNumber = 0;
           allOn = true;
         } else {
           data[channelNumber] = FULL_ON;
         }
       }      
     }

     /* Send new data to the TLCs and trigger the data to be latched */
     WriteDataToShields();
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

    value += (5*direction);

    if (value > 4095)
    {
      value = 4095;
      direction = -1;
    }
    if (value < 0)
    {  
      value = 0;
      direction = 1;
    }
 
    for (uint32_t channel = 0; channel < NUMBER_OF_CHANNELS*number_of_boards; channel++)
    {
      data[channel] = value;
    } 
     WriteDataToShields();
     delay(20);
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
  int newInterfaceType;
  int newNumberOfBoards;
  int newLatchPin[MAX_NUMBER_OF_SHIELDS];
  int newStartingAddress;
  int newRFChannel;
  int newZeroCrossDelay;
  
  Serial.println("Arduino Light Controller configuration:");

  Serial.print("  Interface Type (0=Generic Serial, 1=nRF24L01+) ("); Serial.print(interface_type); Serial.print("): ");
  int newValue = ReadNumberFromSerial();
  Serial.println("");
  if ((newValue >= 0) && (newValue <= 1))
  {
    newInterfaceType = newValue;
  } else {
    newInterfaceType = interface_type;
    Serial.print("Invalid setting for interface type: "); Serial.print(newValue); Serial.println("");
  }
  
   /* Ask user for Number of Boards */
  Serial.print("  Number of Shields Connected ("); Serial.print(number_of_boards); Serial.print("): ");
  newValue = ReadNumberFromSerial();
  Serial.println("");
  if ((newValue > 0) && (newValue <= MAX_NUMBER_OF_SHIELDS))
  {
    newNumberOfBoards = newValue;
  } else {
    newNumberOfBoards = number_of_boards;
    Serial.print("Invalid setting for numberOfBoards: "); Serial.print(newValue); Serial.println("");
  }
  
  /* Ask user for Latch Pin setting */
  for (int i=0; i < newNumberOfBoards; i++)
  {
    Serial.print("  Latch Pin for Shield "); Serial.print(i); Serial.print(" ("); Serial.print(latch_pin[i]); Serial.print("): ");
    int newValue = ReadNumberFromSerial();
    Serial.println("");
    if ((newValue >= 4) && (newValue <= 7))
    {
      newLatchPin[i] = newValue;
    } else {
      newLatchPin[i] = latch_pin[i];
      Serial.print("Invalid setting for latch pin: "); Serial.print(newValue); Serial.println("");
    }
  }
  
  /* Ask user for Starting Address setting */
  Serial.print("  Starting Address ("); Serial.print(starting_address); Serial.print("): ");
  newValue = ReadNumberFromSerial();
  Serial.println("");
  if ((newValue >= 0) && (newValue <= 255))
  {
    newStartingAddress = newValue;
  } else {
    newStartingAddress = starting_address;
    Serial.print("Invalid setting for Starting Address: "); Serial.print(newValue); Serial.println("");
  }

  /* Ask user for Zero Cross Delay setting */  
  Serial.print("  Zero Cross Delay ("); Serial.print(zeroCrossDelay); Serial.print("): ");
  newValue = ReadNumberFromSerial();
  Serial.println("");
  if ((newValue >= 0) && (newValue <= 4000))
  {
    newZeroCrossDelay = newValue;
  } else {
    Serial.print("Invalid setting for zero cross delay: "); Serial.print(newValue); Serial.println("");
    newRFChannel = zeroCrossDelay;
  } 
  
  /* Ask user for RF Channel setting */  
  Serial.print("  RF Channel ("); Serial.print(rf_channel); Serial.print("): ");
  newValue = ReadNumberFromSerial();
  Serial.println("");
  if ((newValue >= 2) && (newValue <= 80))
  {
    newRFChannel = newValue;
  } else {
    Serial.print("Invalid setting for rf channel: "); Serial.print(newValue); Serial.println("");
    newRFChannel = rf_channel;
  }  
  
  while (Serial.available() > 0)
  {
    Serial.read();
  }
 
  Serial.println(""); 
  Serial.println("New settings: ");
  Serial.print("Interface Type: "); Serial.print(newInterfaceType); Serial.println("");
  Serial.print("Number of Shields: "); Serial.print(newNumberOfBoards); Serial.println("");
  for (uint32_t board = 0; board < newNumberOfBoards; board++)
  {
    Serial.print("Latch Pin for Shield "); Serial.print(board); Serial.print(": "); Serial.print(newLatchPin[board]); Serial.println("");
  }
  Serial.print("Starting Address: "); Serial.print(newStartingAddress); Serial.println("");
  Serial.print("Zero Cross Delay: "); Serial.print(newZeroCrossDelay); Serial.println("");
  Serial.print("RF Channel: "); Serial.print(newRFChannel); Serial.println("");
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
      interface_type = newInterfaceType;
      number_of_boards = newNumberOfBoards;
      latch_pin[0] = newLatchPin[0];
      latch_pin[1] = newLatchPin[1];
      latch_pin[2] = newLatchPin[2];
      latch_pin[3] = newLatchPin[3];
      starting_address = newStartingAddress;
      rf_channel = newRFChannel;
      zeroCrossDelay = newZeroCrossDelay;
      
      EEPROM.write(0, interface_type);
      EEPROM.write(1, number_of_boards);
      EEPROM.write(2, latch_pin[0]);
      EEPROM.write(3, latch_pin[1]);
      EEPROM.write(4, latch_pin[2]);
      EEPROM.write(5, latch_pin[3]);
      EEPROM.write(6, starting_address);
      EEPROM.write(7, zeroCrossDelay / 256);
      EEPROM.write(8, (zeroCrossDelay & 0x00FF));
      EEPROM.write(9,  rf_channel);
      EEPROM.write(10, Checksum(0,9));
     
      mode = INTERACTIVE_MODE;
      break;
    default:
      mode = INTERACTIVE_MODE;
      break;
  }
}

void PrintConfiguration()
{
    if (interface_type == RF_NRF24L01)
    {
      radio.printDetails();
    }
    Serial.println(""); 
    Serial.println("Settings: ");
    Serial.print("Interface Type: "); Serial.print(interface_type); Serial.println("");
    Serial.print("Number of Shields: "); Serial.print(number_of_boards); Serial.println("");
    for (uint32_t board = 0; board < number_of_boards; board++)
    {
      Serial.print("Latch Pin for Shield "); Serial.print(board); Serial.print(": "); Serial.print(latch_pin[board]); Serial.println("");
    }
    Serial.print("Starting Address: "); Serial.print(starting_address); Serial.println("");
    Serial.print("Zero Cross Delay: "); Serial.print(zeroCrossDelay); Serial.println("");
    Serial.print("RF Channel: "); Serial.print(rf_channel); Serial.println("");
    Serial.println("");
}

/* Function: ReadConfigData
 * Description: Read configuration data from EEPROM.  If checksum is valid,
 *   use the configuration data.  If the checksum is invalid, use default values.
 */
void ReadConfigData()
{
  /* Check for valid checksum.  If valid, read  config data from EEPROM. */
  if ( Checksum(0, 10) == 0) 
  {
    interface_type = EEPROM.read(0);
    number_of_boards = EEPROM.read(1);
    latch_pin[0] = EEPROM.read(2);
    latch_pin[1] = EEPROM.read(3);
    latch_pin[2] = EEPROM.read(4);
    latch_pin[3] = EEPROM.read(5);
    starting_address = EEPROM.read(6);
    zeroCrossDelay = ((uint16_t)EEPROM.read(7) * 256) | (EEPROM.read(8) );
    rf_channel = EEPROM.read(9);
  } else {
    Serial.print("Using default configuration data. Checksum: ");
    Serial.println(Checksum(0,10), HEX);
    
    interface_type = GENERIC_SERIAL;
    number_of_boards = 1;
    latch_pin[0] = _LATCH_PIN;
    latch_pin[1] = 0;
    latch_pin[2] = 0;
    latch_pin[3] = 0;
    starting_address = 0;
    zeroCrossDelay = 0;
    rf_channel = 0x4c;
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
