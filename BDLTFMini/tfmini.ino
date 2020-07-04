// Try three times to get the firmware version number
// which is saved as 'tfmp.version', a three byte array.

void initMiniPlus()
{
    printf_begin();          // Initialize printf.
//    printf("\r\nTFMPlus Library Example - 07JUN2019\r\n");  // say 'hello'

    mySerial.begin( 115200);  // Initialize TFMPLus device serial port.
    delay(20);               // Give port time to initalize
    tfmP.begin( &mySerial);   // Initialize device library object and...
                             // pass device serial port to the object.

    // Send commands to device during setup.
  //factoryReset();     // reset to default settings
    firmwareVersion();  // get firmware version
    frameRate( 100);    // set device data frame-rate to 100Hz
    saveSettings();     // save the frame-rate setting

    // Initialize the variables for this example
    tfDist = 0;            // Clear device data variables.
    tfFlux = 0;
    tfTemp = 0;

    delay(500);            // And wait for half a second.
}

void printDistance()
{
  for( uint8_t fvi = 1; fvi < 6; ++fvi)
  {
      if( tfmP.getData( tfDist, tfFlux, tfTemp)) // Get data from the device.
      {
        printf( " Dist:%04u \n", tfDist);           // Display the distance.
        break;                                   // Escape this sub-loop
      }
      else                        // If the command fails...
      {
//        tfmP.printStatus( true);  // display the error.
      }
  }
}

// Try three times to get the firmware version number
// which is saved as 'tfmp.version', a three byte array.
void firmwareVersion()
{
    for( uint8_t fvi = 1; fvi < 4; ++fvi)
    {
        if( tfmP.sendCommand( OBTAIN_FIRMWARE_VERSION, 0))
        {
            // If successful, display the version number...
            printf( "Lidar firmware version: %1u.%1u.%1u\r\n",
                tfmP.version[ 0], tfmP.version[ 1], tfmP.version[ 2]);
            break;      // and brreak out of loop.
        }
        else
        {
            // If not successful, display the attempt number
            // and the error: HEADER, CHERCKSUM, SERIAL, tec.
            printf( "Get firmware version failed. "); // Display error message...
            printf( "Attempt: %u", fvi);              // attempt number..
            tfmP.printStatus( false);                 // and error status.
            printf("\r\n");
        }
        delay(100);  // Wait to try again
    }
}

void factoryReset()
{
    printf( "Lidar factory reset ");
    if( tfmP.sendCommand( RESTORE_FACTORY_SETTINGS, 0))
    {
        printf( "passed.\r\n");
    }
    else
    {
        printf( "failed.");
        tfmP.printStatus( false);
    }
}

void frameRate( uint16_t rate)
{
    printf( "Lidar frame rate ");
    if( tfmP.sendCommand( SET_FRAME_RATE, rate))
    {
        printf( "set to %2uHz.\r\n", rate);
    }
    else
    {
        printf( "command failed.");
        tfmP.printStatus( false);
    }
}

void saveSettings()
{
    printf( "Lidar device settings ");
    if( tfmP.sendCommand( SAVE_SETTINGS, 0))
    {
        printf( "saved.\r\n");
    }
    else
    {
        printf( "not saved.");
        tfmP.printStatus( false);
    }
}
