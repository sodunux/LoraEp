// Note: logs are saved into "LogFile.log"

//-------------------------------------------
// FUNCTION FOR FLASH PROGRAMMING A SINGLE BYTE
//
//    This function is called using LOAD button
//-------------------------------------------
void FlashByteProg (unsigned long int inCodeAddress, unsigned char *inData) {
    unsigned char theBufferStatus;
    unsigned char theData;
    char theLog[256];
    _LOG("------------------ FlashByteProg -----------------\n");

    // Flash 16bit mode
    _WBYTE_SFR(0x9A, 0xFF);

    // Send Buffered Program Command
    theData = 0x40;
    //WARNING: only write command at odd address
    _WNBYTES_CODE(inCodeAddress | 0x01, &theData, 1);

    // Write Data
    _WNBYTES_CODE(inCodeAddress , inData, 1);

    sprintf(theLog, ": Write 0x%X @0x%X\n", *inData, inCodeAddress);
    _LOG(theLog);


    // Check Status Register
    do {
        // Check Buffer Status
        //WARNING: only write command at odd address

        _RNBYTES_CODE(inCodeAddress | 0x01, &theBufferStatus, 1);

        sprintf(theLog, ": End Buffer Status 0x%X\n", theBufferStatus);
        _LOG(theLog);
    } while ((theBufferStatus & 0x80) != 0x80);

    // Return to Read Array Mode
    theData = 0xFF;
    //WARNING: only write command at odd address
    _WNBYTES_CODE(inCodeAddress | 0x01, &theData, 1);
} /* End of FlashByteProg */


//-------------------------------------------
// FUNCTION FOR FLASH PROGRAMMING A SECTOR
//    OR SEVERAL CONSECUTIVE DATA
//
//    This function is called using LOAD button
//-------------------------------------------
void FlashSectorProg (unsigned long int inCodeAddress, unsigned char *inData, unsigned int inCount) {
    unsigned char *theBuffer;
    unsigned char theBufferStatus;
    unsigned long int theCodeAddress;
    unsigned char theData, theWordCount;
    unsigned int theCount;
    unsigned char isDataToWrite;
    char theLog[256];
    int i;

    theBuffer = inData;

    _LOG("------------------ FlashSectorProg -----------------\n");
    sprintf(theLog, "Code Address 0x%X, Count 0x%X\n", inCodeAddress, inCount);
    _LOG(theLog);
    
    // Select Flash memory byte mode
    _WBYTE_SFR(0x9A, 0x00);
    
    
    // Buffer Size is 0xFF. Operation must be done
    theCount = inCount - 1;
    theCodeAddress = inCodeAddress;
    while (theCount > 0) {
        if (theCount < 0xFF) {
             theWordCount = theCount;
            }
        else {
            theWordCount = 0xFF;
        }
        theCount = theCount - 0xFF;

        // Check buffer to avoid writing empty buffers
        isDataToWrite = 0;
        for (i = 0; i <= theWordCount ; i++) {
            if (*(theBuffer+i) != 0xFF) {
                isDataToWrite = 1;
            }
        }

        if (isDataToWrite) {

            // Send Buffered Program Command
            theData = 0xE8;
            //WARNING: only write command at odd address
            _WNBYTES_CODE(theCodeAddress | 0x01, &theData, 1);
    
            sprintf(theLog, "=> Write Buffer Prog Command @%X\n" , theCodeAddress);
            _LOG(theLog);

            _LOG("=> Check Status\n");
            do {

                // Check Buffer Status
                //WARNING: only write command at odd address
                _RNBYTES_CODE(theCodeAddress | 0x01, &theBufferStatus, 1);
                sprintf(theLog, ": Buffer Status after Command 0x%X\n", theBufferStatus);
                _LOG(theLog);
            } while ((theBufferStatus & 0x80) != 0x80);

            // Write Count
            // WARNING: only write command at odd address

            sprintf(theLog, "=> Write Count 0x%X\n", theWordCount);
            _LOG(theLog);
            _WNBYTES_CODE(theCodeAddress | 0x01, &theWordCount, 1);

            // Program Buffer
            _WNBYTES_CODE(theCodeAddress, theBuffer, theWordCount+1);

            // Confirm Program Operation
            theData = 0xD0;
            //WARNING: only write command at odd address
            _WNBYTES_CODE(theCodeAddress | 0x01, &theData, 1);

            // Check Status Register
            do {
                // Check Buffer Status
                //WARNING: only write command at odd address
                _RNBYTES_CODE(theCodeAddress | 0x01, &theBufferStatus, 1);
            } while ((theBufferStatus & 0x80) != 0x80);
    
            sprintf(theLog, ": Buffer Status after Prog 0x%X\n", theBufferStatus);
            _LOG(theLog);
        }
        theCodeAddress = theCodeAddress + theWordCount + 1;
        theBuffer = theBuffer + theWordCount + 1;
    }
    // Return to Flash 16bit mode
    _WBYTE_SFR(0x9A, 0xFF);

    // Return to Read Array Mode
    theData = 0xFF;
    //WARNING: only write command at odd address
    _WNBYTES_CODE(inCodeAddress | 0x01, &theData, 1);
} /* End of FlashSectorProg */


//-------------------------------------------
// FUNCTION FOR FLASH ERASE
//
//    This function is called using ERASE or LOAD buttons
//-------------------------------------------
void FlashErase (void) {
    
    unsigned char theBufferStatus;
    unsigned long int theCodeAddress;
    unsigned char theData;
    _LOG("------------------ FlashErase -----------------\n");
    // Buffer Size is 0xFF. Operation must be done
    theCodeAddress = 0x800000;
    // Send Sector Erase  Command
    theData = 0x20;
    //WARNING: only write command at odd address
    _WNBYTES_CODE(theCodeAddress | 0x01, &theData, 1);

    // Confirm Sector Erase  Command
    theData = 0xD0;
    //WARNING: only write command at odd address
    _WNBYTES_CODE(theCodeAddress | 0x01, &theData, 1);

    // Check Status Register
    do {
        // Check Buffer Status
        //WARNING: only write command at odd address
        _RNBYTES_CODE(theCodeAddress | 0x01, &theBufferStatus, 1);
    } while ((theBufferStatus & 0x80) != 0x80);

    // Return to Read Array Mode
    theData = 0xFF;
    //WARNING: only write command at odd address
    _WNBYTES_CODE(theCodeAddress | 0x01, &theData, 1);

    // Check Status Register
    do {
        // Check Buffer Status
        //WARNING: only write command at odd address
        _RNBYTES_CODE(theCodeAddress | 0x01, &theBufferStatus, 1);
    } while ((theBufferStatus & 0x80) != 0x80);

} /* End of FlashErase */


//-------------------------------------------
// FUNCTION FOR FLASH SECTOR ERASE
//
//    This function is called by the BIRD Debug Software
//    to erase the sector corresponding to the given
//    address
//-------------------------------------------
void FlashSectorErase (unsigned long int inCodeAddress) {
    
    unsigned char theBufferStatus;
    unsigned long int theCodeAddress;
    unsigned char theData;
    _LOG("------------------ FlashSectorErase -----------------\n");
    // Buffer Size is 0xFF. Operation must be done
    theCodeAddress = inCodeAddress;
    // Send Sector Erase  Command
    theData = 0x20;
    //WARNING: only write command at odd address
    _WNBYTES_CODE(theCodeAddress | 0x01, &theData, 1);

    // Confirm Sector Erase  Command
    theData = 0xD0;
    //WARNING: only write command at odd address
    _WNBYTES_CODE(theCodeAddress | 0x01, &theData, 1);

    // Check Status Register
    do {
        // Check Buffer Status
        //WARNING: only write command at odd address
        _RNBYTES_CODE(theCodeAddress | 0x01, &theBufferStatus, 1);
    } while ((theBufferStatus & 0x80) != 0x80);

    // Return to Read Array Mode
    theData = 0xFF;
    //WARNING: only write command at odd address
    _WNBYTES_CODE(theCodeAddress | 0x01, &theData, 1);

} /* End of FlashSectorErase */


//-------------------------------------------
// FUNCTION FOR FLASH READ
//
//    This function is called to configure
//    the flash in read mode
//-------------------------------------------
void FlashSetupRead (void) {
    _LOG("------------------ FlashSetupRead -----------------\n");
} /* End of FlashSetupRead */

