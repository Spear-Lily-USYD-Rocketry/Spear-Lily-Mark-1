//#include "gnss.h"
//
//
//// Get the latest Position/Velocity/Time solution and fill all global variables
//bool getPVT(uint16_t maxWait, ubxPacket *packetCfg)
//{
//  if (packetUBXNAVPVT == NULL)
//    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
//  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
//    return (false);
//
//  if (packetUBXNAVPVT->automaticFlags.flags.bits.automatic && packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate)
//  {
//    // The GPS is automatically reporting, we just check whether we got unread data
//    //  if (_printDebug == true)
//    //  {
//    //    _debugSerial->println(F("getPVT: Autoreporting"));
//    //  }
//    checkUbloxInternal(packetCfg, UBX_CLASS_NAV, UBX_NAV_PVT);
//    return packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all;
//  }
//  else if (packetUBXNAVPVT->automaticFlags.flags.bits.automatic && !packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate)
//  {
//    // Someone else has to call checkUblox for us...
//    //  if (_printDebug == true)
//    //  {
//    //    _debugSerial->println(F("getPVT: Exit immediately"));
//    //  }
//    return (false);
//  }
//  else
//  {
//    // if (_printDebug == true)
//    // {
//    //   _debugSerial->println(F("getPVT: Polling"));
//    // }
//
//    // The GPS is not automatically reporting navigation position so we have to poll explicitly
//    packetCfg->cls = UBX_CLASS_NAV;
//    packetCfg->id = UBX_NAV_PVT;
//    packetCfg->len = 0;
//    packetCfg->startingSpot = 0;
//    // packetCfg.startingSpot = 20; //Begin listening at spot 20 so we can record up to 20+packetCfgPayloadSize = 84 bytes Note:now hard-coded in processUBX
//
//    // The data is parsed as part of processing the response
//    sfe_ublox_status_e retVal = sendCommand(packetCfg, maxWait, false);
//
//    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
//      return (true);
//
//    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
//    {
//      // if (_printDebug == true)
//      // {
//      //   _debugSerial->println(F("getPVT: data in packetCfg was OVERWRITTEN by another message (but that's OK)"));
//      // }
//      return (true);
//    }
//
//    // if (_printDebug == true)
//    // {
//    //   _debugSerial->print(F("getPVT retVal: "));
//    //   _debugSerial->println(statusString(retVal));
//    // }
//    return (false);
//  }
//}
//
//
//int32_t getLatitude(uint16_t maxWait)
//{
//  if (packetUBXNAVPVT == NULL)
//    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
//  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
//    return 0;
//
//  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lat == false)
////    getPVT(maxWait);
//  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lat = false; // Since we are about to give this to user, mark this data as stale
//  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
//  return (packetUBXNAVPVT->data.lat);
//}
//
//bool checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
//{
//
//	return (checkUbloxSpi(incomingUBX, requestedClass, requestedID));
//
//}
//
//// Given a packet and payload, send everything including CRC bytes via I2C port
//sfe_ublox_status_e sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait, bool expectACKonly)
//{
//  sfe_ublox_status_e retVal = SFE_UBLOX_STATUS_SUCCESS;
//
//  calcChecksum(outgoingUBX); // Sets checksum A and B bytes of the packet
//
//   sendSpiCommand(outgoingUBX);
//
//  if (maxWait > 0)
//  {
//    // Depending on what we just sent, either we need to look for an ACK or not
//    if ((outgoingUBX->cls == UBX_CLASS_CFG) || (expectACKonly == true))
//    {
//      retVal = waitForACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
//    }
//    else
//    {
//      retVal = waitForNoACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
//    }
//  }
//  return retVal;
//}
//
//
//
//
//
//
//
//
////**********************************************88
////SPI read/write
//// Checks SPI for data, passing any new bytes to process()
//bool checkUbloxSpi(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
//{
//  // Process the contents of the SPI buffer if not empty!
//  for (uint8_t i = 0; i < spiBufferIndex; i++)
//  {
//    process(spiBuffer[i], incomingUBX, requestedClass, requestedID);
//  }
//  spiBufferIndex = 0;
//
////  _spiPort->beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));
////  digitalWrite(_csPin, LOW);
////  uint8_t byteReturned = _spiPort->transfer(0xFF);
////
////  // Note to future self: I think the 0xFF check might cause problems when attempting to process (e.g.) RAWX data
////  // which could legitimately contain 0xFF within the data stream. But the currentSentence check will certainly help!
////
////  // If we are not receiving a sentence (currentSentence == NONE) and the byteReturned is 0xFF,
////  // i.e. the module has no data for us, then delay for
////  if ((byteReturned == 0xFF) && (currentSentence == NONE))
////  {
////    digitalWrite(_csPin, HIGH);
////    _spiPort->endTransaction();
////    delay(spiPollingWait);
////    return (true);
////  }
////
////  while ((byteReturned != 0xFF) || (currentSentence != NONE))
////  {
////    process(byteReturned, incomingUBX, requestedClass, requestedID);
////    byteReturned = _spiPort->transfer(0xFF);
////  }
////  digitalWrite(_csPin, HIGH);
////  _spiPort->endTransaction();
//  return (true);
//
//} // end checkUbloxSpi()
//
//
//
//void spiTransfer(uint8_t byteToTransfer)
//{
//  uint8_t returnedByte = _spiPort->transfer(byteToTransfer);
//  if ((spiBufferIndex < getSpiTransactionSize()) && (returnedByte != 0xFF || currentSentence != NONE))
//  {
//    spiBuffer[spiBufferIndex] = returnedByte;
//    spiBufferIndex++;
//  }
//}
//
//
//
//// Send a command via SPI
//void sendSpiCommand(ubxPacket *outgoingUBX)
//{
//  if (spiBuffer == NULL)
//  {
//
//    return;
//  }
//
//  // Start at the beginning of the SPI buffer
//  spiBufferIndex = 0;
////
////  _spiPort->beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));
////  digiuint8_t *payloadAuto,talWrite(_csPin, LOW);
////  // Write header bytes
////  spiTransfer(UBX_SYNCH_1); //μ - oh ublox, you're funny. I will call you micro-blox from now on.
////  spiTransfer(UBX_SYNCH_2); // b
////
////  spiTransfer(outgoingUBX->cls);
////  spiTransfer(outgoingUBX->id);
////  spiTransfer(outgoingUBX->len & 0xFF); // LSB
////  spiTransfer(outgoingUBX->len >> 8);
////
////  // Write payload.
////  spiTransfer(outgoingUBX->checksumA);
////  spiTransfer(outgoingUBX->checksumB);
////  digitalWrite(_csPin, HIGH);
////  for (uint16_t i = 0; i < outgoingUBX->len; i++)
////  {
////    spiTransfer(outgoingUBX->payload[i]);
////
////  }uint8_t *payloadAuto,
////
////  // Write checksum
////  spiTransfer(outgoingUBX->checksumA);
////  spiTransfer(outgoingUBX->checksumB);
////  digitalWrite(_csPin, HIGH);
////  _spiPort->endTransaction();
//
//}
//
//// Processes NMEA and UBX binary sentences one byte at a time
//// Take a given byte and file it into the proper array
//void process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
//{
////  if (_outputPort != NULL)
////    _outputPort->write(incoming); // Echo this byte to the serial port
//  if ((currentSentence == NONE) || (currentSentence == NMEA))
//  {
//    if (incoming == UBX_SYNCH_1) // UBX binary frames start with 0xB5, aka μ
//    {
//      // This is the start of a binary sentence. Reset flags.
//      // We still don't know the response class
//      ubxFrameCounter = 0;
//      currentSentence = UBX;
//      // Reset the packetBuf.counter even though we will need to reset it again when ubxFrameCounter == 2
//      packetBuf.counter = 0;
//      ignoreThisPayload = false; // We should not ignore this payload - yet
//      // Store data in packetBuf until we know if we have a requested class and ID match
//      activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;
//    }
//    else if (incoming == '$')
//    {
//      nmeaByteCounter = 0; // Reset the NMEA byte counter
//      currentSentence = NMEA;
//    }
//    else if (incoming == 0xD3) // RTCM frames start with 0xD3
//    {
//      rtcmFrameCounter = 0;
//      currentSentence = RTCM;
//    }
//    else
//    {
//      // This character is unknown or we missed the previous start of a sentence
//    }
//  }
//
//  // Depending on the sentence, pass the character to the individual processor
//  if (currentSentence == UBX)
//  {
//    // Decide what type of response this is
//    if ((ubxFrameCounter == 0) && (incoming != UBX_SYNCH_1))      // ISO 'μ'
//      currentSentence = NONE;                                     // Something went wrong. Reset.
//    else if ((ubxFrameCounter == 1) && (incoming != UBX_SYNCH_2)) // ASCII 'b'
//      currentSentence = NONE;                                     // Something went wrong. Reset.
//    // Note to future self:
//    // There may be some duplication / redundancy in the next few lines as processUBX will also
//    // load information into packetBuf, but we'll do it here too for clarity
//    else if (ubxFrameCounter == 2) // Class
//    {
//      // Record the class in packetBuf until we know what to do with it
//      packetBuf.cls = incoming; // (Duplication)
//      rollingChecksumA = 0;     // Reset our rolling checksums here (not when we receive the 0xB5)
//      rollingChecksumB = 0;
//      packetBuf.counter = 0;                                   // Reset the packetBuf.counter (again)
//      packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // Reset the packet validity (redundant?)
//      packetBuf.startingSpot = incomingUBX->startingSpot;      // Copy the startingSpot
//    }
//    else if (ubxFrameCounter == 3) // ID
//    {
//      // Record the ID in packetBuf until we know what to do with it
//      packetBuf.id = incoming; // (Duplication)
//      // We can now identify the type of response
//      // If the packet we are receiving is not an ACK then check for a class and ID match
//      if (packetBuf.cls != UBX_CLASS_ACK)
//      {
//        // This is not an ACK so check for a class and ID match
//        if ((packetBuf.cls == requestedClass) && (packetBuf.id == requestedID))
//        {
//          // This is not an ACK and we have a class and ID match
//          // So start diverting data into incomingUBX (usually packetCfg)
//          activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
//          incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
//          incomingUBX->id = packetBuf.id;
//          incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
//        }
//        // This is not an ACK and we do not have a complete class and ID match
//        // So let's check if this is an "automatic" message which has its own storage defined
//        else if (checkAutomatic(packetBuf.cls, packetBuf.id))
//        {
//          // This is not the message we were expecting but it has its own storage and so we should process it anyway.
//          // We'll try to use packetAuto to buffer the message (so it can't overwrite anything in packetCfg).
//          // We need to allocate memory for the packetAuto payload (payloadAuto) - and delete it once
//          // reception is complete.
//          uint16_t maxPayload = getMaxPayloadSize(packetBuf.cls, packetBuf.id); // Calculate how much RAM we need
//          if (maxPayload == 0)
//          {
////#ifndef SFE_UBLOX_REDUCED_PROG_MEM
////            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
////            {
////              _debugSerial->print(F("process: getMaxPayloadSize returned ZERO!! Class: 0x"));
////              _debugSerial->print(packetBuf.cls);
////              _debugSerial->print(F(" ID: 0x"));
////              _debugSerial->println(packetBuf.id);
////            }
////#endif
//          }
//          if (payloadAuto != NULL) // Check if memory is already allocated - this should be impossible!
//          {
//#ifndef SFE_UBLOX_REDUCED_PROG_MEM
////            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
////            {
////              _debugSerial->println(F("process: memory is already allocated for payloadAuto! Deleting..."));
////            }
//#endif
//            free(payloadAuto); // Created with new[]
//            payloadAuto = NULL;   // Redundant?
//            packetAuto.payload = payloadAuto;
//          }
//          payloadAuto = (uint8_t*)malloc(maxPayload); // Allocate RAM for payloadAuto
//          packetAuto.payload = payloadAuto;
//          if (payloadAuto == NULL) // Check if the alloc failed
//          {
////#ifndef SFE_UBLOX_REDUCED_PROG_MEM
////            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
////            {
////              _debugSerial->print(F("process: memory allocation failed for \"automatic\" message: Class: 0x"));
////              _debugSerial->print(packetBuf.cls, HEX);
////              _debugSerial->print(F(" ID: 0x"));
////              _debugSerial->println(packetBuf.id, HEX);
////              _debugSerial->println(F("process: \"automatic\" message could overwrite data"));
////            }
////#endif
//            // The RAM allocation failed so fall back to using incomingUBX (usually packetCfg) even though we risk overwriting data
//            activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
//            incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
//            incomingUBX->id = packetBuf.id;
//            incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
//          }
//          else
//          {
//            // The RAM allocation was successful so we start diverting data into packetAuto and process it
//            activePacketBuffer = SFE_UBLOX_PACKET_PACKETAUTO;
//            packetAuto.cls = packetBuf.cls; // Copy the class and ID into packetAuto
//            packetAuto.id = packetBuf.id;
//            packetAuto.counter = packetBuf.counter;           // Copy over the .counter too
//            packetAuto.startingSpot = packetBuf.startingSpot; // And the starting spot? (Probably redundant)
////#ifndef SFE_UBLOX_REDUCED_PROG_MEM
////            if (_printDebug == true)
////            {
////              _debugSerial->print(F("process: incoming \"automatic\" message: Class: 0x"));
////              _debugSerial->print(packetBuf.cls, HEX);
////              _debugSerial->print(F(" ID: 0x"));
////              _debugSerial->println(packetBuf.id, HEX);
////            }
////#endif
//          }
//        }
//        else
//        {
//          // This is not an ACK and we do not have a class and ID match
//          // so we should keep diverting data into packetBuf and ignore the payload
//          ignoreThisPayload = true;
//        }
//      }
//      else
//      {
//        // This is an ACK so it is to early to do anything with it
//        // We need to wait until we have received the length and data bytes
//        // So we should keep diverting data into packetBuf
//      }
//    }
//    else if (ubxFrameCounter == 4) // Length LSB
//    {
//      // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
//      packetBuf.len = incoming; // (Duplication)
//    }
//    else if (ubxFrameCounter == 5) // Length MSB
//    {
//      // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
//      packetBuf.len |= incoming << 8; // (Duplication)
//    }
//    else if (ubxFrameCounter == 6) // This should be the first byte of the payload unless .len is zero
//    {
//      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
//      {
////#ifndef SFE_UBLOX_REDUCED_PROG_MEM
////        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
////        {
////          _debugSerial->print(F("process: ZERO LENGTH packet received: Class: 0x"));
////          _debugSerial->print(packetBuf.cls, HEX);
////          _debugSerial->print(F(" ID: 0x"));
////          _debugSerial->println(packetBuf.id, HEX);
////        }
////#endif
//        // If length is zero (!) this will be the first byte of the checksum so record it
//        packetBuf.checksumA = incoming;
//      }
//      else
//      {
//        // The length is not zero so record this byte in the payload
//        packetBuf.payload[0] = incoming;
//      }
//    }
//    else if (ubxFrameCounter == 7) // This should be the second byte of the payload unless .len is zero or one
//    {
//      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
//      {
//        // If length is zero (!) this will be the second byte of the checksum so record it
//        packetBuf.checksumB = incoming;
//      }
//      else if (packetBuf.len == 1) // Check if length is one
//      {
//        // The length is one so this is the first byte of the checksum
//        packetBuf.checksumA = incoming;
//      }
//      else // Length is >= 2 so this must be a payload byte
//      {
//        packetBuf.payload[1] = incoming;
//      }
//      // Now that we have received two payload bytes, we can check for a matching ACK/NACK
//      if ((activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF) // If we are not already processing a data packet
//          && (packetBuf.cls == UBX_CLASS_ACK)                // and if this is an ACK/NACK
//          && (packetBuf.payload[0] == requestedClass)        // and if the class matches
//          && (packetBuf.payload[1] == requestedID))          // and if the ID matches
//      {
//        if (packetBuf.len == 2) // Check if .len is 2
//        {
//          // Then this is a matching ACK so copy it into packetAck
//          activePacketBuffer = SFE_UBLOX_PACKET_PACKETACK;
//          packetAck.cls = packetBuf.cls;
//          packetAck.id = packetBuf.id;
//          packetAck.len = packetBuf.len;
//          packetAck.counter = packetBuf.counter;
//          packetAck.payload[0] = packetBuf.payload[0];
//          packetAck.payload[1] = packetBuf.payload[1];
//        }
//        else // Length is not 2 (hopefully this is impossible!)
//        {
////#ifndef SFE_UBLOX_REDUCED_PROG_MEM
////          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
////          {
////            _debugSerial->print(F("process: ACK received with .len != 2: Class: 0x"));
////            _debugSerial->print(packetBuf.payload[0], HEX);
////            _debugSerial->print(F(" ID: 0x"));
////            _debugSerial->print(packetBuf.payload[1], HEX);
////            _debugSerial->print(F(" len: "));
////            _debugSerial->println(packetBuf.len);
////          }
////#endif
//        }
//      }
//    }
//
//    // Divert incoming into the correct buffer
//    if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETACK)
//      processUBX(incoming, &packetAck, requestedClass, requestedID);
//    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
//      processUBX(incoming, incomingUBX, requestedClass, requestedID);
//    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF)
//      processUBX(incoming, &packetBuf, requestedClass, requestedID);
//    else // if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
//      processUBX(incoming, &packetAuto, requestedClass, requestedID);
//
//    // Finally, increment the frame counter
//    ubxFrameCounter++;
//  }
//  else if (currentSentence == NMEA) // Process incoming NMEA mesages. Selectively log if desired.
//  {
//    if ((nmeaByteCounter == 0) && (incoming != '$'))
//    {
//      currentSentence = NONE; // Something went wrong. Reset. (Almost certainly redundant!)
//    }
//    else if ((nmeaByteCounter == 1) && (incoming != 'G'))
//    {
//      currentSentence = NONE; // Something went wrong. Reset.
//    }
//    else if ((nmeaByteCounter >= 0) && (nmeaByteCounter <= 5))
//    {
//      nmeaAddressField[nmeaByteCounter] = incoming; // Store the start character and NMEA address field
//    }
//
//    if (nmeaByteCounter == 5)
//    {
////      if (!_signsOfLife) // If _signsOfLife is not already true, set _signsOfLife to true if the NMEA header is valid
////      {
////        _signsOfLife = isNMEAHeaderValid();
////      }
//
//#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
//      // Check if we have automatic storage for this message
//      if (isThisNMEAauto())
//      {
//        uint8_t *lengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
//        uint8_t *nmeaPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
//        uint8_t nmeaMaxLength = getNMEAMaxLength();
//        *lengthPtr = 6;                           // Set the working copy length
//        memset(nmeaPtr, 0, nmeaMaxLength);        // Clear the working copy
//        memcpy(nmeaPtr, &nmeaAddressField[0], 6); // Copy the start character and address field into the working copy
//      }
////      else
//#endif
////      {
////        // if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
////        // {
////        //   _debugSerial->println(F("process: non-auto NMEA message"));
////        // }
////      }
//
//      // We've just received the end of the address field. Check if it is selected for logging
//      if (logThisNMEA())
//      {
//        storeFileBytes(&nmeaAddressField[0], 6); // Add start character and address field to the file buffer
//      }
//      // Check if it should be passed to processNMEA
//      if (processThisNMEA())
//      {
//        processNMEA(nmeaAddressField[0]); // Process the start character and address field
//        processNMEA(nmeaAddressField[1]);
//        processNMEA(nmeaAddressField[2]);
//        processNMEA(nmeaAddressField[3]);
//        processNMEA(nmeaAddressField[4]);
//        processNMEA(nmeaAddressField[5]);
//      }
//    }
//
//    if ((nmeaByteCounter > 5) || (nmeaByteCounter < 0)) // Should we add incoming to the file buffer and/or pass it to processNMEA?
//    {
//#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
//      if (isThisNMEAauto())
//      {
//        uint8_t *lengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
//        uint8_t *nmeaPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
//        uint8_t nmeaMaxLength = getNMEAMaxLength();
//        if (*lengthPtr < nmeaMaxLength)
//        {
//          *(nmeaPtr + *lengthPtr) = incoming; // Store the character
//          *lengthPtr = *lengthPtr + 1;        // Increment the length
//          if (*lengthPtr == nmeaMaxLength)
//          {
//          }
//        }
//      }
//#endif
//      if (logThisNMEA())
//        storeFileBytes(&incoming, 1); // Add incoming to the file buffer
//      if (processThisNMEA())
//        processNMEA(incoming); // Pass incoming to processNMEA
//    }
//
//    if (incoming == '*')
//      nmeaByteCounter = -5; // We are expecting * plus two checksum bytes plus CR and LF
//
//    nmeaByteCounter++; // Increment the byte counter
//
//    if (nmeaByteCounter == maxNMEAByteCount) // Check if we have processed too many bytes
//      currentSentence = NONE;                // Something went wrong. Reset.
//
//    if (nmeaByteCounter == 0) // Check if we are done
//    {
//#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
//      if (isThisNMEAauto())
//      {
//        uint8_t *workingLengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
//        uint8_t *workingNMEAPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
//        uint8_t nmeaMaxLength = getNMEAMaxLength();
//
//        // Check the checksum: the checksum is the exclusive-OR of all characters between the $ and the *
//        uint8_t nmeaChecksum = 0;
//        uint8_t charsChecked = 1; // Start after the $
//        uint8_t thisChar = '\0';
//        while ((charsChecked < (nmeaMaxLength - 1)) && (charsChecked < ((*workingLengthPtr) - 4)) && (thisChar != '*'))
//        {
//          thisChar = *(workingNMEAPtr + charsChecked); // Get a char from the working copy
//          if (thisChar != '*')                         // Ex-or the char into the checksum - but not if it is the '*'
//            nmeaChecksum ^= thisChar;
//          charsChecked++; // Increment the counter
//        }
//        if (thisChar == '*') // Make sure we found the *
//        {
//          uint8_t expectedChecksum1 = (nmeaChecksum >> 4) + '0';
//          if (expectedChecksum1 >= ':') // Handle Hex correctly
//            expectedChecksum1 += 'A' - ':';
//          uint8_t expectedChecksum2 = (nmeaChecksum & 0x0F) + '0';
//          if (expectedChecksum2 >= ':') // Handle Hex correctly
//            expectedChecksum2 += 'A' - ':';
//          if ((expectedChecksum1 == *(workingNMEAPtr + charsChecked)) && (expectedChecksum2 == *(workingNMEAPtr + charsChecked + 1)))
//          {
//            uint8_t *completeLengthPtr = getNMEACompleteLengthPtr();    // Get a pointer to the complete copy length
//            uint8_t *completeNMEAPtr = getNMEACompleteNMEAPtr();        // Get a pointer to the complete copy NMEA data
//            memset(completeNMEAPtr, 0, nmeaMaxLength);                  // Clear the previous complete copy
//            memcpy(completeNMEAPtr, workingNMEAPtr, *workingLengthPtr); // Copy the working copy into the complete copy
//            *completeLengthPtr = *workingLengthPtr;                     // Update the length
//            nmeaAutomaticFlags *flagsPtr = getNMEAFlagsPtr();           // Get a pointer to the flags
//            nmeaAutomaticFlags flagsCopy = *flagsPtr;
//            flagsCopy.flags.bits.completeCopyValid = 1; // Set the complete copy valid flag
//            flagsCopy.flags.bits.completeCopyRead = 0;  // Clear the complete copy read flag
//            *flagsPtr = flagsCopy;                      // Update the flags
//            // Callback
//            if (doesThisNMEAHaveCallback()) // Do we need to copy the data into the callback copy?
//            {
//              if (flagsCopy.flags.bits.callbackCopyValid == 0) // Has the callback copy valid flag been cleared (by checkCallbacks)
//              {
//                uint8_t *callbackLengthPtr = getNMEACallbackLengthPtr();    // Get a pointer to the callback copy length
//                uint8_t *callbackNMEAPtr = getNMEACallbackNMEAPtr();        // Get a pointer to the callback copy NMEA data
//                memset(callbackNMEAPtr, 0, nmeaMaxLength);                  // Clear the previous callback copy
//                memcpy(callbackNMEAPtr, workingNMEAPtr, *workingLengthPtr); // Copy the working copy into the callback copy
//                *callbackLengthPtr = *workingLengthPtr;                     // Update the length
//                flagsCopy.flags.bits.callbackCopyValid = 1;                 // Set the callback copy valid flag
//                *flagsPtr = flagsCopy;                                      // Update the flags
//              }
//            }
//          }
//        }
//      }
//#endif
//      currentSentence = NONE; // All done!
//    }
//  }
//  else if (currentSentence == RTCM)
//  {
//    processRTCMframe(incoming); // Deal with RTCM bytes
//  }
//}
//
//// PRIVATE: Check if we have storage allocated for an incoming "automatic" message
//bool checkAutomatic(uint8_t Class, uint8_t ID)
//{
//  bool result = false;
//  switch (Class)
//  {
//  case UBX_CLASS_NAV:
//  {
//    switch (ID)
//    {
//    case UBX_NAV_POSECEF:
//      if (packetUBXNAVPOSECEF != NULL)
//        result = true;
//      break;
//    case UBX_NAV_STATUS:
//      if (packetUBXNAVSTATUS != NULL)
//        result = true;
//      break;
//    case UBX_NAV_DOP:
//      if (packetUBXNAVDOP != NULL)
//        result = true;
//      break;
//    case UBX_NAV_ATT:
//      if (packetUBXNAVATT != NULL)
//        result = true;
//      break;
//    case UBX_NAV_PVT:
//      if (packetUBXNAVPVT != NULL)
//        result = true;
//      break;
//    case UBX_NAV_ODO:
//      if (packetUBXNAVODO != NULL)
//        result = true;
//      break;
//    case UBX_NAV_VELECEF:
//      if (packetUBXNAVVELECEF != NULL)
//        result = true;
//      break;
//    case UBX_NAV_VELNED:
//      if (packetUBXNAVVELNED != NULL)
//        result = true;
//      break;
//    case UBX_NAV_HPPOSECEF:
//      if (packetUBXNAVHPPOSECEF != NULL)
//        result = true;
//      break;
//    case UBX_NAV_HPPOSLLH:
//      if (packetUBXNAVHPPOSLLH != NULL)
//        result = true;
//      break;
//    case UBX_NAV_PVAT:
//      if (packetUBXNAVPVAT != NULL)
//        result = true;
//      break;
//    case UBX_NAV_CLOCK:
//    	if (packetUBXNAVCLOCK != NULL)
//        result = true;
//      break;
//    case UBX_NAV_TIMELS:
//      if (packetUBXNAVTIMELS != NULL)
//        result = true;
//      break;
//    case UBX_NAV_SVIN:
//      if (packetUBXNAVSVIN != NULL)
//        result = true;
//      break;
//    case UBX_NAV_SAT:
//      if (packetUBXNAVSAT != NULL)
//        result = true;
//      break;
//    case UBX_NAV_RELPOSNED:
//      if (packetUBXNAVRELPOSNED != NULL)
//        result = true;
//      break;
//    case UBX_NAV_AOPSTATUS:
//      if (packetUBXNAVAOPSTATUS != NULL)
//        result = true;
//      break;
//    }
//  }
//  break;
//  case UBX_CLASS_RXM:
//  {
//    switch (ID)
//    {
//    case UBX_RXM_SFRBX:
//      if (packetUBXRXMSFRBX != NULL)
//        result = true;
//      break;
//    case UBX_RXM_RAWX:
//      if (packetUBXRXMRAWX != NULL)
//        result = true;
//      break;
//    case UBX_RXM_PMP:
//      if ((packetUBXRXMPMP != NULL) || (packetUBXRXMPMPmessage != NULL))
//        result = true;
//      break;
//    case UBX_RXM_COR:
//      if (packetUBXRXMCOR != NULL)
//        result = true;
//      break;
//    }
//  }
//  break;
//  case UBX_CLASS_CFG:
//  {
//    switch (ID)
//    {
//    case UBX_CFG_PRT:
//      if (packetUBXCFGPRT != NULL)
//        result = true;
//      break;
//    case UBX_CFG_RATE:
//      if (packetUBXCFGRATE != NULL)
//        result = true;
//      break;
//    }
//  }
//  break;
//  case UBX_CLASS_TIM:
//  {
//    switch (ID)
//    {
//    case UBX_TIM_TM2:
//      if (packetUBXTIMTM2 != NULL)
//        result = true;
//      break;
//    }
//  }
//  break;
//  case UBX_CLASS_ESF:
//  {
//    switch (ID)
//    {
//    case UBX_ESF_ALG:
//      if (packetUBXESFALG != NULL)
//        result = true;
//      break;
//    case UBX_ESF_INS:
//      if (packetUBXESFINS != NULL)
//        result = true;
//      break;
//    case UBX_ESF_MEAS:
//      if (packetUBXESFMEAS != NULL)
//        result = true;
//      break;
//    case UBX_ESF_RAW:
//      if (packetUBXESFRAW != NULL)
//        result = true;
//      break;
//    case UBX_ESF_STATUS:
//      if (packetUBXESFSTATUS != NULL)
//        result = true;
//      break;
//    }
//  }
//  break;
//  case UBX_CLASS_MGA:
//  {
//    switch (ID)
//    {
//    case UBX_MGA_ACK_DATA0:
//      if (packetUBXMGAACK != NULL)
//        result = true;
//      break;
//    case UBX_MGA_DBD:
//      if (packetUBXMGADBD != NULL)
//        result = true;
//      break;
//    }
//  }
//  break;
//  case UBX_CLASS_HNR:
//  {
//    switch (ID)
//    {
//    case UBX_HNR_PVT:
//      if (packetUBXHNRPVT != NULL)
//        result = true;
//      break;
//    case UBX_HNR_ATT:
//      if (packetUBXHNRATT != NULL)
//        result = true;
//      break;
//    case UBX_HNR_INS:
//      if (packetUBXHNRINS != NULL)
//        result = true;
//      break;
//    }
//  }
//  break;
//  }
//  return (result);
//}
//
//// PRIVATE: Calculate how much RAM is needed to store the payload for a given automatic message
//uint16_t getMaxPayloadSize(uint8_t Class, uint8_t ID)
//{
//  uint16_t maxSize = 0;
//  if (Class == UBX_CLASS_NAV)
//  {
//      if (ID == UBX_NAV_POSECEF)
//          maxSize = UBX_NAV_POSECEF_LEN;
//      else if (ID == UBX_NAV_STATUS)
//          maxSize = UBX_NAV_STATUS_LEN;
//      else if (ID == UBX_NAV_DOP)
//          maxSize = UBX_NAV_DOP_LEN;
//      else if (ID == UBX_NAV_ATT)
//          maxSize = UBX_NAV_ATT_LEN;
//      else if (ID == UBX_NAV_PVT)
//          maxSize = UBX_NAV_PVT_LEN;
//      else if (ID == UBX_NAV_ODO)
//          maxSize = UBX_NAV_ODO_LEN;
//      else if (ID == UBX_NAV_VELECEF)
//          maxSize = UBX_NAV_VELECEF_LEN;
//      else if (ID == UBX_NAV_VELNED)
//          maxSize = UBX_NAV_VELNED_LEN;
//      else if (ID == UBX_NAV_HPPOSECEF)
//          maxSize = UBX_NAV_HPPOSECEF_LEN;
//      else if (ID == UBX_NAV_HPPOSLLH)
//          maxSize = UBX_NAV_HPPOSLLH_LEN;
//      else if (ID == UBX_NAV_PVAT)
//          maxSize = UBX_NAV_PVAT_LEN;
//      else if (ID == UBX_NAV_CLOCK)
//          maxSize = UBX_NAV_CLOCK_LEN;
//      else if (ID == UBX_NAV_TIMELS)
//          maxSize = UBX_NAV_TIMELS_LEN;
//      else if (ID == UBX_NAV_SVIN)
//          maxSize = UBX_NAV_SVIN_LEN;
//      else if (ID == UBX_NAV_SAT)
//          maxSize = UBX_NAV_SAT_MAX_LEN;
//      else if (ID == UBX_NAV_RELPOSNED)
//          maxSize = UBX_NAV_RELPOSNED_LEN_F9;
//      else if (ID == UBX_NAV_AOPSTATUS)
//          maxSize = UBX_NAV_AOPSTATUS_LEN;
//      else
//          maxSize = 0;
//  }
//  else
//      maxSize = 0;
//
//  return (maxSize);
//}
//
//sfe_ublox_status_e waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
//{
//  outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
//  packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
//  packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
//  packetAuto.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
//  outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
//  packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
//  packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
//  packetAuto.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
//
//  unsigned long startTime = millis();
//  while (millis() < (startTime + (unsigned long)maxTime))
//  {
//    if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
//    {
//      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
//      // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
//      // then we can be confident that the data in outgoingUBX is valid
//      if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
//      {
//        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and a correct ACK!
//      }
//
//      // We can be confident that the data packet (if we are going to get one) will always arrive
//      // before the matching ACK. So if we sent a config packet which only produces an ACK
//      // then outgoingUBX->classAndIDmatch will be NOT_DEFINED and the packetAck.classAndIDmatch will VALID.
//      // We should not check outgoingUBX->valid, outgoingUBX->cls or outgoingUBX->id
//      // as these may have been changed by an automatic packet.
//      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID))
//      {
//        return (SFE_UBLOX_STATUS_DATA_SENT); // We got an ACK but no data...
//      }
//
//      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
//      // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
//      // valid data but it has been or is currently being overwritten by an automatic packet (e.g. PVT).
//      // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
//      // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
//      // So we cannot use outgoingUBX->valid as part of this check.
//      // Note: the addition of packetBuf should make this check redundant!
//      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
//      {
//        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
//      }
//
//      // If packetAck.classAndIDmatch is VALID but both outgoingUBX->valid and outgoingUBX->classAndIDmatch
//      // are NOT_VALID then we can be confident we have had a checksum failure on the data packet
//      else if ((packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
//      {
//        return (SFE_UBLOX_STATUS_CRC_FAIL); // Checksum fail
//      }
//
//      // If our packet was not-acknowledged (NACK) we do not receive a data packet - we only get the NACK.
//      // So you would expect outgoingUBX->valid and outgoingUBX->classAndIDmatch to still be NOT_DEFINED
//      // But if a full PVT packet arrives afterwards outgoingUBX->valid could be VALID (or just possibly NOT_VALID)
//      // but outgoingUBX->cls and outgoingUBX->id would not match...
//      // So I think this is telling us we need a special state for packetAck.classAndIDmatch to tell us
//      // the packet was definitely NACK'd otherwise we are possibly just guessing...
//      // Note: the addition of packetBuf changes the logic of this, but we'll leave the code as is for now.
//      else if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_NOTACKNOWLEDGED)
//      {
//        return (SFE_UBLOX_STATUS_COMMAND_NACK); // We received a NACK!
//      }
//
//      // If the outgoingUBX->classAndIDmatch is VALID but the packetAck.classAndIDmatch is NOT_VALID
//      // then the ack probably had a checksum error. We will take a gamble and return DATA_RECEIVED.
//      // If we were playing safe, we should return FAIL instead
//      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
//      {
//        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and an invalid ACK!
//      }
//
//      // If the outgoingUBX->classAndIDmatch is NOT_VALID and the packetAck.classAndIDmatch is NOT_VALID
//      // then we return a FAIL. This must be a double checksum failure?
//      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
//      {
//        return (SFE_UBLOX_STATUS_FAIL); // We received invalid data and an invalid ACK!
//      }
//
//      // If the outgoingUBX->classAndIDmatch is VALID and the packetAck.classAndIDmatch is NOT_DEFINED
//      // then the ACK has not yet been received and we should keep waiting for it
//      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED))
//      {
//        // if (_printDebug == true)
//        // {
//        //   _debugSerial->print(F("waitForACKResponse: valid data after "));
//        //   _debugSerial->print(millis() - startTime);
//        //   _debugSerial->println(F(" msec. Waiting for ACK."));
//        // }
//      }
//
//    } // checkUbloxInternal == true
//
//    delay(1); // Allow an RTOS to get an elbow in (#11)
//  }           // while (millis() < (startTime + (unsigned long)maxTime))
//
//  // We have timed out...
//  // If the outgoingUBX->classAndIDmatch is VALID then we can take a gamble and return DATA_RECEIVED
//  // even though we did not get an ACK
//  if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
//  {
//    return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data... But no ACK!
//  }
//
//
//  return (SFE_UBLOX_STATUS_TIMEOUT);
//}
