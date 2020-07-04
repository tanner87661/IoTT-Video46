void sendSensor(uint16_t Address, uint8_t State)
{
  uint8_t d1 = (Address & 0xFE)>>1;
  uint8_t d2 = ((Address & 0x0F00)>>8) | ((Address & 0x01)<<5) | ((State & 0x01)<<4);
  printf("LN: %i %i %i\n", 0xB2, d1, d2);
  LocoNet.send(0xB2, d1, d2);
}

void sendSwitch(uint16_t Address, uint8_t Position, uint8_t State)
{
  Address -= 1; //subtract the DCC Ofset
  uint8_t d1 = (Address & 0x7F);
  uint8_t d2 = ((Address & 0x0780)>>7) | ((Position & 0x01)<<5) | ((State & 0x01)<<4);
  printf("LN Swi: %i %i %i\n", 0xB0, d1, d2);
  LocoNet.send(0xB0, d1, d2);
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Sensor messages
void notifySensor( uint16_t Address, uint8_t State ) {
  Serial.print("Sensor: ");
  Serial.print(Address, DEC);
  Serial.print(" - ");
  Serial.println( State ? "Active" : "Inactive" );
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Request messages
void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction ) {
  Serial.print("Switch Request: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
  if (Address == polSwi)
  {
      digitalWrite(relPin1, Direction);
      digitalWrite(relPin2, Direction);
  }
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Output Report messages
void notifySwitchOutputsReport( uint16_t Address, uint8_t ClosedOutput, uint8_t ThrownOutput) {
  Serial.print("Switch Outputs Report: ");
  Serial.print(Address, DEC);
  Serial.print(": Closed - ");
  Serial.print(ClosedOutput ? "On" : "Off");
  Serial.print(": Thrown - ");
  Serial.println(ThrownOutput ? "On" : "Off");
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Sensor Report messages
void notifySwitchReport( uint16_t Address, uint8_t State, uint8_t Sensor ) {
  Serial.print("Switch Sensor Report: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Sensor ? "Switch" : "Aux");
  Serial.print(" - ");
  Serial.println( State ? "Active" : "Inactive" );
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch State messages
void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction ) {
  Serial.print("Switch State: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
}
