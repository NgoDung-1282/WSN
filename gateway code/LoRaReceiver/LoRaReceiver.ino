#include <SPI.h>
#include <LoRa.h>
#define ss 5
#define rst 14
#define dio0 17

void setup() {
  Serial.begin(9600);
  while (!Serial);

Serial.println("LoRa Receiver");
LoRa.setPins(ss, rst, dio0); // setup LoRa transceiver module
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.setTxPower(17);
  LoRa.setSyncWord(0x34);
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
