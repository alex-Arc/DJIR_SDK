#include <Arduino.h>

/*-----------------*/
//  ETHERNET
/*-----------------*/
// #include <NativeEthernet.h>
// #include <NativeEthernetUdp.h>
#include <QNEthernet.h>
#include <QNEthernetUDP.h>
using namespace qindesign::network;
constexpr uint32_t kDHCPTimeout = 10000; // 10 seconds
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(172, 16, 3, 20);

unsigned int localPort = 8888; // local port to listen on
EthernetUDP Udp;

/*-----------------*/
//  CAN BUS
/*-----------------*/
// #include <FlexCAN_T4.h>
// FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

#include <DJIR_SDK.h>

DJIR_SDK::DJIRonin dji;

struct yrp_t
{
  int16_t yaw;
  int16_t roll;
  int16_t pitch;
};

union heading_t
{
  uint8_t array[6];
  struct
  {
    int16_t yaw;
    int16_t roll;
    int16_t pitch;
  };
};

heading_t heading;
heading_t target_heading;

void setup()
{
  Serial.begin(115200);
  delay(400);
  if (CrashReport)
  {
    Serial.print(CrashReport);
  }
  // start the Ethernet
  // Unlike the Arduino API (which you can still use), Ethernet uses
  // the Teensy's internal MAC address by default
  uint8_t mac[6];
  Ethernet.macAddress(mac);
  Serial.printf("MAC = %02x:%02x:%02x:%02x:%02x:%02x\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.printf("Starting Ethernet with DHCP...\n");
  if (!Ethernet.begin())
  {
    Serial.printf("Failed to start Ethernet\n");
    return;
  }
  if (!Ethernet.waitForLocalIP(kDHCPTimeout))
  {
    Serial.printf("Failed to get IP address from DHCP\n");
  }
  else
  {
    IPAddress ip = Ethernet.localIP();
    Serial.printf("    Local IP    = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.subnetMask();
    Serial.printf("    Subnet mask = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.gatewayIP();
    Serial.printf("    Gateway     = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.dnsServerIP();
    Serial.printf("    DNS         = %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);

    Udp.begin(7001);
  }

  Serial.println("-------------START-------------------");
  // Check for Ethernet hardware present
  // if (Ethernet.hardwareStatus() == EthernetNoHardware)
  // {
  //   Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
  //   while (true)
  //   {
  //     delay(1); // do nothing, no point running without Ethernet hardware
  //   }
  // }
  // if (Ethernet.linkStatus() == LinkOFF)
  // {
  //   Serial.println("Ethernet cable is not connected.");
  // }

  dji.connect();
  // dji.set_move_mode(DJIR_SDK::MoveMode::ABSOLUTE_CONTROL);
}

void loop(void)
{

  // std::vector<uint8_t> cmd = dji.move_to(heading.yaw, heading.roll, heading.pitch, 100);
  // send_cmd(cmd);
  dji.update();

  static int32_t timeout4 = millis();
  if (millis() - timeout4 > 15)
  {
    // int32_t t = micros();
    if (dji.get_current_position(heading.yaw, heading.roll, heading.pitch))
    {
    //   // t = micros() - t;
    //   // Serial.println("got");
      // Serial.printf("Y %d, R %d, P %d\n", heading.yaw, heading.roll, heading.pitch);
      Udp.beginPacket(IPAddress(172, 16, 3, 59), 7000);
      Udp.write(heading.array, 6);
      Udp.endPacket();
    }
    // Serial.printf("get_current_position micro sec %d\n", t);
    // Serial.println("test");
    // dji.move_to(heading.yaw, heading.roll, heading.pitch, 100);
    timeout4 = millis();
  }
  // if (Udp.parsePacket())
  // {
  //   Udp.read(target_heading.array, 6);
  //   // Serial.printf("Y %d, R %d, P %d\n", heading.yaw, heading.roll, heading.pitch);
  //   // dji.move_to(target_heading.yaw, target_heading.roll, target_heading.pitch, 1);
  // }

  // Serial.println("after delay");
}
