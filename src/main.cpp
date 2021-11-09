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

// const uint8_t FRAME_LEN = 8;

// void send_cmd(std::vector<uint8_t> cmd)
// {
//   //TODO: getpos its the same msg every time?
//   int data_len = (int)cmd.size();
//   int frame_num = 0;
//   int full_frame_num = data_len / FRAME_LEN;
//   int left_len = data_len % FRAME_LEN;

//   if (left_len == 0)
//     frame_num = full_frame_num;
//   else
//     frame_num = full_frame_num + 1;

//   CAN_message_t *send_buf = new CAN_message_t[frame_num];

//   int data_offset = 0;
//   for (int i = 0; i < (int)(full_frame_num); i++)
//   {
//     send_buf[i].id = 0x223;
//     send_buf[i].flags.extended = 0;
//     send_buf[i].len = FRAME_LEN;

//     for (int j = 0; j < FRAME_LEN; j++)
//     {
//       send_buf[i].buf[j] = cmd[data_offset + j];
//     }
//     data_offset += FRAME_LEN;
//   }

//   if (left_len > 0)
//   {
//     send_buf[frame_num - 1].id = 0x223;
//     send_buf[frame_num - 1].flags.extended = 0;
//     send_buf[frame_num - 1].len = left_len;

//     for (int j = 0; j < left_len; j++)
//       send_buf[frame_num - 1].buf[j] = cmd[data_offset + j];
//   }

//   CAN_message_t inMsg;
//   inMsg.buf[0] = 1;

//   for (int k = 0; k < frame_num; k++)
//   {
//     Can0.write(send_buf[k]);
//     for (int l = 0; l < send_buf[k].len; l++)
//       Serial.printf("0x%0X ", send_buf[k].buf[l]);
//     Serial.println();
//   }
// }

void loop(void)
{

  // std::vector<uint8_t> cmd = dji.move_to(heading.yaw, heading.roll, heading.pitch, 100);
  // send_cmd(cmd);
  dji.update();

  static int32_t timeout4 = millis();
  if (millis() - timeout4 > 15)
  {
    if (dji.get_current_position(heading.yaw, heading.roll, heading.pitch))
    {
      // Serial.println("got");
      // Serial.printf("Y %d, R %d, P %d\n", heading.yaw, heading.roll, heading.pitch);
      Udp.beginPacket(IPAddress(172, 16, 3, 39), 7000);
      Udp.write(heading.array, 6);
      Udp.endPacket();
    }
    // Serial.println("test");
    // dji.move_to(heading.yaw, heading.roll, heading.pitch, 100);
    timeout4 = millis();
  }

  if (Udp.parsePacket())
  {
    Udp.read(target_heading.array, 6);
    // Serial.printf("Y %d, R %d, P %d\n", heading.yaw, heading.roll, heading.pitch);
    dji.move_to(target_heading.yaw, target_heading.roll, target_heading.pitch, 1);

  }

  // Serial.println("after delay");
}
