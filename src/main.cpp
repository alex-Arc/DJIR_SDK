#include <Arduino.h>
#include <DJIR_SDK.h>

/*-----------------*/
//  ETHERNET
/*-----------------*/
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(172, 16, 3, 20);

unsigned int localPort = 8888; // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";       // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

/*-----------------*/
//  CAN BUS
/*-----------------*/
#include <FlexCAN_T4.h>
#include <isotp.h>
#include <CmdCombine.h>

#include "custom_crc16.h"
#include "custom_crc32.h"

isotp<RX_BANKS_16, 512> tp; /* 16 slots for multi-ID support, at 512bytes buffer each payload rebuild */
FlexCAN_T4<CAN2, RX_SIZE_512, TX_SIZE_512> Can0;

int16_t _yaw;
int16_t _roll;
int16_t _pitch;
std::vector<std::vector<uint8_t>> _cmd_list;

bool RXpacket = false;
bool TXpacket = false;

bool get = true;

bool _check_head_crc(std::vector<uint8_t> data)
{
  crc16_t crc16;
  crc16 = crc16_init();
  crc16 = crc16_update(crc16, data.data(), 10);
  crc16 = crc16_finalize(crc16);

  uint16_t recv_crc = (*(uint16_t *)&data.data()[data.size() - 2]);

  if (crc16 == recv_crc)
    return true;
  return false;
}

bool _check_pack_crc(std::vector<uint8_t> data)
{
  crc32_t crc32;
  crc32 = crc32_init();
  crc32 = crc32_update(crc32, data.data(), data.size() - 4);
  crc32 = crc32_finalize(crc32);

  uint32_t recv_crc = (*(uint32_t *)&data.data()[data.size() - 4]);

  if (crc32 == recv_crc)
    return true;
  return false;
}

void _process_cmd(std::vector<uint8_t> data)
{
  uint8_t cmd_type = (uint8_t)data[3];
  bool is_ok = false;
  uint8_t cmd_key[2] = {0, 0};

  // if (RXpacket)
  //   Serial.print("RX: ");

  // If it is a response frame, need to check the corresponding send command
  if (cmd_type == 0x20)
  {
    cmd_key[0] = (uint8_t)data[12];
    cmd_key[1] = (uint8_t)data[13];
    is_ok = true;
    // for (size_t i = 0; i < _cmd_list.size(); i++)
    // {
    //   std::vector<uint8_t> cmd = _cmd_list[i];
    //   if (cmd.size() >= 10)
    //   {
    //     uint16_t last_cmd_crc = *((uint16_t *)&cmd.data()[8]);
    //     uint16_t data_crc = *((uint16_t *)&data.data()[8]);
    //     if (last_cmd_crc == data_crc)
    //     {
    //       cmd_key[0] = (uint8_t)cmd[12];
    //       cmd_key[1] = (uint8_t)cmd[13];
    //       _cmd_list.erase(_cmd_list.begin() + i);
    //       is_ok = true;
    //       break;
    //     }
    //   }
    // }
  }
  else
  {
    cmd_key[0] = (uint8_t)data[12];
    cmd_key[1] = (uint8_t)data[13];
    is_ok = true;
  }

  if (is_ok)
  {
    // Serial.printf("CMD type: 0x%0X ", cmd_type);
    switch (*(uint16_t *)&cmd_key[0])
    {
    case 0x000E:
    {
      // Serial.printf("Control posControl request\t\t");
      _yaw = *(int16_t *)&data[14];
      _roll = *(int16_t *)&data[16];
      _pitch = *(int16_t *)&data[18];
      uint8_t _time = data[21];
      // Serial.printf("yaw %f roll %f pitch %f time %f \n", _yaw * 0.1, _roll * 0.1, _pitch * 0.1, _time * 0.1);

      // for (int i = 14; i < 20; i++)
      //   Serial.printf("0x%0X ", data[i]);
      // Serial.println();
      break;
    }
    case 0x010E:
    {
      Serial.printf("control Gimbal speed\n");
      break;
    }
    case 0x020E:
    {
      _yaw = *(int16_t *)&data[14];
      _roll = *(int16_t *)&data[16];
      _pitch = *(int16_t *)&data[18];

      // Serial.printf("Get Gimbal angle\t");

      if (RXpacket)
      {
        //   if (data[13] == 0x00)
        //     Serial.printf("Data is not ready \t");

        //   if (data[13] == 0x01)
        //     Serial.printf("The current angle is attitude angle\t");

        //   if (data[13] == 0x02)
        //     Serial.printf("The current angle is joint angle\t");

        _yaw = *(int16_t *)&data.data()[16];
        _roll = *(int16_t *)&data.data()[18];
        _pitch = *(int16_t *)&data.data()[20];

        // Serial.printf(" yaw %f roll %f pitch %f \n", _yaw * 0.1, _roll * 0.1, _pitch * 0.1);
        // Serial.printf("%d;%d;%d\n", _yaw, _roll, _pitch);
        // Serial.printf("%f;%f;%f\n", _yaw * 0.1, _roll * 0.1, _pitch * 0.1);
        // send a reply to the IP address and port that sent us the packet we received
        uint8_t packdata[6];
        memcpy(packdata, &data[16], 6);
        Udp.beginPacket(IPAddress(172, 16, 3, 39), 7000);
        Udp.write(packdata, 6);
        Udp.endPacket();

        // usbMIDI.sendSysEx(6, &data[16]);
        get = true;

        // for (int i = 16; i < data.size() - 4; i++)
        //   Serial.printf("%d ", data[i]);
        // Serial.println();
      }
      break;
    }
    case 0x120E:
    {
      // Serial.printf("set Focus Motor CMD");
      break;
    }
    default:
    {
      Serial.printf("get unknown request\n");
      Serial.printf("CmdType 0x%0X\n", cmd_type);
      Serial.printf("CmdKey 0x%0X 0x%0X\n", cmd_key[0], cmd_key[1]);
      for (uint8_t i = 0; i < data.size(); i++)
      {
        Serial.printf("0x%0X ", data[i]);
      }
      Serial.println();
      break;
    }
    }
  }
  RXpacket = false;
  TXpacket = false;
}

std::vector<uint8_t> v1_pack_list = std::vector<uint8_t>();
size_t pack_len = 0;
int step = 0;
std::string canid_raw_str = "";
std::string canid_str = "";

void canSniff(const CAN_message_t &msg)
{
  // Serial.printf("ID: 0x%X \n", msg.id);
  if (msg.id == 0x222 || msg.id == 0x223)
  {
    // Serial.println("\n-------------------------------------------------");
    for (size_t i = 0; i < msg.len; i++)
    {
      if (step == 0)
      {
        if ((uint8_t)msg.buf[i] == 0xAA)
        // if ((uint8_t)msg.buf[i] == 0x55)
        {
          if (msg.id == 0x222)
          {
            RXpacket = true;
          }
          if (msg.id == 0x223)
          {
            TXpacket = true;
          }
          v1_pack_list.push_back(msg.buf[i]);
          step = 1;
        }
      }
      else if (step == 1)
      {
        pack_len = int(msg.buf[i]);
        v1_pack_list.push_back(msg.buf[i]);
        step = 2;
      }
      else if (step == 2)
      {
        pack_len |= ((int(msg.buf[i]) & 0x3) << 8);
        v1_pack_list.push_back(msg.buf[i]);
        step = 3;
      }
      else if (step == 3)
      {
        v1_pack_list.push_back(msg.buf[i]);
        if (v1_pack_list.size() == 12)
        {
          if (_check_head_crc(v1_pack_list))
          {
            step = 4;
          }
          else
          {
            Serial.println("_check_head_crc FAIL");
            step = 0;
            v1_pack_list.clear();
          }
        }
      }
      else if (step == 4)
      {
        v1_pack_list.push_back(msg.buf[i]);
        if (v1_pack_list.size() == pack_len)
        {
          step = 0;
          if (_check_pack_crc(v1_pack_list))
            _process_cmd(v1_pack_list);
          else
            Serial.println("_check_pack_crc FAIL");
          v1_pack_list.clear();
        }
      }
      else
      {
        step = 0;
        v1_pack_list.clear();
      }
    }

    // Serial.print("MB ");
    // Serial.print(msg.mb);
    // Serial.print("  OVERRUN: ");
    // Serial.print(msg.flags.overrun);
    // Serial.print("  LEN: ");
    // Serial.print(msg.len);
    // Serial.print(" EXT: ");
    // Serial.print(msg.flags.extended);
    // Serial.print(" TS: ");
    // Serial.print(msg.timestamp);
    // Serial.print(" ID: ");
    // Serial.print(msg.id, HEX);
    // Serial.print(" Buffer: ");
  }
}

void *_can_conn;
void *_pack_thread;
uint8_t _position_ctrl_byte;
uint8_t _speed_ctrl_byte;
void *_cmd_cmb;

void setup()
{
  // start the Ethernet
  Ethernet.begin(mac, ip);
  Serial.begin(115200);
  delay(400);
  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
  Can0.mailboxStatus();

  tp.begin();
  tp.setWriteBus(&Can0); /* we write to this bus */

  _position_ctrl_byte = 0;
  _speed_ctrl_byte = 0;

  _position_ctrl_byte |= BIT1; //MoveMode - ABSOLUTE_CONTROL
  _speed_ctrl_byte |= BIT3;    //SpeedControl - DISABLED, FocalControl - DISABLED
  _cmd_cmb = new CmdCombine();

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true)
    {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF)
  {
    Serial.println("Ethernet cable is not connected.");
  }
  // start UDP
  Udp.begin(localPort);
}

int16_t yaw = 10;
int16_t roll = 10;
int16_t pitch = 10;
uint16_t time_ms = 100;

#define FRAME_LEN 8

void loop(void)
{
  Can0.events();

  static uint32_t timeout = millis();
  if (millis() - timeout > 10)
  {
    uint8_t cmd_type = 0x03;
    uint8_t cmd_set = 0x0E;
    uint8_t cmd_id = 0x02;

    std::vector<uint8_t> data_payload = {0x01};
    auto cmd = ((CmdCombine *)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);

    int data_len = (int)cmd.size();
    int frame_num = 0;
    int full_frame_num = data_len / FRAME_LEN;
    int left_len = data_len % FRAME_LEN;

    if (left_len == 0)
      frame_num = full_frame_num;
    else
      frame_num = full_frame_num + 1;

    CAN_message_t *send_buf = new CAN_message_t[frame_num];

    int data_offset = 0;
    for (int i = 0; i < (int)(full_frame_num); i++)
    {
      send_buf[i].id = 0x223;
      send_buf[i].flags.extended = 0;
      send_buf[i].len = FRAME_LEN;

      for (int j = 0; j < FRAME_LEN; j++)
      {
        send_buf[i].buf[j] = cmd[data_offset + j];
      }
      data_offset += FRAME_LEN;
    }

    if (left_len > 0)
    {
      send_buf[frame_num - 1].id = 0x223;
      send_buf[frame_num - 1].flags.extended = 0;
      send_buf[frame_num - 1].len = left_len;

      for (int j = 0; j < left_len; j++)
        send_buf[frame_num - 1].buf[j] = cmd[data_offset + j];
    }

    for (int k = 0; k < frame_num; k++)
      Can0.write(send_buf[k]);

    timeout = millis();
  }
}
