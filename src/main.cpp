#include <Arduino.h>
// #include <DJIR_SDK.h>

#include <FlexCAN_T4.h>
#include <isotp.h>
#include <CmdCombine.h>

#include "custom_crc16.h"
#include "custom_crc32.h"

isotp<RX_BANKS_16, 512> tp; /* 16 slots for multi-ID support, at 512bytes buffer each payload rebuild */
FlexCAN_T4<CAN1, RX_SIZE_512, TX_SIZE_512> Can0;

int16_t _yaw;
int16_t _roll;
int16_t _pitch;
std::vector<std::vector<uint8_t>> _cmd_list;

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

  // If it is a response frame, need to check the corresponding send command
  if (cmd_type == 0x20)
  {
    for (size_t i = 0; i < _cmd_list.size(); i++)
    {
      std::vector<uint8_t> cmd = _cmd_list[i];
      if (cmd.size() >= 10)
      {
        uint16_t last_cmd_crc = *((uint16_t *)&cmd.data()[8]);
        uint16_t data_crc = *((uint16_t *)&data.data()[8]);
        if (last_cmd_crc == data_crc)
        {
          cmd_key[0] = (uint8_t)cmd[12];
          cmd_key[1] = (uint8_t)cmd[13];
          _cmd_list.erase(_cmd_list.begin() + i);
          is_ok = true;
          break;
        }
      }
    }
  }
  else
  {
    cmd_key[0] = (uint8_t)data[12];
    cmd_key[1] = (uint8_t)data[13];
    is_ok = true;
  }

  if (is_ok)
  {
    switch (*(uint16_t *)&cmd_key[0])
    {
    case 0x000E:
    {
      Serial.printf("get posControl request\n");
      break;
    }
    case 0x020E:
    {
      //            printf("get getGimbalInfo request\n");
      //            if (data[13] == 0x00)
      //                std::cout << "Data is not ready\n" << std::endl;
      //            if (data[13] == 0x01)
      //                std::cout << "The current angle is attitude angle\n"<<std::endl;
      //            if (data[13] == 0x02)
      //                std::cout << "The current angle is joint angle\n" << std::endl;

      _yaw = *(int16_t *)&data.data()[14];
      _roll = *(int16_t *)&data.data()[16];
      _pitch = *(int16_t *)&data.data()[18];

      //            std::cout << "yaw = " << _yaw << " roll = " << _roll << " pitch = " << _pitch << std::endl;

      break;
    }
    case 0x010E:
    {
      Serial.printf("set Gimbal speed\n");
      break;
    }
    case 0x120E:
    {
      // Serial.printf("set Focus Motor\n");
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
}

std::vector<uint8_t> v1_pack_list = std::vector<uint8_t>();
size_t pack_len = 0;
int step = 0;
std::string canid_raw_str = "";
std::string canid_str = "";

void canSniff(const CAN_message_t &msg)
{
  if (msg.id)
  {

    for (size_t i = 0; i < msg.len; i++)
    {
      if (step == 0)
      {
        if ((uint8_t)msg.buf[i] == 0xAA)
        // if ((uint8_t)msg.buf[i] == 0x55)
        {
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
}

int16_t yaw = 10;
int16_t roll = 10;
int16_t pitch = 10;
uint16_t time_ms = 100;

void loop(void)
{
  Can0.events();

  // static uint32_t timeout = millis();
  // if (millis() - timeout > 5000)
  // {
  //   uint8_t cmd_type = 0x03;
  //   uint8_t cmd_set = 0x0E;
  //   uint8_t cmd_id = 0x00;
  //   uint8_t time = (uint8_t)(time_ms/100);
  //   std::vector<uint8_t> data_payload =
  //   {
  //       ((uint8_t*)&yaw)[0],((uint8_t*)&yaw)[1],
  //       ((uint8_t*)&roll)[0],((uint8_t*)&roll)[1],
  //       ((uint8_t*)&pitch)[0],((uint8_t*)&pitch)[1],
  //       _position_ctrl_byte, time
  //   };
  //   auto cmd = ((CmdCombine *)_cmd_cmb)->combine(cmd_type, cmd_set, cmd_id, data_payload);

  //   Serial.print("Send CMD: ");
  //   for (int i = 0; i < cmd.size(); i++)
  //   {
  //     Serial.print(" 0x");
  //     Serial.print(cmd[i], HEX);
  //   }
  //   Serial.println(" ");

  //   // uint8_t buf[] = {0xAA,0x1A,0x00,0x03,0x00,0x00,0x00,0x22,0x11,0xA2,0x42,0x0E,0x00,0x20,0x00,0x30,0x00,0x40,0x00,0x01,0x14,0x7,0x40,0x97,0xBE};
  //   ISOTP_data config;
  //   config.id = 0x223;
  //   config.flags.extended = 0;  /* standard frame */
  //   config.separation_time = 0; /* time between back-to-back frames in millisec */
  //   tp.write(config, &cmd[0], cmd.size());
  //   timeout = millis();
  // }
}