#include "Handle.h"
#include <Arduino.h>

// #include "USBCAN_SDK.h"
#include "custom_crc16.h"
#include "custom_crc32.h"

#define FRAME_LEN 8

DJIR_SDK::DataHandle::DataHandle()
{
//   pinMode(LED_BUILTIN, OUTPUT);

    _stopped = false;
    _input_position_ready_flag = false;
    _cmd_list = std::vector<std::vector<uint8_t>>();
    Can0.begin();
    Can0.setBaudRate(1000000);
    // Can0.setMaxMB(16);
    // Can0.enableFIFO();
    // Can0.setFIFOFilter(;)
    // Can0.enableFIFOInterrupt();
    // Can0.mailboxStatus();
    // _rsps = std::vector<std::string>();
    // tp.onReceive(myCallback); /* set callback */
}

void DJIR_SDK::DataHandle::start()
{
    // Can0.onReceive(DJIR_SDK::DataHandle::run);
    // _thread = std::thread(&DataHandle::run, this);
}

void DJIR_SDK::DataHandle::stop()
{
    _stopped = true;
    // _thread.join();
}

//TODO: ADD CAN send
void DJIR_SDK::DataHandle::add_cmd(std::vector<uint8_t> cmd)
{
    //digitalWrite(LED_BUILTIN, HIGH);

    // _rdcontent_lock.lock();
    _cmd_list.push_back(cmd);
    if (_cmd_list.size() > 10)
        _cmd_list.erase(_cmd_list.begin());
    // _rdcontent_lock.unlock();

    //TODO: getpos its the same msg every time?
    int data_len = (int)cmd.size();
    int frame_num = 0;
    int full_frame_num = data_len / FRAME_LEN;
    int left_len = data_len % FRAME_LEN;

    if (left_len == 0)
      frame_num = full_frame_num;
    else
      frame_num = full_frame_num + 1;


    CAN_message_t send_buf[frame_num];


    int data_offset = 0;
    for (int i = 0; i < (int)(full_frame_num); i++)
    {
      send_buf[i].id = 0x223;
      send_buf[i].flags.extended = 0;
      send_buf[i].len = FRAME_LEN;

        // __disable_irq();
        // memcpy(send_buf[i].buf, &cmd[data_offset], FRAME_LEN);
        // __enable_irq();

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
        
        // __disable_irq();
        // memcpy(send_buf[frame_num - 1].buf, &cmd[data_offset], left_len);
        // __enable_irq();

      for (int j = 0; j < left_len; j++)
        send_buf[frame_num - 1].buf[j] = cmd[data_offset + j];
    }

    int ret = 0;

    __disable_irq();
    for (int k = 0; k < frame_num; k++) 
    {
      send_buf[k].seq = 1;
      ret = Can0.write(send_buf[k]);
    }
    __enable_irq();

    // if (ret == -1) {
    //     Can0.mailboxStatus();

    //     Serial.printf("Can0.write failed\n");
    // }
    //digitalWrite(LED_BUILTIN, LOW);

}

bool DJIR_SDK::DataHandle::get_position(int16_t &yaw, int16_t &roll, int16_t &pitch, uint16_t timeout_ms)
{
    // Wait data.
    // std::unique_lock<std::mutex> lk(_input_position_mutex);
    uint32_t timeout = 10000;
    while (!_input_position_ready_flag)
    {
        this->run();

        // if (_input_position_cond_var.wait_for(
        //             lk, std::chrono::milliseconds(timeout_ms)) == std::cv_status::timeout)
        // {
        // Reset data ready flag.
        if (timeout < 1)
        {
            Serial.println("TIMEOUT");
            _input_position_ready_flag = false;
            // Unlock mutex.
            // lk.unlock();
            return false;
        }
        timeout--;

        // }
    }
    // lk.unlock();

    // Reset data ready flag.
    _input_position_ready_flag = false;
    yaw = _yaw;
    roll = _roll;
    pitch = _pitch;
    return true;
}

void DJIR_SDK::DataHandle::run()
{
    CAN_message_t frame;
    // std::string canid_raw_str = "";
    // std::string canid_str = "";
    // USBCAN_SDK::CANConnection* dev = (USBCAN_SDK::CANConnection*)_dev;
    // while (!_stopped)
        __disable_irq();
        int ret = Can0.read(frame);
        __enable_irq();

    if (ret)
    {
        // Serial.printf("ID:0x%0X \n", frame.id);
        if (frame.id == 0x222) {
            //digitalWrite(LED_BUILTIN, HIGH);
        // auto frame = dev->get_tunnel()->pop_data_from_recv_queue();
            for (size_t i = 0; i < frame.len; i++)
            {
                if (_step == 0)
                {

                    if (frame.buf[i] == 0xAA)
                    {
                        uint8_t tmp = frame.buf[i];
                        _v1_pack_list.push_back(tmp);
                        _step = 1;
                    }
                }
                else if (_step == 1)
                {
                    _pack_len = int(frame.buf[i]);
                    uint8_t tmp = frame.buf[i];
                    _v1_pack_list.push_back(tmp);
                    _step = 2;
                }
                else if (_step == 2)
                {
                    _pack_len |= ((int(frame.buf[i]) & 0x3) << 8);
                    uint8_t tmp = frame.buf[i];
                    _v1_pack_list.push_back(tmp);
                    _step = 3;
                }
                else if (_step == 3)
                {
                    uint8_t tmp = frame.buf[i];
                    _v1_pack_list.push_back(tmp);
                    if (_v1_pack_list.size() == 12)
                    {
                        if (_check_head_crc(_v1_pack_list))
                            _step = 4;
                        else
                        {
                            _step = 0;
                            _v1_pack_list.clear();
                        }
                    }
                }
                else if (_step == 4)
                {
                    uint8_t tmp = frame.buf[i];
                    _v1_pack_list.push_back(tmp);
                    if (_v1_pack_list.size() == _pack_len)
                    {

                        _step = 0;
                        if (_check_pack_crc(_v1_pack_list)) 
                            _process_cmd(_v1_pack_list);
                        
                        _v1_pack_list.clear();
                    }
                }
                else
                {
                    _step = 0;
                    _v1_pack_list.clear();
                }
            }
            //digitalWrite(LED_BUILTIN, LOW);

        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void DJIR_SDK::DataHandle::_process_cmd(std::vector<uint8_t> data)
{
    uint8_t cmd_type = (uint8_t)data[3];
    bool is_ok = false;
    uint8_t cmd_key[2] = {0, 0};

    // If it is a response frame, need to check the corresponding send command
    if (cmd_type == 0x20)
    {
        // _rdcontent_lock.lock();
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
        // _rdcontent_lock.unlock();
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
        case 0x000e:
        {
            // Serial.printf("get posControl request\n");
            break;
        }
        case 0x020e:
        {
            //            printf("get getGimbalInfo request\n");
            //            if (data[13] == 0x00)
            //                std::cout << "Data is not ready\n" << std::endl;
            //            if (data[13] == 0x01)
            //                std::cout << "The current angle is attitude angle\n"<<std::endl;
            //            if (data[13] == 0x02)
            //                std::cout << "The current angle is joint angle\n" << std::endl;

            _yaw = *(int16_t *)&data.data()[16];
            _roll = *(int16_t *)&data.data()[18];
            _pitch = *(int16_t *)&data.data()[20];

            //            std::cout << "yaw = " << _yaw << " roll = " << _roll << " pitch = " << _pitch << std::endl;

            _input_position_ready_flag = true;
            // _input_position_cond_var.notify_one();

            break;
        }
        default:
        {
            Serial.printf("get unknown request\n");
            break;
        }
        }
    }
}

bool DJIR_SDK::DataHandle::_check_head_crc(std::vector<uint8_t> data)
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

bool DJIR_SDK::DataHandle::_check_pack_crc(std::vector<uint8_t> data)
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
