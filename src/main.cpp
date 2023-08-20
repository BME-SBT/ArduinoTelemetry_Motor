#include <Arduino.h>
#include <CAN.h>
#include <SoftwareSerial.h>

#include "sensor.h"

SoftwareSerial motor_serial(3, 4);

Sensor<uint16_t> s_motor_rpm(0b00001010010, 10, CAN);
Sensor<uint16_t> s_motor_current(0b01111110010, 10, CAN);
Sensor<uint16_t> s_motor_temp(0b00010110010, 10, CAN);
Sensor<uint16_t> s_motor_controller_temp(0b00011010010, 10, CAN);
Sensor<int32_t> s_motor_power(0b10000110010, 10, CAN);
Sensor<uint8_t> s_motor_heartbeat(0b11100110010, 1, CAN);

void dump_bytes(uint8_t *ptr, size_t size)
{
    for (size_t pos = 0; pos < size; pos++)
    {
        if (ptr[pos] < 0x10)
            Serial.print('0');
        Serial.print(ptr[pos], HEX);
        Serial.print(' ');
        if ((pos + 1) % 16 == 0)
            Serial.println();
    }
    Serial.println();
}

void setup()
{
    // USB debugging
    Serial.begin(115200);

    // CAN init
    CAN.setPins(10, 2);
    CAN.setSPIFrequency(250000);
    CAN.setClockFrequency(8000000);

    while (!CAN.begin(500000))
    {
        Serial.println("Starting CAN failed!");
        delay(100);
    }

    // Motor UART init
    motor_serial.begin(19200);
    motor_serial.setTimeout(50);
}

bool success = false;
static uint8_t motor_data_raw[47];

void read_motor_data()
{
    // Clear buffer
    while (motor_serial.available())
    {
        motor_serial.read();
    }
    
    memset(motor_data_raw, 0, 45);
    motor_serial.write(0x80);
    motor_serial.write(0x8d);
    delay(5);
    motor_serial.readBytes(motor_data_raw, 45);
    // dump_bytes(motor_data_raw, 45);

    if (motor_data_raw[0] == 0x7c && motor_data_raw[1] == 0x8d && motor_data_raw[43] == 0x7d)
    {
        // Packet seems valid
        success = true;
        uint16_t controller_temp = motor_data_raw[16] - 20;
        uint16_t motor_temp = motor_data_raw[17] - 20;
        uint16_t motor_rpm = (((uint16_t)motor_data_raw[22] << 8) | motor_data_raw[21]) * 10;
        float motor_current = (((uint16_t)motor_data_raw[29] << 8) | motor_data_raw[28]) / 10.0;
        float motor_voltage = (((uint16_t)motor_data_raw[31] << 8) | motor_data_raw[30]) / 10.0;
        int32_t motor_power_mW = motor_current * motor_voltage * 1000;

        s_motor_rpm.set_value(motor_rpm);
        s_motor_current.set_value((uint16_t)(floor(motor_current * 10)));
        s_motor_temp.set_value(motor_temp * 10);
        s_motor_controller_temp.set_value(controller_temp * 10);
        s_motor_power.set_value(motor_power_mW);
        s_motor_heartbeat.set_value((char)1);
    }
    else
    {
        s_motor_rpm.disable();
        s_motor_current.disable();
        s_motor_temp.disable();
        s_motor_controller_temp.disable();
        s_motor_power.disable();
        s_motor_heartbeat.set_value((char)0);
        success = false;
    }
}

void loop()
{
    Serial.println("loop");
    read_motor_data();
    s_motor_rpm.send();
    s_motor_current.send();
    s_motor_temp.send();
    s_motor_controller_temp.send();
    s_motor_power.send();

    CAN.clearWriteError();
    s_motor_heartbeat.send();

    CAN.clearWriteError();
    delay(1);
}