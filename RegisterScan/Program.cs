﻿//---------------------------------------------------------------------------------
// Copyright (c) April 2020, devMobile Software
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// 
//---------------------------------------------------------------------------------
//#define ST_STM32F429I_DISCOVERY       //nanoff --target ST_STM32F429I_DISCOVERY --update
#define ESP32_WROOM_32_LORA_1_CHANNEL   //nanoff --target ESP32_WROOM_32 --serialport COM4 --update
namespace devMobile.IoT.Rfm9x.RegisterScan
{
   using System;
   using System.Threading;

   using Windows.Devices.Spi;

#if ESP32_WROOM_32_LORA_1_CHANNEL
   using nanoFramework.Hardware.Esp32;
#endif
   using nanoFramework.Runtime.Native;

   public sealed class Rfm9XDevice
   {
      private SpiDevice rfm9XLoraModem;

      public Rfm9XDevice(string SpiPort, int chipSelectPin)
      {

         var settings = new SpiConnectionSettings(chipSelectPin)
         {
            ClockFrequency = 500000,
            Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
            SharingMode = SpiSharingMode.Shared
         };

         rfm9XLoraModem = SpiDevice.FromId(SpiPort, settings);
      }

      public Byte RegisterReadByte(byte registerAddress)
      {
         byte[] writeBuffer = new byte[] { registerAddress, 0x0 };
         byte[] readBuffer = new byte[writeBuffer.Length];

         rfm9XLoraModem.TransferFullDuplex(writeBuffer, readBuffer);

         return readBuffer[1];
      }
   }

   public class Program
   {
#if ST_STM32F429I_DISCOVERY
      private const string SpiBusId = "SPI5";
#endif
#if ESP32_WROOM_32_LORA_1_CHANNEL
      private const string SpiBusId = "SPI1";
#endif

      public static void Main()
      {
#if ST_STM32F429I_DISCOVERY
         int chipSelectPinNumber = PinNumber('C', 2);
#endif
#if ESP32_WROOM_32_LORA_1_CHANNEL
         int chipSelectPinNumber = Gpio.IO16;
#endif

         try
         {
#if ESP32_WROOM_32_LORA_1_CHANNEL
            Configuration.SetPinFunction(nanoFramework.Hardware.Esp32.Gpio.IO12, DeviceFunction.SPI1_MISO);
            Configuration.SetPinFunction(nanoFramework.Hardware.Esp32.Gpio.IO13, DeviceFunction.SPI1_MOSI);
            Configuration.SetPinFunction(nanoFramework.Hardware.Esp32.Gpio.IO14, DeviceFunction.SPI1_CLOCK);
#endif
            Rfm9XDevice rfm9XDevice = new Rfm9XDevice(SpiBusId, chipSelectPinNumber);

            Thread.Sleep(500);

            while (true)
            {
               for (byte registerIndex = 0; registerIndex <= 0x42; registerIndex++)
               {
                  byte registerValue = rfm9XDevice.RegisterReadByte(registerIndex);

                  Debug.WriteLine($"Register 0x{registerIndex:x2} - Value 0X{registerValue:x2}");
               }
               Debug.WriteLine("");

               Thread.Sleep(10000);
            }
         }
         catch (Exception ex)
         {
            Debug.WriteLine(ex.Message);
         }
      }

#if ST_STM32F429I_DISCOVERY
      static int PinNumber(char port, byte pin)
      {
         if (port < 'A' || port > 'J')
            throw new ArgumentException();

         return ((port - 'A') * 16) + pin;
      }
#endif
   }
}
