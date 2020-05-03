//---------------------------------------------------------------------------------
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
namespace devMobile.IoT.Rfm9x.ShieldSPI
{
   using System;
   using System.Threading;

   using Windows.Devices.Gpio;
   using Windows.Devices.Spi;

#if ESP32_WROOM_32_LORA_1_CHANNEL
   using nanoFramework.Hardware.Esp32;
#endif

   public class Program
   {
      private const byte RegVersion = 0x42;
#if ST_STM32F429I_DISCOVERY
      private const string SpiBusId = "SPI5";
#endif
#if ESP32_WROOM_32_LORA_1_CHANNEL
      private const string SpiBusId = "SPI1";
#endif

      public static void Main()
      {
#if ST_STM32F429I_DISCOVERY
         int ledPinNumber  = PinNumber('G', 14);
         int chipSelectPinNumber = PinNumber('C', 2);
#endif
#if ESP32_WROOM_32_LORA_1_CHANNEL
         int ledPinNumber = Gpio.IO17;
         int chipSelectPinNumber = Gpio.IO16;
#endif

         try
         {
            GpioController gpioController = GpioController.GetDefault();
            GpioPin led = gpioController.OpenPin(ledPinNumber);
            led.SetDriveMode(GpioPinDriveMode.Output);

#if ESP32_WROOM_32_LORA_1_CHANNEL
            Configuration.SetPinFunction(nanoFramework.Hardware.Esp32.Gpio.IO12, DeviceFunction.SPI1_MISO);
            Configuration.SetPinFunction(nanoFramework.Hardware.Esp32.Gpio.IO13, DeviceFunction.SPI1_MOSI);
            Configuration.SetPinFunction(nanoFramework.Hardware.Esp32.Gpio.IO14, DeviceFunction.SPI1_CLOCK);
#endif

            var settings = new SpiConnectionSettings(chipSelectPinNumber)
            {
               ClockFrequency = 500000,
               Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
               SharingMode = SpiSharingMode.Shared,
            };

            using (SpiDevice device = SpiDevice.FromId(SpiBusId, settings))
            {
               Thread.Sleep(500);

               while (true)
               {
                  byte[] writeBuffer = new byte[] { RegVersion };
                  byte[] readBuffer = new byte[writeBuffer.Length];

                  device.TransferSequential(writeBuffer, readBuffer); // STM32_F429_DISCO worked, ESP32_LORA_1_CHANNEL worked
                  //device.TransferFullDuplex(writeBuffer, readBuffer); // STM32_F429_DISCO didn't work, ESP32_LORA_1_CHANNEL didn't work

                  Console.WriteLine(String.Format("Register 0x{0:x2} - Value 0X{1:x2}", RegVersion, readBuffer[0]));

                  led.Toggle();
                  Thread.Sleep(10000);
               }
            }
         }
         catch (Exception ex)
         {
            Console.WriteLine(ex.Message);
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
