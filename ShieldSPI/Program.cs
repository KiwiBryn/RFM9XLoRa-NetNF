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
//#define ESP32_WROOM_32_LORA_1_CHANNEL   //nanoff --target ESP32_WROOM_32 --serialport COM4 --update
//#define NETDUINO3_WIFI   // nanoff --target NETDUINO3_WIFI --update
//NOTE May 2020 MBN_QUAIL device doesn't work something broken in SPI configuration
//#define MBN_QUAIL // nanoff --target MBN_QUAIL --update
//NOTE May 2020 ST_NUCLEO64_F091RC device doesn't work something broken in SPI configuration
//#define ST_NUCLEO64_F091RC // nanoff --target ST_NUCLEO64_F091RC --update
#define ST_NUCLEO144_F746ZG //nanoff --target ST_NUCLEO144_F746ZG --update
//#define ST_STM32F429I_DISCOVERY       //nanoff --target ST_STM32F429I_DISCOVERY --update
//NOTE May 2020 ST_STM32F769I_DISCOVERY device doesn't work SPI2 mappings broken 
//#define ST_STM32F769I_DISCOVERY      // nanoff --target ST_STM32F769I_DISCOVERY --update 
namespace devMobile.IoT.Rfm9x.ShieldSPI
{
   using System;
   using System.Diagnostics;
   using System.Threading;

   using Windows.Devices.Gpio;
   using Windows.Devices.Spi;

#if ESP32_WROOM_32_LORA_1_CHANNEL
   using nanoFramework.Hardware.Esp32;
#endif

   public class Program
   {
      private const byte RegVersion = 0x42;
#if ESP32_WROOM_32_LORA_1_CHANNEL
      private const string SpiBusId = "SPI1";
#endif
#if NETDUINO3_WIFI
      private const string SpiBusId = "SPI2";
#endif
#if MBN_QUAIL
      private const string SpiBusId = "SPI1";
#endif
#if ST_NUCLEO64_F091RC
      private const string SpiBusId = "SPI1";
#endif
#if ST_NUCLEO144_F746ZG
      private const string SpiBusId = "SPI1";
#endif
#if ST_STM32F429I_DISCOVERY
      private const string SpiBusId = "SPI5";
#endif
#if ST_STM32F769I_DISCOVERY
      private const string SpiBusId = "SPI5";
#endif

      public static void Main()
      {
#if ESP32_WROOM_32_LORA_1_CHANNEL // No reset line for this device as it isn't connected on SX127X
         int ledPinNumber = Gpio.IO17;
         int chipSelectPinNumber = Gpio.IO16;
#endif
#if NETDUINO3_WIFI
         int ledPinNumber  = PinNumber('A', 10);
         // Arduino D10->PB10
         int chipSelectPinNumber = PinNumber('B', 10);
         // Arduino D9->PE5
         int resetPinNumber = PinNumber('E', 5);
#endif
#if MBN_QUAIL
         int ledPinNumber  = PinNumber('E', 15);
         // CS on socket 1
         int chipSelectPinNumber = PinNumber('A', 3);
         // CS on socket 2
         //int chipSelectPinNumber = PinNumber('E', 0);
         // RST on socket 1
         int resetPinNumber = PinNumber('A', 2);
         // RST on socket 2
         //int resetPinNumber = PinNumber('E', 1);
#endif
#if ST_NUCLEO64_F091RC // No LED for this device as driven by D13 the SPI CLK line
         // Arduino D10->PB6
         int chipSelectPinNumber = PinNumber('B', 6);
         // Arduino D9->PC7
         int resetPinNumber = PinNumber('C', 7);
#endif
#if ST_STM32F429I_DISCOVERY // No reset line for this device as I didn't bother with jumper to SX127X pin
         int ledPinNumber  = PinNumber('G', 14);
         int chipSelectPinNumber = PinNumber('C', 2);
#endif
#if ST_NUCLEO144_F746ZG
         int ledPinNumber = PinNumber('B', 7);
         // Arduino D10->PD14
         int chipSelectPinNumber = PinNumber('D', 14);
         // Arduino D9->PD15
         int resetPinNumber = PinNumber('D', 15);
#endif
#if ST_STM32F769I_DISCOVERY
         int ledPinNumber  = PinNumber('J', 5);
         // Arduino D10->PA11
         int chipSelectPinNumber = PinNumber('A', 11);
         // Arduino D9->PH6
         int resetPinNumber = PinNumber('H', 6);
#endif
         Debug.WriteLine("devMobile.IoT.Rfm9x.ShieldSPI starting");

         try
         {
            GpioController gpioController = GpioController.GetDefault();

#if NETDUINO3_WIFI|| MBN_QUAIL || ST_NUCLEO64_F091RC || ST_NUCLEO144_F746ZG || ST_STM32F769I_DISCOVERY
            // Setup the reset pin
            GpioPin resetGpioPin = gpioController.OpenPin(resetPinNumber);
            resetGpioPin.SetDriveMode(GpioPinDriveMode.Output);
            resetGpioPin.Write(GpioPinValue.High);
#endif

#if ESP32_WROOM_32_LORA_1_CHANNEL || MBN_QUAIL || NETDUINO3_WIFI || ST_NUCLEO144_F746ZG || ST_STM32F429I_DISCOVERY || ST_STM32F769I_DISCOVERY
            // Setup the onboard LED
            GpioPin led = gpioController.OpenPin(ledPinNumber);
            led.SetDriveMode(GpioPinDriveMode.Output);
#endif

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
                  byte[] writeBuffer = new byte[] { RegVersion, 0x0 };
                  byte[] readBuffer = new byte[writeBuffer.Length];

                  device.TransferFullDuplex(writeBuffer, readBuffer);

                  Debug.WriteLine(String.Format("Register 0x{0:x2} - Value 0X{1:x2}", RegVersion, readBuffer[1]));

                  #if ESP32_WROOM_32_LORA_1_CHANNEL || MBN_QUAIL || NETDUINO3_WIFI || ST_NUCLEO144_F746ZG || ST_STM32F429I_DISCOVERY || ST_STM32F769I_DISCOVERY
                     led.Toggle();
                  #endif
                  Thread.Sleep(10000);
               }
            }
         }
         catch (Exception ex)
         {
            Debug.WriteLine(ex.Message);
         }
      }

#if  NETDUINO3_WIFI || MBN_QUAIL || ST_NUCLEO64_F091RC || ST_NUCLEO144_F746ZG || ST_STM32F429I_DISCOVERY || ST_STM32F769I_DISCOVERY
      static int PinNumber(char port, byte pin)
      {
         if (port < 'A' || port > 'J')
            throw new ArgumentException();

         return ((port - 'A') * 16) + pin;
      }
#endif
   }
}
