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
namespace devMobile.IoT.Rfm9x.ShieldSPI
{
   using System;
   using System.Threading;

   using Windows.Devices.Gpio;
   using Windows.Devices.Spi;

   public class Program
   {
      public static void Main()
      {
         try
         {
            GpioController gpioController = GpioController.GetDefault();

            GpioPin led = gpioController.OpenPin(PinNumber('G', 14));
            led.SetDriveMode(GpioPinDriveMode.Output);

            GpioPin chipSelectGpioPin = gpioController.OpenPin(PinNumber('C', 2));
            chipSelectGpioPin.SetDriveMode(GpioPinDriveMode.Output);

            var settings = new SpiConnectionSettings(chipSelectGpioPin.PinNumber)
            {
               ClockFrequency = 500000,
               Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
               SharingMode = SpiSharingMode.Shared,
            };

            using (SpiDevice device = SpiDevice.FromId("SPI5", settings))
            {
               Thread.Sleep(500);

               while (true)
               {
                  byte[] writeBuffer = new byte[] { 0x42 };
                  byte[] readBuffer = new byte[1];

                  device.TransferSequential(writeBuffer, readBuffer);

                  Console.WriteLine(String.Format("Register 0x{0:x2} - Value 0X{1:x2}", 0x42, readBuffer[0]));

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

      static int PinNumber(char port, byte pin)
      {
         if (port < 'A' || port > 'J')
            throw new ArgumentException();

         return ((port - 'A') * 16) + pin;
      }
   }
}
