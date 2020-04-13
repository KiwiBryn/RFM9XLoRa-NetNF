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
namespace devMobile.IoT.Rfm9x.RegisterScan
{
   using System;
   using System.Threading;

   using Windows.Devices.Spi;

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
         byte[] writeBuffer = new byte[] { registerAddress };
         byte[] readBuffer = new byte[1];

         rfm9XLoraModem.TransferSequential(writeBuffer, readBuffer);

         return readBuffer[0];
      }
   }

   public class Program
   {
      public static void Main()
      {
         try
         {
            Rfm9XDevice rfm9XDevice = new Rfm9XDevice("SPI5", PinNumber('C', 2));

            Thread.Sleep(500);

            while (true)
            {
               for (byte registerIndex = 0; registerIndex <= 0x42; registerIndex++)
               {
                  byte registerValue = rfm9XDevice.RegisterReadByte(registerIndex);

                  Console.WriteLine($"Register 0x{registerIndex:x2} - Value 0X{registerValue:x2}");
               }
               Console.WriteLine("");

               Thread.Sleep(10000);
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
