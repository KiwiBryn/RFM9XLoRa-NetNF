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
namespace devMobile.IoT.Rfm9x.RegisterReadAndWrite
{
   using System;
   using System.Threading;

   using Windows.Devices.Gpio;
   using Windows.Devices.Spi;

   public sealed class Rfm9XDevice
   {
      private SpiDevice rfm9XLoraModem;
      private const byte RegisterAddressReadMask = 0X7f;
      private const byte RegisterAddressWriteMask = 0x80;

      public Rfm9XDevice(string spiPort, int chipSelectPin, int resetPin)
      {
         var settings = new SpiConnectionSettings(chipSelectPin)
         {
            ClockFrequency = 1000000,
            Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
            SharingMode = SpiSharingMode.Shared
         };

         rfm9XLoraModem = SpiDevice.FromId(spiPort, settings);

         // Factory reset pin configuration
         GpioController gpioController = GpioController.GetDefault();
         GpioPin resetGpioPin = gpioController.OpenPin(resetPin);
         resetGpioPin.SetDriveMode(GpioPinDriveMode.Output);
         resetGpioPin.Write(GpioPinValue.Low);
         Thread.Sleep(10);
         resetGpioPin.Write(GpioPinValue.High);
         Thread.Sleep(10);
      }

      public Byte RegisterReadByte(byte registerAddress)
      {
         byte[] writeBuffer = new byte[] { registerAddress &= RegisterAddressReadMask, 0x0 };
         byte[] readBuffer = new byte[writeBuffer.Length];

         rfm9XLoraModem.TransferFullDuplex(writeBuffer, readBuffer);

         return readBuffer[1];
      }

      public ushort RegisterReadWord(byte address)
      {
         byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask, 0x0, 0x0 };
         byte[] readBuffer = new byte[writeBuffer.Length];

         rfm9XLoraModem.TransferFullDuplex(writeBuffer, readBuffer);

         return (ushort)(readBuffer[2] + (readBuffer[1] << 8));
      }

      public byte[] RegisterRead(byte address, int length)
      {
         byte[] writeBuffer = new byte[length + 1];
         byte[] readBuffer = new byte[length + 1];
         byte[] repyBuffer = new byte[length];

         writeBuffer[0] = address &= RegisterAddressReadMask;

         rfm9XLoraModem.TransferFullDuplex(writeBuffer, readBuffer);

         Array.Copy(readBuffer, 1, repyBuffer, 0, length);

         return repyBuffer;
      }

      public void RegisterWriteByte(byte address, byte value)
      {
         byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, value };

         rfm9XLoraModem.Write(writeBuffer);
      }

      public void RegisterWriteWord(byte address, ushort value)
      {
         byte[] valueBytes = BitConverter.GetBytes(value);
         byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, valueBytes[0], valueBytes[1] };

         rfm9XLoraModem.Write(writeBuffer);
      }

      public void RegisterWrite(byte address, byte[] bytes)
      {
         byte[] writeBuffer = new byte[1 + bytes.Length];

         Array.Copy(bytes, 0, writeBuffer, 1, bytes.Length);
         writeBuffer[0] = address |= RegisterAddressWriteMask;

         rfm9XLoraModem.Write(writeBuffer);
      }

      public void RegisterDump()
      {
         Console.WriteLine("Register dump");
         for (byte registerIndex = 0; registerIndex <= 0x42; registerIndex++)
         {
            byte registerValue = this.RegisterReadByte(registerIndex);

            Console.WriteLine($"Register 0x{registerIndex:x2} - Value 0X{registerValue:x2}");
         }
      }
   }

   class Program
   {
      static void Main()
      {
         Rfm9XDevice rfm9XDevice = new Rfm9XDevice("SPI5", PinNumber('C', 2), PinNumber('C', 3));

         rfm9XDevice.RegisterDump();

         while (true)
         {
            Console.WriteLine("Read RegOpMode (read byte)");
            Byte regOpMode1 = rfm9XDevice.RegisterReadByte(0x1);
            Console.WriteLine($"RegOpMode 0x{regOpMode1:x2}");

            Console.WriteLine("Set LoRa mode and sleep mode (write byte)");
            rfm9XDevice.RegisterWriteByte(0x01, 0b10000000);

            Console.WriteLine("Read RegOpMode (read byte)");
            Byte regOpMode2 = rfm9XDevice.RegisterReadByte(0x1);
            Console.WriteLine($"RegOpMode 0x{regOpMode2:x2}");

            Console.WriteLine("Read the preamble (read word)");
            ushort preamble = rfm9XDevice.RegisterReadWord(0x20);
            Console.WriteLine($"Preamble 0x{preamble:x2}");

            Console.WriteLine("Set the preamble to 0x80 (write word)");
            rfm9XDevice.RegisterWriteWord(0x20, 0x80);

            Console.WriteLine("Read the center frequency (read byte array)");
            byte[] frequencyReadBytes = rfm9XDevice.RegisterRead(0x06, 5);
            Console.WriteLine($"Frequency Msb 0x{frequencyReadBytes[0]:x2} Mid 0x{frequencyReadBytes[1]:x2} Lsb 0x{frequencyReadBytes[2]:x2}");

            Console.WriteLine("Set the center frequency to 916MHz (write byte array)");
            byte[] frequencyWriteBytes = { 0xE4, 0xC0, 0x00 };
            rfm9XDevice.RegisterWrite(0x06, frequencyWriteBytes);

            rfm9XDevice.RegisterDump();

            Thread.Sleep(30000);
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
