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
#define ST_STM32F429I_DISCOVERY       //nanoff --target ST_STM32F429I_DISCOVERY --update
//#define ESP32_WROOM_32_LORA_1_CHANNEL   //nanoff --target ESP32_WROOM_32 --serialport COM4 --update
namespace devMobile.IoT.Rfm9x.TransmitInterrupt
{
   using System;
   using System.Text;
   using System.Threading;

   using Windows.Devices.Gpio;
   using Windows.Devices.Spi;

#if ESP32_WROOM_32_LORA_1_CHANNEL
   using nanoFramework.Hardware.Esp32;
#endif

   public sealed class Rfm9XDevice
   {
      private SpiDevice rfm9XLoraModem;
      private GpioPin InterruptGpioPin = null;
      private const byte RegisterAddressReadMask = 0X7f;
      private const byte RegisterAddressWriteMask = 0x80;

      public Rfm9XDevice(string spiPort, int chipSelectPin, int resetPin, int interruptPin)
      {
         var settings = new SpiConnectionSettings(chipSelectPin)
         {
            ClockFrequency = 500000,
            //DataBitLength = 8,
            Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
            SharingMode = SpiSharingMode.Shared,
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

         // Interrupt pin for RX message & TX done notification 
         InterruptGpioPin = gpioController.OpenPin(interruptPin);
         InterruptGpioPin.SetDriveMode(GpioPinDriveMode.Input);

         InterruptGpioPin.ValueChanged += InterruptGpioPin_ValueChanged;
      }

      public Rfm9XDevice(string spiPort, int chipSelectPin, int interruptPin)
      {
         var settings = new SpiConnectionSettings(chipSelectPin)
         {
            ClockFrequency = 500000,
            Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
            SharingMode = SpiSharingMode.Shared,
         };

         rfm9XLoraModem = SpiDevice.FromId(spiPort, settings);

         // Interrupt pin for RX message & TX done notification 
         GpioController gpioController = GpioController.GetDefault();
         InterruptGpioPin = gpioController.OpenPin(interruptPin);
         InterruptGpioPin.SetDriveMode(GpioPinDriveMode.Input);

         InterruptGpioPin.ValueChanged += InterruptGpioPin_ValueChanged;
      }

      private void InterruptGpioPin_ValueChanged(object sender, GpioPinValueChangedEventArgs e)
      {
         if (e.Edge != GpioPinEdge.RisingEdge)
         {
            return;
         }

         byte irqFlags = this.RegisterReadByte(0x12); // RegIrqFlags
         Console.WriteLine($"RegIrqFlags 0X{irqFlags:x2}");

         if ((irqFlags & 0b00001000) == 0b00001000)  // TxDone
         {
            Console.WriteLine("Transmit-Done");
         }

         this.RegisterWriteByte(0x12, 0xff);// RegIrqFlags      
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
         byte[] readBuffer = new byte[writeBuffer.Length];
         byte[] replyBuffer = new byte[length];

         writeBuffer[0] = address &= RegisterAddressReadMask;

         rfm9XLoraModem.TransferFullDuplex(writeBuffer, readBuffer);

         Array.Copy(readBuffer, 1, replyBuffer, 0, length);

         return replyBuffer;
      }

      public void RegisterWriteByte(byte address, byte value)
      {
         byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, value };
         byte[] readBuffer = new byte[writeBuffer.Length];

         rfm9XLoraModem.TransferFullDuplex(writeBuffer, readBuffer);
      }

      public void RegisterWriteWord(byte address, ushort value)
      {
         byte[] valueBytes = BitConverter.GetBytes(value);
         byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, valueBytes[0], valueBytes[1] };
         byte[] readBuffer = new byte[writeBuffer.Length];

         rfm9XLoraModem.TransferFullDuplex(writeBuffer, readBuffer);
      }

      public void RegisterWrite(byte address, byte[] bytes)
      {
         byte[] writeBuffer = new byte[1 + bytes.Length];
         byte[] readBuffer = new byte[writeBuffer.Length];

         Array.Copy(bytes, 0, writeBuffer, 1, bytes.Length);
         writeBuffer[0] = address |= RegisterAddressWriteMask;

         rfm9XLoraModem.TransferFullDuplex(writeBuffer, readBuffer);
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
#if ST_STM32F429I_DISCOVERY
      private const string SpiBusId = "SPI5";
#endif
#if ESP32_WROOM_32_LORA_1_CHANNEL
      private const string SpiBusId = "SPI1";
#endif

      static void Main()
      {
         int SendCount = 0;
#if ST_STM32F429I_DISCOVERY
         int chipSelectPinNumber = PinNumber('C', 2);
         int resetPinNumber = PinNumber('C', 3);
         int interruptPinNumber = PinNumber('A', 4);
#endif
#if ESP32_WROOM_32_LORA_1_CHANNEL
         int chipSelectPinNumber = Gpio.IO16;
         int interruptPinNumber = Gpio.IO26;
#endif

         try
         {
#if ESP32_WROOM_32_LORA_1_CHANNEL
            Configuration.SetPinFunction(Gpio.IO12, DeviceFunction.SPI1_MISO);
            Configuration.SetPinFunction(Gpio.IO13, DeviceFunction.SPI1_MOSI);
            Configuration.SetPinFunction(Gpio.IO14, DeviceFunction.SPI1_CLOCK);
            Rfm9XDevice rfm9XDevice = new Rfm9XDevice(SpiBusId, chipSelectPinNumber, interruptPinNumber);
#endif
#if ST_STM32F429I_DISCOVERY
            Rfm9XDevice rfm9XDevice = new Rfm9XDevice(SpiBusId, chipSelectPinNumber, resetPinNumber, interruptPinNumber);
#endif
            Thread.Sleep(500);

            // Put device into LoRa + Standby mode
            rfm9XDevice.RegisterWriteByte(0x01, 0b10000000); // RegOpMode 

            // Set the frequency to 915MHz
            byte[] frequencyWriteBytes = { 0xE4, 0xC0, 0x00 }; // RegFrMsb, RegFrMid, RegFrLsb
            rfm9XDevice.RegisterWrite(0x06, frequencyWriteBytes);

            // More power PA Boost
            rfm9XDevice.RegisterWriteByte(0x09, 0b10000000); // RegPaConfig

            // Interrupt on TxDone
            rfm9XDevice.RegisterWriteByte(0x40, 0b01000000); // RegDioMapping1 0b00000000 DI0 TxDone

            while (true)
            {
               // Set the Register Fifo address pointer
               rfm9XDevice.RegisterWriteByte(0x0E, 0x00); // RegFifoTxBaseAddress 

               // Set the Register Fifo address pointer
               rfm9XDevice.RegisterWriteByte(0x0D, 0x0); // RegFifoAddrPtr 

               string messageText = $"Hello LoRa {SendCount += 1}!";

               // load the message into the fifo
               byte[] messageBytes = UTF8Encoding.UTF8.GetBytes(messageText);
               rfm9XDevice.RegisterWrite(0x0, messageBytes); // RegFifo 

               // Set the length of the message in the fifo
               rfm9XDevice.RegisterWriteByte(0x22, (byte)messageBytes.Length); // RegPayloadLength
               Console.WriteLine($"Sending {messageBytes.Length} bytes message {messageText}");
               rfm9XDevice.RegisterWriteByte(0x01, 0b10000011); // RegOpMode 

               Thread.Sleep(10000);
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
