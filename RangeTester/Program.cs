//---------------------------------------------------------------------------------
// Copyright (c) May 2020, devMobile Software
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
//#define ADDRESSED_MESSAGES_PAYLOAD
//#define ESP32_WROOM_32_LORA_1_CHANNEL   //nanoff --target ESP32_WROOM_32 --serialport COM4 --update
#define NETDUINO3_WIFI   // nanoff --target NETDUINO3_WIFI --update
//#define ST_STM32F429I_DISCOVERY       //nanoff --target ST_STM32F429I_DISCOVERY --update
namespace devMobile.IoT.Rfm9x.RangeTester
{
	using System;
	using System.Diagnostics;
	using System.Text;
	using System.Threading;

#if ESP32_WROOM_32_LORA_1_CHANNEL
	using nanoFramework.Hardware.Esp32;
#endif
	using Windows.Devices.Gpio;

	using devMobile.IoT.Rfm9x;

	class Program
	{
		private const double Frequency = 915000000.0;
		private static GpioPin led = null;
#if ST_STM32F429I_DISCOVERY
		private const string DeviceName = "Disco429";
		private const string SpiBusId = "SPI5";
#endif
#if ESP32_WROOM_32_LORA_1_CHANNEL
		private const string DeviceName = "ESP32";
		private const string SpiBusId = "SPI1";
#endif
#if NETDUINO3_WIFI
		private const string DeviceName = "N3WA";
		//private const string DeviceName = "N3WB";
		private const string SpiBusId = "SPI2";
#endif
#if ADDRESSED_MESSAGES_PAYLOAD
		private const string HostName = "LoRaIoT1";
#endif

		static void Main()
		{
			byte MessageCount = System.Byte.MaxValue;
#if ST_STM32F429I_DISCOVERY
			int ledPinNumber = PinNumber('G', 14);
			int chipSelectPinNumber = PinNumber('C', 2);
			int resetPinNumber = PinNumber('C', 3);
			int interruptPinNumber = PinNumber('A', 4);
#endif
#if ESP32_WROOM_32_LORA_1_CHANNEL
			int ledPinNumber = Gpio.IO17;
			int chipSelectPinNumber = Gpio.IO16;
         int interruptPinNumber = Gpio.IO26;

			Configuration.SetPinFunction(Gpio.IO12, DeviceFunction.SPI1_MISO);
			Configuration.SetPinFunction(Gpio.IO13, DeviceFunction.SPI1_MOSI);
			Configuration.SetPinFunction(Gpio.IO14, DeviceFunction.SPI1_CLOCK);

			Rfm9XDevice rfm9XDevice = new Rfm9XDevice(SpiBusId, chipSelectPinNumber, interruptPinNumber);
#endif
#if NETDUINO3_WIFI
			int ledPinNumber = PinNumber('A', 10);
			// Arduino D10->PB10
			int chipSelectPinNumber = PinNumber('B', 10);
			// Arduino D9->PE5
			int resetPinNumber = PinNumber('E', 5);
			// Arduino D2->PA3
			int interruptPinNumber = PinNumber('A', 3);
#endif
			Debug.WriteLine("devMobile.IoT.Rfm9x.LoRaDeviceClient starting");

			GpioController gpioController = GpioController.GetDefault();

			led = gpioController.OpenPin(ledPinNumber);
			led.SetDriveMode(GpioPinDriveMode.Output);
			led.Write(GpioPinValue.Low);

#if ST_STM32F429I_DISCOVERY || NETDUINO3_WIFI
			Rfm9XDevice rfm9XDevice = new Rfm9XDevice(SpiBusId, chipSelectPinNumber, resetPinNumber, interruptPinNumber);
#endif
			rfm9XDevice.Initialise( Frequency
				, rxPayloadCrcOn: true
				);

#if DEBUG
			rfm9XDevice.RegisterDump();
#endif

			rfm9XDevice.OnReceive += Rfm9XDevice_OnReceive;
#if ADDRESSED_MESSAGES_PAYLOAD
			rfm9XDevice.Receive(UTF8Encoding.UTF8.GetBytes(DeviceName));
#else
			rfm9XDevice.Receive();
#endif
			rfm9XDevice.OnTransmit += Rfm9XDevice_OnTransmit;

			Thread.Sleep(10000);

			while (true)
			{
				string messageText = string.Format("Hello from {0} ! {1}", DeviceName, MessageCount);
				MessageCount -= 1;

				byte[] messageBytes = UTF8Encoding.UTF8.GetBytes(messageText);
				Debug.WriteLine(string.Format("{0}-TX {1} byte message {2}", DateTime.UtcNow.ToString("HH:mm:ss"), messageBytes.Length, messageText));
#if ADDRESSED_MESSAGES_PAYLOAD
				rfm9XDevice.Send(UTF8Encoding.UTF8.GetBytes(HostName), messageBytes);
#else
				rfm9XDevice.Send(messageBytes);
#endif
				Thread.Sleep(10000);
			}
		}

		private static void Rfm9XDevice_OnReceive(object sender, Rfm9XDevice.OnDataReceivedEventArgs e)
		{
			led.Toggle();

			try
			{
				// Remove unprintable characters from messages
				for (int index = 0; index < e.Data.Length; index++)
				{
					if ((e.Data[index] < 0x20) || (e.Data[index] > 0x7E))
					{
						e.Data[index] = 0x20;
					}
				}

				string messageText = UTF8Encoding.UTF8.GetString(e.Data, 0, e.Data.Length);

#if ADDRESSED_MESSAGES_PAYLOAD
				string addressText = UTF8Encoding.UTF8.GetString(e.Address, 0, e.Address.Length);

				Debug.WriteLine(string.Format(@"{0}-RX From {1} PacketSnr {2} Packet RSSI {3}dBm RSSI {4}dBm ={5} ""{6}""", DateTime.UtcNow.ToString("HH:mm:ss"), addressText, e.PacketSnr, e.PacketRssi, e.Rssi, e.Data.Length, messageText));
#else
				Debug.WriteLine(string.Format(@"{0}-RX PacketSnr {1} Packet RSSI {2}dBm RSSI {3}dBm ={4} ""{5}""", DateTime.UtcNow.ToString("HH:mm:ss"), e.PacketSnr, e.PacketRssi, e.Rssi, e.Data.Length, messageText));
#endif
			}
			catch (Exception ex)
			{
				Debug.WriteLine(ex.Message);
			}
		}

		private static void Rfm9XDevice_OnTransmit(object sender, Rfm9XDevice.OnDataTransmitedEventArgs e)
		{
			Debug.WriteLine(string.Format("{0}-TX Done", DateTime.UtcNow.ToString("HH:mm:ss")));
		}

#if ST_STM32F429I_DISCOVERY || NETDUINO3_WIFI
		static int PinNumber(char port, byte pin)
		{
			if (port < 'A' || port > 'J')
				throw new ArgumentException();

			return ((port - 'A') * 16) + pin;
		}
#endif
	}
}

