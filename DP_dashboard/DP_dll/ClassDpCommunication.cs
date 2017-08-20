using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using SerialPort_dll;
using System.Diagnostics;
using System.Runtime.CompilerServices;

public interface IGUI
{
    void UpdateTraceInfo(string msg);

    void devicesDetected();
    void showScanButton();
}

namespace DpCommunication
{

    public class DpInfo
    {

        private const byte SERIAL_NUMBER_LENGTH = 0x0A;

        public string  DeviseSerialNumber;
        public string  DeviceBarcode;
        public string DeviceMacAddress;
        public SByte CurrentTemp;
        public float   S1Pressure;
        public float   S2Pressure;
        public UInt16  LeftA2D;
        public UInt16  RightA2D;
        public byte    Calibrated;         
    }

    public class DpIncomingInformation
    {
        public bool   newControlTransmitMessage = false;
        public string controlTransmitMessage; // TX line
        public bool   newControlReceiverMessage = false;
        public string controlReceiverMessage; // RX line
    }

    public class DpCalibPoint
    {
        public float temp;                    // temperature of the current pressure value
        public float pressure;                // Physical pressure of the current a2d pressure value
        public int a2dPressureValue;          // A2D value



        public DpCalibPoint()
        {

        }

        public DpCalibPoint(float Pressure, float Temp)
        {
            temp = Temp;
            pressure = Pressure;
        }
    }

    public class DpCalibPointsInTemperature
    {
       public List<DpCalibPoint> oneTempLine = new List<DpCalibPoint>();
    }


    public class ClassDpCommunication
    {
        private bool NewDpInfoEvent = false;
        public bool LicenseAck = false;
        


        public DpInfo dpInfo = new DpInfo();
        // DP info Offset 
        private const byte SERIAL_NUMBER_LENGTH             = 0x0a;
        private const byte MAC_ADDRESS_LENGTH               = 0x0C;
        private const byte BARCODE_LENGTH                   = 0x0C;

        private const byte DEVICE_INFO_SERIAL_NUMBER_NUMBER_OFFSET = 0x03;
        private const byte DEVICE_INFO_CURRENT_TEMP_OFFSET       = DEVICE_INFO_SERIAL_NUMBER_NUMBER_OFFSET     + SERIAL_NUMBER_LENGTH;
        private const byte DEVICE_INFO_S1_PRESSURE_OFFSET        = DEVICE_INFO_CURRENT_TEMP_OFFSET           + 0x01;
        private const byte DEVICE_INFO_S2_PRESSURE_OFFSET        = DEVICE_INFO_S1_PRESSURE_OFFSET            + 0x04;
        private const byte DEVICE_INFO_CALIBRATED_OFFSET         = DEVICE_INFO_S2_PRESSURE_OFFSET            + 0x04;
        private const byte DEVICE_INFO_BARCODE_OFFSET      = DEVICE_INFO_CALIBRATED_OFFSET             + 0x01;
        private const byte DEVICE_INFO_A2D1_OFFSET               = DEVICE_INFO_BARCODE_OFFSET          + BARCODE_LENGTH;
        private const byte DEVICE_INFO_A2D2_OFFSET               = DEVICE_INFO_A2D1_OFFSET                   + 0x02;
        private const byte DEVICE_INFO_MAC_ADDRESS_OFFSET               = DEVICE_INFO_A2D2_OFFSET            + 0x02;
        //end DP info Offset 




        private const byte MAX_TEMP_POINTS = 0x05;
        private const byte MAX_PRESSURE_POINTS = 0x0f;

        //op cod's//
        private const byte API_MSG_DP_SET_BARCODE             = 0x00;
        private const byte API_MSG_DP_GET_DP_INFO             = 0x01;  //opcode
        private const byte API_MSG_DP_GET_SELF_TEST           = 0x02;  //opcode
        private const byte API_MSG_DP_GET_CURRENT_TEMP        = 0x03;  //opcode
        private const byte API_MSG_DP_GET_CURRENT_PRESSURE    = 0x04;  //opcode
        private const byte API_MSG_DP_SEND_PRESSURE_TO_DP     = 0x05;  //opcode
        private const byte API_MSG_DP_CALIBRATION_DONE        = 0x06;  //opcode
        private const byte API_MSG_DP_ACK_OK                  = 0x07;  //opcode
        private const byte API_MSG_DP_SEND_SERIAL_NUMBER      = 0x08;  //opcode
        private const byte API_MSG_DP_SEND_CALIBRATION_START  = 0x09;  //opcode
        private const byte API_MSG_DP_SEND_LICENSE            = 0x0A;  //opcode
        private const byte API_MSG_DP_LICENSE_ACK             = 0x0B;  //opcode
        private const byte API_MSG_DP_RESET                   = 0x0E;
        


        public List<DpCalibPointsInTemperature> DPPressuresTable = new List<DpCalibPointsInTemperature>();
        private const byte API_RECEIVE_MSG_MAX_SIZE = 255;
        private DateTime communicationPacketTimeLast;

        private const byte COM_PACKET_MESSAGE_TX = 0x01;  //parameter
        private const byte COM_PACKET_MESSAGE_RX = 0x00;  //parameter
        private const byte API_MSG_DP_BASIC_MASSEGE_LENGTH = 0x04;
        //optional preambles bytes that represent the start of the received packet
        private const byte API_MSG_PREAMBLE = 248;  // 0xf8

        //optional indexes in the received packet
        private const byte COM_PACKET_INDEX_START_BYTE = 0; //index of the preamble byte data in the received packet
        private const byte COM_PACKET_INDEX_MESSAGE_SIZE = 1;	//index of the packet size data in the received packet
        private const byte COM_PACKET_INDEX_MESSAGE_TYPE = 2;	//index of the message type data in the received packet
        private const byte COM_PACKET_INDEX_MESSAGE_ID = 3; //index of the message id data in the received packet


        //general constants
        private const byte SIZE_OF_FLOAT = 0x04;

        private DpIncomingInformation incomingInfo;
        public classSerial SerialPortInstanse;
        private Thread IncomingCommunicationBufferHandlerThread;
        private IGUI _gui;

        public ClassDpCommunication(string portName, int baud, DpIncomingInformation info, IGUI gui)
        {
            IncomingCommunicationBufferHandlerThread = new Thread(ApiTask);
            SerialPortInstanse = new classSerial(portName, 115200, null);
            incomingInfo = info;

            IncomingCommunicationBufferHandlerThread.IsBackground = false;
            IncomingCommunicationBufferHandlerThread.Start();
            _gui = gui;
        }
        public string GetCurrentTime()
        {
            DateTime dt = new DateTime();
            return dt.Second.ToString();
        }

        public void CloseComPort()
        {
            if (SerialPortInstanse.port.IsOpen)
            {
                SerialPortInstanse.port.Close();
                SerialPortInstanse.ComPortOk = false;
            }
            IncomingCommunicationBufferHandlerThread = null;
        }
               
        private bool WaitForResponse(int mSecToWait)
        {
            Stopwatch sw = new Stopwatch();
            sw.Start();

            do
            {
                if (NewDpInfoEvent == true)
                {
                    NewDpInfoEvent = false;
                    return true;
                }                
                Thread.Sleep(10);

            } while (sw.ElapsedMilliseconds <  mSecToWait);
            return false;
        }
        

        private void ApiTask()
        {

            byte[] incomingCommunicationBuffer = new byte[API_RECEIVE_MSG_MAX_SIZE]; //incoming communication buffer

            while (SerialPortInstanse.ComPortOk)
            {
                try
                {
                    if (SerialPortInstanse.port.BytesToRead > 0)
                    {
                        do
                        {
                            SerialPortInstanse.port.Read(incomingCommunicationBuffer, 0, 1);
                        } while (incomingCommunicationBuffer[0] != API_MSG_PREAMBLE);

                        SerialPortInstanse.port.Read(incomingCommunicationBuffer, 1, 1);

                        for (int i = 2; i < incomingCommunicationBuffer[1]; i++)
                        {
                            SerialPortInstanse.port.Read(incomingCommunicationBuffer, i, 1);
                        }
                        analyzeIncomingCommunicationPacket(incomingCommunicationBuffer);
                        Array.Clear(incomingCommunicationBuffer, 0, incomingCommunicationBuffer.Length);
                        SerialPortInstanse.Semaphore = true;
                    }

                    else
                    {
                        Thread.Sleep(1);
                    }
                }
                catch (Exception ex)
                {
                    _gui.UpdateTraceInfo("Serial port error " + ex.StackTrace + "\r\n");
                    //SerialPortInstanse.port.DiscardInBuffer();
                    //Array.Clear(incomingCommunicationBuffer, 0, incomingCommunicationBuffer.Length);

                    //SerialPortInstanse.ComPortOk = false;
                    //SerialPortInstanse.ComPortErrorMessage = string.Format("Error: {0} connection error. function - DP comunication.", SerialPortInstanse.port.PortName);
                }                        
            }

        }



        void analyzeIncomingCommunicationPacket(byte[] incomingData)
        {
            //check that received packet is legal
            //1) packet preamble byte is API_MSG_PREAMBLE
            //2) packet length is none zero
            //3) check sum is valid
            if ((incomingData[COM_PACKET_INDEX_START_BYTE] == API_MSG_PREAMBLE) &&
            (incomingData[COM_PACKET_INDEX_MESSAGE_SIZE] > 0) &&
            (incomingData[incomingData[COM_PACKET_INDEX_MESSAGE_SIZE] - 1] == CheckCum(incomingData, incomingData[COM_PACKET_INDEX_MESSAGE_SIZE])))
            {
                SendRxTxStringAsHex(incomingData, COM_PACKET_MESSAGE_RX, incomingData[COM_PACKET_INDEX_MESSAGE_SIZE]);
                incomingInfo.newControlReceiverMessage = true;


                int packetType;
                //initialize variable
                packetType = incomingData[COM_PACKET_INDEX_MESSAGE_TYPE];

                Console.WriteLine("\rRX Packet type " + incomingData[COM_PACKET_INDEX_MESSAGE_TYPE].ToString());

                communicationPacketTimeLast = DateTime.Now;

                switch (packetType)
                {
                    case API_MSG_DP_ACK_OK:
                        {

                        }
                        break;
                    case API_MSG_DP_SEND_PRESSURE_TO_DP:
                        {


                        }
                        break;
                    case API_MSG_DP_GET_DP_INFO:
                        {
                            dpInfo.DeviceBarcode = "";
                            dpInfo.DeviceMacAddress = "";
                            
                            string temSerialNumber = System.Text.Encoding.UTF8.GetString(incomingData, DEVICE_INFO_SERIAL_NUMBER_NUMBER_OFFSET, 10);
                            dpInfo.DeviseSerialNumber = temSerialNumber.Replace("\0", "");

                            dpInfo.CurrentTemp = (SByte)incomingData[DEVICE_INFO_CURRENT_TEMP_OFFSET];
                            dpInfo.S1Pressure = System.BitConverter.ToSingle(incomingData, DEVICE_INFO_S1_PRESSURE_OFFSET);
                            dpInfo.S2Pressure = System.BitConverter.ToSingle(incomingData, DEVICE_INFO_S2_PRESSURE_OFFSET);
                            dpInfo.Calibrated = incomingData[DEVICE_INFO_CALIBRATED_OFFSET];
                            dpInfo.DeviceBarcode = System.Text.Encoding.UTF8.GetString(incomingData, DEVICE_INFO_BARCODE_OFFSET, BARCODE_LENGTH);

                            dpInfo.LeftA2D = BitConverter.ToUInt16(incomingData, DEVICE_INFO_A2D1_OFFSET);
                            dpInfo.RightA2D = BitConverter.ToUInt16(incomingData, DEVICE_INFO_A2D2_OFFSET);

                            Byte[] mac = new byte[6];
                            mac[0] = incomingData[DEVICE_INFO_MAC_ADDRESS_OFFSET];
                            mac[1] = incomingData[DEVICE_INFO_MAC_ADDRESS_OFFSET + 1];
                            mac[2] = incomingData[DEVICE_INFO_MAC_ADDRESS_OFFSET + 2];
                            mac[3] = incomingData[DEVICE_INFO_MAC_ADDRESS_OFFSET + 3];
                            mac[4] = incomingData[DEVICE_INFO_MAC_ADDRESS_OFFSET + 4];
                            mac[5] = incomingData[DEVICE_INFO_MAC_ADDRESS_OFFSET + 5];

                            dpInfo.DeviceMacAddress += String.Format("{0:X2}", mac[0]);
                            dpInfo.DeviceMacAddress += String.Format("{0:X2}", mac[1]);
                            dpInfo.DeviceMacAddress += String.Format("{0:X2}", mac[2]);
                            dpInfo.DeviceMacAddress += String.Format("{0:X2}", mac[3]);
                            dpInfo.DeviceMacAddress += String.Format("{0:X2}", mac[4]);
                            dpInfo.DeviceMacAddress += String.Format("{0:X2}", mac[5]);

                            _gui.UpdateTraceInfo("Got DP info " + dpInfo.DeviceMacAddress + " " + dpInfo.CurrentTemp+" "+dpInfo.LeftA2D + " "+ dpInfo.RightA2D+" "+dpInfo.S1Pressure+" "+dpInfo.S2Pressure+"\r\n");

                            NewDpInfoEvent = true; 
                        }
                        break;
                    case API_MSG_DP_LICENSE_ACK:
                        {
                            LicenseAck = incomingData[COM_PACKET_INDEX_MESSAGE_TYPE + 1] == 1? true: false;
                            Console.WriteLine("Got license ack!!! "+LicenseAck);
                        }
                        break;
                    default:
                        break;
                }
            }
        }
        
        public void SendRxTxStringAsHex(byte[] data, byte dataDirection, byte length)
        {
            incomingInfo.controlReceiverMessage = "";
            incomingInfo.controlTransmitMessage = "";

            for (int i = 0; i < length - 1; i++)
            {
                if (dataDirection == COM_PACKET_MESSAGE_TX)
                {
                    incomingInfo.controlTransmitMessage += string.Format("{0:x}", data[i]);
                }
                else if (dataDirection == COM_PACKET_MESSAGE_RX)
                {
                    incomingInfo.controlReceiverMessage += string.Format("{0:x}", data[i]);
                }
            }

        }

        private byte CheckCum(byte[] data, int length)
        {
            int sum;
            int i;
            byte sumByteValue;

            //initialize variable
            sum = 0;
            i = 0;

            //getting packet sum
            for (i = 0; i < length - 1; i++)
                sum += data[i];


            //convert to byte value range
            sumByteValue = (byte)(sum % 256);

            //return value
            return (byte)~sumByteValue;
        }

        public static string ByteArrayToString(byte[] ba)
        {
            StringBuilder hex = new StringBuilder(ba.Length * 2);
            foreach (byte b in ba)
                hex.AppendFormat("{0:x2}", b);
            return hex.ToString();
        }


        public bool DpWritePressurePointToDeviceSync(float tempUnderTest, byte TempN, float extPressure, byte PreesureN, int waitTime)
        {
            lock (this)
            {
                DpWritePressurePointToDevice(tempUnderTest, TempN, extPressure, PreesureN);

                return WaitForResponse(waitTime);
            }
        }

        public void DpWritePressurePointToDevice(float tempUnderTest, byte TempN, float extPressure,byte PreesureN)
        {

            byte[] TargetTempArray = new byte[SIZE_OF_FLOAT];
            TargetTempArray = BitConverter.GetBytes(tempUnderTest);

            byte[] extPressArray = new byte[SIZE_OF_FLOAT];
            extPressArray = BitConverter.GetBytes(extPressure);

            byte[] data = new byte[API_MSG_DP_BASIC_MASSEGE_LENGTH + 10];
            data[0] = API_MSG_PREAMBLE;
            data[1] = (byte)data.Count();
            data[2] = API_MSG_DP_SEND_PRESSURE_TO_DP;  //opcode

            data[3] = TargetTempArray[0];  // byte 1 from the float temp value
            data[4] = TargetTempArray[1];  // byte 2 from the float temp value
            data[5] = TargetTempArray[2];  // byte 3 from the float temp value
            data[6] = TargetTempArray[3];  // byte 4 from the float temp value
            data[7] = TempN;

            data[8]  = extPressArray[0];   // byte 1 from the float pressure value
            data[9]  = extPressArray[1];   // byte 2 from the float pressure value
            data[10] = extPressArray[2];   // byte 3 from the float pressure value
            data[11] = extPressArray[3];   // byte 4 from the float pressure value
            data[12] = PreesureN;

            data[data.Count() - 1] = CheckCum(data, data.Count());

            _gui.UpdateTraceInfo("Send data to DP "+ByteArrayToString(data)+"\r\n");
            SerialPortInstanse.port.DiscardInBuffer();
            SerialPortInstanse.Send(data, data.Count());
        }

        public void Simulation()
        {
            for (int i = 0; i < MAX_TEMP_POINTS; i++)
            {
                DpCalibPointsInTemperature newTempLine = new DpCalibPointsInTemperature();
                for (int j = 0; j < MAX_PRESSURE_POINTS; j++)
                {
                    DpCalibPoint newPoint = new DpCalibPoint();
                    newPoint.a2dPressureValue = 1000 * j;
                    newTempLine.oneTempLine.Add(newPoint);
                }
                DPPressuresTable.Add(newTempLine);
            }

        }
        public bool DPgetDpInfoSync(int timeout)
        {
            lock (this)
            {
                _gui.UpdateTraceInfo("Send get info \r\n");
                Console.Write("Send get info \r\n");
                byte[] data = new byte[API_MSG_DP_BASIC_MASSEGE_LENGTH];
                data[0] = API_MSG_PREAMBLE;
                data[1] = (byte)data.Count();
                data[2] = API_MSG_DP_GET_DP_INFO;  //opcode

                data[data.Count() - 1] = CheckCum(data, data.Count());

                SerialPortInstanse.Send(data, data.Count());

               return WaitForResponse(timeout);
            }
        }


        public void SendPressuresTableToDP()
        {
            byte TempCount = 0;
            byte PressureInTempCount = 0;

            if (DPPressuresTable.Capacity > 0)
            {
                foreach (DpCalibPointsInTemperature currentTempLine in DPPressuresTable)
                {
                    foreach (DpCalibPoint currentPoint in currentTempLine.oneTempLine)
                    {
                        //public void DpWritePressurePointToDevice(byte PreesureN, float extPressure, byte TempN, float tempUnderTest)
                        //DpWritePressurePointToDevice(currentPoint.a2dPressureValue, TempCount, PressureInTempCount);
                        PressureInTempCount++;
                    }
                    TempCount++;
                    PressureInTempCount = 0;
                }
            }
        }

        public void SendDpSerialNumber(byte[] serialNumber)
        {
            byte[] data = new byte[API_MSG_DP_BASIC_MASSEGE_LENGTH + serialNumber.Count()];
            data[0] = API_MSG_PREAMBLE;
            data[1] = (byte)data.Count();
            data[2] = API_MSG_DP_SEND_SERIAL_NUMBER;  //opcode
            Array.Copy(serialNumber, 0, data, API_MSG_DP_BASIC_MASSEGE_LENGTH - 1, serialNumber.Count());
            data[data.Count() - 1] = CheckCum(data, data.Count());

            SerialPortInstanse.Send(data, data.Count());
        }

        public void SendEndCalibration()
        {
            byte[] data = new byte[API_MSG_DP_BASIC_MASSEGE_LENGTH];
            data[0] = API_MSG_PREAMBLE;
            data[1] = (byte)data.Count();
            data[2] = API_MSG_DP_CALIBRATION_DONE;  //opcode
            data[data.Count() - 1] = CheckCum(data, data.Count());
            SerialPortInstanse.Send(data, data.Count());
        }

        public void SendStartCalibration()
        {
            byte[] data = new byte[API_MSG_DP_BASIC_MASSEGE_LENGTH];
            data[0] = API_MSG_PREAMBLE;
            data[1] = (byte)data.Count();
            data[2] = API_MSG_DP_SEND_CALIBRATION_START;  //opcode
            data[data.Count() - 1] = CheckCum(data, data.Count());
            SerialPortInstanse.Send(data, data.Count());
        }

        public void SendDpLicense(byte[] license)
        {
            byte[] data = new byte[API_MSG_DP_BASIC_MASSEGE_LENGTH + license.Length];
            data[0] = API_MSG_PREAMBLE;
            data[1] = (byte)data.Count();
            data[2] = API_MSG_DP_SEND_LICENSE;  //opcode

            Array.Copy(license, 0, data, API_MSG_DP_BASIC_MASSEGE_LENGTH - 1, license.Count());


            data[data.Count() - 1] = CheckCum(data, data.Count());
            SerialPortInstanse.Send(data, data.Count());
        }

        public void reset()
        {
            lock (this)
            {
                
                byte[] data = new byte[API_MSG_DP_BASIC_MASSEGE_LENGTH];
                data[0] = API_MSG_DP_RESET;
                data[1] = (byte)data.Count();
                data[2] = API_MSG_DP_SET_BARCODE;  //opcode


                data[data.Count() - 1] = CheckCum(data, data.Count());
                SerialPortInstanse.Send(data, data.Count());
            }
        }

        public void setFinalBarcode(byte[] SN)
        {
            byte[] data = new byte[API_MSG_DP_BASIC_MASSEGE_LENGTH + SN.Length];
            data[0] = API_MSG_DP_SET_BARCODE;
            data[1] = (byte)data.Count();
            data[2] = API_MSG_DP_RESET;  //opcode
            Array.Copy(SN, 0, data, API_MSG_DP_BASIC_MASSEGE_LENGTH - 1, SN.Count());
            
            data[data.Count() - 1] = CheckCum(data, data.Count());
            SerialPortInstanse.Send(data, data.Count());
            Thread.Sleep(100);
            reset();
        }
    }
}
/*
case etOpcodeSetBarCodeFinal :  

    //Exctract device bar code
    osal_memcpy((void*)barCode, msg->data, SERIAL_BARCODE_LENGTH);
    //Save device bar code to SNV 
    set_param(EE_BARCODE_ITEM_ID, SERIAL_BARCODE_LENGTH, barCode);

    //Set device name to bar code number
    //safe_osal_snv_write(DEVICE_NAME_ITEM_ID, SERIAL_BARCODE_LENGTH , barCode);
    write_device_name(barCode);
    //read device name
    read_device_name();
  
  case   etOpcodeResetDevice:
    //Reset the device (To apply new name)    
    HAL_SYSTEM_RESET();
    
    break;
  */