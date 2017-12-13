using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SerialPort_dll;
using System.Threading;
namespace multiplexing_dll
{

   public enum ChaneleNum { chanel_0, chanel_1, chanel_2, chanel_3, chanel_4, chanel_5, chanel_6, chanel_7, chanel_8, chanel_9, chanel_10, chanel_11, chanel_12, chanel_13, chanel_14, chanel_15,all_disconnected};

    public class MultiplexingIncomingInformation
    {
        public bool newControlTransmitMessage = false;
        public string controlTransmitMessage; // TX line
        public bool newControlReceiverMessage = false;
        public string controlReceiverMessage; // RX line
    }

    public class classMultiplexing
    {

        // op code's
        private const byte API_MSG_CONNECT_CH = 0x01;
        private const byte API_MSG_DISCONNECT_CH = 0x02;
        private const byte API_MSG_CONNECT_ALL_CH = 0x03;
        private const byte API_MSG_DISCONNECT_ALL_CH = 0x04;



        private const byte API_RECEIVE_MSG_MAX_SIZE = 255;
        private DateTime communicationPacketTimeLast;

        private const byte COM_PACKET_MESSAGE_TX = 0x01;  //parameter
        private const byte COM_PACKET_MESSAGE_RX = 0x00;  //parameter
        private const byte API_MSG_MAG_BASIC_MASSEGE_LENGTH = 0x04;
        //optional preambles bytes that represent the start of the received packet
        private const byte API_MSG_PREAMBLE = 170;

        //optional indexes in the received packet
        private const byte COM_PACKET_INDEX_START_BYTE = 0; //index of the preamble byte data in the received packet
        private const byte COM_PACKET_INDEX_MESSAGE_SIZE = 1;	//index of the packet size data in the received packet
        private const byte COM_PACKET_INDEX_MESSAGE_TYPE = 2;	//index of the message type data in the received packet
        private const byte COM_PACKET_INDEX_MESSAGE_ID = 3; //index of the message id data in the received packet


        private MultiplexingIncomingInformation incomingInfo;
        public classSerial SerialPortInstanse;
        private Thread IncomingCommunicationBufferHandlerThread;

       public ChaneleNum ConnectedChanel = ChaneleNum.all_disconnected;




        public classMultiplexing(string portName, int baud, MultiplexingIncomingInformation info)
        {
            IncomingCommunicationBufferHandlerThread = new Thread(ApiTask);
            SerialPortInstanse = new classSerial(portName, 115200, null);
            incomingInfo = info;
            IncomingCommunicationBufferHandlerThread.IsBackground = false;
            IncomingCommunicationBufferHandlerThread.Start();
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
            try
            {
                IncomingCommunicationBufferHandlerThread.Abort();
                IncomingCommunicationBufferHandlerThread = null;
            }
            catch (Exception e)
            {
                if (e == null)
                    e = new Exception("The exception was null ????");
                throw e;
            } 

        }

        private void ApiTask()
        {

            byte[] incomingCommunicationBuffer = new byte[API_RECEIVE_MSG_MAX_SIZE]; //incoming communication buffer

            while (SerialPortInstanse.ComPortOk)
            {
                if (!SerialPortInstanse.port.IsOpen)
                {
                    System.Diagnostics.Trace.WriteLine(string.Format("{0} is not open",SerialPortInstanse.port.PortName));
                    break;
                }
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
                    }
                    else
                    {
                        Thread.Sleep(10);
                    }
                }
                catch (Exception ex)
                {
                    //SerialPortInstanse.port.DiscardInBuffer();
                    //Array.Clear(incomingCommunicationBuffer, 0, incomingCommunicationBuffer.Length);

                    SerialPortInstanse.ComPortErrorMessage = string.Format("Error: {0} connection error. function - DP Multiplexer." + Environment.NewLine + ex.Message, SerialPortInstanse.port.PortName);                                                            
                    SerialPortInstanse.ComPortOk = false;
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


                communicationPacketTimeLast = DateTime.Now;

                switch (packetType)
                {
                    case API_MSG_CONNECT_CH:
                        {

                        }
                        break;
                    case API_MSG_DISCONNECT_CH:
                        {

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


        public void ConnectDpDevice(byte DpId)
        {

            byte[] data = new byte[API_MSG_MAG_BASIC_MASSEGE_LENGTH + 1];
            data[0] = API_MSG_PREAMBLE;
            data[1] = (byte)data.Count();
            data[2] = API_MSG_CONNECT_CH;//opcode
            data[3] = DpId;
            data[data.Count() - 1] = CheckCum(data, data.Count());

            SerialPortInstanse.Send(data, data.Count());

            ConnectedChanel = (ChaneleNum)DpId;
            Thread.Sleep(250);
        }


        public void DisConnectAllDp()
        {

            byte[] data = new byte[API_MSG_MAG_BASIC_MASSEGE_LENGTH];
            data[0] = API_MSG_PREAMBLE;
            data[1] = (byte)data.Count();
            data[2] = API_MSG_DISCONNECT_ALL_CH;//opcode
            data[data.Count() - 1] = CheckCum(data, data.Count());

            SerialPortInstanse.Send(data, data.Count());

            ConnectedChanel = ChaneleNum.all_disconnected;
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
    }

}
