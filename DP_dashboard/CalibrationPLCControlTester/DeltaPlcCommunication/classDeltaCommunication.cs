using System;
using System.Collections.Generic;
using Utils;

namespace DeltaPlcCommunication
{
    #region enumrators
    public enum DeltaMemType { S = 0, X, Y, T, M, C, D };

    public enum DeltaMsgType
    {
        ReadCoilStatus = 0x1,
        ReadInputStatus = 0x2,
        ReadHoldingRegisters = 0x3,
        ForceSingleCoil = 0x5,
        PresetSingleRegister = 0x6,
        ForceMultipleCoils = 0x15,
        PresetMultipleRegister = 0x10,
        ReportSlaveID = 0x17
    }

    #endregion
       

    public struct DeltaReturnedData
    {
        public bool[] BitStatus;
        public int[] IntValue;
        public float[] Realvalue;
    }

    public class DeltaIncomingInformation
    {
        public List<String> listDebugInfo = new List<String>();
    }

    public class classDeltaProtocol
    {
#region Constants
        private const string STX = ":01";   //address of device
        private const int MAX_QUIRY = 11;
        public string RxMsg;

        public const int S_OFFSET = 0x00;   //~3ff
        public const int X_OFFSET = 0x400;  //~04FF
        public const int Y_OFFSET = 0x500;  //~05FF
        public const int T_OFFSET = 0x600;  //~06FF
        public const int M_OFFSET = 0x800;  //~08FF
        public const int C_OFFSET = 0xE00;  //~0EFF
        public const int D_OFFSET = 0x1000; //~14FF
        public const byte MAX_16BIT_SIZE = 18;
        public const byte MAX_32BIT_SIZE = 9;
        public classSerial serial;
        private DeltaIncomingInformation incomingInfo;




        public bool newPressureTableReceive = false;
        private IGUI _gui;

        #endregion
        #region Methods
        public classDeltaProtocol(string portName, int baud, DeltaIncomingInformation info, IGUI gui)
        {
            incomingInfo = info;
            serial = new classSerial(portName, baud, null);
            _gui = gui;
        }

        public bool IsComOpen()
        {
            return serial.IsComOpen();
        }

        public void CloseComPort()
        {
            if (serial.port.IsOpen)
            {
                serial.port.Close();
                serial.ComPortOk = false;
            }
        }

        public DeltaReturnedData SendNewMessage(DeltaMsgType bCommand, DeltaMemType bType, int iAddress, byte bSize,List<Int16> oDataArrayToWrite = null)
        {
            byte bcheckSum;
            string stSendBuf;
            DeltaReturnedData Incoming = new DeltaReturnedData();

            bcheckSum = 0;
            stSendBuf = ":01";
            bcheckSum = 1; //Hex

            //kind of command
            switch (bCommand)
            {
                case DeltaMsgType.ReadCoilStatus:
                    bcheckSum += (byte)DeltaMsgType.ReadCoilStatus;
                    stSendBuf += Convert.ToString((byte)DeltaMsgType.ReadCoilStatus, 16).PadLeft(2, '0');
                    break;
                case DeltaMsgType.ReadInputStatus:
                    bcheckSum += (byte)DeltaMsgType.ReadInputStatus;
                    stSendBuf += Convert.ToString((byte)DeltaMsgType.ReadInputStatus, 16).PadLeft(2, '0');
                    break;
                case DeltaMsgType.ReadHoldingRegisters:
                    bcheckSum += (byte)DeltaMsgType.ReadHoldingRegisters;
                    stSendBuf += Convert.ToString((byte)DeltaMsgType.ReadHoldingRegisters, 16).PadLeft(2, '0');
                    break;
                case DeltaMsgType.ForceSingleCoil:
                    bcheckSum += (byte)DeltaMsgType.ForceSingleCoil;
                    stSendBuf += Convert.ToString((byte)DeltaMsgType.ForceSingleCoil, 16).PadLeft(2, '0');
                    break;
                case DeltaMsgType.PresetSingleRegister:
                    bcheckSum += (byte)DeltaMsgType.PresetSingleRegister;
                    stSendBuf += Convert.ToString((byte)DeltaMsgType.PresetSingleRegister, 16).PadLeft(2, '0');
                    break;
                case DeltaMsgType.ForceMultipleCoils:
                    bcheckSum += (byte)DeltaMsgType.ForceMultipleCoils;
                    stSendBuf += Convert.ToString((byte)DeltaMsgType.ForceMultipleCoils, 16).PadLeft(2, '0');
                    break;
                case DeltaMsgType.PresetMultipleRegister:
                    bcheckSum += (byte)DeltaMsgType.PresetMultipleRegister;
                    stSendBuf += Convert.ToString((byte)DeltaMsgType.PresetMultipleRegister, 16).PadLeft(2, '0');
                    break;
                case DeltaMsgType.ReportSlaveID:
                    bcheckSum += (byte)DeltaMsgType.ReportSlaveID;
                    stSendBuf += Convert.ToString((byte)DeltaMsgType.ReportSlaveID, 16).PadLeft(2, '0');
                    break;
            }

            //hi and lo address
            switch (bType)
            {
                case DeltaMemType.C:
                    //Hibyte
                    stSendBuf += Convert.ToString(((iAddress + C_OFFSET) / 256), 16).PadLeft(2, '0');
                    bcheckSum += (byte)((iAddress + C_OFFSET) / 256);
                    //Lowbyte
                    stSendBuf += Convert.ToString(((iAddress + C_OFFSET) & 255), 16).PadLeft(2, '0');
                    bcheckSum += (byte)((iAddress + C_OFFSET) & 255);
                    break;
                case DeltaMemType.X:
                    //Hibyte
                    stSendBuf += Convert.ToString(((iAddress + X_OFFSET) / 256), 16).PadLeft(2, '0');
                    bcheckSum += (byte)((iAddress + X_OFFSET) / 256);
                    //Lowbyte
                    stSendBuf += Convert.ToString(((iAddress + X_OFFSET) & 255), 16).PadLeft(2, '0');
                    bcheckSum += (byte)((iAddress + X_OFFSET) & 255);
                    break;
                case DeltaMemType.Y:
                    //Hibyte
                    stSendBuf += Convert.ToString(((iAddress + Y_OFFSET) / 256), 16).PadLeft(2, '0');
                    bcheckSum += (byte)((iAddress + Y_OFFSET) / 256);
                    //Lowbyte
                    stSendBuf += Convert.ToString(((iAddress + Y_OFFSET) & 255), 16).PadLeft(2, '0');
                    bcheckSum += (byte)((iAddress + Y_OFFSET) & 255);
                    break;
                case DeltaMemType.T:
                    //Hibyte
                    stSendBuf += Convert.ToString(((iAddress + T_OFFSET) / 256), 16).PadLeft(2, '0');
                    bcheckSum += (byte)((iAddress + T_OFFSET) / 256);
                    //Lowbyte
                    stSendBuf += Convert.ToString(((iAddress + T_OFFSET) & 255), 16).PadLeft(2, '0');
                    bcheckSum += (byte)((iAddress + T_OFFSET) & 255);
                    break;
                case DeltaMemType.M:
                    //Hibyte
                    stSendBuf += Convert.ToString(((iAddress + M_OFFSET) / 256), 16).PadLeft(2, '0');
                    bcheckSum += (byte)((iAddress + M_OFFSET) / 256);
                    //Lowbyte
                    stSendBuf += Convert.ToString(((iAddress + M_OFFSET) & 255), 16).PadLeft(2, '0');
                    bcheckSum += (byte)((iAddress + M_OFFSET) & 255);
                    break;
                case DeltaMemType.D:
                    //Hibyte
                    stSendBuf += Convert.ToString(((iAddress + D_OFFSET) / 256), 16).PadLeft(2, '0');
                    bcheckSum += (byte)((iAddress + D_OFFSET) / 256);
                    //Lowbyte
                    stSendBuf += Convert.ToString(((iAddress + D_OFFSET) & 255), 16).PadLeft(2, '0');
                    bcheckSum += (byte)((iAddress + D_OFFSET) & 255);
                    break;
            }

            switch (bCommand)
            {
                case DeltaMsgType.ReadInputStatus:
                    //Hibyte Size
                    stSendBuf += Convert.ToString((bSize / 256), 16).PadLeft(2, '0');
                    bcheckSum += (byte)(bSize / 256);
                    //Lowbyte Size
                    stSendBuf += Convert.ToString(((bSize) & 255), 16).PadLeft(2, '0');
                    bcheckSum += (byte)((bSize) & 255);
                    break;
                case DeltaMsgType.ReadHoldingRegisters:
                    //Hibyte Size
                    stSendBuf += Convert.ToString((bSize / 256), 16).PadLeft(2, '0');
                    bcheckSum += (byte)(bSize / 256);
                    //Lowbyte Size
                    stSendBuf += Convert.ToString((bSize & 255), 16).PadLeft(2, '0');
                    bcheckSum += (byte)(bSize & 255);
                    break;
                case DeltaMsgType.PresetSingleRegister:
                    //Hibyte Size
                    stSendBuf += Convert.ToString((oDataArrayToWrite[0] / 256), 16).PadLeft(2, '0');
                    bcheckSum += (byte)(oDataArrayToWrite[0] / 256);
                    //Lowbyte Size
                    stSendBuf += Convert.ToString((oDataArrayToWrite[0] & 255), 16).PadLeft(2, '0');
                    bcheckSum += (byte)(oDataArrayToWrite[0] & 255);
                    break;
                case DeltaMsgType.PresetMultipleRegister:
                    //Hibyte Size
                    int registersNum = bSize;
                    bSize += bSize;

                    stSendBuf += Convert.ToString((registersNum / 256), 16).PadLeft(2, '0');
                    bcheckSum += (byte)(registersNum / 256);
                    //Lowbyte Size
                    stSendBuf += Convert.ToString((registersNum & 255), 16).PadLeft(2, '0');
                    bcheckSum += (byte)(registersNum & 255);

                    stSendBuf += Convert.ToString((bSize & 255), 16).PadLeft(2, '0');
                    bcheckSum += (byte)(bSize & 255);


                    for(int i = 0;i< oDataArrayToWrite.Count;i++)
                    {
                        stSendBuf += Convert.ToString((oDataArrayToWrite[i] / 256), 16).PadLeft(2, '0');
                        bcheckSum += (byte)(oDataArrayToWrite[i] / 256);
                        //Lowbyte Size
                        stSendBuf += Convert.ToString((oDataArrayToWrite[i] & 255), 16).PadLeft(2, '0');
                        bcheckSum += (byte)(oDataArrayToWrite[i] & 255);
                    }
                   
                    break;
                case DeltaMsgType.ForceSingleCoil:
                    if (bSize == 0) //false
                    {
                        //Hibyte Size
                        stSendBuf += "0000";

                    }
                    else    //true
                    {
                        stSendBuf += "FF00";
                        bcheckSum += 0xFF;
                    }
                    break;
            }
            
            stSendBuf += Convert.ToString((0x100 - bcheckSum), 16).PadLeft(2, '0');
            stSendBuf = stSendBuf.ToUpper();

            RxMsg = SendAndRecieveData(stSendBuf);

#if  DELTA_14SS
#else       //DELTA_12SA2
            RxMsg = ":0" + RxMsg;
#endif

            if (RxMsg == "")
            {
                _gui.UpdateTraceInfo("No response from PLC ");
                incomingInfo.listDebugInfo.Add("No Response From unit -Check The Connection");
                return Incoming;
            }
            // exit method if message is not valid
            //System.Diagnostics.Debug.Print(stSendBuf);
            //System.Diagnostics.Debug.Print(RxMsg);
            if (!CheckValidMessage(RxMsg))
            {
                _gui.UpdateTraceInfo("Response from PLC is invalid "+RxMsg);
                return Incoming;
            }
            //kind of command
            switch (bCommand)
            {
                case DeltaMsgType.ReadInputStatus:
                    Incoming = ParseReceivedBuf(RxMsg, bSize, 1);
                    break;
                case DeltaMsgType.ReadCoilStatus:
                    Incoming = ParseReceivedBuf(RxMsg, bSize, 1);
                    break;
                case DeltaMsgType.ReadHoldingRegisters:
                    Incoming = ParseReceivedBuf(RxMsg, bSize, 0);
                    newPressureTableReceive = true;
                    break;

                case DeltaMsgType.ForceSingleCoil:
                    if (!stSendBuf.Equals(RxMsg.Substring(0, RxMsg.Length - 1)))
                    {
                       incomingInfo.listDebugInfo.Add("Received message is different then expected");
                    }
                    break;

                case DeltaMsgType.PresetSingleRegister:

                    break;
                case DeltaMsgType.ForceMultipleCoils:

                    break;
                case DeltaMsgType.PresetMultipleRegister:

                    break;
                case DeltaMsgType.ReportSlaveID:

                    break;
            }
            RxMsg = "";
            return Incoming;
        }

        public DeltaReturnedData SendLongSizeRequest(DeltaMsgType bCommand, DeltaMemType bType, int iAddress, int iSize)
        {
            // Size is bigger then allowed - we have to send few messages
            DeltaReturnedData Result = new DeltaReturnedData();
            Result.IntValue = new int[iSize];
            int TmpSize = iSize;

            while (TmpSize > MAX_16BIT_SIZE)
            {
                DeltaReturnedData Incoming = this.SendNewMessage(bCommand, bType, (iAddress + iSize - TmpSize), MAX_16BIT_SIZE);
                if (Incoming.IntValue == null) return Result;
                Array.Copy(Incoming.IntValue, 0, Result.IntValue, iSize - TmpSize, MAX_16BIT_SIZE);
                TmpSize -= MAX_16BIT_SIZE;
            }
            if (TmpSize > 0)
            {
                DeltaReturnedData Incoming = this.SendNewMessage(bCommand, bType, (byte)(iAddress + iSize - TmpSize), (byte)TmpSize);
                Array.Copy(Incoming.IntValue, 0, Result.IntValue, iSize - TmpSize, TmpSize);
            }
            return Result;

        }

        private DeltaReturnedData ParseReceivedBuf(string RxMsg, int Size, byte DataType)
        {
            DeltaReturnedData DRD = new DeltaReturnedData();
                      


            if (RxMsg.Substring(0, 3).Equals(STX))
            {
                switch (DataType)
                {
                    case 0:
                        if (int.Parse(RxMsg.Substring(5, 2), System.Globalization.NumberStyles.HexNumber) / 2 == Size)
                        {
                            DRD.IntValue = new int[Size];
                            for (int i = 0; i < Size; i++)
                            {
                                DRD.IntValue[i] = int.Parse(RxMsg.Substring(7 + (i * 4), 4), System.Globalization.NumberStyles.HexNumber);
                                //if (DRD.IntValue[)
                            }

                            
                        }
                        break;
                    case 1:
                        if (int.Parse(RxMsg.Substring(5, 2), System.Globalization.NumberStyles.HexNumber) == Size / 8)
                        {
                            DRD.BitStatus = new bool[Size];
                            for (int i = 0; i < Size / 8; i++)
                            {
                                byte temp = byte.Parse(RxMsg.Substring(7 + (i * 2), 2), System.Globalization.NumberStyles.HexNumber);
                                for (int j = 0; j < 8; j++)
                                {
                                    DRD.BitStatus[(i * 8) + j] = Convert.ToBoolean(temp & 0x1);
                                    temp >>= 1;
                                }
                            }
                        }
                        break;
                    case 2:
                        if (int.Parse(RxMsg.Substring(5, 2), System.Globalization.NumberStyles.HexNumber) == (Size / 8) + 1)
                        {
                            //if (Size < 9) Size = 8;
                            DRD.BitStatus = new bool[Size];
                            for (int i = 0; i < (Size / 8); i++)
                            {
                                byte temp = byte.Parse(RxMsg.Substring(7 + (i * 2), 2), System.Globalization.NumberStyles.HexNumber);
                                for (int j = 0; j < 8; j++)
                                {
                                    DRD.BitStatus[(i * 8) + j] = Convert.ToBoolean(temp & 0x1);
                                    temp >>= 1;

                                }
                            }
                        }
                        break;
                }
            }
            return DRD;
        }
        private bool CheckValidMessage(string RxMsg)
        {
            byte bTemp = 0;
            try
            {
                //check if response messsage is valid
                if (RxMsg.Substring(3, 1).Equals("8"))
                {
                    switch (int.Parse(RxMsg.Substring(4, 1), System.Globalization.NumberStyles.HexNumber))
                    {
                        case 1:
                            incomingInfo.listDebugInfo.Add("Illegal command code\r");
                            break;
                        case 2:
                            incomingInfo.listDebugInfo.Add("Illegal device address\r");
                            break;
                        case 3:
                            incomingInfo.listDebugInfo.Add("Illegal device value\r");
                            break;
                        case 7:
                            incomingInfo.listDebugInfo.Add("Check Sum Error\r");
                            break;
                    }
                    return false;
                }

                //calculate checksum
                //if(RxMsg.IndexOf("\r\n")==-1) RxMsg=RxMsg+ "\r\n";

                for (int i = 0; i < RxMsg.Length / 2 - 1; i++)
                    bTemp += Byte.Parse(RxMsg.Substring((i * 2) + 1, 2), System.Globalization.NumberStyles.HexNumber);

                if (bTemp == 0)
                    return true;
                else
                {
                    incomingInfo.listDebugInfo.Add("illigal CheckSum\r");
                    return false;
                }
            }
            catch
            {
                incomingInfo.listDebugInfo.Add("Parity Error\r");
                return false;
            }
        }

        private string SendAndRecieveData(string tx)
        {
            string rx = string.Empty;

            incomingInfo.listDebugInfo.Add(string.Format("Tx:{0}", tx));

            try
            {
                serial.port.DiscardInBuffer();
                serial.port.WriteLine(tx + "\r\n");
                rx = serial.port.ReadLine().Substring(2);
                incomingInfo.listDebugInfo.Add(string.Format("Rx:{0}\r", rx));
            }
            catch(Exception ex)
            {
                incomingInfo.listDebugInfo.Add(string.Format("response timeout" + Environment.NewLine + ex.Message));
            }
            return rx;
        }




        public string WritePressureTableToPLC(List<Int16> PressureTable)
        {
            SendNewMessage(DeltaMsgType.PresetMultipleRegister, DeltaMemType.D, 300, (byte)PressureTable.Count, PressureTable);
            return RxMsg;
        }

        public void classDeltaWriteSetpoint(List<Int16> SetPoint)
        {
            SendNewMessage(DeltaMsgType.PresetMultipleRegister, DeltaMemType.D, 305,1, SetPoint);
        }

#if xx
        public string WriteWordToDataRegister(int iAddress, int iSize, int value)
        {
            byte bcheckSum;
            string stSendBuf;
            string rx = string.Empty;
            incomingInfo.listDebugInfo.Add(string.Format("Tx:{0}", value));

            bcheckSum = 0;
            stSendBuf = ":";

            stSendBuf += "01"; // slave address
            bcheckSum += 01; //Hex

            stSendBuf += "06"; // CMD
            bcheckSum += 6; //Hex



            //Hibyte of address
            stSendBuf += Convert.ToString(((iAddress + D_OFFSET) / 256), 16).PadLeft(2, '0');
            bcheckSum += (byte)((iAddress + D_OFFSET) / 256);
            //Lowbyte of address
            stSendBuf += Convert.ToString(((iAddress + D_OFFSET) & 255), 16).PadLeft(2, '0');
            bcheckSum += (byte)((iAddress + D_OFFSET) & 255);


            //Hibyte  of data
            stSendBuf += Convert.ToString(((value) / 256), 16).PadLeft(2, '0');
            bcheckSum += (byte)((value) / 256);
            //Lowbyte  of data
            stSendBuf += Convert.ToString(((value) & 255), 16).PadLeft(2, '0');
            bcheckSum += (byte)((value) & 255);



            stSendBuf += Convert.ToString((0x100 - bcheckSum), 16).PadLeft(2, '0');
            stSendBuf = stSendBuf.ToUpper();

            try
            {
                serial.port.WriteLine(stSendBuf + "\r\n");
                rx = serial.port.ReadLine().Substring(2);
                incomingInfo.listDebugInfo.Add(string.Format("Rx:{0}\r", rx));
            }
            catch (Exception ex)
            {
                incomingInfo.listDebugInfo.Add(string.Format("response timeout\r\n"));
            }
            return rx;
        }
#endif
        #endregion
    }
}

