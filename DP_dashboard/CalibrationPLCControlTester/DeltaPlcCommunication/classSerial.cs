using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO.Ports;
using System.Diagnostics;

namespace DeltaPlcCommunication
{
    public class classSerial
    {
        public SerialPort port;
        public bool ComPortOk = true;
        public string ComPortErrorMessage = "";


        public bool IsComOpen()
        {
            return port.IsOpen;
        }

        public classSerial( string name, int baud, SerialDataReceivedEventHandler handler)
        {
            port = new SerialPort(name, baud,Parity.Even,7,StopBits.One);
            port.Handshake = Handshake.None;

            port.ReadTimeout = 3000;
           // source port.ReadTimeout = 1000;
            if (handler != null)
            {
                port.DataReceived += handler;
            }
            try
            {
                if (!port.IsOpen)
                {
                    port.Open();
                }
            }
            catch (Exception ex)
            {
                //throw new System.ArgumentException(ex.Message);
                ComPortOk = false;
                ComPortErrorMessage = string.Format("Error: {0} not exist. COM function - DELTA(PLC)." + Environment.NewLine + ex.Message, port.PortName);
            }

        }

        public void Send(byte[] data, int size)
        {
            try
            {
                if (port.IsOpen)
                {
#if XX
                    for (int i = 0; i < size; i++)
                    {

                        if(size > 20 )
                        {
                            if(i % 10 ==0 )
                            {
                                System.Threading.Thread.Sleep(1);
                            }
                        }
                        port.Write( data, i, 1 );
                    }
#endif
                    port.Write( data, 0, size );
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine(ex.Message);
            }
        }

        public int Recieve(byte[] data, int size, int timeoutMili = 100 )
        {
            DateTime start = DateTime.Now;
            TimeSpan delta;
            try
            {
                if (port.IsOpen)
                {
                    while (port.BytesToRead < size)
                    {
                        delta = DateTime.Now - start;
                        if (delta.TotalMilliseconds > timeoutMili)
                        {
                            return 0;
                        }
                    }
                    port.Read(data, 0, size);
                }
                return size;
            }
            catch (Exception ex)
            {
                Debug.WriteLine(ex.Message);
                return 0;
            }
        }
    }
}
