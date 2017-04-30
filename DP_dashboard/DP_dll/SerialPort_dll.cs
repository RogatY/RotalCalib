using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SerialPort_dll
{
    public class classSerial
    {
        public SerialPort port;
        public bool ComPortOk = true;
        public string ComPortErrorMessage = "";
        public object MessageBox { get; private set; }

        public bool IsComOpen()
        {
            return port.IsOpen;
        }

        public classSerial(string name, int baud, SerialDataReceivedEventHandler handler)
        {
            port = new SerialPort(name, baud);
            port.Handshake = Handshake.None;
            port.ReadTimeout = 1000;
            if (handler != null)
            {
                port.DataReceived += handler;
            }
            try
            {
                if (!port.IsOpen)
                {
                    port.Open();
                    ComPortOk = true;

                    FixComunicationProblem();
                }
            }
            catch (Exception ex)
            {
                //MessageBox.Show(ex.Message);
                ComPortOk = false;
                ComPortErrorMessage = string.Format("Error:{0} not exist. COM function - DP comunication." + Environment.NewLine + ex.Message, name);
            }

        }

        public void Send(byte[] data, int size)
        {
            try
            {
                if (port.IsOpen)
                {
                    port.Write(data, 0, size);
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine(ex.Message);
            }
        }

        public int Recieve(byte[] data, int size, int timeoutMili = 100)
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

        private void FixComunicationProblem()
        {
            while (port.BytesToRead > 0)
            {
                int temp;
                temp = port.ReadByte();
            }

        }

    }
}




