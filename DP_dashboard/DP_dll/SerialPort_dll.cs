using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace SerialPort_dll
{
    public class classSerial
    {
        public SerialPort port;
        public bool ComPortOk = true;
        public string ComPortErrorMessage = "";
        public object MessageBox { get; private set; }
        private volatile bool _transmitSemaphore = true;
        private object locker = new object();
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
            bool transmit_ok = false;
            lock (locker)
            {
                if (_transmitSemaphore)
                {
                    _transmitSemaphore = false;
                    transmit_ok = true;
                }

            }

            if (!transmit_ok)
            {
                Stopwatch sw = new Stopwatch();
                sw.Start();
                while ((sw.ElapsedMilliseconds < 500) && (!_transmitSemaphore))
                {
                    Thread.Sleep(10);
                }
            }
            
            try
            {
                if (port.IsOpen)
                {
                    _transmitSemaphore = false;
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
                            _transmitSemaphore = true;
                            return 0;
                        }
                    }
                    port.Read(data, 0, size);
                    _transmitSemaphore = true;
                }
                return size;
            }
            catch (Exception ex)
            {
                Debug.WriteLine(ex.Message);
                _transmitSemaphore = true;
                return 0;
            }
        }

        public bool Semaphore
        {
             get { return _transmitSemaphore; } 
             set { _transmitSemaphore = value; } 
        }

        private void FixComunicationProblem()
        {
            _transmitSemaphore = true;
            while (port.BytesToRead > 0)
            {
                int temp;
                temp = port.ReadByte();
            }

        }

    }
}




