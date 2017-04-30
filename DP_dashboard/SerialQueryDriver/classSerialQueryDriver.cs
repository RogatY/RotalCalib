using System;
using System.Collections.Generic;
using System.Management;


namespace SerialQueryDriver
{

    public class Device
    {
        public string comName { get; set; }
        public string id { get; set; }
        public bool paired = false;
        public string pairedTo = "-";

        public override string ToString()
        {
            string deviceStr = string.Format("\r\n\r\nDevice detiles:\r\nid = {0}.\r\nCom name = {1}.\r\nCom is busy - {2}\r\nControll the {3}.\r\n\r\n", id, comName, paired == true ? "YES." : "NO.", pairedTo);
            return deviceStr;
        }
    }

    public static class classSerialQueryDriver
    {

        static public List<Device> devices = new List<Device>();

        public static void GetComPortName(ref string multiplexerComName, string multiplexerID, ref string plcComName, string plcID, ref string dpComName, string dpID, ref string tempControllerComName, string tempControllerID)
        {
            //Below is code pasted from WMICodeCreator
            try
            {
                ManagementObjectSearcher searcher =
                new ManagementObjectSearcher("root\\WMI",
                "SELECT * FROM MSSerial_PortName");

                string[] MultiplexerSplitId = multiplexerID.Split('\\');
                string[] PlcSplitId = plcID.Split('\\');
                string[] DpSplitId = dpID.Split('\\');
                string[] TempControllerSplitId = tempControllerID.Split('\\');


                string[] currentSplitId = new string[4];


                foreach (ManagementObject queryObj in searcher.Get())
                {



                    Device newDevice = new Device();
                    newDevice.comName = queryObj["PortName"].ToString();
                    newDevice.id = queryObj["InstanceName"].ToString();


                    currentSplitId = queryObj["InstanceName"].ToString().Split('\\');


                    if (MultiplexerSplitId[0] == currentSplitId[0] && MultiplexerSplitId[1] == currentSplitId[1])
                    {
                        multiplexerComName = queryObj["PortName"].ToString();
                        newDevice.pairedTo = "Multiplexer";
                        newDevice.paired = true;
                    }

                    if (PlcSplitId[0] == currentSplitId[0] && PlcSplitId[1] == currentSplitId[1])
                    {
                        plcComName = queryObj["PortName"].ToString();
                        newDevice.pairedTo = "PLC";
                        newDevice.paired = true;
                    }

                    if (DpSplitId[0] == currentSplitId[0] && DpSplitId[1] == currentSplitId[1])
                    {
                        dpComName = queryObj["PortName"].ToString();
                        newDevice.pairedTo = "DP";
                        newDevice.paired = true;
                    }

                    if (TempControllerSplitId[0] == currentSplitId[0] && TempControllerSplitId[1] == currentSplitId[1])
                    {
                        tempControllerComName = queryObj["PortName"].ToString();
                        newDevice.pairedTo = "Oven";
                        newDevice.paired = true;
                    }

                    Console.WriteLine("InstanceName: {0}", queryObj["InstanceName"]);
                    Console.WriteLine("PortName: {0}", queryObj["PortName"]);
                    if (queryObj["InstanceName"].ToString().Contains("USB"))
                    {
                        Console.WriteLine(queryObj["PortName"] + " is a USB to SERIAL adapter / converter");
                    }

                    devices.Add(newDevice);
                }
            }
            catch (ManagementException ex)
            {
                System.Windows.Forms.MessageBox.Show("Fail to scan comports.\r\n\r\n(1)Try restart the application as a administrator\r\n(2)Check that you connected well the comport cable." + Environment.NewLine + ex.Message);
            }



#if false  //this is an example of existing serial devices query

                    public void GetComPortName(string device)
        {
            //Below is code pasted from WMICodeCreator
            try
            {
                ManagementObjectSearcher searcher =
                    new ManagementObjectSearcher("root\\WMI",
                    "SELECT * FROM MSSerial_PortName");

                foreach (ManagementObject queryObj in searcher.Get())
                {
                    Console.WriteLine("-----------------------------------");
                    Console.WriteLine("MSSerial_PortName instance");
                    Console.WriteLine("-----------------------------------");
                    Console.WriteLine("InstanceName: {0}", queryObj["InstanceName"]);

                    Console.WriteLine("-----------------------------------");
                    Console.WriteLine("MSSerial_PortName instance");
                    Console.WriteLine("-----------------------------------");
                    Console.WriteLine("PortName: {0}", queryObj["PortName"]);

                    //If the serial port's instance name contains USB 
                    //it must be a USB to serial device
                    if (queryObj["InstanceName"].ToString().Contains("USB"))
                    {
                        Console.WriteLine(queryObj["PortName"] + " is a USB to SERIAL adapter / converter");
                    }
                }
            }
            catch (ManagementException ex)
            {
              //MessageBox.Show("An error occurred while querying for WMI data: " + ex.Message);
            }

#endif

        }
    }
}
