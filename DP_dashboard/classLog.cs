using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Windows.Forms;
using DpCommunication;

namespace DP_dashboard
{
    class classLog
    {
        private string logFileName;
        private StreamWriter file;
        private string sPath;

        private string GetTime()
        {
            DateTime now = DateTime.Now;
            return string.Format("{0}:{1}:{2}:{3}",now.Hour,now.Minute,now.Second,now.Millisecond);
        }

        void CreateDirectoryIfNotExist(string path)
        {
            bool exists = System.IO.Directory.Exists(path);

            if (!exists)
                System.IO.Directory.CreateDirectory(path);
        }


        public string OpenFileForLogging(List<float> pressuresUnderTeat,string path,ClassDevice dpDevice)
        {
            string title1 = ",";
            DateTime now = DateTime.Now;

            sPath = path;

            logFileName = string.Format("{0:0000}_{1:00}_{2:00}_{3:00}_{4:00}_{5:00}_{6}",now.Year,now.Month,now.Day,now.Hour,now.Minute,now.Second, dpDevice.DeviceSerialNumber) + ".csv";
            dpDevice.CSVFileName = logFileName;
            CreateDirectoryIfNotExist(path);

            try
            {
                file = new StreamWriter(path + @"\" + logFileName, false);
                file.WriteLine("");



                title1 = string.Format("DP Name(SN),{0}", dpDevice.DeviceSerialNumber);
                file.WriteLine(title1);

                title1 = string.Format("MAC address,{0}", dpDevice.DeviceMacAddress);
                file.WriteLine(title1);

                title1 = string.Format("Calibration date,{0}", now);
                file.WriteLine(title1);

                file.WriteLine("");
                file.WriteLine("");
            }





            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine(ex.Message);
            }
            return logFileName;
        }

        public string PrintLogRecordToFile(ClassDevice device, List<float> pressuresUnderTest, string path, byte tempIndex)
        {
            string path1 = path + @"\" + device.CSVFileName;

            try
            {
                file = new StreamWriter(path1, true);
            }
            catch
            {
                return "Log error: fail to open " + device.CSVFileName + " file to logging\r\nCheck Device name is correct!\r\n" + "Current device position is " + device.PositionOnBoard;
            }

            if (file != null)
            {
                //write temp header
                string line = string.Format(",,,Temp#{0},", tempIndex);
                file.WriteLine(line);

                line = ",TIME,PLC_Pressure,A2D_Right,A2D_Left,A2D Delta,Temp on DP,";
                file.WriteLine(line);


                for (int i = 0; i < pressuresUnderTest.Count; i++)
                {
                    line = device.CalibrationData[tempIndex, i].time.ToString();
                    line += "," + string.Format("Pressure {0} = {1}[bar]", i,pressuresUnderTest[i]);
                    line += "," + device.CalibrationData[tempIndex, i].extA2dPressureValue;
                    line += "," + device.CalibrationData[tempIndex,i].RightA2DValue;
                    line += "," + device.CalibrationData[tempIndex, i].LeftA2DValue;
                    line += "," + Math.Abs(device.CalibrationData[tempIndex, i].RightA2DValue - device.CalibrationData[tempIndex, i].LeftA2DValue);
                    line += "," + device.CalibrationData[tempIndex, i].tempOnDevice;


                    file.WriteLine(line);
                }
                file.WriteLine("");
                file.WriteLine("");
                file.Close();
            }
            return "";
        }

        public void CloseFileForLogging()
        {
            try
            {
                file.Close();
                file = null;
            }
            catch//(Exception e)
            {

            }
        }
    }
}
