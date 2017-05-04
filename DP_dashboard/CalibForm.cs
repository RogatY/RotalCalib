using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;
using multiplexing_dll;
using DeltaPlcCommunication;
using DpCommunication;
using System.Diagnostics;
using TempController_dll;
using SerialQueryDriver;
using log4net;
using System.Reflection;
using System.Threading;

namespace DP_dashboard
{

    public struct SwVersion
    {
        public string MultiPlexerFw;
        public string DpFw;
        public string CalibrationTool;

        public SwVersion(string multiPlexerFw, string dpFw, string calibrationTool)
        {
            MultiPlexerFw = multiPlexerFw;
            DpFw = dpFw;
            CalibrationTool = calibrationTool;
        }
    }

    public partial class CalibForm : Form
    {
        private static readonly ILog Logger = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);


        public static CalibForm currentForm;

        //cosntantsC:\work\Rotal\Rotal calibration\project\RotalCalib\DP_dashboard\CalibForm.cs
        private const int MAX_PRESSURE_POINT = 0x0f;

        // mult plexing protocol instance
        private MultiplexingIncomingInformation MultiplexingInfo;
        public classMultiplexing classMultiplexing;
        public string MutiplexingComPortName = "NULL";
        // end

        // plc protocol instance
        public classDeltaProtocol ClassDeltaProtocol;
        private DeltaIncomingInformation PLCinfo;
        private DeltaReturnedData IncumingParametersFromPLC;
        public string PlcComPortName = "NULL";
        // end   

        // DP protocol instance  
        public ClassDpCommunication classDpCommunication;
        private DpIncomingInformation DPinfo;
        public string DpComPortName = "NULL";
        // end  

        // temp controller protocol instance
        public TempControllerProtocol tempControllerInstanse;
        public string TempControllerComPortName = "NULL";
        // end  
         

        private classLog log = new classLog();
        public ClassCalibrationInfo classCalibrationInfo;
        private string CurrentSnDeviceIsFocus = "";

        ConfigForm ConfigFormInstanse;
        DateTime UpdateTempTime = new DateTime();

        SwVersion swVersions;

        public CalibForm()
        {
            InitializeComponent();
            currentForm = this;

            // check the names of the comports...
            /*
            classSerialQueryDriver.GetComPortName(
                                                    ref MutiplexingComPortName,
                                                    Properties.Settings.Default.MultiplexerComPortID,
                                                    ref PlcComPortName,
                                                    Properties.Settings.Default.PlcComPortID,
                                                    ref DpComPortName,
                                                    Properties.Settings.Default.DpComPortID,
                                                    ref TempControllerComPortName,
                                                    Properties.Settings.Default.TempControllerComPortID);
                                                    */



            // plc protocol init          
            PlcComPortName = Properties.Settings.Default.plcComPort;
            PLCinfo = new DeltaIncomingInformation();
            ClassDeltaProtocol = new classDeltaProtocol(PlcComPortName, 9600, PLCinfo);

            // multplexing protocol init 
            MutiplexingComPortName = Properties.Settings.Default.multiplexingComPort;
            MultiplexingInfo = new MultiplexingIncomingInformation();
            classMultiplexing = new classMultiplexing(MutiplexingComPortName, 115200, MultiplexingInfo);


            // DP protocol init
            DpComPortName = Properties.Settings.Default.dpComPort;
            DPinfo = new DpIncomingInformation();
            classDpCommunication = new ClassDpCommunication(DpComPortName, 115200, DPinfo);


            // Temp controller protocol init
            TempControllerComPortName = Properties.Settings.Default.TempControllerComPort;
            tempControllerInstanse = new TempControllerProtocol(TempControllerComPortName, 9600, true);
            //tempControllerInstanse = new TempControllerProtocol("COM16", 9600);

            //update versions
            swVersions = new SwVersion("1.0.0", "", Application.ProductVersion);
            
            // Calibration class init           
            classCalibrationInfo = new ClassCalibrationInfo(tempControllerInstanse, classDpCommunication, classMultiplexing, ClassDeltaProtocol, swVersions);



            UpdateTempTime = DateTime.Now;


            foreach (Device dev in classSerialQueryDriver.devices)
            {
                    rtb_info.AppendText(dev.ToString());
                    Console.WriteLine(dev.ToString());
            }
            rtb_info.ScrollToCaret();
            timer1.Enabled = true;
        }


        private void Form1_Load(object sender, EventArgs e)
        {
            this.Text += "  Ver " + swVersions.CalibrationTool;
            //LoadDefoultCalibPointToList();

            tb_logsPath.Text = Properties.Settings.Default.LogPath;

            //ClassMail.SendCalibrationDone();
        }

        private void bt_writePressureTableToPlc_Click(object sender, EventArgs e)
        {

            //List<Int16> PressureTable = new List<Int16>();
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest1 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest2 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest3 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest4 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest5 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest6 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest7 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest8 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest9 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest10 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest11 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest12 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest13 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest14 * 10));
            //PressureTable.Add((Int16)(Properties.Settings.Default.PressureUnderTest15 * 10));

            //rtb_info.Text += "PLC ->  " +  classCalibrationInfo.classDeltaProtocolInstanse.WritePressureTableToPLC(PressureTable) +  "\r\n";


            // SHLOM integration
            //List<Int16> SetPointPressure = new List<Int16>();

            //SetPointPressure.Add(Int16.Parse(tb_setpoint.Text));
            //classCalibrationInfo.classDeltaProtocolInstanse.classDeltaWriteSetpoint(SetPointPressure);
        }

        private void bt_readPressureTableFromPlc_Click(object sender, EventArgs e)
        {
            //IncumingParametersFromPLC = DeltaProtocolInstanse.SendNewMessage(DeltaMsgType.ReadHoldingRegisters, DeltaMemType.D, 300, MAX_PRESSURE_POINT);

            //if (DeltaProtocolInstanse.newPressureTableReceive == true)
            //{
            //    DeltaProtocolInstanse.newPressureTableReceive = false;


            //    for (int i = 1; i <= MAX_PRESSURE_POINT; i++)
            //    {
            //        //dgv_prressureTable.Rows[i - 1].Cells[1].Value = IncumingParametersFromPLC.IntValue[i - 1].ToString();
            //    }

            //}

            //classCalibrationInfo.classDeltaProtocolInstanse.SendNewMessage
            //Shalom integration
            IncumingParametersFromPLC = classCalibrationInfo.classDeltaProtocolInstanse.SendNewMessage(DeltaMsgType.ReadHoldingRegisters, DeltaMemType.D, 300, 1);
            rtb_info.Text += "flag register " + IncumingParametersFromPLC.IntValue[0].ToString() + "\r\n";

            IncumingParametersFromPLC = classCalibrationInfo.classDeltaProtocolInstanse.SendNewMessage(DeltaMsgType.ReadHoldingRegisters, DeltaMemType.D, 301, 1);
            rtb_info.Text += "PV = " + IncumingParametersFromPLC.IntValue[0].ToString() + "\r\n";



        }



        private void timer1_Tick(object sender, EventArgs e)
        {
            CheckComPorts();
#if true
            if (!classCalibrationInfo.CriticalStates && !classCalibrationInfo.ConnectingToDP)
            {
                if (CheckTimout(UpdateTempTime, 1))
                {
                    classCalibrationInfo.classDpCommunicationInstanse.DPgetDpInfo();
                    classCalibrationInfo.classDpCommunicationInstanse.NewDpInfoEvent = false;

                    tb_temperatureOnDP.Text = classCalibrationInfo.classDpCommunicationInstanse.dpInfo.CurrentTemp.ToString();

                    UpdateTempTime = DateTime.Now;
                }

            }

#endif

            UpdateGUI();
            if(classCalibrationInfo.TraceInfo != "")
            {
                rtb_info.AppendText(classCalibrationInfo.TraceInfo);
                rtb_info.ScrollToCaret();
                classCalibrationInfo.TraceInfo = "";
            }

            if (classCalibrationInfo.FinishCalibrationEvent)
            {
                classCalibrationInfo.FinishCalibrationEvent = false;
                UpdateColorStatus();
                UpdateDataTable(CurrentSnDeviceIsFocus);
                MessageBox.Show("Calibration done!");
            }

            if (classCalibrationInfo.TempTimoutErrorEvent)
            {
                classCalibrationInfo.TempTimoutErrorEvent = false;

                DialogResult result = MessageBox.Show("Fail to set temperature!", "Error", MessageBoxButtons.OK);
                if (result == DialogResult.OK)
                {
                    classCalibrationInfo.NextAfterTempTimoutErrorEvent = true;
                }
            }

            if (classCalibrationInfo.PressureTimoutErrorEvent)
            {
                classCalibrationInfo.PressureTimoutErrorEvent = false;

                DialogResult result = MessageBox.Show("Fail to set pressure!", "Error", MessageBoxButtons.OK);
                if (result == DialogResult.OK)
                {
                    classCalibrationInfo.NextAfterPressureTimoutErrorEvent = true;
                }
            }


            if (classCalibrationInfo.classCalibrationSettings.AlertToTechnican)
            {
                classCalibrationInfo.classCalibrationSettings.AlertToTechnican = false;
                DialogResult result = MessageBox.Show("Pressure stable. the system wait to tachnicatan approve.", "Pressure info", MessageBoxButtons.OK);
                if (result == DialogResult.OK)
                {
                    classCalibrationInfo.classCalibrationSettings.TechnicianApproveGoNext = true;
                }
            }



            if (classCalibrationInfo.ErrorEvent)
            {
                classCalibrationInfo.ErrorEvent = false;
                rtb_info.Text += "Error:   " + classCalibrationInfo.ErrorMessage + DateTime.Now.ToString() + "\r\n";
            }

            if (classCalibrationInfo.IncermentCalibPointStep)
            {
                classCalibrationInfo.IncermentCalibPointStep = false;
                string Message = string.Format("Calibration in process :  {0}  Temp index:{1}. Pressure index:{2}", DateTime.Now.ToString(), classCalibrationInfo.CurrentCalibTempIndex.ToString(), classCalibrationInfo.CurrentCalibPressureIndex.ToString());
                rtb_info.Text += Message + "\r\n";

                UpdateDataTable(CurrentSnDeviceIsFocus);
            }

            if (classCalibrationInfo.ClassTempControllerInstanse.TempControllerConnectionEvent)
            {
                classCalibrationInfo.ClassTempControllerInstanse.TempControllerConnectionEvent = false;
                if (classCalibrationInfo.ClassTempControllerInstanse.TempControllerConnectionStatus)
                {
                    classCalibrationInfo.ClassTempControllerInstanse.TempControllerConnectionStatus = false;
                }
            }

            if (classCalibrationInfo != null && classCalibrationInfo.classDeltaProtocolInstanse.serial.ComPortOk && classCalibrationInfo.ClassTempControllerInstanse.ComPortOk)
            {
                classCalibrationInfo.UpdateRealTimeData(this);

                //UpdateCurrentTemp(classCalibrationInfo.CurrentTempControllerValue.ToString());
                tb_pressCurrentPressure.Text = classCalibrationInfo.CurrentPLCPressure.ToString();
            }


            

        }
        public void UpdateCurrentTemp(string value)
        {
            if (InvokeRequired)
            {
                this.Invoke(new Action<string>(UpdateCurrentTemp), new object[] { value });
                return;
            }
            tb_currentTemperature.Text = value;
        }

        private void UpdateGUI()
        {

            if (classCalibrationInfo.DoCalibration)
            {
                try
                {
                    tb_timeFromSendTemp.Text = DateTime.Now.Subtract(classCalibrationInfo.TimeFromSetTempPointRequest).Minutes.ToString();

                    UpdateCalibrationGUI();
                }
                catch (Exception ex)
                {
                    rtb_info.Text += "update GUI error\r\n";
                    rtb_info.Text += ex.Message;
                }

            }


            UpdateProgressBar();
            CalibrationButtonHandele();
            MarkConnectedDevice();
        }

        private void UpdateProgressBar()
        {
            if (classCalibrationInfo.DoCalibration)
            {
                pb_calibProgressBar.Visible = true;

                int TotalCalibPoints = classCalibrationInfo.classCalibrationSettings.TempUnderTestList.Count * classCalibrationInfo.classCalibrationSettings.PressureUnderTestList.Count;
                int FinishCalibPoints = classCalibrationInfo.CurrentCalibTempIndex * classCalibrationInfo.classCalibrationSettings.PressureUnderTestList.Count + classCalibrationInfo.CurrentCalibPressureIndex;

                float Precent = ((float)FinishCalibPoints / (float)TotalCalibPoints) * 100;
                pb_calibProgressBar.Value = Convert.ToInt16(Precent);
            }
            else
            {
                pb_calibProgressBar.Visible = false;
                pb_calibProgressBar.Value = 0;
            }

        }

        //private void pnl_plcControl_Paint(object sender, PaintEventArgs e)
        //{

        //}

        //private void bt_connectToDp_Click(object sender, EventArgs e)
        //{
        //    //byte DpId = (byte)(int.Parse(cmb_dpDeviceNumber.SelectedItem.ToString()));
        //    //MultiplexingProtocolInstanse.ConnectDpDevice(DpId);            
        //}

        private void bt_disconnect_Click(object sender, EventArgs e)
        {
            classMultiplexing.DisConnectAllDp();
        }

        private bool FlashDpDevice(string fileName)
        {
            string path = @"C:\Program Files (x86)\Texas Instruments\SmartRF Tools\Flash Programmer\bin\SmartRFProgConsole.exe";
            string args = string.Format("S() EPV F={0}", fileName);
            Process burn = new Process();
            burn.StartInfo.FileName = path;
            burn.StartInfo.Arguments = args;
            burn.StartInfo.WindowStyle = ProcessWindowStyle.Normal;
            burn.Start();
            burn.WaitForExit();
            return burn.ExitCode == 0;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            bool res = FlashDpDevice(@"C:\work\grow_me\projects\BLE-CC254x-1.4.0\Projects\ble\grow_me\CC2541DB\CC2541\Exe\SimpleBLEPeripheral.hex");

            if (!res)
            {
                MessageBox.Show("burn fail");
            }
        }

        private void bt_writePressursToDP_Click(object sender, EventArgs e)
        {
            // DpProtocolInstanse.SendDpSerialNumber(System.Text.Encoding.ASCII.GetBytes(tb_dpSerialNumber.Text));

            classDpCommunication.SendPressuresTableToDP();
        }

        private void bt_exportPressursTableToCSVfile_Click(object sender, EventArgs e)
        {
            //log.OpenFileForLogging(Application.StartupPath + @"\Logs", "1.0.0", "1.0.0","6778899");
            //log.PrintLogRecordToFile(DpProtocolInstanse);
            //log.CloseFileForLogging();
        }

        //private void panel1_Paint(object sender, PaintEventArgs e)
        //{

        //}

        private void bt_getDPinfo_Click(object sender, EventArgs e)
        {
            classDpCommunication.DPgetDpInfo();
        }

        private void bt_configuration_Click(object sender, EventArgs e)
        {
            ConfigForm configForm = new ConfigForm(classDpCommunication, this, swVersions.CalibrationTool);
            this.Hide();
            configForm.Show();
        }

        //private void rtbLog_TextChanged(object sender, EventArgs e)
        //{

        //}

        private void bt_startCalibration_Click(object sender, EventArgs e)
        {
            if (ValidStationAccess())
            {
                try
                {
                    //check connection to database.....
                    int userId = RIT_QA.ClassDal.GetFirstUserID();

                    classCalibrationInfo.DetectFlag = true;
                    classCalibrationInfo.InitDetectTread();

                    //wait to finish the detect.
                    while (!classCalibrationInfo.EndDetectEvent)
                    {
                        Application.DoEvents();
                    }

                    classCalibrationInfo.EndDetectEvent = false;
                    UpdateDeviceTable();


                    if (Properties.Settings.Default.DebugMode == false)
                    {
                        if (classCalibrationInfo.DpCountAxist > 0 &&
                            classCalibrationInfo.classCalibrationSettings.PressureUnderTestList.Count > 0 &&
                            classCalibrationInfo.classCalibrationSettings.TempUnderTestList.Count > 0 &&
                            classCalibrationInfo.classCalibrationSettings.Versions.DpFw != "")
                        {
                            UpdateDeviceTable();


                            classDpCommunication.SendStartCalibration();

                            classCalibrationInfo.ResetStateMachine();
                            classCalibrationInfo.DoCalibration = true;
                            classCalibrationInfo.CalibrationPaused = false;
                            classCalibrationInfo.InitCalibTread();
                            classCalibrationInfo.CreateLogFiles();
                            ClearColorIndication();
                        }
                        else
                        {
                            if (classCalibrationInfo.DpCountAxist == 0)
                            {
                                MessageBox.Show("No exist dp devices!");
                            }
                            else if (classCalibrationInfo.classCalibrationSettings.PressureUnderTestList.Count == 0)
                            {
                                MessageBox.Show("Load configuration file befor you calibration start");
                            }
                            else if (classCalibrationInfo.classCalibrationSettings.TempUnderTestList.Count == 0)
                            {
                                MessageBox.Show("Load configuration file befor you calibration start");
                            }

                        }
                    }

                }
                catch (Exception ex)
                {
                    rtb_info.AppendText(ex.StackTrace.ToString() + "\r\n\r\n" + ex.InnerException.ToString() + ".\r\n\r\n");
                    Logger.Error("Faile to add table to database.   " + ex.StackTrace.ToString() + "       " + ex.InnerException.ToString());
                }
            }else
            {
                DialogResult result = MessageBox.Show("Enter User name and Station id in config Form", "Warning", MessageBoxButtons.OK);
            }
        }

        private bool ValidStationAccess()
        {
            if (classCalibrationInfo.classCalibrationSettings.StationId > 0 && classCalibrationInfo.classCalibrationSettings.UserName != "")
                return true;
                    return false;
        }

        private void UpdateDeviceTable()
        {
            dgv_devicesQueue.Rows.Clear();
            for (int i = 0; i < classCalibrationInfo.DpCountAxist; i++)
            {
                dgv_devicesQueue.Rows.Add(i.ToString(),
                    classCalibrationInfo.classDevices[i].DeviceMacAddress.ToString(),
                    classCalibrationInfo.classDevices[i].DeviceBarcode.ToString(),
                    classCalibrationInfo.classDevices[i].PositionOnBoard.ToString(),
                    classCalibrationInfo.classDevices[i].BoardNumber.ToString());
            }
        }
        private void UpdateDataTable(string barcode)
        {
            bool ExistDevice = false;
            int i = 0;
            for (i = 0; i < classCalibrationInfo.DpCountAxist; i++)
            {
                if (classCalibrationInfo.classDevices[i].DeviceBarcode == barcode)
                {
                    ExistDevice = true;
                    break;
                }
            }

            int deviceIndex = i;

            if (ExistDevice)
            {
                dgv_deviceData.Rows.Clear();
                ExistDevice = false;
                string[] dataRow = new string[classCalibrationInfo.classCalibrationSettings.TempUnderTestList.Count * 2 + 1];

                for (i = 0; i < classCalibrationInfo.classCalibrationSettings.PressureUnderTestList.Count; i++)
                {
                    dataRow[0] = classCalibrationInfo.classCalibrationSettings.PressureUnderTestList[i].ToString();
                    for (int j = 0; j < classCalibrationInfo.classCalibrationSettings.TempUnderTestList.Count; j++)
                    {
                        dataRow[j * 2 + 1] = classCalibrationInfo.classDevices[deviceIndex].CalibrationData[j, i].LeftA2DValue.ToString();
                        dataRow[j * 2 + 2] = classCalibrationInfo.classDevices[deviceIndex].CalibrationData[j, i].RightA2DValue.ToString();
                    }
                    dgv_deviceData.Rows.Add(dataRow);
                }
            }

        }

        private void dgv_devicesQueue_CellEnter(object sender, DataGridViewCellEventArgs e)
        {
            if (dgv_devicesQueue.Rows[e.RowIndex].Cells[2].Value != null)
            {
                CurrentSnDeviceIsFocus = dgv_devicesQueue.Rows[e.RowIndex].Cells[2].Value.ToString();
            }

            UpdateDataTable(CurrentSnDeviceIsFocus);

        }


        void UpdateCalibrationGUI()
        {
            //temp controller
            tb_targetTemperature.Text = classCalibrationInfo.classCalibrationSettings.TempUnderTestList[classCalibrationInfo.CurrentCalibTempIndex].ToString();
            tb_currentSkipTime.Text = ((classCalibrationInfo.classCalibrationSettings.TempSkipStartTime[classCalibrationInfo.CurrentCalibTempIndex]) / 60).ToString() + " [min]";
            //tb_temperatureOnDP.Text = classCalibrationInfo.CurrentTempOnDP.ToString();

            //pressure
            tb_pressTargetPressure.Text = classCalibrationInfo.PlcBar2Adc(classCalibrationInfo.classCalibrationSettings.PressureUnderTestList[classCalibrationInfo.CurrentCalibPressureIndex]).ToString();            

            //disable dp selection
            if(classCalibrationInfo.ConnectingToDP == true)
            {
                pnl_dpSelection.Enabled = false;
            }
            else
            {
                pnl_dpSelection.Enabled = true;
            }

            if (classCalibrationInfo.PressureStableFlag)
            {
                tb_preeStable.Text = "Yes";
                tb_preeStable.BackColor = Color.Green;

            }
            else
            {
                tb_preeStable.Text = "No";
                tb_preeStable.BackColor = Color.White;
            }

        }


        private void dgv_deviceData_CellContentClick(object sender, DataGridViewCellEventArgs e)
        {
           
        }

        private void bt_stopCalibration_Click(object sender, EventArgs e)
        {
            classCalibrationInfo.DoCalibration = false;
            classCalibrationInfo.StateMachineReset();
        }

        //private void pnl_TempData_Paint(object sender, PaintEventArgs e)
        //{

        //}

        //private void label7_Click(object sender, EventArgs e)
        //{

        //}

        //private void label5_Click(object sender, EventArgs e)
        //{

        //}

        //private void label6_Click(object sender, EventArgs e)
        //{

        //}

        private void bt_clear_Click(object sender, EventArgs e)
        {
            rtb_info.Text = "";
        }


        private bool CheckTimout(DateTime startTime, int timeout)
        {
            if (DateTime.Now.Subtract(startTime).TotalSeconds > timeout)
            {
                return true;
            }
            return false;
        }

        private void bt_connectDP_Click(object sender, EventArgs e)
        {
            byte DpId = (byte)(int.Parse(cmb_dpList.SelectedItem.ToString()));
            classCalibrationInfo.classMultiplexingInstanse.ConnectDpDevice(DpId);
        }

        private void bt_disConnectDP_Click(object sender, EventArgs e)
        {
            classCalibrationInfo.classMultiplexingInstanse.DisConnectAllDp();
        }

        //private void dgv_devicesQueue_CellContentClick(object sender, DataGridViewCellEventArgs e)
        //{

        //}

        //private void pnl_calibrationPanel_Paint(object sender, PaintEventArgs e)
        //{

        //}

        private void bt_settings_Click(object sender, EventArgs e)
        {
            this.Hide();
            if (ConfigFormInstanse == null)
            {
                ConfigFormInstanse = new ConfigForm(classDpCommunication, this, swVersions.CalibrationTool);
            }
            ConfigFormInstanse.Show();
        }

        private void bt_detectDp_Click(object sender, EventArgs e)
        {
            classCalibrationInfo.DetectFlag = true;
            classCalibrationInfo.InitDetectTread();
        }



        private void DGVSetCellColor(DataGridView dgv, int col, int row, Color color)
        {
            DataGridViewCellStyle style = new DataGridViewCellStyle();
            style.BackColor = color;
            dgv_devicesQueue[col, row].Style = style;
        }

        private void ClearColorIndication()
        {
            for (int i = 0; i < dgv_devicesQueue.Rows.Count; i++)
            {
                DGVSetCellColor(dgv_devicesQueue, 1, i, Color.White);
            }
        }


        private void UpdateColorStatus()
        {
            for (int i = 0; i < dgv_devicesQueue.Rows.Count - 1; i++)
            {
                int j = 0;
                while (j < classCalibrationInfo.DpCountAxist)
                {
                    if (dgv_devicesQueue.Rows[i].Cells[2].Value.ToString().Equals(classCalibrationInfo.classDevices[j].DeviceSerialNumber.ToString()))
                    {
                        DGVSetCellColor(dgv_devicesQueue, 1, i, classCalibrationInfo.classDevices[j].deviceStatus == DeviceStatus.Pass ? Color.Green : Color.Red);
                        break;
                    }
                    j++;
                }
            }
        }

        private void CalibForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            classCalibrationInfo.DoCalibration = false;
            if (classCalibrationInfo.CalibrationTaskHandlerThread != null)
            {
                classCalibrationInfo.CalibrationTaskHandlerThread.Abort();
                classCalibrationInfo.CalibrationTaskHandlerThread = null;
            }

            classCalibrationInfo.DetectFlag = false;
            if (classCalibrationInfo.DetectDevicesTaskHandlerThread != null)
            {
                classCalibrationInfo.DetectDevicesTaskHandlerThread.Abort();
                classCalibrationInfo.DetectDevicesTaskHandlerThread = null;
            }
            classCalibrationInfo.EndDetectEvent = true;


            ClassDeltaProtocol.CloseComPort();
            classDpCommunication.CloseComPort();
            classMultiplexing.CloseComPort();
            tempControllerInstanse.CloseComPort();


            //if (ConfigFormInstanse != null)
            //{
            //    ConfigFormInstanse.Close();
            //    ConfigFormInstanse = null;
            //}
            //Application.Exit();
        }

        private void CalibrationButtonHandele()
        {
            if (classCalibrationInfo != null)
            {
                if (classCalibrationInfo.DoCalibration == true)
                {
                    bt_startCalibration.Enabled = false;

                    if (!classCalibrationInfo.ConnectingToDP)
                    {
                        if (classCalibrationInfo.CalibrationPaused)
                        {
                            bt_pauseStartCalib.Text = "Start";
                        }
                        else
                        {
                            bt_pauseStartCalib.Text = "Pause";
                        }
                        bt_pauseStartCalib.Enabled = true;

                        bt_stopCalibration.Enabled = true;
                    }
                    else
                    {
                        if (classCalibrationInfo.CalibrationPaused)
                        {
                            bt_startCalibration.Enabled = false;
                        }

                        bt_pauseStartCalib.Text = "Pause";
                        bt_pauseStartCalib.Enabled = false;

                        bt_stopCalibration.Enabled = false;
                    }

                }
                else
                {
                    bt_startCalibration.Enabled = true;

                    bt_pauseStartCalib.Text = "Pause";
                    bt_pauseStartCalib.Enabled = false;

                    bt_stopCalibration.Enabled = false;
                }

            }
        }

        private void bt_pauseStartCalib_Click(object sender, EventArgs e)
        {
            classCalibrationInfo.CalibrationPaused = !classCalibrationInfo.CalibrationPaused;

            if (classCalibrationInfo.CalibrationPaused == false)
            {
                byte tempIndex;
                if (tb_tempIndexAfterPause.Text.ToString() != "")
                {
                    tempIndex = byte.Parse(tb_tempIndexAfterPause.Text.ToString());
                }
                else
                {
                    tempIndex = 0;
                }
                classCalibrationInfo.StateMachineResetAfterPause(tempIndex);
            }
        }

        private void button1_Click_1(object sender, EventArgs e)
        {
            string CutSn = tb_tempIndexAfterPause.Text.Substring(4);

            byte[] SN = System.Text.Encoding.ASCII.GetBytes(CutSn);
            classDpCommunication.SendDpSerialNumber(SN);
        }

        //private void button2_Click(object sender, EventArgs e)
        //{
        //    //TEST
        //    classCalibrationInfo.classDpCommunicationInstanse.DPgetDpInfo();
        //}

        //private void dgv_devicesQueue_CellClick(object sender, DataGridViewCellEventArgs e)
        //{

        //}

        //private void backgroundWorker1_DoWork(object sender, DoWorkEventArgs e)
        //{

        //}

        public void MarkConnectedDevice()
        {
            try
            {
                DataGridViewCellStyle spatialStyle = new DataGridViewCellStyle();
                DataGridViewCellStyle normalStyle = new DataGridViewCellStyle();
                spatialStyle.ForeColor = Color.Blue;
                normalStyle.ForeColor = Color.Black;


                for (int RowCount = 0; RowCount < dgv_devicesQueue.Rows.Count; RowCount++)
                {
                    foreach (DataGridViewCell cell in dgv_devicesQueue.Rows[RowCount].Cells)
                    {
                        //if (RowCount == Convert.ToInt32(classMultiplexing.ConnectedChanel))
                        //{
                        //    cell.Style.ForeColor = spatialStyle.ForeColor;
                        //}
                        //else
                        //{
                        //    cell.Style.ForeColor = normalStyle.ForeColor;
                        //}

                        if(classCalibrationInfo.classDevices[RowCount].PositionOnBoard == Convert.ToInt32(classMultiplexing.ConnectedChanel))
                        {
                            cell.Style.ForeColor = spatialStyle.ForeColor;
                        }
                        else
                        {
                            cell.Style.ForeColor = normalStyle.ForeColor;
                        }
                    }                  
                }
            }
            catch//(Exception ex)
            { }
          }

        private void chb_pressureAutoMode_CheckedChanged(object sender, EventArgs e)
        {
            classCalibrationInfo.classCalibrationSettings.PressureAutoMode = chb_pressureAutoMode.Checked;
        }

        private void bt_writeSN_Click(object sender, EventArgs e)
        {
            try
            {
                //string CutSn = tb_dpSN.Text.Substring(4);

                byte[] SN = System.Text.Encoding.ASCII.GetBytes(tb_dpSN.Text);
                classDpCommunication.SendDpSerialNumber(SN);

                classDpCommunication.DPgetDpInfo();
            }
            catch(Exception ex)
            {
                if (tb_dpSN.Text.Length < 1)
                {
                    MessageBox.Show("Please enter valid SN." + Environment.NewLine + ex.Message);
                }

            }
        }

        private void button1_Click_2(object sender, EventArgs e)
        {
            float pressure = classCalibrationInfo.ReadPressureFromPlc();
            tb_testReadPressure.Text =   pressure.ToString();
        }

        private void button2_Click_1(object sender, EventArgs e)
        {
            try
            {
                short PressureinBar = classCalibrationInfo.PlcBar2Adc(float.Parse(tb_newsetPresssure.Text));

                List < short > l = new List<short>();
                l.Clear();
                l.Add(PressureinBar);
                classCalibrationInfo.classDeltaProtocolInstanse.classDeltaWriteSetpoint(l);
                                                                
            }
            catch//(Exception ex)
            {

            }
        }

        public void CheckComPorts()
        {
            if (classCalibrationInfo != null)
            {
                //DP
                if (!classCalibrationInfo.classDpCommunicationInstanse.SerialPortInstanse.ComPortOk && classCalibrationInfo.classDpCommunicationInstanse.SerialPortInstanse.ComPortErrorMessage != string.Empty)
                {
                    string Message = string.Copy(classCalibrationInfo.classDpCommunicationInstanse.SerialPortInstanse.ComPortErrorMessage);
                    classCalibrationInfo.classDpCommunicationInstanse.SerialPortInstanse.ComPortErrorMessage = "";

                    MessageBox.Show(Message);

                }

                //MultiPlexer
                if (!classCalibrationInfo.classMultiplexingInstanse.SerialPortInstanse.ComPortOk && classCalibrationInfo.classMultiplexingInstanse.SerialPortInstanse.ComPortErrorMessage != string.Empty)
                {
                    string Message = string.Copy(classCalibrationInfo.classMultiplexingInstanse.SerialPortInstanse.ComPortErrorMessage);
                    classCalibrationInfo.classMultiplexingInstanse.SerialPortInstanse.ComPortErrorMessage = "";

                    MessageBox.Show(Message);

                }

                //PLC-Delta
                if (!classCalibrationInfo.classDeltaProtocolInstanse.serial.ComPortOk && classCalibrationInfo.classDeltaProtocolInstanse.serial.ComPortErrorMessage != string.Empty)
                {
                    string Message = string.Copy(classCalibrationInfo.classDeltaProtocolInstanse.serial.ComPortErrorMessage);
                    classCalibrationInfo.classDeltaProtocolInstanse.serial.ComPortErrorMessage = "";

                    MessageBox.Show(Message);

                }

                //temp controller
                if (!classCalibrationInfo.ClassTempControllerInstanse.ComPortOk && classCalibrationInfo.ClassTempControllerInstanse.ComPortErrorMessage != string.Empty)
                {
                    string Message = string.Copy(classCalibrationInfo.ClassTempControllerInstanse.ComPortErrorMessage);
                    classCalibrationInfo.ClassTempControllerInstanse.ComPortErrorMessage = "";

                    MessageBox.Show(Message);

                }
            }

        }

        //private void tb_logsPath_TextChanged(object sender, EventArgs e)
        //{
        //    //System.Diagnostics.Process.Start(Properties.Settings.Default.LogPath);
        //}

        private void tb_logsPath_Click(object sender, EventArgs e)
        {
            System.Diagnostics.Process.Start(Properties.Settings.Default.LogPath);
        }

        //private void panel1_Paint_1(object sender, PaintEventArgs e)
        //{

        //}

        //private void button3_Click(object sender, EventArgs e)
        //{

        //}

        private void button3_Click_1(object sender, EventArgs e)
        {
            classCalibrationInfo.classDpCommunicationInstanse.LicenseAck = false;
            byte[] license = new LicenceSupport().GetKey("17", "");
            if (license.Length > 0)
                classCalibrationInfo.classDpCommunicationInstanse.SendDpLicense(license);

            Thread.Sleep(1000);
            if (classCalibrationInfo.classDpCommunicationInstanse.LicenseAck)
            {
                rtb_info.AppendText("succeess " + System.Text.Encoding.UTF8.GetString(license, 0, license.Length));
            }
            else
            {
                MessageBox.Show("error");
            }
        }

        private void button3_Click(object sender, EventArgs e)
        {
           float temp =  classCalibrationInfo.TempControllerReadTemp();
            rtb_info.AppendText(temp.ToString());
            rtb_info.ScrollToCaret();

            classCalibrationInfo.classDpCommunicationInstanse.DPgetDpInfo();
        }

        private void pnl_calibrationPanel_Paint(object sender, PaintEventArgs e)
        {

        }

        private void button3_Click_2(object sender, EventArgs e)
        {
            classCalibrationInfo.ValidOvenTargetSp(4.7f);
        }
    }
}           
