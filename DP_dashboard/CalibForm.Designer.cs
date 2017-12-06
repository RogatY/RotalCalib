namespace DP_dashboard
{
    partial class CalibForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(CalibForm));
            this.pnl_calibrationPanel = new System.Windows.Forms.Panel();
            this.ScanButton = new System.Windows.Forms.Button();
            this.tb_logsPath = new System.Windows.Forms.TextBox();
            this.pb_calibProgressBar = new System.Windows.Forms.ProgressBar();
            this.bt_pauseStartCalib = new System.Windows.Forms.Button();
            this.bt_clear = new System.Windows.Forms.Button();
            this.rtb_info = new System.Windows.Forms.RichTextBox();
            this.bt_stopCalibration = new System.Windows.Forms.Button();
            this.bt_startCalibration = new System.Windows.Forms.Button();
            this.dgv_deviceData = new System.Windows.Forms.DataGridView();
            this.col_extPressure = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.col_Temp1_1 = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.col_Temp1_2 = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.col_temp2_p1 = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.col_Temp2_2 = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.col_temp3_p1 = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.col_Temp3_2 = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.col_temp4_p1 = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.col_Temp4_2 = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.col_temp5_p1 = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.col_Temp5_2 = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.dgv_devicesQueue = new System.Windows.Forms.DataGridView();
            this.col_no = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.col_deviceMacAddress = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.col_serialNumber = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.DevicePositionOnBoard = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.BoardNumber = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.bt_settings = new System.Windows.Forms.Button();
            this.tb_newsetPresssure = new System.Windows.Forms.TextBox();
            this.button2 = new System.Windows.Forms.Button();
            this.tb_testReadPressure = new System.Windows.Forms.TextBox();
            this.button1 = new System.Windows.Forms.Button();
            this.pnl_TempData = new System.Windows.Forms.Panel();
            this.label13 = new System.Windows.Forms.Label();
            this.tb_timeFromSendTemp = new System.Windows.Forms.TextBox();
            this.tb_currentSkipTime = new System.Windows.Forms.TextBox();
            this.label12 = new System.Windows.Forms.Label();
            this.tb_temperatureOnDP = new System.Windows.Forms.TextBox();
            this.label10 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.tb_targetTemperature = new System.Windows.Forms.TextBox();
            this.tb_currentTemperature = new System.Windows.Forms.TextBox();
            this.label7 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.pnl_dpSelection = new System.Windows.Forms.Panel();
            this.btnRelief = new System.Windows.Forms.Button();
            this.Reset = new System.Windows.Forms.Button();
            this.tb_dpSN = new System.Windows.Forms.TextBox();
            this.cmb_dpList = new System.Windows.Forms.ComboBox();
            this.bt_writeSN = new System.Windows.Forms.Button();
            this.bt_disConnectDP = new System.Windows.Forms.Button();
            this.bt_connectDP = new System.Windows.Forms.Button();
            this.panel2 = new System.Windows.Forms.Panel();
            this.chb_pressureAutoMode = new System.Windows.Forms.CheckBox();
            this.tb_preeStable = new System.Windows.Forms.TextBox();
            this.label9 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.tb_pressTargetPressure = new System.Windows.Forms.TextBox();
            this.tb_pressCurrentPressure = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.backgroundWorker1 = new System.ComponentModel.BackgroundWorker();
            this.pnl_calibrationPanel.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.dgv_deviceData)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.dgv_devicesQueue)).BeginInit();
            this.pnl_TempData.SuspendLayout();
            this.pnl_dpSelection.SuspendLayout();
            this.panel2.SuspendLayout();
            this.SuspendLayout();
            // 
            // pnl_calibrationPanel
            // 
            this.pnl_calibrationPanel.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.pnl_calibrationPanel.Controls.Add(this.ScanButton);
            this.pnl_calibrationPanel.Controls.Add(this.tb_logsPath);
            this.pnl_calibrationPanel.Controls.Add(this.pb_calibProgressBar);
            this.pnl_calibrationPanel.Controls.Add(this.bt_pauseStartCalib);
            this.pnl_calibrationPanel.Controls.Add(this.bt_clear);
            this.pnl_calibrationPanel.Controls.Add(this.rtb_info);
            this.pnl_calibrationPanel.Controls.Add(this.bt_stopCalibration);
            this.pnl_calibrationPanel.Controls.Add(this.bt_startCalibration);
            this.pnl_calibrationPanel.Controls.Add(this.dgv_deviceData);
            this.pnl_calibrationPanel.Controls.Add(this.dgv_devicesQueue);
            this.pnl_calibrationPanel.Location = new System.Drawing.Point(12, 12);
            this.pnl_calibrationPanel.Name = "pnl_calibrationPanel";
            this.pnl_calibrationPanel.Size = new System.Drawing.Size(933, 674);
            this.pnl_calibrationPanel.TabIndex = 15;
            this.pnl_calibrationPanel.Paint += new System.Windows.Forms.PaintEventHandler(this.pnl_calibrationPanel_Paint);
            // 
            // ScanButton
            // 
            this.ScanButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ScanButton.ForeColor = System.Drawing.Color.Black;
            this.ScanButton.Location = new System.Drawing.Point(840, 485);
            this.ScanButton.Name = "ScanButton";
            this.ScanButton.Size = new System.Drawing.Size(72, 33);
            this.ScanButton.TabIndex = 22;
            this.ScanButton.Text = "Scan";
            this.ScanButton.UseVisualStyleBackColor = true;
            this.ScanButton.Click += new System.EventHandler(this.ScanButton_Click);
            // 
            // tb_logsPath
            // 
            this.tb_logsPath.Location = new System.Drawing.Point(716, 534);
            this.tb_logsPath.Margin = new System.Windows.Forms.Padding(2);
            this.tb_logsPath.Name = "tb_logsPath";
            this.tb_logsPath.ReadOnly = true;
            this.tb_logsPath.Size = new System.Drawing.Size(207, 20);
            this.tb_logsPath.TabIndex = 21;
            this.tb_logsPath.Text = " ";
            this.tb_logsPath.Click += new System.EventHandler(this.tb_logsPath_Click);
            // 
            // pb_calibProgressBar
            // 
            this.pb_calibProgressBar.Location = new System.Drawing.Point(139, 591);
            this.pb_calibProgressBar.Margin = new System.Windows.Forms.Padding(2);
            this.pb_calibProgressBar.Name = "pb_calibProgressBar";
            this.pb_calibProgressBar.Size = new System.Drawing.Size(507, 19);
            this.pb_calibProgressBar.Step = 1;
            this.pb_calibProgressBar.TabIndex = 20;
            this.pb_calibProgressBar.Visible = false;
            // 
            // bt_pauseStartCalib
            // 
            this.bt_pauseStartCalib.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.bt_pauseStartCalib.ForeColor = System.Drawing.Color.Black;
            this.bt_pauseStartCalib.Location = new System.Drawing.Point(717, 485);
            this.bt_pauseStartCalib.Name = "bt_pauseStartCalib";
            this.bt_pauseStartCalib.Size = new System.Drawing.Size(72, 33);
            this.bt_pauseStartCalib.TabIndex = 8;
            this.bt_pauseStartCalib.Text = "Pause";
            this.bt_pauseStartCalib.UseVisualStyleBackColor = true;
            this.bt_pauseStartCalib.Click += new System.EventHandler(this.bt_pauseStartCalib_Click);
            // 
            // bt_clear
            // 
            this.bt_clear.Location = new System.Drawing.Point(6, 511);
            this.bt_clear.Margin = new System.Windows.Forms.Padding(2);
            this.bt_clear.Name = "bt_clear";
            this.bt_clear.Size = new System.Drawing.Size(50, 33);
            this.bt_clear.TabIndex = 7;
            this.bt_clear.Text = "Clear";
            this.bt_clear.UseVisualStyleBackColor = true;
            this.bt_clear.Click += new System.EventHandler(this.bt_clear_Click);
            // 
            // rtb_info
            // 
            this.rtb_info.AcceptsTab = true;
            this.rtb_info.Location = new System.Drawing.Point(61, 437);
            this.rtb_info.Name = "rtb_info";
            this.rtb_info.Size = new System.Drawing.Size(650, 131);
            this.rtb_info.TabIndex = 6;
            this.rtb_info.Text = "";
            // 
            // bt_stopCalibration
            // 
            this.bt_stopCalibration.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.bt_stopCalibration.ForeColor = System.Drawing.Color.Black;
            this.bt_stopCalibration.Location = new System.Drawing.Point(717, 437);
            this.bt_stopCalibration.Name = "bt_stopCalibration";
            this.bt_stopCalibration.Size = new System.Drawing.Size(72, 31);
            this.bt_stopCalibration.TabIndex = 5;
            this.bt_stopCalibration.Text = "Stop Calibration";
            this.bt_stopCalibration.UseVisualStyleBackColor = true;
            this.bt_stopCalibration.Click += new System.EventHandler(this.bt_stopCalibration_Click);
            // 
            // bt_startCalibration
            // 
            this.bt_startCalibration.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.bt_startCalibration.ForeColor = System.Drawing.Color.Black;
            this.bt_startCalibration.Location = new System.Drawing.Point(840, 437);
            this.bt_startCalibration.Name = "bt_startCalibration";
            this.bt_startCalibration.Size = new System.Drawing.Size(74, 31);
            this.bt_startCalibration.TabIndex = 3;
            this.bt_startCalibration.Text = "Start Calibration";
            this.bt_startCalibration.UseVisualStyleBackColor = true;
            this.bt_startCalibration.Click += new System.EventHandler(this.bt_startCalibration_Click);
            // 
            // dgv_deviceData
            // 
            this.dgv_deviceData.BackgroundColor = System.Drawing.Color.Silver;
            this.dgv_deviceData.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.dgv_deviceData.Columns.AddRange(new System.Windows.Forms.DataGridViewColumn[] {
            this.col_extPressure,
            this.col_Temp1_1,
            this.col_Temp1_2,
            this.col_temp2_p1,
            this.col_Temp2_2,
            this.col_temp3_p1,
            this.col_Temp3_2,
            this.col_temp4_p1,
            this.col_Temp4_2,
            this.col_temp5_p1,
            this.col_Temp5_2});
            this.dgv_deviceData.Location = new System.Drawing.Point(326, 7);
            this.dgv_deviceData.Name = "dgv_deviceData";
            this.dgv_deviceData.Size = new System.Drawing.Size(598, 416);
            this.dgv_deviceData.TabIndex = 1;
            this.dgv_deviceData.CellContentClick += new System.Windows.Forms.DataGridViewCellEventHandler(this.dgv_deviceData_CellContentClick);
            // 
            // col_extPressure
            // 
            this.col_extPressure.HeaderText = "Ext P";
            this.col_extPressure.Name = "col_extPressure";
            this.col_extPressure.ReadOnly = true;
            this.col_extPressure.Width = 50;
            // 
            // col_Temp1_1
            // 
            this.col_Temp1_1.HeaderText = "Temp1  Left A2D";
            this.col_Temp1_1.Name = "col_Temp1_1";
            this.col_Temp1_1.ReadOnly = true;
            this.col_Temp1_1.Width = 50;
            // 
            // col_Temp1_2
            // 
            this.col_Temp1_2.HeaderText = "Temp1 Right A2D";
            this.col_Temp1_2.Name = "col_Temp1_2";
            this.col_Temp1_2.ReadOnly = true;
            this.col_Temp1_2.Width = 50;
            // 
            // col_temp2_p1
            // 
            this.col_temp2_p1.HeaderText = "Temp2 Left A2D";
            this.col_temp2_p1.Name = "col_temp2_p1";
            this.col_temp2_p1.ReadOnly = true;
            this.col_temp2_p1.Width = 50;
            // 
            // col_Temp2_2
            // 
            this.col_Temp2_2.HeaderText = "Temp2 Right A2D";
            this.col_Temp2_2.Name = "col_Temp2_2";
            this.col_Temp2_2.ReadOnly = true;
            this.col_Temp2_2.Width = 50;
            // 
            // col_temp3_p1
            // 
            this.col_temp3_p1.HeaderText = "Temp3 Left A2D";
            this.col_temp3_p1.Name = "col_temp3_p1";
            this.col_temp3_p1.ReadOnly = true;
            this.col_temp3_p1.Width = 50;
            // 
            // col_Temp3_2
            // 
            this.col_Temp3_2.HeaderText = "Temp3 Right A2D";
            this.col_Temp3_2.Name = "col_Temp3_2";
            this.col_Temp3_2.ReadOnly = true;
            this.col_Temp3_2.Width = 50;
            // 
            // col_temp4_p1
            // 
            this.col_temp4_p1.HeaderText = "Temp4 Left Right A2D";
            this.col_temp4_p1.Name = "col_temp4_p1";
            this.col_temp4_p1.ReadOnly = true;
            this.col_temp4_p1.Width = 50;
            // 
            // col_Temp4_2
            // 
            this.col_Temp4_2.HeaderText = "Temp4 Right A2D";
            this.col_Temp4_2.Name = "col_Temp4_2";
            this.col_Temp4_2.ReadOnly = true;
            this.col_Temp4_2.Width = 50;
            // 
            // col_temp5_p1
            // 
            this.col_temp5_p1.HeaderText = "Temp5 Left A2D";
            this.col_temp5_p1.Name = "col_temp5_p1";
            this.col_temp5_p1.ReadOnly = true;
            this.col_temp5_p1.Width = 50;
            // 
            // col_Temp5_2
            // 
            this.col_Temp5_2.HeaderText = "Temp5 Right A2D";
            this.col_Temp5_2.Name = "col_Temp5_2";
            this.col_Temp5_2.ReadOnly = true;
            this.col_Temp5_2.Width = 50;
            // 
            // dgv_devicesQueue
            // 
            this.dgv_devicesQueue.BackgroundColor = System.Drawing.Color.Silver;
            this.dgv_devicesQueue.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.dgv_devicesQueue.Columns.AddRange(new System.Windows.Forms.DataGridViewColumn[] {
            this.col_no,
            this.col_deviceMacAddress,
            this.col_serialNumber,
            this.DevicePositionOnBoard,
            this.BoardNumber});
            this.dgv_devicesQueue.Location = new System.Drawing.Point(6, 5);
            this.dgv_devicesQueue.Name = "dgv_devicesQueue";
            this.dgv_devicesQueue.Size = new System.Drawing.Size(314, 418);
            this.dgv_devicesQueue.TabIndex = 0;
            this.dgv_devicesQueue.CellEnter += new System.Windows.Forms.DataGridViewCellEventHandler(this.dgv_devicesQueue_CellEnter);
            // 
            // col_no
            // 
            this.col_no.HeaderText = "NO";
            this.col_no.Name = "col_no";
            this.col_no.ReadOnly = true;
            this.col_no.Width = 20;
            // 
            // col_deviceMacAddress
            // 
            this.col_deviceMacAddress.HeaderText = "Mac address";
            this.col_deviceMacAddress.Name = "col_deviceMacAddress";
            this.col_deviceMacAddress.ReadOnly = true;
            this.col_deviceMacAddress.Width = 80;
            // 
            // col_serialNumber
            // 
            this.col_serialNumber.HeaderText = "Barcode";
            this.col_serialNumber.Name = "col_serialNumber";
            this.col_serialNumber.ReadOnly = true;
            this.col_serialNumber.Width = 80;
            // 
            // DevicePositionOnBoard
            // 
            this.DevicePositionOnBoard.HeaderText = "Board position ";
            this.DevicePositionOnBoard.Name = "DevicePositionOnBoard";
            this.DevicePositionOnBoard.ReadOnly = true;
            this.DevicePositionOnBoard.Width = 40;
            // 
            // BoardNumber
            // 
            this.BoardNumber.HeaderText = "Board number";
            this.BoardNumber.Name = "BoardNumber";
            this.BoardNumber.ReadOnly = true;
            this.BoardNumber.Width = 40;
            // 
            // bt_settings
            // 
            this.bt_settings.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(192)))), ((int)(((byte)(192)))), ((int)(((byte)(255)))));
            this.bt_settings.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.bt_settings.ForeColor = System.Drawing.Color.Black;
            this.bt_settings.Location = new System.Drawing.Point(9, 193);
            this.bt_settings.Name = "bt_settings";
            this.bt_settings.Size = new System.Drawing.Size(118, 31);
            this.bt_settings.TabIndex = 19;
            this.bt_settings.Text = "Settings";
            this.bt_settings.UseVisualStyleBackColor = false;
            this.bt_settings.Click += new System.EventHandler(this.bt_settings_Click);
            // 
            // tb_newsetPresssure
            // 
            this.tb_newsetPresssure.Location = new System.Drawing.Point(164, 92);
            this.tb_newsetPresssure.Margin = new System.Windows.Forms.Padding(2);
            this.tb_newsetPresssure.Name = "tb_newsetPresssure";
            this.tb_newsetPresssure.Size = new System.Drawing.Size(98, 20);
            this.tb_newsetPresssure.TabIndex = 13;
            this.tb_newsetPresssure.Text = "0";
            // 
            // button2
            // 
            this.button2.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F);
            this.button2.ForeColor = System.Drawing.Color.Black;
            this.button2.Location = new System.Drawing.Point(8, 87);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(119, 31);
            this.button2.TabIndex = 12;
            this.button2.Text = "Write pressure [Bar]";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.button2_Click_1);
            // 
            // tb_testReadPressure
            // 
            this.tb_testReadPressure.Location = new System.Drawing.Point(164, 51);
            this.tb_testReadPressure.Margin = new System.Windows.Forms.Padding(2);
            this.tb_testReadPressure.Name = "tb_testReadPressure";
            this.tb_testReadPressure.Size = new System.Drawing.Size(98, 20);
            this.tb_testReadPressure.TabIndex = 11;
            this.tb_testReadPressure.Text = "0";
            // 
            // button1
            // 
            this.button1.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F);
            this.button1.ForeColor = System.Drawing.Color.Black;
            this.button1.Location = new System.Drawing.Point(8, 48);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(119, 31);
            this.button1.TabIndex = 10;
            this.button1.Text = "Read pressure [A2D]";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click_2);
            // 
            // pnl_TempData
            // 
            this.pnl_TempData.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.pnl_TempData.Controls.Add(this.label13);
            this.pnl_TempData.Controls.Add(this.tb_timeFromSendTemp);
            this.pnl_TempData.Controls.Add(this.tb_currentSkipTime);
            this.pnl_TempData.Controls.Add(this.label12);
            this.pnl_TempData.Controls.Add(this.tb_temperatureOnDP);
            this.pnl_TempData.Controls.Add(this.label10);
            this.pnl_TempData.Controls.Add(this.label8);
            this.pnl_TempData.Controls.Add(this.tb_targetTemperature);
            this.pnl_TempData.Controls.Add(this.tb_currentTemperature);
            this.pnl_TempData.Controls.Add(this.label7);
            this.pnl_TempData.Controls.Add(this.label6);
            this.pnl_TempData.Location = new System.Drawing.Point(951, 12);
            this.pnl_TempData.Name = "pnl_TempData";
            this.pnl_TempData.Size = new System.Drawing.Size(283, 199);
            this.pnl_TempData.TabIndex = 16;
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label13.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.label13.Location = new System.Drawing.Point(4, 168);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(122, 13);
            this.label13.TabIndex = 13;
            this.label13.Text = "Setpoint temp timer[min] ";
            // 
            // tb_timeFromSendTemp
            // 
            this.tb_timeFromSendTemp.Location = new System.Drawing.Point(186, 132);
            this.tb_timeFromSendTemp.Name = "tb_timeFromSendTemp";
            this.tb_timeFromSendTemp.ReadOnly = true;
            this.tb_timeFromSendTemp.Size = new System.Drawing.Size(75, 20);
            this.tb_timeFromSendTemp.TabIndex = 10;
            // 
            // tb_currentSkipTime
            // 
            this.tb_currentSkipTime.Location = new System.Drawing.Point(186, 168);
            this.tb_currentSkipTime.Name = "tb_currentSkipTime";
            this.tb_currentSkipTime.ReadOnly = true;
            this.tb_currentSkipTime.Size = new System.Drawing.Size(74, 20);
            this.tb_currentSkipTime.TabIndex = 12;
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label12.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.label12.Location = new System.Drawing.Point(4, 132);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(117, 13);
            this.label12.TabIndex = 9;
            this.label12.Text = "Wait for temp timer[min]";
            // 
            // tb_temperatureOnDP
            // 
            this.tb_temperatureOnDP.Location = new System.Drawing.Point(186, 58);
            this.tb_temperatureOnDP.Name = "tb_temperatureOnDP";
            this.tb_temperatureOnDP.ReadOnly = true;
            this.tb_temperatureOnDP.Size = new System.Drawing.Size(75, 20);
            this.tb_temperatureOnDP.TabIndex = 8;
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label10.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.label10.Location = new System.Drawing.Point(2, 62);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(94, 13);
            this.label10.TabIndex = 7;
            this.label10.Text = "Temperture on DP";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label8.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.label8.Location = new System.Drawing.Point(62, 2);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(88, 13);
            this.label8.TabIndex = 6;
            this.label8.Text = "Temperature Info";
            // 
            // tb_targetTemperature
            // 
            this.tb_targetTemperature.Location = new System.Drawing.Point(186, 96);
            this.tb_targetTemperature.Name = "tb_targetTemperature";
            this.tb_targetTemperature.ReadOnly = true;
            this.tb_targetTemperature.Size = new System.Drawing.Size(75, 20);
            this.tb_targetTemperature.TabIndex = 5;
            // 
            // tb_currentTemperature
            // 
            this.tb_currentTemperature.Location = new System.Drawing.Point(186, 28);
            this.tb_currentTemperature.Name = "tb_currentTemperature";
            this.tb_currentTemperature.ReadOnly = true;
            this.tb_currentTemperature.Size = new System.Drawing.Size(75, 20);
            this.tb_currentTemperature.TabIndex = 4;
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label7.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.label7.Location = new System.Drawing.Point(1, 30);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(86, 13);
            this.label7.TabIndex = 2;
            this.label7.Text = "Oven temperture";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.label6.Location = new System.Drawing.Point(2, 96);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(38, 13);
            this.label6.TabIndex = 1;
            this.label6.Text = "Target";
            // 
            // pnl_dpSelection
            // 
            this.pnl_dpSelection.BackColor = System.Drawing.SystemColors.Control;
            this.pnl_dpSelection.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.pnl_dpSelection.Controls.Add(this.btnRelief);
            this.pnl_dpSelection.Controls.Add(this.Reset);
            this.pnl_dpSelection.Controls.Add(this.tb_dpSN);
            this.pnl_dpSelection.Controls.Add(this.tb_newsetPresssure);
            this.pnl_dpSelection.Controls.Add(this.cmb_dpList);
            this.pnl_dpSelection.Controls.Add(this.button2);
            this.pnl_dpSelection.Controls.Add(this.bt_writeSN);
            this.pnl_dpSelection.Controls.Add(this.bt_settings);
            this.pnl_dpSelection.Controls.Add(this.tb_testReadPressure);
            this.pnl_dpSelection.Controls.Add(this.bt_disConnectDP);
            this.pnl_dpSelection.Controls.Add(this.button1);
            this.pnl_dpSelection.Controls.Add(this.bt_connectDP);
            this.pnl_dpSelection.Location = new System.Drawing.Point(951, 417);
            this.pnl_dpSelection.Name = "pnl_dpSelection";
            this.pnl_dpSelection.Size = new System.Drawing.Size(283, 269);
            this.pnl_dpSelection.TabIndex = 13;
            // 
            // btnRelief
            // 
            this.btnRelief.Location = new System.Drawing.Point(9, 230);
            this.btnRelief.Name = "btnRelief";
            this.btnRelief.Size = new System.Drawing.Size(118, 23);
            this.btnRelief.TabIndex = 22;
            this.btnRelief.Text = "Relief";
            this.btnRelief.UseVisualStyleBackColor = true;
            this.btnRelief.Click += new System.EventHandler(this.btnRelief_Click);
            // 
            // Reset
            // 
            this.Reset.Location = new System.Drawing.Point(9, 165);
            this.Reset.Name = "Reset";
            this.Reset.Size = new System.Drawing.Size(118, 23);
            this.Reset.TabIndex = 21;
            this.Reset.Text = "Reset";
            this.Reset.UseVisualStyleBackColor = true;
            this.Reset.Click += new System.EventHandler(this.Reset_Click);
            // 
            // tb_dpSN
            // 
            this.tb_dpSN.Location = new System.Drawing.Point(164, 13);
            this.tb_dpSN.Margin = new System.Windows.Forms.Padding(2);
            this.tb_dpSN.Name = "tb_dpSN";
            this.tb_dpSN.Size = new System.Drawing.Size(98, 20);
            this.tb_dpSN.TabIndex = 10;
            // 
            // cmb_dpList
            // 
            this.cmb_dpList.DisplayMember = "0";
            this.cmb_dpList.FormattingEnabled = true;
            this.cmb_dpList.Items.AddRange(new object[] {
            "0",
            "1",
            "2",
            "3",
            "4",
            "5",
            "6",
            "7",
            "8",
            "9",
            "10",
            "11",
            "12",
            "13",
            "14",
            "15"});
            this.cmb_dpList.Location = new System.Drawing.Point(164, 181);
            this.cmb_dpList.Margin = new System.Windows.Forms.Padding(2);
            this.cmb_dpList.Name = "cmb_dpList";
            this.cmb_dpList.Size = new System.Drawing.Size(98, 21);
            this.cmb_dpList.TabIndex = 11;
            this.cmb_dpList.Text = "0";
            // 
            // bt_writeSN
            // 
            this.bt_writeSN.Location = new System.Drawing.Point(8, 7);
            this.bt_writeSN.Margin = new System.Windows.Forms.Padding(2);
            this.bt_writeSN.Name = "bt_writeSN";
            this.bt_writeSN.Size = new System.Drawing.Size(119, 31);
            this.bt_writeSN.TabIndex = 20;
            this.bt_writeSN.Text = "Write SN(Name)";
            this.bt_writeSN.UseVisualStyleBackColor = true;
            this.bt_writeSN.Click += new System.EventHandler(this.bt_writeSN_Click);
            // 
            // bt_disConnectDP
            // 
            this.bt_disConnectDP.Location = new System.Drawing.Point(164, 132);
            this.bt_disConnectDP.Margin = new System.Windows.Forms.Padding(2);
            this.bt_disConnectDP.Name = "bt_disConnectDP";
            this.bt_disConnectDP.Size = new System.Drawing.Size(96, 31);
            this.bt_disConnectDP.TabIndex = 10;
            this.bt_disConnectDP.Text = "Disconnect";
            this.bt_disConnectDP.UseVisualStyleBackColor = true;
            this.bt_disConnectDP.Click += new System.EventHandler(this.bt_disConnectDP_Click);
            // 
            // bt_connectDP
            // 
            this.bt_connectDP.Location = new System.Drawing.Point(9, 129);
            this.bt_connectDP.Margin = new System.Windows.Forms.Padding(2);
            this.bt_connectDP.Name = "bt_connectDP";
            this.bt_connectDP.Size = new System.Drawing.Size(118, 31);
            this.bt_connectDP.TabIndex = 9;
            this.bt_connectDP.Text = "Connect DP NO";
            this.bt_connectDP.UseVisualStyleBackColor = true;
            this.bt_connectDP.Click += new System.EventHandler(this.bt_connectDP_Click);
            // 
            // panel2
            // 
            this.panel2.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.panel2.Controls.Add(this.chb_pressureAutoMode);
            this.panel2.Controls.Add(this.tb_preeStable);
            this.panel2.Controls.Add(this.label9);
            this.panel2.Controls.Add(this.label4);
            this.panel2.Controls.Add(this.tb_pressTargetPressure);
            this.panel2.Controls.Add(this.tb_pressCurrentPressure);
            this.panel2.Controls.Add(this.label1);
            this.panel2.Controls.Add(this.label2);
            this.panel2.Location = new System.Drawing.Point(950, 217);
            this.panel2.Name = "panel2";
            this.panel2.Size = new System.Drawing.Size(284, 194);
            this.panel2.TabIndex = 17;
            // 
            // chb_pressureAutoMode
            // 
            this.chb_pressureAutoMode.AutoSize = true;
            this.chb_pressureAutoMode.Checked = true;
            this.chb_pressureAutoMode.CheckState = System.Windows.Forms.CheckState.Checked;
            this.chb_pressureAutoMode.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.chb_pressureAutoMode.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.chb_pressureAutoMode.Location = new System.Drawing.Point(8, 153);
            this.chb_pressureAutoMode.Margin = new System.Windows.Forms.Padding(2);
            this.chb_pressureAutoMode.Name = "chb_pressureAutoMode";
            this.chb_pressureAutoMode.Size = new System.Drawing.Size(120, 17);
            this.chb_pressureAutoMode.TabIndex = 9;
            this.chb_pressureAutoMode.Text = "Pressure auto mode";
            this.chb_pressureAutoMode.UseVisualStyleBackColor = true;
            this.chb_pressureAutoMode.CheckedChanged += new System.EventHandler(this.chb_pressureAutoMode_CheckedChanged);
            // 
            // tb_preeStable
            // 
            this.tb_preeStable.Location = new System.Drawing.Point(192, 112);
            this.tb_preeStable.Name = "tb_preeStable";
            this.tb_preeStable.ReadOnly = true;
            this.tb_preeStable.Size = new System.Drawing.Size(74, 20);
            this.tb_preeStable.TabIndex = 8;
            this.tb_preeStable.Text = "no";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label9.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.label9.Location = new System.Drawing.Point(7, 114);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(37, 13);
            this.label9.TabIndex = 7;
            this.label9.Text = "Stable";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label4.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.label4.Location = new System.Drawing.Point(72, 11);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(68, 13);
            this.label4.TabIndex = 6;
            this.label4.Text = "Pressure info";
            // 
            // tb_pressTargetPressure
            // 
            this.tb_pressTargetPressure.Location = new System.Drawing.Point(192, 75);
            this.tb_pressTargetPressure.Name = "tb_pressTargetPressure";
            this.tb_pressTargetPressure.ReadOnly = true;
            this.tb_pressTargetPressure.Size = new System.Drawing.Size(74, 20);
            this.tb_pressTargetPressure.TabIndex = 5;
            // 
            // tb_pressCurrentPressure
            // 
            this.tb_pressCurrentPressure.Location = new System.Drawing.Point(192, 41);
            this.tb_pressCurrentPressure.Name = "tb_pressCurrentPressure";
            this.tb_pressCurrentPressure.ReadOnly = true;
            this.tb_pressCurrentPressure.Size = new System.Drawing.Size(74, 20);
            this.tb_pressCurrentPressure.TabIndex = 4;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.label1.Location = new System.Drawing.Point(4, 42);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(112, 13);
            this.label1.TabIndex = 2;
            this.label1.Text = "Current  pressure [Bar]";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.label2.Location = new System.Drawing.Point(4, 77);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(106, 13);
            this.label2.TabIndex = 1;
            this.label2.Text = "Target pressure [Bar]";
            // 
            // CalibForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1244, 698);
            this.Controls.Add(this.panel2);
            this.Controls.Add(this.pnl_dpSelection);
            this.Controls.Add(this.pnl_calibrationPanel);
            this.Controls.Add(this.pnl_TempData);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.Fixed3D;
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.MaximizeBox = false;
            this.Name = "CalibForm";
            this.Text = "Calibration";
            this.TransparencyKey = System.Drawing.Color.Lime;
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.CalibForm_FormClosing);
            this.Load += new System.EventHandler(this.Form1_Load);
            this.pnl_calibrationPanel.ResumeLayout(false);
            this.pnl_calibrationPanel.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.dgv_deviceData)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.dgv_devicesQueue)).EndInit();
            this.pnl_TempData.ResumeLayout(false);
            this.pnl_TempData.PerformLayout();
            this.pnl_dpSelection.ResumeLayout(false);
            this.pnl_dpSelection.PerformLayout();
            this.panel2.ResumeLayout(false);
            this.panel2.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion
        private System.Windows.Forms.Panel pnl_calibrationPanel;
        private System.Windows.Forms.Button bt_stopCalibration;
        private System.Windows.Forms.Button bt_startCalibration;
        private System.Windows.Forms.DataGridView dgv_deviceData;
        private System.Windows.Forms.DataGridView dgv_devicesQueue;
        private System.Windows.Forms.RichTextBox rtb_info;
        private System.Windows.Forms.Panel pnl_TempData;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox tb_targetTemperature;
        private System.Windows.Forms.TextBox tb_currentTemperature;
        private System.Windows.Forms.Button bt_clear;
        private System.Windows.Forms.Panel pnl_dpSelection;
        private System.Windows.Forms.ComboBox cmb_dpList;
        private System.Windows.Forms.Button bt_disConnectDP;
        private System.Windows.Forms.Button bt_connectDP;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Panel panel2;
        private System.Windows.Forms.TextBox tb_preeStable;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox tb_pressTargetPressure;
        private System.Windows.Forms.TextBox tb_pressCurrentPressure;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Button bt_settings;
        private System.Windows.Forms.Button bt_pauseStartCalib;
        private System.Windows.Forms.TextBox tb_temperatureOnDP;
        private System.Windows.Forms.Label label10;
        private System.ComponentModel.BackgroundWorker backgroundWorker1;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_extPressure;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_Temp1_1;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_Temp1_2;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_temp2_p1;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_Temp2_2;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_temp3_p1;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_Temp3_2;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_temp4_p1;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_Temp4_2;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_temp5_p1;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_Temp5_2;
        private System.Windows.Forms.CheckBox chb_pressureAutoMode;
        private System.Windows.Forms.Button bt_writeSN;
        private System.Windows.Forms.TextBox tb_dpSN;
        private System.Windows.Forms.TextBox tb_testReadPressure;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.TextBox tb_newsetPresssure;
        private System.Windows.Forms.ProgressBar pb_calibProgressBar;
        private System.Windows.Forms.TextBox tb_logsPath;
        private System.Windows.Forms.TextBox tb_timeFromSendTemp;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.TextBox tb_currentSkipTime;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_no;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_deviceMacAddress;
        private System.Windows.Forms.DataGridViewTextBoxColumn col_serialNumber;
        private System.Windows.Forms.DataGridViewTextBoxColumn DevicePositionOnBoard;
        private System.Windows.Forms.DataGridViewTextBoxColumn BoardNumber;
        private System.Windows.Forms.Button Reset;
        private System.Windows.Forms.Button ScanButton;
        private System.Windows.Forms.Button btnRelief;
    }
}

