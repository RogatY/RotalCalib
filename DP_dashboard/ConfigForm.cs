using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using DpCommunication;
using System.Data.OleDb;
using System.IO;


namespace DP_dashboard
{
    public partial class ConfigForm : Form
    {
        enum LICENSE_BITS
        {
            DPS = 1,
            DPT = 2,
            PS = 4,
            PT = 8,
            DATA_LOGGER = 0x10,
            WASH_CONTROL = 0x20,
            MASTER = 0x40,
            MANUFACTURE = 0x80
        };

        // Config file parameters offset
        private const byte CF_PRESSURE_COLUME_INDEX                                       = 0x06;
        private const byte CF_TEMPERATURE_COLUME_INDEX                                    = CF_PRESSURE_COLUME_INDEX + 1;
        private const byte CF_TEMPERATURE_TIMOUT_COLUME_INDEX                             = CF_TEMPERATURE_COLUME_INDEX + 1;

        private const byte CF_TEMPERATURE_SKIP_TIME_COLUME_INDEX                         = CF_TEMPERATURE_TIMOUT_COLUME_INDEX + 3;
        private const byte CF_TEMPERATURE_SKIP_TIME_ROW_INDEX                            = 7;
                            
        private const byte CF_TEMPERATURE_SAMPLES_AMOUNT_COLUME_INDEX                    = CF_TEMPERATURE_TIMOUT_COLUME_INDEX + 3;
        private const byte CF_TEMPERATURE_SAMPLES_AMOUNT_ROW_INDEX                       = CF_TEMPERATURE_SKIP_TIME_ROW_INDEX + 1;
                            
        private const byte CF_TEMPERATURE_SAMPLE_INTERVAL_COLUME_INDEX                   = CF_TEMPERATURE_TIMOUT_COLUME_INDEX + 3;
        private const byte CF_TEMPERATURE_SAMPLE_INTERVAL_ROW_INDEX                      = CF_TEMPERATURE_SAMPLES_AMOUNT_ROW_INDEX + 1;
                            
        private const byte CF_TEMPERATURE_DELTA_COLUME_INDEX                             = CF_TEMPERATURE_TIMOUT_COLUME_INDEX + 3;
        private const byte CF_TEMPERATURE_DELTA_TIME_ROW_INDEX                           = CF_TEMPERATURE_SAMPLE_INTERVAL_ROW_INDEX + 1;
                            
        private const byte CF_TEMPERATURE_MAX_WAIT_TO_TEMP_STABLE_TIME_COLUME_INDEX      = CF_TEMPERATURE_TIMOUT_COLUME_INDEX + 3;
        private const byte CF_TEMPERATURE_MAX_WAIT_TO_TEMP_STABLE_TIME_ROW_INDEX         = CF_TEMPERATURE_DELTA_TIME_ROW_INDEX + 1;

// END 

        public static CalibForm calibForm;
        ClassDpCommunication DpProtocolInstanse;

        public ConfigForm(ClassDpCommunication dpProtocolInstanse, CalibForm calibFormcaller, string ver)
        {
            calibForm = calibFormcaller;
            InitializeComponent();
            DpProtocolInstanse = dpProtocolInstanse;

            Text += " Ver " + ver;
            tb_batch.Text = Properties.Settings.Default.Batch;
            tb_fwVersion.Text = Properties.Settings.Default.FwVersion;
            tb_stationId.Text = Properties.Settings.Default.StationId.ToString();
            tb_userName.Text = Properties.Settings.Default.UserId;

        }

        private void dataGridView1_CellContentClick(object sender, DataGridViewCellEventArgs e)
        {

        }

        private void ConfigForm_Load(object sender, EventArgs e)
        {
         
        }

        private void bt_Cancel_Click(object sender, EventArgs e)
        {
            this.Hide();
            calibForm.Show();
        }



        private void bt_saveCalibPoint_Click(object sender, EventArgs e)
        {

            Properties.Settings.Default.Batch = tb_batch.Text;
            Properties.Settings.Default.FwVersion = tb_fwVersion.Text;
            try { Properties.Settings.Default.StationId = Int32.Parse(tb_stationId.Text); } catch { }
            Properties.Settings.Default.UserId = tb_userName.Text;

            calibForm.classCalibrationInfo.classCalibrationSettings.TempUnderTestList.Clear();
            calibForm.classCalibrationInfo.classCalibrationSettings.PressureUnderTestList.Clear();
            calibForm.classCalibrationInfo.classCalibrationSettings.TempSkipStartTime.Clear();


            calibForm.classCalibrationInfo.classCalibrationSettings.Versions.DpFw = tb_fwVersion.Text;

            //save temp points
            for (int i = 0; i < dgv_calibTempPointsTable.Rows.Count; i++)
            {
                DataGridViewCheckBoxCell EnabelTempCell = new DataGridViewCheckBoxCell();
                EnabelTempCell = (DataGridViewCheckBoxCell)dgv_calibTempPointsTable.Rows[i].Cells[0];

                if (EnabelTempCell.Value != null)
                {
                    if ((bool)(dgv_calibTempPointsTable.Rows[i].Cells[0].Value))
                    {
                        calibForm.classCalibrationInfo.classCalibrationSettings.TempUnderTestList.Add(float.Parse(dgv_calibTempPointsTable.Rows[i].Cells[1].Value.ToString()));
                        calibForm.classCalibrationInfo.classCalibrationSettings.TempSkipStartTime.Add(Int32.Parse(dgv_calibTempPointsTable.Rows[i].Cells[2].Value.ToString()) * 60);
                    }
                }
            }



            //save pressure points
            for (int i = 0; i < dgv_calibPressuresPointsTable.Rows.Count; i++)
            {
                DataGridViewCheckBoxCell EnabelTempCell = new DataGridViewCheckBoxCell();
                EnabelTempCell = (DataGridViewCheckBoxCell)dgv_calibPressuresPointsTable.Rows[i].Cells[0];
                if (EnabelTempCell.Value != null)
                {
                    if ((bool)(dgv_calibPressuresPointsTable.Rows[i].Cells[0].Value))
                    {
                        calibForm.classCalibrationInfo.classCalibrationSettings.PressureUnderTestList.Add(float.Parse(dgv_calibPressuresPointsTable.Rows[i].Cells[1].Value.ToString()));
                    }
                }

            }


            // update jig configuration
            if (cmb_jigConfiguration.SelectedItem != null)
            {

                if (cmb_jigConfiguration.SelectedItem.ToString() != "")
                {
                    calibForm.classCalibrationInfo.classCalibrationSettings.JigConfiguration = int.Parse(cmb_jigConfiguration.SelectedItem.ToString());
                }
                else
                {
                    calibForm.classCalibrationInfo.classCalibrationSettings.JigConfiguration = 8;
                }
            }
            else
            {
                calibForm.classCalibrationInfo.classCalibrationSettings.JigConfiguration = 8;
            }

            // update temp sample settings
            calibForm.classCalibrationInfo.classCalibrationSettings.TempSampleInterval = Convert.ToInt32(tb_temSpampleInterval.Text);
            calibForm.classCalibrationInfo.classCalibrationSettings.TempDeltaRange = float.Parse(tb_tempDeltaRange.Text);
            calibForm.classCalibrationInfo.classCalibrationSettings.MaxTimeWaitToTemp = Convert.ToInt32(tb_tempMaxWaitTime.Text) * 60;
            calibForm.classCalibrationInfo.classCalibrationSettings.TempSampleAmount = Convert.ToInt32(tb_tempSampleNum.Text);



            //update license type

            byte _capabilities = 0;
            _capabilities |= chkDPS.Checked ? (byte)LICENSE_BITS.DPS : (byte)0;
            _capabilities |= chkDPT.Checked ? (byte)LICENSE_BITS.DPT : (byte)0;
            _capabilities |= chkPS.Checked ? (byte)LICENSE_BITS.PS : (byte)0;
            _capabilities |= chkPT.Checked ? (byte)LICENSE_BITS.PT : (byte)0;
            _capabilities |= chkDataLogger.Checked ? (byte)LICENSE_BITS.DATA_LOGGER : (byte)0;
            _capabilities |= chkWash.Checked ? (byte)LICENSE_BITS.WASH_CONTROL : (byte)0;
            calibForm.classCalibrationInfo.classCalibrationSettings.DeviceLicens = _capabilities.ToString("X2");

            //save station detiles
            calibForm.classCalibrationInfo.classCalibrationSettings.UserName = tb_userName.Text;
            calibForm.classCalibrationInfo.classCalibrationSettings.StationId = int.Parse(tb_stationId.Text);

            //save batch
            calibForm.classCalibrationInfo.classCalibrationSettings.Batch = tb_batch.Text;


            this.Hide();
            calibForm.Show();
        }

        private void dgv_calibTempPointsTable_CellEndEdit(object sender, DataGridViewCellEventArgs e)
        {
            if (dgv_calibTempPointsTable.Rows[e.RowIndex].Cells[1].Value != null)
            {
                if (dgv_calibTempPointsTable.Rows[e.RowIndex].Cells[1].Value.ToString() != "")
                {
                    dgv_calibTempPointsTable.Rows[e.RowIndex].Cells[0].Value = true;
                }
                else
                {
                    dgv_calibTempPointsTable.Rows[e.RowIndex].Cells[0].Value = false;
                }
            }
            else
            {
                dgv_calibTempPointsTable.Rows[e.RowIndex].Cells[0].Value = false;
            }

        }





        private void dgv_calibPressuresPointsTable_CellEndEdit(object sender, DataGridViewCellEventArgs e)
        {
            if (dgv_calibPressuresPointsTable.Rows[e.RowIndex].Cells[1].Value != null)
            {
                if (dgv_calibPressuresPointsTable.Rows[e.RowIndex].Cells[1].Value.ToString() != "")
                {
                    dgv_calibPressuresPointsTable.Rows[e.RowIndex].Cells[0].Value = true;
                }
                else
                {
                    dgv_calibPressuresPointsTable.Rows[e.RowIndex].Cells[0].Value = false;
                }
            }
            else
            {
                dgv_calibPressuresPointsTable.Rows[e.RowIndex].Cells[0].Value = false;
            }

        }

        private void bt_loadDefoult_Click(object sender, EventArgs e)
        {


           
        }

        private void bt_loadConfigFile_Click(object sender, EventArgs e)
        {
            string filePath = string.Empty;
            string fileExt = string.Empty;
            OpenFileDialog file = new OpenFileDialog(); //open dialog to choose file 
            if (file.ShowDialog() == System.Windows.Forms.DialogResult.OK) //if there is a file choosen by the user  
            {
                filePath = file.FileName; //get the path of the file  
                fileExt = Path.GetExtension(filePath); //get the file extension  
                if (fileExt.CompareTo(".xls") == 0 || fileExt.CompareTo(".xlsx") == 0)
                {
                    try
                    {
                        DataTable dtExcel = new DataTable();
                        dtExcel = ReadExcel(filePath, fileExt); //read excel file  

                        dgv_calibPressuresPointsTable.Visible = true;
                       // dgv_calibPressuresPointsTable.DataSource = dtExcel;

                        BindConfigParameters(dtExcel);


                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show(ex.Message.ToString());
                    }
                }
                else
                {
                    MessageBox.Show("Please choose .xls or .xlsx file only.", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Error); //custom messageBox to show error  
                }
            }
        }


        public void BindConfigParameters(DataTable dataSource)
        {

            //pressure
            dgv_calibPressuresPointsTable.Rows.Clear();
            foreach (DataRow dr in dataSource.Rows)
            {
                if (dr[6].ToString() != "" && dr[6].ToString() != null)
                {
                    dgv_calibPressuresPointsTable.Rows.Add(true, dr[6].ToString());
                }
            }

            //temp
            dgv_calibTempPointsTable.Rows.Clear();
            foreach (DataRow dr in dataSource.Rows)
            {
                if (dr[7].ToString() != ""  && dr[7].ToString() != null)
                {
                    if (dr[8].ToString() != "" && dr[8].ToString() != null)
                    {
                        dgv_calibTempPointsTable.Rows.Add(true, dr[7].ToString(), dr[8].ToString());
                    }
                    else
                    {
                        dgv_calibTempPointsTable.Rows.Add(true, dr[7].ToString(),"60");
                    }
                }
            }

            //if (dataSource.Rows[CF_TEMPERATURE_SKIP_TIME_ROW_INDEX][CF_TEMPERATURE_SKIP_TIME_COLUME_INDEX].ToString() != "" && dataSource.Rows[CF_TEMPERATURE_SKIP_TIME_ROW_INDEX][CF_TEMPERATURE_SKIP_TIME_COLUME_INDEX].ToString() != null)
            //{
            //    tb_tempSkipTime.Text = dataSource.Rows[CF_TEMPERATURE_SKIP_TIME_ROW_INDEX][CF_TEMPERATURE_SKIP_TIME_COLUME_INDEX].ToString();
            //}

            if (dataSource.Rows[CF_TEMPERATURE_SAMPLES_AMOUNT_ROW_INDEX][CF_TEMPERATURE_SAMPLES_AMOUNT_COLUME_INDEX].ToString() != "" && dataSource.Rows[CF_TEMPERATURE_SAMPLES_AMOUNT_ROW_INDEX][CF_TEMPERATURE_SAMPLES_AMOUNT_COLUME_INDEX].ToString() != null)
            {
                tb_tempSampleNum.Text = dataSource.Rows[CF_TEMPERATURE_SAMPLES_AMOUNT_ROW_INDEX][CF_TEMPERATURE_SAMPLES_AMOUNT_COLUME_INDEX].ToString();
            }

            if (dataSource.Rows[CF_TEMPERATURE_SAMPLE_INTERVAL_ROW_INDEX][CF_TEMPERATURE_SAMPLE_INTERVAL_COLUME_INDEX].ToString() != "" && dataSource.Rows[CF_TEMPERATURE_SAMPLE_INTERVAL_ROW_INDEX][CF_TEMPERATURE_SAMPLE_INTERVAL_COLUME_INDEX].ToString() != null)
            {
                tb_temSpampleInterval.Text = dataSource.Rows[CF_TEMPERATURE_SAMPLE_INTERVAL_ROW_INDEX][CF_TEMPERATURE_SAMPLE_INTERVAL_COLUME_INDEX].ToString();
            }

            if (dataSource.Rows[CF_TEMPERATURE_DELTA_TIME_ROW_INDEX][CF_TEMPERATURE_DELTA_COLUME_INDEX].ToString() != "" && dataSource.Rows[CF_TEMPERATURE_DELTA_TIME_ROW_INDEX][CF_TEMPERATURE_DELTA_COLUME_INDEX].ToString() != null)
            {
                tb_tempDeltaRange.Text = dataSource.Rows[CF_TEMPERATURE_DELTA_TIME_ROW_INDEX][CF_TEMPERATURE_DELTA_COLUME_INDEX].ToString();
            }

            if (dataSource.Rows[CF_TEMPERATURE_MAX_WAIT_TO_TEMP_STABLE_TIME_ROW_INDEX][CF_TEMPERATURE_MAX_WAIT_TO_TEMP_STABLE_TIME_COLUME_INDEX].ToString() != "" && dataSource.Rows[CF_TEMPERATURE_MAX_WAIT_TO_TEMP_STABLE_TIME_ROW_INDEX][CF_TEMPERATURE_MAX_WAIT_TO_TEMP_STABLE_TIME_COLUME_INDEX].ToString() != null)
            {
                tb_tempMaxWaitTime.Text = dataSource.Rows[CF_TEMPERATURE_MAX_WAIT_TO_TEMP_STABLE_TIME_ROW_INDEX][CF_TEMPERATURE_MAX_WAIT_TO_TEMP_STABLE_TIME_COLUME_INDEX].ToString();
            }

        }

        public DataTable ReadExcel(string fileName, string fileExt)
        {

            string conn = string.Empty;
            DataTable dtexcel = new DataTable();
            if (fileExt.CompareTo(".xls") == 0)
                conn = @"provider=Microsoft.Jet.OLEDB.4.0;Data Source=" + fileName + ";Extended Properties='Excel 8.0;HRD=Yes;IMEX=1';"; //for below excel 2007  
            else
                conn = @"Provider=Microsoft.ACE.OLEDB.12.0;Data Source=" + fileName + ";Extended Properties='Excel 12.0;HDR=NO';"; //for above excel 2007  
            using (OleDbConnection con = new OleDbConnection(conn))
            {
                try
                {
                    OleDbDataAdapter oleAdpt = new OleDbDataAdapter("select * from [Sheet1$]", con); //here we read data from sheet1  
                    oleAdpt.Fill(dtexcel); //fill excel data into dataTable  
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.ToString());
                }
            }
            return dtexcel;

        }

        private void ConfigForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            calibForm.classCalibrationInfo.DoCalibration = false;
            calibForm.classCalibrationInfo.DetectFlag = false;

            calibForm.ClassDeltaProtocol.CloseComPort();
            calibForm.classDpCommunication.CloseComPort();
            calibForm.classMultiplexing.CloseComPort();
            calibForm.tempControllerInstanse.CloseComPort();

            calibForm.Close();

            Application.Exit();

            //calibForm.Show();
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            UpdateGui();
        }




        private void UpdateGui()
        {
            if (calibForm.classCalibrationInfo.DoCalibration)
            {
                pnl_settingsPanel.Enabled = false;
            }
            else
            {
                pnl_settingsPanel.Enabled = true;
            }
        }
    }
}


        

