using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using log4net;
using System.Reflection;

namespace DP_dashboard
{
    static class Program
    {
        private static readonly ILog Logger = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        static System.Threading.Mutex singleton = new System.Threading.Mutex(true, "RitCalibrationTool");
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            if (!singleton.WaitOne(TimeSpan.Zero, true))
            {
                Logger.Warn("there is already another instance running!");
                MessageBox.Show("There is already another instance running!", "Error", MessageBoxButtons.OK, MessageBoxIcon.Stop);
                Application.Exit();
            }
            else
            {
                Application.EnableVisualStyles();
                Application.SetCompatibleTextRenderingDefault(false);
                Logger.Info("Program started");
                Application.Run(new CalibForm());
                singleton.ReleaseMutex();
            }

        }
    }
}
