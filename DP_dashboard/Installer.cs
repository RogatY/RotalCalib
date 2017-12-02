using System;
using System.Collections;
using System.ComponentModel;
using System.IO;
using System.Windows.Forms;

namespace DP_dashboard
{
    [RunInstaller(true)]
    public partial class Installer : System.Configuration.Install.Installer
    {
        public Installer()
        {
            InitializeComponent();
        }

        public override void Install(System.Collections.IDictionary stateSaver)
        {
            string path = this.Context.Parameters["targetdir"];
            if (File.Exists(path + "\\DP_dashboard.exe.config"))
            {
                string Date = DateTime.Now.ToString("yyyy-MM-dd_HH-mm");
                File.Copy(path + "\\DP_dashboard.exe.config", path + Date + "_DP_dashboard.exe.config" + ".bak");
            }
            base.Install(stateSaver);
        }

        public override void Commit(IDictionary savedState)
        {
            string path = this.Context.Parameters["targetdir"];
            if (File.Exists(path + "\\DP_dashboard.exe.config"))
            {
                string Date = DateTime.Now.ToString("yyyy-MM-dd_HH-mm");
                File.Copy(path + "\\DP_dashboard.exe.config", path + Date + "_DP_dashboard.exe.config" + ".bak");
            }
            base.Commit(savedState);
        }

        public override void Rollback(IDictionary savedState)
        {
            base.Rollback(savedState);
        }

        public override void Uninstall(IDictionary savedState)
        {
            
            string path = this.Context.Parameters["targetdir"];
            if (File.Exists(path + "\\DP_dashboard.exe.config"))
            {
                DriveInfo di = new DriveInfo(path);
                string Date = DateTime.Now.ToString("yyyy-MM-dd_HH-mm");

                File.Copy(path + "\\DP_dashboard.exe.config", di.Name + Date + "__DP_dashboard.exe.config" + ".bak");
            }
            base.Uninstall(savedState);
        }


    }
}
