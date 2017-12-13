using System;
using System.Collections;
using System.ComponentModel;
using System.IO;

namespace DP_dashboard
{
    [RunInstaller(true)]
    public partial class Installer : System.Configuration.Install.Installer
    {
        public Installer()
        {
            InitializeComponent();
        }

        protected override void OnBeforeInstall(IDictionary savedState)
        {
            base.OnBeforeInstall(savedState);
            string path = this.Context.Parameters["targetdir"];
            if (File.Exists(path + "\\DP_dashboard.exe.config"))
            {
                string Date = DateTime.Now.ToString("yyyy-MM-dd_HH-mm");
                File.Copy(path + "\\DP_dashboard.exe.config", path + Date + "_DP_dashboard.exe.config" + ".bak");
            }
        }

        public override void Uninstall(IDictionary savedState)
        {
            string path = this.Context.Parameters["targetdir"];
            if (File.Exists(path + "\\DP_dashboard.exe.config"))
            {
                string Date = DateTime.Now.ToString("yyyy-MM-dd_HH-mm");
                File.Copy(path + "\\DP_dashboard.exe.config", path + Date + "_DP_dashboard.exe.config" + ".bak");
            }
            base.Uninstall(savedState);
        }
    }
}
