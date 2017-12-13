using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Utils
{

    public interface IGUI
    {
        void UpdateTraceInfo(string msg);

        void devicesDetected();
        void showScanButton();
    }

}
