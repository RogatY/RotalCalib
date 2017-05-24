using System;
using System.Diagnostics;
using System.Net.Http;
using System.Threading;
using System.Threading.Tasks;

namespace DP_dashboard
{
    public class LicenceSupport
    {

        private static string licenseServerUrl = Properties.Settings.Default.LicenseServerURL;
        private static bool ready = false;
        private static byte[] license = null;
        private static string licenseType = "";
        private static string dpMac = "";
        //private static bool waiting = false;

        public LicenceSupport(string Url = "")
        {
            if(Url != "")
            licenseServerUrl = Url;
        }

        /// <summary>
        /// Return license as byte[] or null if timeout 
        /// </summary>
        /// <returns></returns>
        public byte [] GetKey(string licTypy, string mac)
        {
             if (licTypy != "")
            {
                licenseType = licTypy;
                dpMac = mac;

                return Generate();
            }           
            return null;
        }

        private static byte[] Generate()
        {
            ready = false;
            // ... Use HttpClient.
            using (HttpClient client = new HttpClient())
            {
                var tResp = client.GetAsync(licenseServerUrl + dpMac + "&capabilities=" + licenseType);
                tResp.Wait();
                HttpResponseMessage response = tResp.Result;
                using (HttpContent content = response.Content)
                {
                    try
                    {
                        var t = content.ReadAsStringAsync();
                        t.Wait();
                        string result = t.Result;


                        int len = result.IndexOf('}') - result.IndexOf('{') + 1;
                        result = result.Substring(result.IndexOf('{'), len);
                        key k = new System.Web.Script.Serialization.JavaScriptSerializer().Deserialize<key>(result);

                        if (k.Validation)
                        {
                            Console.WriteLine("License !!!! " + k.Key);
                            license = StringToByteArray(k.Key);
                        }
                        else
                        {
                            license = null;
                        }
                    }
                    catch //(Exception ex)
                    {
                        license = null;
                    }
                    ready = true;
                }
            }

            return license;
        }

        public class key
        {
            public string Key { get; set; }
            public bool Validation
            {
                get { return Key != null; }
                set { }
            }


        }
        public static byte[] StringToByteArray(string hex)
        {
            if (hex.Length % 2 == 1)
                throw new Exception("The binary key cannot have an odd number of digits");

            byte[] arr = new byte[hex.Length >> 1];

            for (int i = 0; i < hex.Length >> 1; ++i)
            {
                arr[i] = (byte)((GetHexVal(hex[i << 1]) << 4) + (GetHexVal(hex[(i << 1) + 1])));
            }

            return arr;
        }

        public static int GetHexVal(char hex)
        {
            int val = (int)hex;
            //For uppercase A-F letters:
            return val - (val < 58 ? 48 : 55);
            //For lowercase a-f letters:
            //return val - (val < 58 ? 48 : 87);
            //Or the two combined, but a bit slower:
            //return val - (val < 58 ? 48 : (val < 97 ? 55 : 87));
        }
    }
}
