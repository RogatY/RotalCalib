using System;
using System.Linq;
//using log4net;
using System.Reflection;
using System.Collections.Generic;
using log4net;

namespace DP_dashboard.RIT_QA
{
    static public class ClassDal
    {
        private static readonly ILog Logger = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        public static bool AddCalibrationRow(CalibrationData row)
        {
            using (RIT_QAEntities1 db = new RIT_QAEntities1())
            {
                Device dev = db.Devices.Find(row.Barcode);
                if (dev == null)
                {
                    dev = new Device();
                    dev.Barcode = row.Barcode;
                    dev.MAC = row.MAC_ADDRESS;
                    dev.Date = DateTime.Now;
                    db.Devices.Add(dev);
                }
                else
                {
                    dev.MAC = row.MAC_ADDRESS;
                    if (dev.Date == null)
                        dev.Date = DateTime.Now;
                }
                SaveDbChenges(db);

                db.CalibrationDatas.Add(row);

                return SaveDbChenges(db);
            }
        }


        public static bool AddOneTempertureTable(List<CalibrationData> tampertureTable, ClassDevice[] devices)
        {
            foreach (CalibrationData row in tampertureTable)
            {
                AddCalibrationRow(row);
            }
            return true;
        }



        static public Device GetDeviceBySerialNumber(string serialNo)
        {
            Device dev;
            using (RIT_QAEntities1 db = new RIT_QAEntities1())
            {
                dev = db.Devices.Find(serialNo);
            }
            return dev;
        }


        static public User GetUserByID(int id)
        {
            User user;
            using (RIT_QAEntities1 db = new RIT_QAEntities1())
            {
                user = db.Users.Find(id);
            }
            return user;
        }


        public static int GetFirstUserID()
        {
            using (RIT_QAEntities1 db = new RIT_QAEntities1())
            {
                User user = db.Users.FirstOrDefault();
                if (user == null)
                {
                    user = new User();
                    user.Name = "david";
                    user.Password = "123456";
                    db.Users.Add(user);

                    SaveDbChenges(db);
                }
                return user.Id;
            }
        }

        static bool SaveDbChenges(RIT_QAEntities1 db)
        {
            try
            {
                db.SaveChanges();

                Logger.Info("Save chenges success");

                return true;
            }
            catch (Exception ex)
            {
                Logger.Error("Save chenges failed" + "    " + ex.StackTrace.ToString() + ex.InnerException.ToString());

                return false;
            }
        }


    }
}