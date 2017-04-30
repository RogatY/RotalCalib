using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net.Mail;
using System.Net;

namespace DP_dashboard
{
    static public class ClassMail
    {



        static public bool SendCalibrationDone()
        {           
            return SendMessage("Calibration done", string.Format("{0} - Calibration done.", DateTime.Now.ToString()));
        }

        static public bool SendCalibrationFailed()
        {
            return SendMessage("Calibration Fail", string.Format("{0} - Calibration fail.", DateTime.Now.ToString()));
        }



        static public bool SendMessage(string Sub, string Body)
        {
            var fromAddress = new MailAddress("dotnet4918@gmail.com", "David");
            var toAddress = new MailAddress("dudi938@gmail.com", "Amir");
            const string fromPassword = "49184918dot";
            string subject = Sub;
            string body = Body;

            var smtp = new SmtpClient
            {
                Host = "smtp.gmail.com",
                Port = 587,
                EnableSsl = true,
                DeliveryMethod = SmtpDeliveryMethod.Network,
                UseDefaultCredentials = false,
                Credentials = new NetworkCredential(fromAddress.Address, fromPassword)
            };
            using (var message = new MailMessage(fromAddress, toAddress)
            {
                Subject = subject,
                Body = body
            })
            {
                try
                {
                    smtp.Send(message);
                    return true;
                }
                catch //(Exception ex)
                {
                    return false;
                }
            }
        }                       
    }
}

