//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated from a template.
//
//     Manual changes to this file may cause unexpected behavior in your application.
//     Manual changes to this file will be overwritten if the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

namespace DP_dashboard.RIT_QA
{
    using System;
    using System.Collections.Generic;
    
    public partial class CalibrationData
    {
        public string SerialNo { get; set; }
        public Nullable<int> UserID { get; set; }
        public Nullable<int> StationID { get; set; }
        public Nullable<double> PressureSP { get; set; }
        public Nullable<double> PressurePLC { get; set; }
        public Nullable<double> TempSP { get; set; }
        public Nullable<double> TempDP { get; set; }
        public Nullable<int> RightA2D { get; set; }
        public Nullable<int> LeftA2D { get; set; }
        public int id { get; set; }
        public Nullable<System.DateTime> Datetime { get; set; }
        public string CalibrationToolVer { get; set; }
        public string DpFwVer { get; set; }
        public string MultiPlexerFwVer { get; set; }
    
        public virtual Device Device { get; set; }
        public virtual User User { get; set; }
    }
}
