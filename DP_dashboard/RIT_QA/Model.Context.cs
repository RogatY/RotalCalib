﻿//------------------------------------------------------------------------------
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
    using System.Data.Entity;
    using System.Data.Entity.Infrastructure;
    
    public partial class RIT_QAEntities1 : DbContext
    {
        public RIT_QAEntities1()
            : base("name=RIT_QAEntities1")
        {
        }
    
        protected override void OnModelCreating(DbModelBuilder modelBuilder)
        {
            throw new UnintentionalCodeFirstException();
        }
    
        public virtual DbSet<CalibrationData> CalibrationDatas { get; set; }
        public virtual DbSet<Device> Devices { get; set; }
        public virtual DbSet<User> Users { get; set; }
    }
}