
enum ENCOS_ERRO
{
    ENCOS_ERROR_NONE = 0,
    ENCOS_ERROR_OVER_TEMP = 1,
    ENCOS_ERROR_CURRENT_LIMIT_VIOLATION =2,
    ENCOS_ERROR_DC_BUS_UNDER_VOLTAGE =3,
    ENCOS_ERROR_ABS_SPI_COM_FAIL=4,
    ENCOS_ERROR_DC_BUS_OVER_VOLTAGE =6,
    ENCOS_ERROR_DRV_FAULT=7,


    /* drv8301的硬件错误信息 */
    DRV8301_FaultType_NoFault  = (0 << 0),  //!< No fault
    DRV8301_FaultType_FETLC_OC = (1 << 0),  //!< FET Low side, Phase C Over Current fault
    DRV8301_FaultType_FETHC_OC = (1 << 1),  //!< FET High side, Phase C Over Current fault
    DRV8301_FaultType_FETLB_OC = (1 << 2),  //!< FET Low side, Phase B Over Current fault
    DRV8301_FaultType_FETHB_OC = (1 << 3),  //!< FET High side, Phase B Over Current fault
    DRV8301_FaultType_FETLA_OC = (1 << 4),  //!< FET Low side, Phase A Over Current fault
    DRV8301_FaultType_FETHA_OC = (1 << 5),  //!< FET High side, Phase A Over Current fault
    DRV8301_FaultType_OTW      = (1 << 6),  //!< Over Temperature Warning fault
    DRV8301_FaultType_OTSD     = (1 << 7),  //!< Over Temperature Shut Down fault
    DRV8301_FaultType_PVDD_UV  = (1 << 8),  //!< Power supply Vdd Under Voltage fault
    DRV8301_FaultType_GVDD_UV  = (1 << 9),  //!< DRV8301 Vdd Under Voltage fault
    DRV8301_FaultType_GVDD_OV  = (1 << 10)  //!< DRV8301 Vdd Over Voltage fault
   
};

