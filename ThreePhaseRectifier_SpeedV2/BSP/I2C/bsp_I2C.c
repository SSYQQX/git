#include "bsp_I2C.h"

extern int COM_flag;
extern int I2C_ERROR_FLAG;//I2C���ϱ�־

///////////////////////////////BMS����//////////////////////////////////////////////////

//�Ĵ�������ֵ
int Ram_SetFlag=0;
Uint16 Alarm_Set=0;//��������

Uint16  FET_Status=0;//FET״̬  ALRT_PIN DDSG_PIN DCHG_PIN PDSG_FET DSG_FET PCHG_FET CHG_FET

Uint16  Pump_Control=0;//

Uint16 Safety_Alert_A=0xFF;//A�ྯ��
Uint16 Safety_Status_A=0xFF;//A�����
Uint16 Safety_Alert_B=0xFF;//B�ྯ��
Uint16 Safety_Status_B=0xFF;//B�����
Uint16 Safety_Alert_C=0xFF;//C�ྯ��
Uint16 Safety_Status_C=0xFF;//C�����

Uint16  PF_Alert_A=0xFF;//
Uint16  PF_Status_A=0xFF;//
Uint16  PF_Alert_B=0xFF;//
Uint16  PF_Status_B=0xFF;//
Uint16  PF_Alert_C=0xFF;//
Uint16  PF_Status_C=0xFF;//
Uint16  PF_Alert_D=0xFF;//
Uint16  PF_Status_D=0xFF;//
//��ѹ����
Uint16  Cell1_Voltage_mv=0;//��о1��ѹ
Uint16  Cell2_Voltage_mv=0;//��о2��ѹ
Uint16  Cell3_Voltage_mv=0;//��о3��ѹ
Uint16  Cell4_Voltage_mv=0;//��о4��ѹ
Uint16  Cell5_Voltage_mv=0;//��о5��ѹ
Uint16  Cell6_Voltage_mv=0;//��о6��ѹ
Uint16  Cell7_Voltage_mv=0;//��о7��ѹ
Uint16  Cell8_Voltage_mv=0;//��о8��ѹ
Uint16  Cell9_Voltage_mv=0;//��о9��ѹ
Uint16  Cell10_Voltage_mv=0;//��о10��ѹ
Uint16  Stack_Voltage_mv=0;//����鶥����ѹ
Uint16  PACK_Pin_Voltage_mv=0;//PACK�����ϵ�ѹ
Uint16  CC2_Current_mA=0;//CC2 �˲������ɵĵ�������ֵ
int16  CC2_Current_mA_Real=0;//CC2 �˲������ɵĵ�������ֵ
//�¶Ȳ���ֵ
int16 Int_Temperature_Value=0;//�ڲ��¶�  ��λ0.1k
int16 TS1_Temperature_Value=0;//Ts1�¶�
int16 TS2_Temperature_Value=0;//Ts2�¶�
int16 TS3_Temperature_Value=0;//Ts3�¶�
//��ؾ���
int16 CB_ACTIVE_CELLS=0;//
//���״̬
Uint16 Battery_Status=0;

Uint16 I2C_MODE_Val=0;//I2C ģʽ
Uint16 Vcell_Num=0;//��ؽ���
Uint16 COV_Threshold_mv=0;//��ѹ������ֵ
Uint16 CUV_Threshold_mv=0;//��ѹ������ֵ
Uint16 SCD_Threshold_A=0;//�ŵ��·������ֵ
int OCC_Threshold_A=0;//������������ֵ
Uint16 OCD1_Threshold_A=0;//�ŵ����������ֵ1
int OCD2_Threshold_A=0;//�ŵ����������ֵ2
Uint16 FET_ControlMODE=0;//FET����ģʽ
Uint16 FET_Options=0;//��ַ 0x9308��Ĭ��ֵ 0x0D����FET����ģʽ�����������õ�
Uint16 Protections_A=0;//���ø��ֱ���
Uint16 Manufacturing_Status_Value=0;

//д��������
Uint16 SWAP_I2C_MODE8[4]={0xE7,0x36,0x29,0xDF};//i2c����ģʽ����crc��д��0x3E���˾��CRC
Uint16 I2C_MODE_Addr[2]={0x39,0x92};//i2cģʽ�洢����ַ
//����I2C��ַ
Uint16 I2C_Address_Addr[2]={0x3A,0x92};//0x923A
Uint16 I2C_Address_Value[1]={0x40};//��ַ��0x10��Ϊ0x40
//����CONFIG_UPDATE ģʽ  ������
Uint16 ENTER_CFG_UPDATE_Addr[2]={0x90,0x00};//0x0090
//�˳�CONFIG_UPDATE ģʽ  ������
Uint16 EXIT_CFG_UPDATE_Addr[2]={0x92,0x00};//0x0092
//��λ����
Uint16 RESET_Addr[2]={0x12,0x00};//0x0012
//Uint16 Settings_Protection_A[16]={0x3A,0x92};
Uint16 Settings_Protection_A[16]={0x80,0x91};
//���ݴ洢�����ʡ�����ַ�ĵ�λ�ֽ�д�� 0x3E������ַ�ĸ�λ�ֽ�д�� 0x3F���Ӵ�0x40 ��ʼ��ȡ���ݳ��ȵĻ��������� 0x61 ��ȡ��Ӧ�ĳ���
//��ؽ�������
Uint16 Vcell_Mode_Addr[2]={0x04,0x93};//��ؽ������á���ַ
Uint16 Vcell_Mode_value[2]={0xFF,0x03};//д��ֵ 0x03FF.10��
//��ѹ������ֵ
Uint16 COV_Threshold_Addr[2]={0x78,0x92};//��ѹ������ֵ���á���ַ0x9278
Uint16 COV_Threshold_Value[1]={76};//д��ֵ 76;76*50.6=3800mv  3845
//��ѹ������ֵ
Uint16 CUV_Threshold_Addr[2]={0x75,0x92};//��ѹ������ֵ���á���ַ0x9275
Uint16 CUV_Threshold_Value[1]={49};//д��ֵ 49; 49*50.6=2500mv  2479
//�ŵ��·������ֵ
Uint16 SCD_Threshold_Addr[2]={0x86,0x92};//�ŵ��·������ֵ���á���ַ0x9286  20mR����
Uint16 SCD_Threshold_Value[1]={13};//д��ֵ 13;  20A 400mv ��14��
//������������ֵ
Uint16 OCC_Threshold_Addr[2]={0x80,0x92};//������������ֵ����ַ0x9280
Uint16 OCC_Threshold_Value[1]={35};//д��ֵ 35;  3.5A 70mv=35*2mv
//�����������ӳ�
Uint16 OCC_Delay_Addr[2]={0x81,0x92};//0x9281
Uint16 OCC_Delay_Value[1]={0x0A};//д��ֵ10     10*3.3ms=33ms
//�ŵ����������ֵ1  OCD1������ �ӳ�ʱ��� 300ms
Uint16 OCD1_Threshold_Addr[2]={0x82,0x92};//�ŵ����������ֵ1����ַ0x9282
Uint16 OCD1_Threshold_Value[1]={100};//д��ֵ 100;  10A  200mv=100*2mv
//�ŵ��������1�ӳ�
Uint16 OCD1_Delay_Addr[2]={0x83,0x92};//0x9283
Uint16 OCD1_Delay_Value[1]={0x5B};//д��ֵ91     91*3.3ms=300ms
//�ŵ����������ֵ2  OCD2����С �ӳ�ʱ�䳤 24ms
Uint16 OCD2_Threshold_Addr[2]={0x84,0x92};//�ŵ����������ֵ2����ַ0x9284
Uint16 OCD2_Threshold_Value[1]={35};//д��ֵ 35;  3.5A  70mv=35*2mv
//�ŵ��������2�ӳ�
Uint16 OCD2_Delay_Addr[2]={0x85,0x92};//0x9285
Uint16 OCD2_Delay_Value[1]={0x7F};//д��ֵ127     127*3.3ms=419ms
///////////////�ŵ�����ָ�����
//������ֵ����������һ��ֵ�ָ����ŵ����Ϊ����������Ϊ�� ��λmA
Uint16 OCD_Recovery_CurrentThreshold_Addr[2]={0x8D,0x92};//0x928D
Uint16 OCD_Recovery_CurrentThreshold_Value[2]={0x9C,0XFF};///-100  ����Ϊff9c
//ʱ����ֵ
Uint16 OCD_Recovery_TimeThreshold_Addr[2]={0xAF,0x92};//0x92AF
Uint16 OCD_Recovery_TimeThreshold_Value[1]={0x05};///5 S
/////////////��оƽ������///////��������20s��󣬵�оƽ���㷨���¼���Ҫƽ��ĵ�о��
Uint16 Balancing_Configuration_Addr[2]={0x35,0x93};//0x9335.����ģʽ����
Uint16 Balancing_Configuration_Value[1]={0x01};//���ģʽ��������⡣˯��ģʽ�½�ֹ���⣬��ֹ�����½�ֹ���⡣
//����ͬʱƽ��ĵ�о�� 5
Uint16 Cell_Balance_Max_Cells_Addr[2]={0x3A,0x93};//0x933A
Uint16 Cell_Balance_Max_Cells_Value[1]={5};//5��
//��͵�о��ѹ���ڴ�ֵ,������о����С��о��ѹ�����40mV ʱ ��ʼ����
Uint16 Cell_Balance_Min_Cell_V_Addr[2]={0x3B,0x93};// 0x933B
Uint16 Cell_Balance_Min_Cell_V_Value[2]={0x10,0x0E};//3600mv 0x0e10
//��ȡ���ھ���ĵ��  ������
Uint16 CB_ACTIVE_CELLSA_Addr[2]={0x83,0x00};//0x0083
//��оƽ��ʱ�ĵ�ѹ�����ٶ�  �洢��
Uint16 Power_Config_Addr[2]={0x34,0x92};// 0x9234
Uint16 Power_Config_Value[2]={0xA2,0x29};//0x29A2.///����ʱ��ѹ�����ٶȽ�Ϊ1/4

//CC Gain ������������ֵ   CC_Gain_Value=7.4768/Rs   ��IEEE754 ������ 4 �ֽڸ�ʽ��ʾ
Uint16 CC_Gain_Addr[2]={0xA8,0X91};//0x91A8
Uint16 CC_Gain_Value[4]={0xB7,0x62,0xBF,0x3E};//ʹ�� IEEE754 ������ 4 �ֽڸ�ʽ��С���ֽ����˳��洢����ֵ//20mR 3E BF 62 B7
//Capacity Gain  �����ؼ�������ԭʼ��������ֵת��Ϊͨ���ĵ�� Capacity Gain=CC Gain*298261.6178
Uint16 Capacity_Gain_Addr[2]={0xAC,0X91};//0x91AC
Uint16 Capacity_Gain_Value[4]={0x10,0xC7,0xD9,0x47};//ʹ�� IEEE754 ������ 4 �ֽڸ�ʽ��С���ֽ����˳��洢����ֵ//20mR 47 D9 C7 10
//FET �������ĵ�ǰ״̬
Uint16 Manufacturing_Status_Addr[2]={0x57,0x00};//�������ĵ�ǰ״̬
//�͵�ѹ Shutdown��ֵ����ذ���ѹ��ֵ
Uint16 Shutdown_Stack_Voltage_Addr[2]={0x41,0x92};//0x9241
Uint16 Shutdown_Stack_Voltage_Value[2]={0x00,0x00};//0//��ѹ������Shutdownģʽ
//����FET����ģʽ������ģʽ��
Uint16 FET_ControlMODE_Addr[2]={0x43,0x93};//����ģʽ������ַ0x9343  Manufacturing:Mfg Status Init
Uint16 FET_ControlMODE_Value[2]={0x10,0x00};//д��ֵ 0x0010; ��������ʧЧ��飬����FET��������ģʽ��ע������������
// FET_ENABLE
Uint16 FET_ENABLE_Addr[2]={0x22,0x00};
Uint16 PF_ENABLE_Addr[2]={0x24,0x00};
// FET Options
Uint16 FET_Options_Addr[2]={0x08,0x93};//��ַ0x9308
Uint16 FET_Options_Value[1]={0x1F};//д��ֵ 0x1F; DSG ֮ǰ��ͨ PDSG FET;�������ƣ�����ģʽ;д��0x1F��CHG������SLEEPģʽ�����á�
//Protections A���� SCD OCD2 OCD1 OCC COV CUV
Uint16 Protections_A_Addr[2]={0x61,0x92};//��ַ0x9261
Uint16 Protections_A_Value[1]={0xFC};//д��ֵ 0xFC; ����SCD OCD2 OCD1 OCC COV CUV����
Uint16 DSG_FET_Protections_A[2]={0x69,0x92};//0x9269
Uint16 DSG_FET_Protections_A_Value[1]={0};//DSG FET��������ʹ�ܡ�0xE4ȫ��
Uint16 CHG_FET_Protections_A[2]={0x65,0x92};//0x9265
Uint16 CHG_FET_Protections_A_Value[1]={0};//CHG FET��������ʹ�ܡ�0x98 ȫ��
////�ر�Ԥ���
Uint16 Precharge_Start_Voltage_Addr[2]={0x0A,0x93};//0x930A
Uint16 Precharge_Start_Voltage_Value[2]={0x00,0x00};
Uint16 Precharge_Stop_Voltage_Addr[2]={0x0C,0x93};//0x930C
Uint16 Precharge_Stop_Voltage_Value[2]={0x00,0x00};
///////TS��������   TS1ʹ��Ĭ������
//TS2 0x0B ���������¶Ȳ��������浫�����ڵ�о�¶ȱ���
Uint16 TS2_Config_Addr[2]={0xFE,0x92};//0x92FE
Uint16 TS2_Config_Value[1]={0x0B};//
//TS3 0x0B ���������¶Ȳ��������浫�����ڵ�о�¶ȱ���
Uint16 TS3_Config_Addr[2]={0xFF,0x92};//0x92FF
Uint16 TS3_Config_Value[1]={0x0B};//
//FET����,��������ֽڣ�������
Uint16 ALL_FETS_ON_Addr[2]={0x96,0x00};//
Uint16 ALL_FETS_OFF_Addr[2]={0x95,0x00};//
Uint16 DSG_PDSG_OFF_Adde[2]={0x93,0x00};
Uint16 CHG_PCHG_OFF_Adde[2]={0x94,0x00};
//OTP_WR_CHECK �������ȡ�ж��Ƿ��������OTPд��
Uint16 OTP_WR_CHECK_Addr[2]={0xA0,0x00};//0x00a0
Uint16 OTP_WR_CHECK_Value=0;//�����
//����OTPд��
Uint16 OTP_WRITE_Addr[2]={0xA1,0x00};//0x00A1
Uint16 OTP_WRITE_Value=0;//д����
//Uint16 Vcell_Mode[2]={0x80,0x91};//��ؽ������á�д��ֵ 0x0203.����
//Uint16 Vcell_Mode_value[2]={0x7A,0x30};//
//ֱ�����������
Uint16  Safety_Alert_A_Addr=0x02;//A�ྯ��
Uint16  Safety_Status_A_Addr=0x03;//A�����
Uint16  Safety_Alert_B_Addr=0x04;//B�ྯ��
Uint16  Safety_Status_B_Addr=0x05;//B�����
Uint16  Safety_Alert_C_Addr=0x06;//C�ྯ��
Uint16  Safety_Status_C_Addr=0x07;//C�����
//ֱ���������ѹ�͵���
Uint16  Cell1_Voltage_Addr=0x14;//��о1��ѹ
Uint16  Cell2_Voltage_Addr=0x16;//��о2��ѹ
Uint16  Cell3_Voltage_Addr=0x18;//��о3��ѹ
Uint16  Cell4_Voltage_Addr=0x1A;//��о4��ѹ
Uint16  Cell5_Voltage_Addr=0x1C;//��о5��ѹ
Uint16  Cell6_Voltage_Addr=0x1E;//��о6��ѹ
Uint16  Cell7_Voltage_Addr=0x20;//��о7��ѹ
Uint16  Cell8_Voltage_Addr=0x22;//��о8��ѹ
Uint16  Cell9_Voltage_Addr=0x24;//��о9��ѹ
Uint16  Cell10_Voltage_Addr=0x26;//��о10��ѹ

Uint16  Stack_Voltage_Addr=0x34;//����鶥����ѹ
Uint16  PACK_Pin_Voltage_Addr=0x36;//PACK�����ϵ�ѹ
Uint16  CC2_Current_Addr=0x3A;//CC2 �˲������ɵĵ�������ֵ
//ֱ��������¶�
Uint16   Int_Temperature_Addr=0x68;//�ڲ��¶�  0.1K��λ
Uint16   TS1_Temperature_Addr=0x70;//TS1�¶�
Uint16   TS2_Temperature_Addr=0x72;//TS2�¶�
Uint16   TS3_Temperature_Addr=0x74;//TS3�¶�
//ֱ����������״̬
Uint16  Battery_Status_Addr=0x12;//���״̬
//��FET״̬��ֱ������
Uint16  FET_Status_Addr=0x7F;//FET״̬
//���ù��϶�ȡ��ֱ������
Uint16  PF_Alert_A_Addr=0x0A;//
Uint16  PF_Status_A_Addr=0x0B;//
Uint16  PF_Alert_B_Addr=0x0C;//
Uint16  PF_Status_B_Addr=0x0D;//
Uint16  PF_Alert_C_Addr=0x0E;//
Uint16  PF_Status_C_Addr=0x0F;//
Uint16  PF_Alert_D_Addr=0x10;//
Uint16  PF_Status_D_Addr=0x11;//


//д�����ı���
Uint16 Send_Data[16]={0};
Uint16 SendData_length=0;
Uint16 SendReg_address=0;
//�������ı���
Uint16 Read_Data[16]={0};
Uint16 ReadData_length=0;
Uint16 ReadReg_address=0;

/////////////////////////////////////////////////////////////////////////////////



//��дλ��0д��1����
//
// Globals
////������Ϣ�Ľṹ��
struct I2CMSG I2cMsgOut1={ I2C_MSGSTAT_SEND_WITHSTOP,//��Ϣ״̬����ʾ������Ϣ����һ��ֹͣ����
                           I2C_SLAVE_ADDR,//���豸�ĵ�ַ
                           I2C_NUMBYTES,//���͵������ֽ���
                           REG_ADDR,
                           0x00,                   // Msg Byte 1//Ҫ���͵������ֽڵ�����
                           0x00,
                           0x00,
                           0x00
};                  // Msg Byte 2
//������Ϣ�Ľṹ��
struct I2CMSG I2cMsgIn1={ I2C_MSGSTAT_SEND_NOSTOP,//������Ϣ�󲻷���ֹͣ��������ͨ�������������Ͷ����Ϣ
                          I2C_SLAVE_ADDR,
                          I2C_NUMBYTES,
                          REG_ADDR,
                          };

struct I2CMSG *CurrentMsgPtr;//��ǰ����Ϣָ�롣


void I2CB_GpioInit(void)
{
GPIO_SetupPinMux(40, GPIO_MUX_CPU1, 6);
GPIO_SetupPinMux(41, GPIO_MUX_CPU1, 6);
}

//
// I2CA_Init - Initialize I2CA settings
//
void I2CB_Init(void)
{
    I2cbRegs.I2CSAR.all = 0x0050;     // Slave address - EEPROM control code
//ʱ��Ƶ�ʵļ��㹫ʽ���£�
//
//    I2Cģ��ʱ�ӣ�I2C Module Clock�� = ϵͳʱ�ӣ�SYSCLK�� / (I2CPSC + 1)
//    I2C����ʱ�ӣ�I2C Master Clock�� = I2Cģ��ʱ�ӣ�I2C Module Clock�� / (ICCL + d) + (ICCH + d)
//IPSC = 0, d = 7; IPSC = 1, d = 6; IPSC > 1, d = 5.

    I2cbRegs.I2CPSC.all = 19;         // Prescaler - need 7-12 Mhz on module clk
                                      //19=0x13;IPSC=3 >1, d = 5;
    I2cbRegs.I2CCLKL = 10;            // NOTE: must be non zero 400K 10
    I2cbRegs.I2CCLKH = 5;             // NOTE: must be non zero  5
    I2cbRegs.I2CIER.all = 0x24;       // Enable SCD & ARDY __interrupts
//��I2Cģ��Ӹ�λ״̬���ͷų��������������ڹ���ʱֹͣI2Cģ��;ʹ��I2C
    I2cbRegs.I2CMDR.all = 0x0020;     // Take I2C out of reset
                                      // Stop I2C when suspended
  //  �Ĵ���I2CMDR��XAλ���Ƶ�ַ��λ��
 //  I2cbRegs.I2CMDR.bit.XA=0;//0��7λ��ַ��1:10λ��ַ
//������TXFIFO������FIFO��
    I2cbRegs.I2CFFTX.all = 0x6000;    // Enable FIFO mode and TXFIFO
    I2cbRegs.I2CFFRX.all = 0x2040;    // Enable RXFIFO, clear RXFFINT,
//������RXFIFO������FIFO��

    //���������Ϣ������
    // Clear incoming message buffer
    //
    int i=0;
        for (i = 0; i < I2C_MAX_BUFFER_SIZE; i++)
        {
            I2cMsgIn1.MsgBuffer[i] = 0x0000;
        }
//���ж�
        // Enable I2C __interrupt 1 in the PIE: Group 8 __interrupt 1
        //
        PieCtrlRegs.PIEIER8.bit.INTx3 = 1;//i2cb int8.3
        //

    return;
}

//write func
void Write_Func(void)
{
    Uint16 Error;
    I2cMsgOut1.MsgStatus = I2C_MSGSTAT_SEND_WITHSTOP;
    if(I2cMsgOut1.MsgStatus == I2C_MSGSTAT_SEND_WITHSTOP)
    {
        Error = I2CB_WriteData(&I2cMsgOut1);

        //
        // If communication is correctly initiated, set msg status to busy
        // and update CurrentMsgPtr for the __interrupt service routine.
        // Otherwise, do nothing and try again next loop. Once message is
        // initiated, the I2C __interrupts will handle the rest. Search for
        // ICINTR1A_ISR in the i2c_eeprom_isr.c file.
        //
        if (Error == I2C_SUCCESS)
        {
            CurrentMsgPtr = &I2cMsgOut1;
            //��CurrentMsgPtrָ������Ϊָ��I2cMsgOut1�������жϷ������̾Ϳ��Է��ʵ�ǰ���ڴ������Ϣ
            I2cMsgOut1.MsgStatus = I2C_MSGSTAT_WRITE_BUSY;//��ʾ��Ϣ����æ��д�����
            COM_flag=2;
        }
    }
}

//write func
void Write_Function(Uint16 reg_address,Uint16 *data,Uint16 data_length)
{
    Uint16 Error;
    int i=0;

    I2cMsgOut1.RegAddr=reg_address;
    I2cMsgOut1.NumOfBytes=data_length;
    for(i=0;i<data_length;i++)
    {
        I2cMsgOut1.MsgBuffer[i]=data[i];
    }

    I2cMsgOut1.MsgStatus = I2C_MSGSTAT_SEND_WITHSTOP;
    if(I2cMsgOut1.MsgStatus == I2C_MSGSTAT_SEND_WITHSTOP)
    {
        Error = I2CB_WriteData(&I2cMsgOut1);

        //
        // If communication is correctly initiated, set msg status to busy
        // and update CurrentMsgPtr for the __interrupt service routine.
        // Otherwise, do nothing and try again next loop. Once message is
        // initiated, the I2C __interrupts will handle the rest. Search for
        // ICINTR1A_ISR in the i2c_eeprom_isr.c file.
        //
        if (Error == I2C_SUCCESS)
        {
            CurrentMsgPtr = &I2cMsgOut1;
            //��CurrentMsgPtrָ������Ϊָ��I2cMsgOut1�������жϷ������̾Ϳ��Է��ʵ�ǰ���ڴ������Ϣ
            I2cMsgOut1.MsgStatus = I2C_MSGSTAT_WRITE_BUSY;//��ʾ��Ϣ����æ��д�����

        }
    }
}

//���ݴ洢�Ĵ�������
void  DataMemory_Com_Write(Uint16 *com_address,Uint16 com_len,Uint16 *value,Uint16 val_len )
        {
            Uint16 sum=0;
            Uint16 sum1=0;
            Uint16 sum2=0;
            Uint16 JY_Val=0;
            Uint16 JY_len=0;//���ݳ��ȣ������������������ݵĳ��� + 0x3E �� 0x3F �ĳ��� + 0x60 �� 0x61 �ĳ�
            //�ȣ�

            int i=0;

            Write_Function(0x3E,com_address,com_len);//д��洢����ַ
            DELAY_US(1000);//��λus
            Write_Function(0x40,value,val_len);//д��ֵ
            DELAY_US(1000);//��λus
            //����У���
            for(i=0;i<com_len;i++)//�����
            {
                sum1=sum1+com_address[i];
            }

            for(i=0;i<val_len;i++)//ֵ��
            {
                sum2=sum2+value[i];
            }
            sum=sum1+sum2;//��
            JY_Val=~sum;
            JY_Val=__byte(&JY_Val, 0);//�Ͱ�λ��λȡ����
            Write_Function(0x60,&JY_Val,1);//д��У���
            DELAY_US(1000);//��λus
            JY_len=com_len+val_len+2;
            Write_Function(0x61,&JY_len,1);//д�볤��
        }


//read func
void Read_Func(void)
{
    I2cMsgIn1.MsgStatus=I2C_MSGSTAT_SEND_NOSTOP;

    while(I2cMsgIn1.MsgStatus != I2C_MSGSTAT_INACTIVE)//
    //if(I2cMsgOut1.MsgStatus == I2C_MSGSTAT_INACTIVE)
    {//���I2cMsgOut1�ṹ���е�MsgStatus�ֶΣ���ȷ���Ƿ���Խ��ж�ȡ������
        //���״̬��I2C_MSGSTAT_INACTIVE����ʾû�����ڽ��е�д����������Խ��ж�ȡ��
        //
        // Check incoming message status.
        //
       // w_flag++;
        if(I2cMsgIn1.MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)
        {
            //
            // EEPROM address setup portion
            //
            while(I2CB_ReadData(&I2cMsgIn1) != I2C_SUCCESS)
            {//ѭ�����Է��Ͷ�ȡ��ַ��EEPROM��ֱ�������ɹ���
                //
                // Maybe setup an attempt counter to break an infinite while
                // loop. The EEPROM will send back a NACK while it is
                // performing a write operation. Even though the write
                // communique is complete at this point, the EEPROM could
                // still be busy programming the data. Therefore, multiple
                // attempts are necessary.
                //
            }

            //���µ�ǰ��Ϣָ�����Ϣ״̬:
            // Update current message pointer and message status
            //
            CurrentMsgPtr = &I2cMsgIn1;
            I2cMsgIn1.MsgStatus = I2C_MSGSTAT_SEND_NOSTOP_BUSY;
        }
        //
        // Once message has progressed past setting up the internal address
        // of the EEPROM, send a restart to read the data bytes from the
        // EEPROM. Complete the communique with a stop bit. MsgStatus is
        // updated in the __interrupt service routine.
        //���Ͷ�ȡ��ַ����Ҫ����һ���ظ���ʼ������Restart�������л�I2C�����ϵ����豸��ɫ�ӷ�����ģʽ��������ģʽ��
        else if(I2cMsgIn1.MsgStatus == I2C_MSGSTAT_RESTART)//���жϺ������л�Ϊ��
        {
            //
            // Read data portion
            //
            while(I2CB_ReadData(&I2cMsgIn1) != I2C_SUCCESS)
            {
                //
                // Maybe setup an attempt counter to break an infinite while
                // loop.
                //
            }
            //
            // Update current message pointer and message status
            //
            CurrentMsgPtr = &I2cMsgIn1;
            I2cMsgIn1.MsgStatus = I2C_MSGSTAT_READ_BUSY;
        }
    }
}

//read func
int Read_Function(Uint16 reg_address,Uint16 data_length)
{
    I2cMsgIn1.RegAddr=reg_address;
    I2cMsgIn1.NumOfBytes=data_length;
    I2cMsgIn1.MsgStatus=I2C_MSGSTAT_SEND_NOSTOP;

    int loop1=0;
    while(I2cMsgIn1.MsgStatus != I2C_MSGSTAT_INACTIVE)//�ȴ���ȡ�ɹ�
    {//���I2cMsgOut1�ṹ���е�MsgStatus�ֶΣ���ȷ���Ƿ���Խ��ж�ȡ������
        //���״̬��I2C_MSGSTAT_INACTIVE����ʾû�����ڽ��е�д����������Խ��ж�ȡ��
        //
        // Check incoming message status.
        //
        if(loop1++>20000)
            {
            I2C_ERROR_FLAG=3;
            return 0;//�����ʧ��
            }

        if(I2cMsgIn1.MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)
        {
            //
            // EEPROM address setup portion
            //
            int loop2=0;
            while(I2CB_ReadData(&I2cMsgIn1) != I2C_SUCCESS)
            {//ѭ�����Է��Ͷ�ȡ��ַ���ӻ���ֱ�������ɹ���
                //
                // Maybe setup an attempt counter to break an infinite while
                // loop. The EEPROM will send back a NACK while it is
                // performing a write operation. Even though the write
                // communique is complete at this point, the EEPROM could
                // still be busy programming the data. Therefore, multiple
                // attempts are necessary.
                //
                if(loop2++>20000)
                    {
                    I2C_ERROR_FLAG=1;
                    return 0;//�ӻ���Ӧ��
                    }
            }

            //���µ�ǰ��Ϣָ�����Ϣ״̬:
            // Update current message pointer and message status
            //
            CurrentMsgPtr = &I2cMsgIn1;
            I2cMsgIn1.MsgStatus = I2C_MSGSTAT_SEND_NOSTOP_BUSY;
        }
        //
        // Once message has progressed past setting up the internal address
        // of the EEPROM, send a restart to read the data bytes from the
        // EEPROM. Complete the communique with a stop bit. MsgStatus is
        // updated in the __interrupt service routine.
        //���Ͷ�ȡ��ַ����Ҫ����һ���ظ���ʼ������Restart�������л�I2C�����ϵ����豸��ɫ�ӷ�����ģʽ��������ģʽ��
        else if(I2cMsgIn1.MsgStatus == I2C_MSGSTAT_RESTART)//���жϺ������л�Ϊ�ˣ���˳�����Ҫ�ڴ�ѭ���ж�
        {
            //
            // Read data portion
            //
            int loop3=0;
            while(I2CB_ReadData(&I2cMsgIn1) != I2C_SUCCESS)
            {
                //
                // Maybe setup an attempt counter to break an infinite while
                // loop.
                //
                if(loop3++>20000)
                    {
                    I2C_ERROR_FLAG=2;
                    return 0;//�ظ���ʼʧ��
                    }
            }
            //
            // Update current message pointer and message status
            //
            CurrentMsgPtr = &I2cMsgIn1;
            I2cMsgIn1.MsgStatus = I2C_MSGSTAT_READ_BUSY;
        }
    }
    I2C_ERROR_FLAG=0;
    return 0;
}

//
// I2CA_WriteData - Transmit I2CA message
//
Uint16 I2CB_WriteData(struct I2CMSG *msg)
{
    Uint16 i;

    //
    // Wait until the STP bit is cleared from any previous master communication.
    // Clearing of this bit by the module is delayed until after the SCD bit is
    // set. If this bit is not checked prior to initiating a new message, the
    // I2C could get confused.
    //�ȴ�STPλ���.���STPλΪ1���򷵻�һ��������,��ʾI2Cģ�黹û��׼���÷����µ�����
    // STPλ�ڷ��������ݺ������
    if(I2cbRegs.I2CMDR.bit.STP == 1)
    {
        return I2C_STP_NOT_READY_ERROR;
    }

    //
    // Setup slave address
    //���ô��豸��ַ
    I2cbRegs.I2CSAR.all = msg->SlaveAddress;

    //
    // Check if bus busy
    //���I2C״̬�Ĵ����е�BB������æ��λ�Ƿ�Ϊ1
    if(I2cbRegs.I2CSTR.bit.BB == 1)
    {
        return I2C_BUS_BUSY_ERROR;
    }

    //
    // Setup number of bytes to send
    // MsgBuffer + Address
    //����Ҫ���͵��ֽ����������˼Ĵ�����ַ
    I2cbRegs.I2CCNT = msg->NumOfBytes+1;
//д��֮ǰ�Ȱ�FIFOָ�����㡣
     I2cbRegs.I2CFFTX.bit.TXFFRST = 0;    //��λ TXFIFO ��ָ�븴λ��0000��
     I2cbRegs.I2CFFTX.bit.TXFFRST=1;    //ʹ�� FIFO
    //
    // Setup data to send
    //
 //   I2cbRegs.I2CDXR.all = msg->MemoryHighAddr;//����EEPROM�ĸߵ�ַ
    I2cbRegs.I2CDXR.all = msg->RegAddr;//���ͼĴ�����ַ

    for (i=0; i < msg->NumOfBytes; i++)
    {
        I2cbRegs.I2CDXR.all = *(msg->MsgBuffer+i);//����Ϣ�������е�ÿ���ֽ�д�뵽I2C���ݼĴ����У��Ա㷢��
    }//����I2C������FIFO���У��Ƚ��ȳ����У�ģʽ�����Ḳ�ǡ�

    //
    // Send start as master transmitter
    //���Ϳ�ʼ������Ϊ��������
    // FREE��STT��STP��MST��TRX��IRS
    I2cbRegs.I2CMDR.all = 0x6E20;//I2CMDR:I2Cģʽ���üĴ���

    return I2C_SUCCESS;
}

//
// I2CA_ReadData - Read I2CA Message
//
Uint16 I2CB_ReadData(struct I2CMSG *msg)
{
    //
    // Wait until the STP bit is cleared from any previous master communication.
    // Clearing of this bit by the module is delayed until after the SCD bit is
    // set. If this bit is not checked prior to initiating a new message, the
    // I2C could get confused.
    //�ȴ�STPλ���
    if(I2cbRegs.I2CMDR.bit.STP == 1)
    {
        return I2C_STP_NOT_READY_ERROR;
    }
   // ���ô��豸��ַ
    I2cbRegs.I2CSAR.all = msg->SlaveAddress;
   //��������STOP�źŵ����
    if(msg->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)//������STOP�ź�
    {
        //
        // Check if bus busy
        //�������æ
        if(I2cbRegs.I2CSTR.bit.BB == 1)
        {
            return I2C_BUS_BUSY_ERROR;
        }
        I2cbRegs.I2CCNT = 1;//���ü����Ĵ���
//д��Ĵ�����ַ֮ǰһ��Ҫ�����㷢��FIFOָ��
        I2cbRegs.I2CFFTX.bit.TXFFRST = 0;    //��λ TXFIFO ��ָ�븴λ��0000��
        I2cbRegs.I2CFFTX.bit.TXFFRST=1;    //ʹ�� FIFO
       // I2cbRegs.I2CDXR.all = msg->MemoryHighAddr;
        I2cbRegs.I2CDXR.all = msg->RegAddr;//�Ĵ�����ַ.���ͼĴ����ĵ�ַ��
        I2cbRegs.I2CMDR.all = 0x2620; // Send data to setup EEPROM address//�����ǰ�淢�ͺ�������һ��ֹͣλ
    }//����START����������Ϊ������ģʽ��������STOP�źš�
    // dsp�����ڴ��ַ��״̬�л�Ϊ��I2C_MSGSTAT_RESTART����ʼ��������
    else if(msg->MsgStatus == I2C_MSGSTAT_RESTART)
    {
        I2cbRegs.I2CCNT = msg->NumOfBytes;    // Setup how many bytes to expect//���ü����Ĵ�������ʾҪ���յ��ֽ���
        I2cbRegs.I2CMDR.all = 0x2C20;         // Send restart as master receiver//����RESTART����������Ϊ������ģʽ��
    }

    return I2C_SUCCESS;
}


// �������ڽ�16λ����ת��Ϊԭʼ����ֵ
int convertTwosComplementToDecimal(int twosComplement) {
    // ������λ�Ƿ�Ϊ1��������
    if (twosComplement & 0x8000) {
        // ����Ǹ���������в���ת��
        return -((~twosComplement + 1) & 0xFFFF);
    } else {
        // �����������ֱ�ӷ���
        return twosComplement;
    }
}


/////////////////��ȡ���� //////////////////
/////////////////��ȡ����͹�����Ϣ//////////////////////////////////
void Read_BMS_Information(int flag)
{

      if(flag==100)
        {
        I2cMsgOut1.SlaveAddress=0x20;//���µ�ַ ���ݰ�
        I2cMsgIn1.SlaveAddress=0x20;
        }
        else
        {
            I2cMsgOut1.SlaveAddress=0x08;//���µ�ַ ��ذ�
            I2cMsgIn1.SlaveAddress=0x08;
        }
        Read_Function(Safety_Alert_A_Addr,1);//��ȡA�ྯ��
        Safety_Alert_A=I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms
        Read_Function(Safety_Status_A_Addr,1);//��ȡA�����
        Safety_Status_A=I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms
//
//        Read_Function(Safety_Alert_B_Addr,1);//��ȡB�ྯ��
//        Safety_Alert_B=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//��ʱ1ms
//        Read_Function(Safety_Status_B_Addr,1);//��ȡB�����
//        Safety_Status_B=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//��ʱ1ms
//
//        Read_Function(Safety_Alert_C_Addr,1);//��ȡC�ྯ��
//        Safety_Alert_C=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//��ʱ1ms
//        Read_Function(Safety_Status_C_Addr,1);//��ȡC�����
//        Safety_Status_C=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//��ʱ1ms
//        //���ù���
//        Read_Function(PF_Alert_A_Addr,1);//��ȡC�ྯ��
//        PF_Alert_A=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//��ʱ1ms
//        Read_Function(PF_Status_A_Addr,1);//��ȡC�����
//        PF_Status_A=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//��ʱ1ms
//
//        Read_Function(PF_Alert_B_Addr,1);//��ȡC�ྯ��
//        PF_Alert_B=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//��ʱ1ms
//        Read_Function(PF_Status_B_Addr,1);//��ȡC�����
//        PF_Status_B=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//��ʱ1ms
//
//        Read_Function(PF_Alert_C_Addr,1);//��ȡC�ྯ��
//        PF_Alert_C=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//��ʱ1ms
//        Read_Function(PF_Status_C_Addr,1);//��ȡC�����
//        PF_Status_C=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//��ʱ1ms
//
//        Read_Function(PF_Alert_D_Addr,1);//��ȡC�ྯ��
//        PF_Alert_D=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//��ʱ1ms
//        Read_Function(PF_Status_D_Addr,1);//��ȡC�����
//        PF_Status_D=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//��ʱ1ms

    /////////////////��FET״̬
        Read_Function(FET_Status_Addr,1);//��ȡFET״̬
        FET_Status=I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms

    ////////////����ѹ�����///////////////////////////////////////////////
        Read_Function(Cell1_Voltage_Addr,2);
        Cell1_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms
        Read_Function(Cell2_Voltage_Addr,2);
        Cell2_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms
        Read_Function(Cell3_Voltage_Addr,2);
        Cell3_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms
        Read_Function(Cell4_Voltage_Addr,2);
        Cell4_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms
        Read_Function(Cell5_Voltage_Addr,2);
        Cell5_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms
        Read_Function(Cell6_Voltage_Addr,2);
        Cell6_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms
        Read_Function(Cell7_Voltage_Addr,2);
        Cell7_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms
        Read_Function(Cell8_Voltage_Addr,2);
        Cell8_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms
        Read_Function(Cell9_Voltage_Addr,2);
        Cell9_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms
        Read_Function(Cell10_Voltage_Addr,2);
        Cell10_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms
        //����鶥����ѹ ��λ10mv
        Read_Function(Stack_Voltage_Addr,2);
        Stack_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//��ʱ1ms
        //����
        Read_Function(CC2_Current_Addr,2);
        CC2_Current_mA=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        CC2_Current_mA_Real=convertTwosComplementToDecimal(CC2_Current_mA);
        DELAY_US(1000);//��ʱ1ms
    ////////////���¶�///////////////////////////////////////////////
        Read_Function(Int_Temperature_Addr,2);
        Int_Temperature_Value=convertTwosComplementToDecimal((I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0])*0.1-273.15;
        DELAY_US(1000);//��ʱ1ms

        Read_Function(TS1_Temperature_Addr,2);
        TS1_Temperature_Value=convertTwosComplementToDecimal((I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0])*0.1-273.15;
        DELAY_US(1000);//��ʱ1ms
        Read_Function(TS2_Temperature_Addr,2);
        TS2_Temperature_Value=convertTwosComplementToDecimal((I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0])*0.1-273.15;
        DELAY_US(1000);//��ʱ1ms
        Read_Function(TS3_Temperature_Addr,2);
        TS3_Temperature_Value=convertTwosComplementToDecimal((I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0])*0.1-273.15;
        DELAY_US(1000);//��ʱ1ms
    ////////////����������ĵ��///////////////////////////////////////////////
        Write_Function(0x3E,CB_ACTIVE_CELLSA_Addr,2);//
        DELAY_US(10000);//��ʱ10ms
        Read_Function(0x40,2);
        CB_ACTIVE_CELLS=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
    ////////////�����״̬///////////////////////////////////////////////
//        DELAY_US(1000);//��ʱ1ms
//        Read_Function(Battery_Status_Addr,2);
//        Battery_Status=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//��ʱ1ms
//        //��FET����ģʽFET_ControlMODE
//        Write_Function(0x3E,FET_ControlMODE_Addr,2);//���ع�ѹ��ֵ
//        DELAY_US(10000);//��λus���ȴ��ӻ�׼������
//        Read_Function(0x40,2);//�Ӵӻ������ݻ�����������
//        FET_ControlMODE=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
}
