#ifndef BSP_I2C_H_
#define BSP_I2C_H_
#include "F28x_Project.h"

//
// Defines
//
#define I2C_SLAVE_ADDR        0x08//���豸�ĵ�ַ��7λ��ַ;8λ��ַΪ0x10
#define I2C_NUMBYTES          4  //�����ֽ���
#define REG_ADDR             0x66//

extern struct I2CMSG I2cMsgOut1;//����
extern struct I2CMSG I2cMsgIn1;
extern struct I2CMSG *CurrentMsgPtr;
//�ֽڼ���
extern Uint16 PassCount;
extern Uint16 FailCount;
// Function Prototypes
//��I2C���ߴ��ڿ���״̬ʱ��û�����ݴ��䣩��SCL��SDA�����ָߵ�ƽ��
void I2CB_GpioInit(void);
void   I2CB_Init(void);
Uint16 I2CB_WriteData(struct I2CMSG *msg);
Uint16 I2CB_ReadData(struct I2CMSG *msg);

void Write_Func(void);
void Read_Func(void);

void  DataMemory_Com_Write(Uint16 *com_address,Uint16 com_len,Uint16 *value,Uint16 val_len );
void Write_Function(Uint16 reg_address,Uint16 *data,Uint16 data_length);
int Read_Function(Uint16 reg_address,Uint16 data_length);

void Read_BMS_Information(int flag);//��BMS��Ϣ
// �������ڽ�16λ����ת��Ϊԭʼ����ֵ
int convertTwosComplementToDecimal(int complement);

//BMS ��Ϣ����
extern Uint16  FET_Status;//FET״̬  ALRT_PIN DDSG_PIN DCHG_PIN PDSG_FET DSG_FET PCHG_FET CHG_FET
extern Uint16 Safety_Alert_A;//A�ྯ��
extern Uint16 Safety_Status_A;//A�����
extern Uint16 Safety_Alert_B;//B�ྯ��
extern Uint16 Safety_Status_B;//B�����
extern Uint16 Safety_Alert_C;//C�ྯ��
extern Uint16 Safety_Status_C;//C�����

extern Uint16  PF_Alert_A;//
extern Uint16  PF_Status_A;//
extern Uint16  PF_Alert_B;//
extern Uint16  PF_Status_B;//
extern Uint16  PF_Alert_C;//
extern Uint16  PF_Status_C;//
extern Uint16  PF_Alert_D;//
extern Uint16  PF_Status_D;//
//��ѹ����
extern Uint16  Cell1_Voltage_mv;//��о1��ѹ
extern Uint16  Cell2_Voltage_mv;//��о2��ѹ
extern Uint16  Cell3_Voltage_mv;//��о3��ѹ
extern Uint16  Cell4_Voltage_mv;//��о4��ѹ
extern Uint16  Cell5_Voltage_mv;//��о5��ѹ
extern Uint16  Cell6_Voltage_mv;//��о6��ѹ
extern Uint16  Cell7_Voltage_mv;//��о7��ѹ
extern Uint16  Cell8_Voltage_mv;//��о8��ѹ
extern Uint16  Cell9_Voltage_mv;//��о9��ѹ
extern Uint16  Cell10_Voltage_mv;//��о10��ѹ
extern Uint16  Stack_Voltage_mv;//����鶥����ѹ
extern Uint16  PACK_Pin_Voltage_mv;//PACK�����ϵ�ѹ
extern Uint16  CC2_Current_mA;//CC2 �˲������ɵĵ�������ֵ
extern int16  CC2_Current_mA_Real;//CC2 �˲������ɵĵ�������ֵ
//�¶Ȳ���ֵ
extern int16 Int_Temperature_Value;//�ڲ��¶�  ��λ0.1k
extern int16 TS1_Temperature_Value;//Ts1�¶�
extern int16 TS2_Temperature_Value;//Ts2�¶�
extern int16 TS3_Temperature_Value;//Ts3�¶�
//��ؾ���
extern int16 CB_ACTIVE_CELLS;//
//���״̬
extern Uint16 Battery_Status;

extern Uint16 I2C_MODE_Val;//I2C ģʽ
extern Uint16 Vcell_Num;//��ؽ���
extern Uint16 COV_Threshold_mv;//��ѹ������ֵ
extern Uint16 CUV_Threshold_mv;//��ѹ������ֵ
extern Uint16 SCD_Threshold_A;//�ŵ��·������ֵ
extern int OCC_Threshold_A;//������������ֵ
extern Uint16 OCD1_Threshold_A;//�ŵ����������ֵ1
extern int OCD2_Threshold_A;//�ŵ����������ֵ2
extern Uint16 FET_ControlMODE;//FET����ģʽ
extern Uint16 FET_Options;//��ַ 0x9308��Ĭ��ֵ 0x0D����FET����ģʽ�����������õ�
extern Uint16 Protections_A;//���ø��ֱ���
extern Uint16 Manufacturing_Status_Value;

#endif
