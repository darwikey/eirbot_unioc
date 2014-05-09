#ifndef UNIOC_CONFIG_H
#define UNIOC_CONFIG_H

//Encodeurs
#define U_ENC0 (_SFR_MEM32(0x8000))
#define U_ENC1 (_SFR_MEM32(0x8004))
#define U_ENC2 (_SFR_MEM32(0x8008))
#define U_ENC3 (_SFR_MEM32(0x800C))
/*#define U_ENC1 (_SFR_MEM32(0x8000))
#define U_ENC0 (_SFR_MEM32(0x8004))
#define U_ENC3 (_SFR_MEM32(0x8008))
#define U_ENC2 (_SFR_MEM32(0x800C))*/

//ENC MAN
#define U_ENC_MAN (_SFR_MEM16(0x80FE))

//RESET
#define U_RESET (_SFR_MEM8(0x8040 + 18))


//Position manager

#define U_PM_FLAGS (_SFR_MEM8(0x8041))
#define U_PM_DISTANCE (_SFR_MEM32(0x8042))
#define U_PM_ANGLE (_SFR_MEM32(0x8046))
#define U_PM_X (_SFR_MEM32(0x804A))
#define U_PM_Y (_SFR_MEM32(0x804E))
#define U_PM_2PI (_SFR_MEM32(0x8053))
#define U_PM_VIT_D (_SFR_MEM32(0x8057))
#define U_PM_VIT_G (_SFR_MEM32(0x805B))

#define U_PM_FLAGS_LOCK (1<<0)
#define U_PM_FLAGS_RESET (1<<1)
#define U_PM_FLAGS_SET_REGISTERS (1<<2)


//Asserv



//moteurs
#define U_MOT_INV_G (1<<0)
#define U_MOT_INV_D (1<<1)
#define U_MOT_RESET (1<<3)
#define U_MOT_MODE_ATMEGA (0)
#define U_MOT_MODE_FPGA (1<<2)

#define U_MOT_BASE_ADRESSE 0x8020
#define U_MOT_FLAGS (_SFR_MEM32(U_MOT_BASE_ADRESSE))
#define U_MOT_D_RATIO (_SFR_MEM32(U_MOT_BASE_ADRESSE + 1))
#define U_MOT_G_RATIO (_SFR_MEM32(U_MOT_BASE_ADRESSE + 5))
#define U_MOT_D_PERIODE (_SFR_MEM32(U_MOT_BASE_ADRESSE + 9))
#define U_MOT_G_PERIODE (_SFR_MEM32(U_MOT_BASE_ADRESSE + 13))


#define U_ASSERV_DISTANCE (_SFR_MEM32(0x8100))
#define U_ASSERV_ANGLE (_SFR_MEM32(0x8110))


//RDS
//
#define U_RDS_BAUDRATE (_SFR_MEM32(0x8200))
#define U_RDS_CODE1 (_SFR_MEM8(0x8204))
#define U_RDS_CODE2 (_SFR_MEM8(0x8205))
#define U_RDS_RES1 (_SFR_MEM16(0x8206))
#define U_RDS_RES2 (_SFR_MEM16(0x8208))

#endif //UNIOC_CONFIG_H
