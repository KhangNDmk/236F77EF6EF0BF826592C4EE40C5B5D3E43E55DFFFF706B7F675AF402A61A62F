/*
 * BGC.h
 *
 *  Created on: Feb 19, 2019
 *      Author: MK
 *      Thuc hien thay doi cac mode hoat dong cua Gimbal.*/



#ifndef BGC_H
#define BGC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Noi dung header */
#include "BLDC.h"
#include "MPU9250.h"
#include "bgc_math.h"


/* Dinh nghia cac Mode hoat dong cua BGC */
#define BGC_MODE_INIT           1
#define BGC_MODE_CALIB          2
#define BGC_MODE_SET            3
#define BGC_MODE_RUN            4

/* event type: Loai su kien tac dong vao he thong */
#ifndef BGC_EVENT_TYPE
#define BGC_EVENT_TYPE

#define EV_USER             1            /* event tu nguoi dung*/
#define EV_BGC              2            /* event he thong*/

#endif
/* ket thuc event type */



/* Dinh nghia ma lenh cua cac user-command */
#ifndef BGC_USER_EV_
#define BGC_USER_EV_

#define USER_PRESET              1       /* dua gimbal motor ve vi tri mac dinh */
#define USER_CALIB               2       /*  */
#define USER_START               3       /*  */

#endif
/* ket thuc dinh nghia user-command */


/**/
#ifndef BGC_SYS_EV_
#define BGC_SYS_EV_

#define SYS_PRESET              1       /* dua gimbal motor ve vi tri mac dinh */
#define SYS_CALIB               2       /*  */
#define SYS_START               3       /*  */

#endif
/**/

typedef struct quaternion
{
    float q[4];
}quaternion;

typedef struct
{
    float phi_M1, phi_M2, phi_M3;
    float phi_refM1, phi_refM2, phi_refM3;
    mat3 RM_imu, RM_gimbal, RM_imuRef;
    int mode;
    quaternion q_m, q_r;
}BGC;

void BGC_GetQm(BGC *bgc0, MPU9250 *mpu0);

mat3 BGC_CalRM_gimbal(BGC *bgc0);

void BGC_SetQref(BGC *bgc0, MPU9250 *mpu0);

void BGC_Controller(BGC *bgc0, MPU9250 *mpu0 , int ev_user);

void BGC_init(BGC *bgc0, MPU9250 *mpu0);

mat3 qua2rot(quaternion qua0 );

/*Ket thuc noi dung header*/


/* CPP*/
#ifdef __cplusplus
}
#endif
/* CPP*/

#endif /* BGC_MODE_H_ */
