/*
 * BGCmode.h
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


/* Dinh nghia cac Mode hoat dong cua BGC */
#define BGC_MODE_PRESET         1
#define BGC_MODE_CALIB          2
#define BGC_MODE_RUN            3

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



typedef struct
{
    BLDC motor1;
    BLDC motor2;
    BLDC motor3;
    MPU9250 mpu1;
    int mode;
}BGC;



/*Ket thuc noi dung header*/


/* CPP*/
#ifdef __cplusplus
}
#endif
/* CPP*/

#endif /* BGC_MODE_H_ */
