/*
 * BGCmode.c
 *
 *  Created on: Feb 19, 2019
 *      Author: MK
 */


#include "BGC.h"
#include "i2c.h"
void BGC_init(BGC *bgc0)
{

    MPU9250_Init(&hi2c1, &(bgc0->mpu1), AFS_4G, GFS_250DPS, MFS_16BITS);

    bgc0->mode = BGC_MODE_RUN;
}



void BGC_Controller(BGC *bgc0, int ev_user)
{
    switch (bgc0->mode) {
        case BGC_MODE_RUN:

            break;
        case BGC_MODE_PRESET:

            break;
        case BGC_MODE_CALIB:

            break;

        default:
            break;
    }
}

