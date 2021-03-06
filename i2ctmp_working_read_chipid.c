/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== i2ctmp116.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

/* Example/Board Header files */
#include "Board.h"

#define TASKSTACKSIZE       640

/*
 *  ======== TMP Registers ========
 */
#define ACCL_REG          0x00  /* Die Temp Result Register for TMP116 */
/*0x68 is used when SDO pin is pulled to GND,0x69 if to VCC.what is the SDO pin?*/
#define ACCL_ADDR1      0x68;
#define ACCL_ADDR2      0x69;


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    uint16_t        sample;
    uint16_t        temperature;
    uint8_t         txBuffer[1];
    uint8_t         rxBuffer[1];
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

    /* Call driver init functions */
    GPIO_init();
    I2C_init();

    /* Configure the LED and if applicable, the TMP116_EN pin */
#ifdef Board_GPIO_TMP116_EN/*is this related to the SDO pin?*/
    GPIO_setConfig(Board_GPIO_TMP116_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
    /* 1.5 ms reset time for the TMP116 */
    sleep(1);
#endif

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);

    if (i2c == NULL){
        return (NULL);
    }

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf    = rxBuffer;
    i2cTransaction.readCount  = 1;

    /*
     * Determine which I2C sensor is present.
     * We will prefer sensors in this order: TMP116 (on BoosterPacks),
     * TMP116 (on-board CC32XX LaunchPads), and last TMP006
     * (on older CC32XX LaunchPads).
     */
    /* Try TMP116 values */
    txBuffer[0] = ACCL_REG;
    i2cTransaction.slaveAddress = ACCL_ADDR1;
    if (!I2C_transfer(i2c, &i2cTransaction)) {
        /* Not ADDR1, try ADDR2 */
        i2cTransaction.slaveAddress = ACCL_ADDR2;
        if (!I2C_transfer(i2c, &i2cTransaction)) {
            return (NULL);
        }
    }

    if (rxBuffer[0] == 209){
        sample = 0;
    }

    else{
        sample = 1;
    }


    I2C_close(i2c);

    return (NULL);
}
