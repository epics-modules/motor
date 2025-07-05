/*
FILENAME...     omsMAXvEncFunc.h
USAGE...        Pro-Dex OMS MAXv encoder asyn motor support

*/

/*
 *  Created on: 10/2010
 *      Author: eden
 */

#ifndef OMSMAXVENCFUNC_H_
#define OMSMAXVENCFUNC_H_

#include "omsMAXv.h"


#define ENCFUNCTION_FUNCTMASK     0xFF00  /* 8 function bits  */
#define ENCFUNCTION_ENC_MASK      0x00F0  /* 4 encoder bits (encoder 0 - 15)  */
#define ENCFUNCTION_AXISMASK      0x000F  /* 4 axis bits (axis 0 - 15)  */

#define ENCFUNCTION_NONE          0x00  /* no encoder functions  */
#define ENCFUNCTION_READ          0x01  /* read (auxiliary) encoder  */
#define ENCFUNCTION_REPLACE       0x02  /* replace encoder of axis axismask with encoder encodermask */
#define ENCFUNCTION_AVERAGE       0x03  /* read (auxiliary) encoder  */

#define ENCFUNCTION_AUXENC_0     0x0E  /* aux encoder 0 is defined as encoder 14)  */
#define ENCFUNCTION_AUXENC_1     0x0F  /* aux encoder 1 is defined as encoder 15)  */

#define MAXENCFUNC 8

class omsMAXvEncFunc : public omsMAXv {
public:
    omsMAXvEncFunc(const char*, int, int, const char*, int, int, unsigned int, int, int, const char* );
    omsMAXvEncFunc(const char*, int, int, const char*, int, int );
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus getEncoderPositions(epicsInt32 encPosArr[OMS_MAX_AXES]);

private:
    void initialize();
    int averageChannel[MAXENCFUNC];
    int encFuncIndex[MAXENCFUNC];
    int encRawPosIndex[MAXENCFUNC];
    int encPosIndex[2];
};

#endif /* OMSMAXVENCFUNC_H_ */
