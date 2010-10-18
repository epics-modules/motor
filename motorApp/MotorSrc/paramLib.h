#ifndef PARAM_LIB_H
#define PARAM_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <shareLib.h>

#define PARAM_OK (0)
#define PARAM_ERROR (-1)

typedef unsigned int paramIndex;
typedef struct paramList * PARAMS;
typedef void (*paramCallback)( void *, unsigned int, unsigned int * ); 

typedef struct
{
  PARAMS (*create)    ( paramIndex startVal, paramIndex nvals );
  void (*destroy)     ( PARAMS params );
  int  (*setInteger)  ( PARAMS params, paramIndex index, int value );
  int  (*setDouble)   ( PARAMS params, paramIndex index, double value );
  void (*callCallback)( PARAMS params );
  int  (*getInteger)  ( PARAMS params, paramIndex index, int * value );
  int  (*getDouble)   ( PARAMS params, paramIndex index, double * value );
  int  (*setCallback) ( PARAMS params, paramCallback callback, void * param );
  void (*dump)        ( PARAMS params );
  void (*forceCallback)( PARAMS params );
} paramSupport;

epicsShareExtern paramSupport * motorParam;

#ifdef __cplusplus
}
#endif
#endif
