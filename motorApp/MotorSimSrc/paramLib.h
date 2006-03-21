#ifndef PARAM_LIB_H
#define PARAM_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#define PARAM_OK (0)
#define PARAM_ERROR (-1)

typedef unsigned int paramIndex;
typedef struct paramList * PARAMS;
typedef void (*paramCallback)( void *, unsigned int, unsigned int * ); 

PARAMS paramCreate( paramIndex nvals );
void paramDestroy( PARAMS params );
int paramSetInteger( PARAMS params, paramIndex index, int value );
int paramSetDouble( PARAMS params, paramIndex index, double value );
void paramCallCallback( PARAMS params );
int paramGetInteger( PARAMS params, paramIndex index, int * value );
int paramGetDouble( PARAMS params, paramIndex index, double * value );
int paramSetCallback( PARAMS params, paramCallback callback, void * param );
void paramDump( PARAMS params );

#ifdef __cplusplus
}
#endif
#endif
