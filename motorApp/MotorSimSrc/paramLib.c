/**\mainpage Simple parameter system that can be used for EPICS motor drivers.

This is a simple parameter system designed to make parameter storage and 
callback notification simpler for EPICS motor drivers. The calls are compatible
with the EPICS motor API, so a number of the routines in a motor driver can
be simple pass-through routines calling this library. For an example, see the
drvMotorSim.c code.

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "paramLib.h"

typedef enum { paramUndef, paramDouble, paramInt } paramType;

typedef struct
{
    paramType type;
    union
    {
        double dval;
        int    ival;
    } data;
} paramVal;

typedef struct paramList
{
    paramIndex nvals;
    int * flags;
    paramIndex * set_flags;
    paramVal * vals;
    paramCallback callback;
    void * param;
} paramList;

/** Deletes a parameter system created by paramCreate.

    Allocates data structures for a parameter system with the given number of
    parameters. Parameters stored in the system are are accessed via and index number
    ranging from 0 to the number of values minus 1.

    \param params [in]   Pointer to PARAM handle returned by paramCreate.

    \return void.
*/
void paramDestroy( PARAMS params )
{
    if (params->flags != NULL) free( params->flags );
    if (params->set_flags != NULL) free( params->set_flags );
    if (params->vals != NULL) free( params->vals );
    free( params );
    params = NULL;
}

/** Creates a parameter system with a given number of values

    Allocates data structures for a parameter system with the given number of
    parameters. Parameters stored in the system are are accessed via and index number
    ranging from 0 to the number of values minus 1.

    \param nvals  [in]   Number of parameters.

    \return Handle to be passed to other parameter system routines or NULL if system cannot be created.
*/
PARAMS paramCreate( paramIndex nvals )
{
    PARAMS params = (PARAMS) calloc( 1, sizeof(paramList ));

    if ( nvals > 0 &&
         (params != NULL) &&
         ((params->flags = (int *) calloc( nvals, sizeof(int))) != NULL ) &&
         ((params->set_flags = (paramIndex *) calloc( nvals, sizeof(paramIndex))) != NULL ) &&
         ((params->vals = (paramVal *) calloc( nvals, sizeof(paramVal)) ) != NULL ) )
    {
        params->nvals = nvals;
    }
    else
    {
        paramDestroy( params );
    }

    return params;
}


/** Sets the value of an integer parameter.

    Sets the value of the parameter associated with a given index to an integer value.

    \param params [in]   Pointer to PARAM handle returned by paramCreate.
    \param index  [in]   Index number of the parameter.
    \param value  [in]   Value to be assigned to the parameter.

    \return Integer indicating 0 (PARAM_OK) for success or non-zero for index out of range. 
*/
int paramSetInteger( PARAMS params, paramIndex index, int value )
{
    int status = PARAM_ERROR;

    if (index >= 0 && index < params->nvals)
    {
        if ( params->vals[index].type != paramInt ||
             params->vals[index].data.ival != value )
        {
            params->flags[index] = 1;
            params->vals[index].type = paramInt;
            params->vals[index].data.ival = value;
        }
        status = PARAM_OK;
    }
    return status;
}

/** Sets the value of a double parameter.

    Sets the value of the parameter associated with a given index to a double value.

    \param params [in]   Pointer to PARAM handle returned by paramCreate.
    \param index  [in]   Index number of the parameter.
    \param value  [in]   Value to be assigned to the parameter.

    \return Integer indicating 0 (PARAM_OK) for success or non-zero for index out of range. 
*/
int paramSetDouble( PARAMS params, paramIndex index, double value )
{
    int status = PARAM_ERROR;

    if (index >=0 && index < params->nvals)
    {
        if ( params->vals[index].type != paramDouble ||
             params->vals[index].data.dval != value )
        {
            params->flags[index] = 1;
            params->vals[index].type = paramDouble;
            params->vals[index].data.dval = value;
        }
        status = PARAM_OK;
    }
    return status;
}

/** Gets the value of an integer parameter.

    Returns the value of the parameter associated with a given index as an integer value.

    \param params [in]   Pointer to PARAM handle returned by paramCreate.
    \param index  [in]   Index number of the parameter.
    \param value  [out]  Value of the parameter as a integer.

    \return Integer indicating 0 (PARAM_OK) for success or non-zero for index out of range. 
*/
int paramGetInteger( PARAMS params, paramIndex index, int * value )
{
    int status = PARAM_OK;

    if (index >= 0 && index < params->nvals)
    {
        switch (params->vals[index].type)
        {
        case paramDouble: *value = (int) floor(params->vals[index].data.dval+0.5); break;
        case paramInt: *value = params->vals[index].data.ival; break;
        default: status = 0;
        }
    }
    else status = PARAM_ERROR;

    return status;
}

/** Gets the value of a double parameter.

    Gets the value of the parameter associated with a given index as a double value.

    \param params [in]   Pointer to PARAM handle returned by paramCreate.
    \param index  [in]   Index number of the parameter.
    \param value  [out]  Value of the parameter as a double.

    \return Integer indicating 0 (PARAM_OK) for success or non-zero for index out of range. 
*/
int paramGetDouble( PARAMS params, paramIndex index, double * value )
{
    int status = PARAM_OK;

    if (index >= 0 && index < params->nvals)
    {
        switch (params->vals[index].type)
        {
        case paramDouble: *value = params->vals[index].data.dval; break;
        case paramInt: *value = (double) params->vals[index].data.ival; break;
        default: status = 0;
        }
    }
    else status = PARAM_ERROR;

    return status;
}

/** Sets a callback routing to call when parameters change

    This sets the value of a routine which is called whenever the user calls paramCallCallback and
    a value in the parameter system has been changed.

    \param params   [in]   Pointer to PARAM handle returned by paramCreate.
    \param callback [in]   Index number of the parameter. This must be a routine that
                           takes three parameters and returns void.
                           The first paramet is the pointer passed as the third parameter to this routine.
			   The second is an integer indicating the number of parameters that have changed.
			   The third is an array of parameter indices that indicates the parameters that
			   have changed.
    \param param    [in]   Pointer to a paramemter to be passed to the callback routine.

    \return Integer indicating 0 (PARAM_OK) for success or non-zero for index out of range. 
*/
int paramSetCallback( PARAMS params, paramCallback callback, void * param )
{
    params->callback = callback;
    params->param = param;
    return PARAM_OK;
}

/** Calls the callback routine indicating which parameters have changed.

    This routine should be called whenever you have changed a number of parameters and wish
    to notify someone (via the callback routine) that they have changed.

    \param params   [in]   Pointer to PARAM handle returned by paramCreate.

    \return void
*/
void paramCallCallback( PARAMS params )
{
    unsigned int i;
    int nFlags=0;

    for (i = 0; i < params->nvals; i++)
    {
        if (params->flags[i])
        {
            params->set_flags[nFlags] = i;
            nFlags++;
            params->flags[i] = 0;
        }

    }
    if ( nFlags > 0 && params->callback != NULL ) params->callback( params->param, nFlags, params->set_flags );
}

/** Prints the current values in the parameter system to stdout

    This routine prints all the values in the parameter system to stdout. 
    If the values are currently undefined, this is noted.

    \param params   [in]   Pointer to PARAM handle returned by paramCreate.

    \return void
*/
void paramDump( PARAMS params )
{
    unsigned int i;

    printf( "Number of parameters is: %d\n", params->nvals );
    for (i =0; i < params->nvals; i++)
    {
        switch (params->vals[i].type)
        {
        case paramDouble:
            printf( "Parameter %d is a double, value %f\n", i, params->vals[i].data.dval );
            break;
        case paramInt:
            printf( "Parameter %d is an integer, value %d\n", i, params->vals[i].data.ival );
            break;
        default:
            printf( "Parameter %d is undefined\n", i );
            break;
        }
    }
}
