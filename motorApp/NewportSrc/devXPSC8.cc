/* File: devXPSC8.cc                     */

/* Device Support Routines for motor record for XPS C8 Motor Controller */
/*
 *      Original Author: Jon Kelly
 *
 */


#define VERSION 1.00

#include        <string.h>
#include        <epicsMutex.h>
#include        <epicsExport.h>
#include        <epicsThread.h>
#include        "motorRecord.h"
#include        "motor.h"
#include        "motordevCom.h"
#include        "drvXPSC8.h"

/*#define DLL _declspec(dllexport)*/
#include        "xps_c8_driver.h"

#define STATIC static
extern struct driver_table XPSC8_access;

#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)
#define MIN(a,b) ((a)<(b) ? (a) : (b))
#define ABS(f) ((f)>0 ? (f) : -(f))

#define MOVING 1


#ifdef NODEBUG
#define Debug(L,FMT,V) ;
#else
volatile int devXPSC8Debug = 0;
epicsExportAddress(int, devXPSC8Debug); 
/* To make the var available to the shell */

#define Debug(L,FMT,V...) {  if(L <= devXPSC8Debug) \
                        { printf("%s(%d):",__FILE__,__LINE__); \
                          printf(FMT,##V); } }
#endif

/* Debugging levels:
 *      devXPSC8Debug >= 3  Print new part of command and command string so far
 *                          at the end of XPSC8_build_trans
 */


/* ----------------Create the dsets for devXPSC8----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long XPSC8_init(int);
STATIC long XPSC8_init_record(struct motorRecord *);
STATIC long XPSC8_start_trans(struct motorRecord *);
STATIC RTN_STATUS XPSC8_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS XPSC8_end_trans(struct motorRecord *);

struct motor_dset devXPSC8 =
{
    {8, NULL, (DEVSUPFUN)XPSC8_init, (DEVSUPFUN)XPSC8_init_record, NULL},
    motor_update_values,
    XPSC8_start_trans,
    XPSC8_build_trans,
    XPSC8_end_trans
};
epicsExportAddress(dset,devXPSC8);


/* --------------------------- program data --------------------- */
/* This table is used to define the command types */

static msg_types XPSC8_table[] = {
    MOTION,     /* MOVE_ABS */
    MOTION,     /* MOVE_REL */
    MOTION,     /* HOME_FOR */
    MOTION,     /* HOME_REV */
    IMMEDIATE,  /* LOAD_POS */
    IMMEDIATE,  /* SET_VEL_BASE */
    IMMEDIATE,  /* SET_VELOCITY */
    IMMEDIATE,  /* SET_ACCEL */
    IMMEDIATE,  /* GO */
    IMMEDIATE,  /* SET_ENC_RATIO */
    INFO,       /* GET_INFO */
    MOVE_TERM,  /* STOP_AXIS */
    VELOCITY,   /* JOG */
    IMMEDIATE,  /* SET_PGAIN */
    IMMEDIATE,  /* SET_IGAIN */
    IMMEDIATE,  /* SET_DGAIN */
    IMMEDIATE,  /* ENABLE_TORQUE */
    IMMEDIATE,  /* DISABL_TORQUE */
    IMMEDIATE,  /* PRIMITIVE */
    IMMEDIATE,  /* SET_HIGH_LIMIT */
    IMMEDIATE   /* SET_LOW_LIMIT */
};


static struct board_stat **XPSC8_cards;

/* --------------------------- program data --------------------- */



/* initialize device support for XPSC8 stepper motor */
STATIC long XPSC8_init(int after)
{
    long rtnval;

    Debug(1, "XPSC8_init, after=%d\n", after);
    if (after == 0) {
        drvtabptr = &XPSC8_access;
        Debug(1, "XPSC8_init, calling driver initialization\n");
        (drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, 
                            &XPSC8_cards);
    Debug(1, "XPSC8_init, end of function\n");
    return(rtnval);
}


/* initialize a record instance */
STATIC long XPSC8_init_record(struct motorRecord *mr)
{
    long rtnval;
    struct motor_trans *trans;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct XPSC8controller *control;
    struct XPSC8axis         *cntrl;
    int card, signal;

    Debug(1, "--------XPSC8_init_record \n");
    rtnval = motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, 
                                   drvtabptr, XPSC8_cards);
    /* We have a logic problem here.  motor_init has read in the motor
     * position in engineering units, but it did not yet know the resolution
     * to convert to steps. It thus did a divide by zero.
     * We have to wait till after calling 
     * motor_init_record_com for mr->dpvt to be initialized, and then set 
     * the rmp field of the record again */
    trans = (struct motor_trans *) mr->dpvt;
    motor_call = &(trans->motor_call);
    card = mr->out.value.vmeio.card;
    signal = mr->out.value.vmeio.signal;
    brdptr = (*trans->tabptr->card_array)[card];
    control = (struct XPSC8controller *) brdptr->DevicePrivate;
    cntrl = (struct XPSC8axis *)&control->axis[signal];
    cntrl->resolution = mr->mres;    /* Set the motor resolution */
    mr->rmp = NINT(cntrl->currentposition[1] / cntrl->resolution);
    Debug(1, "XPSC8_init_record: card=%d, signal=%d, currentposition[1]=%f"
          " resolution=%f, mr->rmp=%d\n", 
          card, signal, cntrl->currentposition[1], cntrl->resolution, mr->rmp);
    return(rtnval);
}


/* start building a transaction */
STATIC long XPSC8_start_trans(struct motorRecord *mr)
{
    Debug(1, "--------XPSC8_start_trans\n");
    long rtnval;
    rtnval = motor_start_trans_com(mr, XPSC8_cards);
    return(rtnval);
}


/* end building a transaction */
STATIC RTN_STATUS XPSC8_end_trans(struct motorRecord *mr)
{
    Debug(1, "--------XPSC8_end_trans\n");
    RTN_STATUS rtnval;
    rtnval = motor_end_trans_com(mr, drvtabptr);
    return(rtnval);

}


/* add a part to the transaction */
STATIC RTN_STATUS XPSC8_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct XPSC8controller *control;
    struct XPSC8axis         *cntrl;
    double dval=0.,resolution,steps;
    int ival=0;
    RTN_STATUS rtnval=OK;
    int card, signal;

    int positioner, status;

    positioner = 1; /* Means only move one axis at a time */
    if (parms != NULL){
        dval = parms[0]; /* I assume this is the record DVAL which you set*/
                         /* to move e.g. dval = 10mm and command = MOVE_ABS*/
        ival = NINT(parms[0]);
    }
    
    motor_call = &(trans->motor_call);
    card = motor_call->card;
    signal = motor_call->signal;
    brdptr = (*trans->tabptr->card_array)[card];
    
    Debug(11, "XPSC8_build_trans: After brdptr command\n");
    
    if (brdptr == NULL)
        return(rtnval = ERROR);

    control = (struct XPSC8controller *) brdptr->DevicePrivate;
    cntrl = (struct XPSC8axis *)&control->axis[signal];

    resolution = cntrl->resolution;
    steps = resolution * dval;
    
/*    mr->dllm = cntrl->minlimit;*/        /* set the epics limits to the XPS limits */
/*    mr->dhlm = cntrl->maxlimit; */   
    
    Debug(1, "XPSC8_build_trans: card=%d, signal=%d, command=%d, ival=%d"
          " dval=%f, steps=%f\n", 
          card, signal, command, ival, dval, steps);    
    
    Debug(1, "XPSC8_build_trans: resolution=%f\n",resolution);
    if (XPSC8_table[command] > motor_call->type)
        motor_call->type = XPSC8_table[command];
    Debug(11, "XPSC8_build_trans: After cntrl command\n");
    if (trans->state != BUILD_STATE)
        return(rtnval = ERROR);
    Debug(11, "XPSC8_build_trans: After cntrl command\n");
    /* Take a lock so that only 1 thread can be talking to the XPSC8 at
     *  once.  I don't know if this is needed?  */
    epicsMutexLock(control->XPSC8Lock);

    /* No need to deal with initialization, premove or postmove strings, 
       XPSC8 does not support */

    Debug(1, "build_trans: Top Of Switch command=%d, cntrl->moving=%d"
          " GStat=%d-\n", command,cntrl->moving,cntrl->groupstatus);       

    switch (command) {
    case MOVE_ABS:/* command 0*/
  
        Debug(1, "XPSC8_build_trans: command=%d, Move_ABS moving=%d steps=%f\n", 
              command,cntrl->moving, steps);    

        if ((cntrl->moving) == MOVING) {
            printf("--STILL MOVING--\n"); 
            goto done;
        } /* This is set in drvXPSC8.cc*/

        Debug(11, "Move_ABS socket=%d posiname=%s posit=%d dval=%f\n",
              cntrl->socket, cntrl->positionername, positioner, dval);
        epicsThreadSleep(0.1);
            
        status = GroupMoveAbsolute(cntrl->socket, cntrl->positionername,
                                   positioner, &steps);
        if (status != 0) {
            printf(" Error performing GroupMoveAbsolute\n");
        }

        Debug(1, "--After GroupMoveAbsolute command=%d,Move_ABS moving=%d \n", 
              command,cntrl->moving);  
    
        break;

    case MOVE_REL:
        /* Using the SGamma setings in stages.ini */
        if (cntrl->moving == MOVING) goto done; 
        /* This is set in drvXPSC8.cc*/
        status = GroupMoveRelative(cntrl->socket, cntrl->positionername,
                                   positioner, &steps);
        if (status != 0) printf(" Error performing GroupMoveRelative\n");
        break;

    case HOME_FOR:
    case HOME_REV:
        /* If motion has been killed the group will need to be initialized*/
        /* and homed before the motors can be driven again */
        if (cntrl->groupstatus < 10 ) {
            /* ie not initialized state!*/
            status = GroupInitialize(cntrl->pollsocket,cntrl->groupname);
            if (status != 0) {
                printf("HOME Command Error performing GroupInitialise\n");
            } 
            status = GroupStatusGet(cntrl->pollsocket, cntrl->groupname,
                                    &cntrl->groupstatus);
            if (status != 0) {
                printf(" Error performing GroupStatusGet\n");
            }    
            Debug(1, "XPSC8_build_trans:****** Perform GroupInitialize\n"); 
            status = GroupHomeSearch(cntrl->socket,cntrl->groupname);
            if (status != 0) {
                printf(" Error performing GroupHomeSearch\n");
            } 
            Debug(1, "XPSC8_build_trans:******* Perform GroupHomeSearch\n");
            
            break; 
        }

        status = GroupKill(cntrl->pollsocket,cntrl->groupname);
        if (status != 0) {
            printf(" Error performing GroupKill\n");
        }   
    
        status = GroupInitialize(cntrl->pollsocket,cntrl->groupname);
        if (status != 0) {
            printf("HOME Command Error performing GroupInitialise\n");
        } 
 
 
        status = GroupStatusGet(cntrl->pollsocket, cntrl->groupname,
                                &cntrl->groupstatus);
        if (status != 0) {
            printf(" Error performing GroupStatusGet\n");
        }    
        Debug(1, "XPSC8_build_trans:****** Perform GroupInitialize\n"); 
     
        status = GroupHomeSearch(cntrl->socket,cntrl->groupname);
        if (status != 0) {
            printf(" Error performing GroupHomeSearch\n");
        } 
        Debug(1, "XPSC8_build_trans:******* Perform GroupHomeSearch\n");       
             
                /*}*/
                
        /*    if (cntrl->groupstatus == NOTREF ){    */        
        /* you must kill a group before homing*/
        /*}*/
        break;

    case LOAD_POS:
        /* command 4 I assume it means just update the position value? */
        /*status = GroupPositionCurrentGet(cntrl->socket, 
                                           cntrl->positionername,
                                           positioner, 
                                           &cntrl->currentposition[1]);
        if (status != 0) 
            printf(" Error performing GroupPositionCurrentGet Load Pos\n");*/
        break;

    case SET_VEL_BASE: 
        /* The XPS does not have a different Base velocity!!!*/
        break;
    case SET_VELOCITY: 
        /* The XPS does not have a different Base velocity!!!!!
           So I perform the same operation for VEL_BASE and VEL*/
    
        status = PositionerSGammaParametersSet(cntrl->pollsocket,
                                               cntrl->positionername, steps,
                                               cntrl->accel, 
                                               cntrl->minjerktime,
                                               cntrl->maxjerktime);
        if (status != 0) 
            printf(" Error performing PositionerSGammaParameters Set Vel\n");
        else cntrl->velocity = steps;
        break;

    case SET_ACCEL:    /* command 7 */
        status = PositionerSGammaParametersSet(cntrl->pollsocket,
                                               cntrl->positionername,
                                               cntrl->velocity,
                                               steps, cntrl->minjerktime,
                                               cntrl->maxjerktime);
        if (status != 0) 
            printf(" Error performing PositionerSGammaParameters Set Accel\n");
        else cntrl->accel = steps;
        break;

    case GO:            
        break;

    case SET_ENC_RATIO:        /* These must be set in the Stages.ini file */
        break;

    case GET_INFO:        /* Again I don't know what this is intended to do? */
        break;

    case STOP_AXIS: 
        /* The whole group must stop, not just 1 axis */
        if (cntrl->groupstatus > 42) { 
            /* Then the group is moving! */              
            status = GroupMoveAbort(cntrl->pollsocket,cntrl->groupname);
            if (status != 0) printf(" Error performing GroupMoveAbort(\n");
        } 
        break;

    case JOG:    
        /* I need more commandds to impliment this i.e. enable/disable
           set velocity and acceleration */
        break;

    case SET_PGAIN:
    case SET_IGAIN:
    case SET_DGAIN:
        /* These can be implimented but there are so many variables */
        /* It makes sense to do it in the stages.ini file */
    case ENABLE_TORQUE:
    case DISABL_TORQUE:
        /* The XPSC8 does not support gain or torque commands */
        break;

    case SET_HIGH_LIMIT:
        Debug(1, "XPSC8_build_trans highlimit: socket=%d, posname=%s, "
              "minlim=%f, steps=%f\n", 
              cntrl->socket, cntrl->positionername, cntrl->minlimit, steps);    

        status = PositionerUserTravelLimitsSet(cntrl->pollsocket,
                                               cntrl->positionername,
                                               cntrl->minlimit, steps);
        if (status != 0) 
            printf(" Error performing PositionerUserTravelLimitsGet "
                   " Max Limit status=%d\n",status);
        else cntrl->maxlimit = steps;
        break;
    
    case SET_LOW_LIMIT:
        status = PositionerUserTravelLimitsSet(cntrl->pollsocket,
                                               cntrl->positionername,
                                               steps, cntrl->maxlimit);
        if (status != 0) 
            printf("Error performing PositionerUserTravelLimitsGet Min Limit\n");
        else cntrl->minlimit = steps;
        break;

    default:
        rtnval = ERROR;
    }

done:
    /* Free the lock */
    epicsMutexUnlock(control->XPSC8Lock);
    Debug(1, "End Of Build_trans after Switch\n");
    return (rtnval);
}
