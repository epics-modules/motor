/* File: devXPSC8.cc                     */

/* Device Support Routines for motor record for XPS C8 Motor Controller */
/*
 *      Original Author: Jon Kelly
 *
 * Modification Log:
 * -----------------
 *
 * 13th May 2005
 * 
 * 1) The use of multiaxis groups has been enabled with the addition of 
 * variables to the drvXPSC8.cc: XPSC8Name_config function used in the st.cmd. 
 * 2) A cue feature has now been added so that if a command is sent while
 * a group is busy it is cued until the group becomes static.
 * 3) The driver waits for a time specified in drvXPSC8.h: XPSC8_QUE_PAUSE when
 * a command is issued. It then performs all the motions specified in the lapsed
 * time as a single syncronised motion.  The pause is performed in drvXPSC8.cc:
 * send_mess.
 *
 */


#define VERSION 1.00

#include        <string.h>
#include        <epicsMutex.h>
#include        <epicsThread.h>
#include        "motorRecord.h"
#include        "motor.h"
#include        "motordevCom.h"

#include        "drvXPSC8.h"
#include        "xps_c8_driver.h"
#include        <epicsExport.h>

#define STATIC static
extern struct driver_table XPSC8_access;

#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)
#define MIN(a,b) ((a)<(b) ? (a) : (b))
#define ABS(f) ((f)>0 ? (f) : -(f))

#define MOVING 1

/*----------------debugging-----------------*/

#define DEBUG

#ifdef __GNUG__
    #ifdef	DEBUG
	volatile int devXPSC8Debug = 3;
	#define Debug(L, FMT, V...) { if(L <= devXPSC8Debug) \
			{ printf("%s(%d):",__FILE__,__LINE__); \
			  printf(FMT,##V); } }
	epicsExportAddress(int, devXPSC8Debug);
    #else 
	#define Debug(L, FMT, V...)
    #endif  
#else 
    #define Debug()
#endif



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
extern "C" {epicsExportAddress(dset, devXPSC8);}


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

    Debug(10, "XPSC8_init, after=%d\n", after);
    if (after == 0) {
        drvtabptr = &XPSC8_access;
        Debug(10, "XPSC8_init, calling driver initialization\n");
        (drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, 
                            &XPSC8_cards);
    Debug(10, "XPSC8_init, end of function\n");
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

    Debug(10, "--------XPSC8_init_record \n");
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
    mr->rmp = NINT((cntrl->currentposition) / (cntrl->resolution));
    Debug(1, "XPSC8_init_record: card=%d, signal=%d, currentposition=%f"\
          " resolution=%f, mr->rmp=%d\n",\
          card, signal, cntrl->currentposition, cntrl->resolution, mr->rmp);
    return(rtnval);
}


/* start building a transaction */
STATIC long XPSC8_start_trans(struct motorRecord *mr)
{
    long rtnval;
    rtnval = motor_start_trans_com(mr, XPSC8_cards);
    return(rtnval);
}


/* end building a transaction */
STATIC RTN_STATUS XPSC8_end_trans(struct motorRecord *mr)
{
    RTN_STATUS rtnval;
    rtnval = motor_end_trans_com(mr, drvtabptr);
    return(rtnval);

}


/* add a part to the transaction */
STATIC RTN_STATUS XPSC8_build_trans(motor_cmnd command,\
					 double *parms, \
					 struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct XPSC8controller *control;
    struct XPSC8axis         *cntrl;
    struct XPSC8group   *groupcntrl;	/*XPS group specific data */
    double dval=0.0,resolution,steps;
    int ival=0;
    RTN_STATUS rtnval=OK;
    int card, signal;
    int groupnumber, groupsize;
    int axisingroup, groupstatus;
    int status;

    if (parms != NULL){
        dval = parms[0]; /* This is the record DVAL which you set */
                         /* to move e.g. dval = 10mm and command = MOVE_ABS*/
        ival = NINT(parms[0]);
    }
    
    motor_call = &(trans->motor_call);
    card = motor_call->card;
    signal = motor_call->signal;
    brdptr = (*trans->tabptr->card_array)[card];
    
    Debug(10, "XPSC8_build_trans: After brdptr command\n");
    
    if (brdptr == NULL)
        return(rtnval = ERROR);

    control = (struct XPSC8controller *) brdptr->DevicePrivate;
    cntrl = (struct XPSC8axis *)&control->axis[signal];
    
    groupstatus = cntrl->groupstatus;    /* From XPS controller */
    axisingroup = cntrl->axisingroup;    /* Pull in group info */
    groupnumber = cntrl->groupnumber;
    
    groupcntrl = (struct XPSC8group *)&control->group[groupnumber];
    groupsize = groupcntrl->groupsize;	 /* Number of motors in group */

    cntrl->resolution = mr->mres;    /* Read in the motor resolution */
    resolution = cntrl->resolution;
    steps = resolution * dval;		/* This could be a position or velocity */
    
/*    mr->dllm = cntrl->minlimit;*/        /* set the epics limits to the XPS limits */
/*    mr->dhlm = cntrl->maxlimit; */   
    
    Debug(10, "XPSC8_build_trans: card=%d, signal=%d, command=%d, ival=%d"\
          " dval=%f, steps=%f\n",\
          card, signal, command, ival, dval, steps);    
    
    Debug(10, "XPSC8_build_trans: resolution=%f\n",resolution);
    if (XPSC8_table[command] > motor_call->type)
        motor_call->type = XPSC8_table[command];
    if (trans->state != BUILD_STATE)
        return(rtnval = ERROR);
   
    epicsMutexLock(control->XPSC8Lock);

    /* No need to deal with initialization, premove or postmove strings, 
       XPSC8 does not support */

    Debug(10, "build_trans: Top Of Switch command=%d, cntrl->moving=%d"\
          " \n GroupStat=%d\n", command,cntrl->moving,cntrl->groupstatus);
    
    Debug(5,"Build_trans: com=%d, axis=%d, moving=%d socket=%i psocket=%i\n",\
              command,signal,cntrl->moving,cntrl->socket,cntrl->devpollsocket);
    
    status = GroupStatusGet(cntrl->devpollsocket, cntrl->groupname,
                                &cntrl->groupstatus);
    if (status != 0) 
        printf("BuildTrans Error performing GroupStatusGet status=%d\n", status); 
        	   
    groupstatus = cntrl->groupstatus;	/* Update groupstatus */            	  
	         
    switch (command) {
    case MOVE_ABS:/* command 0*/
  
	
	if ((groupstatus < 10) || (groupstatus == 47)) {
            /* ie not initialized state or Jogging!*/
		break;}
		
	/* If there is no cue, update the cue array to make sure you don't move */
	/* the wrong motors */
	if ((groupcntrl->cuesize == 0) && (groupstatus > 9 && groupstatus < 20)){
	    status = GroupPositionCurrentGet(cntrl->devpollsocket,
                                         cntrl->groupname,
                                         groupsize,
                                         groupcntrl->positionarray); /* Array! */

	    if (status != 0) {
            	printf(" Error performing GroupPositionCurrentGet\n");
            }	    
	}

	
	/* Always add the move to the cue */
	groupcntrl->positionarray[axisingroup] =  steps;   
	++groupcntrl->cuesize;		/* Add 1 to cue total */
	if (groupcntrl->cuesize == 1)
		groupcntrl->cueflag = 1;/* If first call set flag */     
	if (groupcntrl->cuesize > 1)
	    	Debug(2,"******Adding move to an existing cue***\n"); 
		
	 /* The communication and looping has been moved to drvXPSC8 send_mess */
	              
        break;

    case MOVE_REL:/*1*/
    	/* The motor record seems to impliment the relative move by 
	   calculating the new position and calling MOVE_ABS */

    case HOME_FOR: 
        
    case HOME_REV: /*3*/
        if (cntrl->groupstatus == 43 ) {	/* Allready Homing */
	    break;}
	
	/* If motion has been killed the group will need to be initialized*/
        /* and homed before the motors can be driven again */
        
	if (cntrl->groupstatus < 10 ) {
            /* ie not initialized state!*/
            status = GroupInitialize(cntrl->devpollsocket,cntrl->groupname);
            if (status != 0) {
                printf("HOME Command Error performing GroupInitialise\n");
            } 
            
            status = GroupHomeSearch(cntrl->socket,cntrl->groupname);
            if (status != 0) {
                printf(" Error performing GroupHomeSearch\n");
            }
            
            goto home_rev_end; 
        }
	
	/* Else kill all the motions an home */
	
        status = GroupKill(cntrl->devpollsocket,cntrl->groupname);
        if (status != 0) {
            printf(" Error performing GroupKill\n");
        }   
    
        status = GroupInitialize(cntrl->devpollsocket,cntrl->groupname);
        if (status != 0) {
            printf("HOME Command Error performing GroupInitialise\n");
        } 
     
        status = GroupHomeSearch(cntrl->socket,cntrl->groupname);
        if (status != 0) {
            printf(" Error performing GroupHomeSearch\n");
        } 
        Debug(2, "XPSC8_build_trans:******* Perform GroupHomeSearch\n"); 
	Debug(2, "XPSC8_build_trans:groupsize=%i\n",groupsize);      
             
	
home_rev_end:			/* Used for goto statment above */

		/* When a group is homed the drive cue is emptied */
		
	groupcntrl->cuesize = 0;		/* Reset cue */
	groupcntrl->cueflag = 0;
	
        break;

    case LOAD_POS:/* 4*/
        /* command 4 Not used*/
	
        break;

    case SET_VEL_BASE: 
        /* The XPS does not have a different Base velocity!!!*/
        break;
    case SET_VELOCITY: /*6*/
        /* The XPS does not have a different Base velocity!!!!!
           So I perform the same operation for VEL_BASE and VEL*/
    
        status = PositionerSGammaParametersSet(cntrl->devpollsocket,
                                               cntrl->positionername, steps,
                                               cntrl->accel, 
                                               cntrl->minjerktime,
                                               cntrl->maxjerktime);
        if (status != 0) {
            printf(" Error performing PositionerSGammaParameters Set Vel\n");
	    printf(" steps=%f resoulution=%f DVAL=%f\n",steps,resolution,dval);
	}    
        else cntrl->velocity = steps;
        break;

    case SET_ACCEL:    /* command 7 */
        status = PositionerSGammaParametersSet(cntrl->devpollsocket,
                                               cntrl->positionername,
                                               cntrl->velocity,
                                               steps, cntrl->minjerktime,
                                               cntrl->maxjerktime);
        if (status != 0) {
            printf(" Error performing PositionerSGammaParameters Set Accel %i\n",
	    							status);
	    printf(" steps=%f resoulution=%f DVAL=%f\n",steps,resolution,dval);							
	if (status == -17) 
            printf("devXPSC8 BuildTrans: One of the parameters was out of range!");
	      }	  
        else cntrl->accel = steps;
        break;

    case GO:          /* 8 */ 
        if (groupstatus == 20) { /* If disabled then enable */
	    status = GroupMotionEnable(cntrl->devpollsocket,cntrl->groupname);
	    if (status != 0) 
                printf(" Error performing GroupMotionEnable %i\n",status);
	}
        break;

    case SET_ENC_RATIO:        /* These must be set in the Stages.ini file */
        break;

    case GET_INFO:     /* 10 * This is run when you press Go from stop!*/ 
        break;

    case STOP_AXIS: /* 11 */
        /* The whole group must stop, not just 1 axis */
	/* Update status to see if the group is moving */
	
	if (cntrl->groupstatus > 42) { 
            /* Then the group is moving! */              
            status = GroupMoveAbort(cntrl->devpollsocket,cntrl->groupname);
            if (status != 0) {
	        printf(" Error performing GroupMoveAbort = %i %s(\n",\
	    					status,cntrl->groupname);
		
		}		
	/* When a group is stopped the drive cue is emptied and reset*/
	    
	    status = GroupPositionCurrentGet(cntrl->devpollsocket,
                                         cntrl->groupname,
                                         groupsize,
                                         groupcntrl->positionarray); 
	     if (status != 0) {
                printf(" Error performing GroupPositionCurrentGet\n");
             }
	}
	groupcntrl->cuesize = 0;		/* Reset cue */
	groupcntrl->cueflag = 0;
        break;

    case JOG:    
        /* I need more commandds to impliment this i.e. enable/disable jog
           set velocity and acceleration */
        break;

    case SET_PGAIN: /* 13 */
    case SET_IGAIN:
    case SET_DGAIN:
        /* These can be implimented but there are so many variables */
        /* It makes sense to do it in the stages.ini file */
    case ENABLE_TORQUE:
    case DISABL_TORQUE:
        /* The XPSC8 does not support gain or torque commands */
        break;

    case SET_HIGH_LIMIT: /*18 */
        Debug(10, "XPSC8_build_trans highlimit: socket=%d, posname=%s, "\
              "minlim=%f, steps=%f\n",\
              cntrl->socket, cntrl->positionername, cntrl->minlimit, steps);    

        status = PositionerUserTravelLimitsSet(cntrl->devpollsocket,
                                               cntrl->positionername,
                                               cntrl->minlimit, steps);
        if (status != 0) 
            printf(" Error performing PositionerUserTravelLimitsGet "
                   " Max Limit status=%d\n",status);
        else cntrl->maxlimit = steps;
        break;
    
    case SET_LOW_LIMIT:
        status = PositionerUserTravelLimitsSet(cntrl->devpollsocket,
                                               cntrl->positionername,
                                               steps, cntrl->maxlimit);
        if (status != 0) 
            printf("Error performing PositionerUserTravelLimitsGet Min Limit\n");
        else cntrl->minlimit = steps;
        break;

    default:
        rtnval = ERROR;
    }

    /* Free the lock */
    epicsMutexUnlock(control->XPSC8Lock);
    Debug(10, "End Of Build_trans after Switch\n");
    return (rtnval);
}
