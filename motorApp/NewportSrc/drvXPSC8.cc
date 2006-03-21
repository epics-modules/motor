/*
 *      drvXPSC8.cc
 *      Motor record driver level support for Newport XPSC8 motor controller.
 *
 *      By Jon Kelly
 *      2005
 *
 * Modification Log:
 * -----------------
 *  13th May 2005
 * The driver waits for a time specified in drvXPSC8.h: XPSC8_QUE_PAUSE when
 * a command is issued. It then performs all the motions specified in the lasped
 * time as a single syncronised motion.  The pause is performed in drvXPSC8.cc:
 * send_mess.
 *
 * 30th June 2005
 * The driver is converted to asyn.  The XPS some times returns an accel=zero in 
 * readStatus so I have added a do loop to repeat the command until a non zero
 * value is returned
 */

#include        <string.h>
#include        <stdio.h>
#include        <epicsThread.h>
#include        <epicsMutex.h>
#include        <epicsString.h>
#include        <drvSup.h>
#include        "motor.h"

#include        "drvXPSC8.h"
#include        "xps_c8_driver.h"
#include        <epicsExport.h>

#define STATIC static

/*----------------debugging-----------------*/
#define DEBUG

#ifdef __GNUG__
    #ifdef        DEBUG
        #define Debug(l, f, args...) { if(l<=drvXPSC8Debug) printf(f,## args); }
    #else
        #define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif
volatile int drvXPSC8Debug = 0;
/* --- Local data. --- */ 
int XPSC8_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"

/*----------------functions-----------------*/
STATIC int recv_mess(int, char *, int);
STATIC RTN_STATUS send_mess(int, const char *, char *);
STATIC void start_status(int card);
/*STATIC void XPSC8Status(int card);*/
STATIC int set_status(int card, int signal);
static long report(int level);
static long init();
STATIC int motor_init();
STATIC void query_done(int, int, struct mess_node *);
STATIC void readXPSC8Status(int card);

/*----------------functions-----------------*/

struct driver_table XPSC8_access =
{
    motor_init,
    motor_send,
    motor_free,
    motor_card_info,
    motor_axis_info,
    &mess_queue,
    &queue_lock,
    &free_list,
    &freelist_lock,
    &motor_sem,
    &motor_state,
    &total_cards,
    &any_motor_in_motion,
    send_mess,
    recv_mess,
    set_status,
    query_done,
    start_status,
    &initialized,
    NULL
};


struct
{
    long number;
#ifdef __cplusplus
    long (*report) (int);
    long (*init) (void);
#else
    DRVSUPFUN report;
    DRVSUPFUN init;
#endif
} drvXPSC8 = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvXPSC8);}

/* I don't know where the XPSC8_access is set?*/
STATIC struct thread_args targs = {SCAN_RATE, &XPSC8_access, 0.0};


/*********************************************************
 * Print out driver status report
 *********************************************************/

static long report(int level)   /*##########  XPSC8 Version */
{
    int card;

    if (XPSC8_num_cards <=0)
        printf("    No XPSC8 controllers configured.\n");
    else {
        for (card = 0; card < XPSC8_num_cards; card++) {
            struct controller *brdptr = motor_state[card];
            if (brdptr == NULL)
                printf("    XPSC8 controller %d connection failed.\n", card);
            else {
                struct XPSC8controller *control;

                control = (struct XPSC8controller *) brdptr->DevicePrivate;
                printf("    XPSC8 driver %d, axis:%s \n",
                       card, brdptr->ident);
            }
        }
    }
    return (0);
}


static long init()
{
    if (XPSC8_num_cards <= 0)
    {
        Debug(10, "init() XPSC8etup() missing from startup script.%i\n",
              XPSC8_num_cards);
    }
    return ((long) 0);
}



STATIC void query_done(int card, int axis, struct mess_node *nodeptr)
{
}

/*********************************************************
 * Read the status and position of all motors on a card
 * start_status(int card)
 *            if card == -1 then start all cards
 *********************************************************/

STATIC void start_status(int card)
{
    int itera;

    Debug(10, "start_status: card=%d  total_cards=%d\n", card,total_cards);
    if (card >= 0) {
        readXPSC8Status(card);
    } else {
        for (itera = 0; itera < total_cards; itera++) {
            if (motor_state[itera]) readXPSC8Status(itera);
        }
    }
}


/*****************************************************************
 * Read in the motion parameters such as velocity, Accel, Position etc
 *****************************************************************/ 

STATIC void readXPSC8Status(int card)
{
    struct XPSC8controller *control =
                (struct XPSC8controller *) motor_state[card]->DevicePrivate;
    struct XPSC8axis *cntrl;
    int status,statuserror;
    int XPSC8_num_axes;
    int i; 
    struct mess_node *nodeptr; /* These are to print the DONE flag */
    register struct mess_info *motor_info;

    int groupsize, groupnumber; /* To be read in from struct */
    struct XPSC8group *groupcntrl;        /*XPS group specific data */     
    double position[XPSC8_NUM_CHANNELS]; /* To temp store position array */
    int axisingroup;
    int loop;        /* Variables used when the XPS returns accel=zero */
    int max_loop = 10;        /* This creates an error when we set the vel/accel */
    
    msta_field statusflags;
    status = 0;
    statuserror = 0; /* this = 1 if an error occurs */
    XPSC8_num_axes = motor_state[card]->total_axis;

    Debug(10, "XPSC8:readXPSC8Status card=%d num_axes=%d\n",card,XPSC8_num_axes);
    
    epicsMutexLock(control->XPSC8Lock);
    /*epicsThreadSleep(10);*/
    
    for (i=0; i<XPSC8_num_axes;i++) {    

        motor_info = &(motor_state[card]->motor_info[i]); /*To print DONE flag*/
        nodeptr = motor_info->motor_motion;
        statusflags.All = motor_info->status.All;

        Debug(10, "XPSC8:readXPSC8Status RA_DONE=%d, RA_MOVING=%d, "
                 "RA_PROBLEM=%d\n",
              statusflags.Bits.RA_DONE, statusflags.Bits.RA_MOVING,
              statusflags.Bits.RA_PROBLEM);

        control = (struct XPSC8controller *) motor_state[card]->DevicePrivate;
        cntrl = (struct XPSC8axis *)&control->axis[i];

        axisingroup = cntrl->axisingroup;
        if ((axisingroup < 0) || (axisingroup > 7) ) {
            printf(" Error Axis In Group Out Of Range status=%d\n", status); 
            statuserror =1;
        }

        groupnumber = cntrl->groupnumber;
        if ((groupnumber < 0) || (groupnumber > 7) ) {
            printf(" Error Group Number Out Of Range status=%d\n", status); 
            statuserror =1;
        }
        groupcntrl = (struct XPSC8group *)&control->group[groupnumber];

        groupsize = groupcntrl->groupsize;
        if ((groupsize < 1) || (groupsize > 8) ) {
            printf(" Error Group Size Out Of Range status=%d\n", status); 
            statuserror =1;
        }
        Debug(10, "XPSC8:readXPSC8Status groupsize=%i groupnumber=%i\n",
              groupsize,groupnumber);
        Debug(10, "XPSC8:readXPSC8Status card=%d axis=%d sock=%d gp=%s\n",card,i,
              cntrl->socket,cntrl->groupname);
    
        status = GroupStatusGet(cntrl->pollsocket, cntrl->groupname,
                                &cntrl->groupstatus);
        if (status != 0) {
            printf(" ReadStatus Error performing GroupStatusGet status=%d\n", status); 
            statuserror =1;
        }

        /* This do loop is required because some times the XPS returns accel=zero*/
        loop = 0;
        do {
            status = PositionerSGammaParametersGet(cntrl->pollsocket,
                                               cntrl->positionername,
                                               &cntrl->velocity,
                                               &cntrl->accel,
                                               &cntrl->minjerktime,
                                               &cntrl->maxjerktime);
            loop++;
        }
        while ((cntrl->accel < 0.00001) && (loop < max_loop));

        if (status != 0) {
            printf(" Error performing PositionerSGammaParametersGet\n");
            statuserror =1;
        }
        /* Is the XPS sending the wrong accel and vel data? */
        if (cntrl->accel <= 0.00001 )
            printf("drvXPSC8 Error PositionerSGammaParametersGet accel=%f vel=%f loop=%i\n",
                   cntrl->accel,cntrl->velocity,loop);

/*        Not implimented due to lack of epics motor commands     
        status = GroupJogParametersGet(cntrl->pollsocket,
                                         cntrl->groupname,
                                         positioner,
                                         &cntrl->jogvelocity,
                                         &cntrl->jogaccel);
        if (status != 0) {
            printf(" Error performing GroupJogParametersGet\n");
            statuserror =1;
        }
*/

        status = GroupPositionCurrentGet(cntrl->pollsocket,
                                         cntrl->groupname,
                                         groupsize,
                                         position); /* Array! */
        if (status != 0) {
            printf(" Error performing GroupPositionCurrentGet\n");
            statuserror =1;
        } else {
        /* Place the position data into the axis structure */
        cntrl->currentposition = position[axisingroup];}


        status = GroupPositionTargetGet(cntrl->pollsocket,
                                        cntrl->groupname,
                                        groupsize,
                                         position); /* Re use Array! */
        if (status != 0) {
            printf(" Error performing GroupPositionTargetGet\n");
            statuserror =1;
        } else {
        /* Place the position data into the axis structure */
        cntrl->targetposition = position[axisingroup];}


        status = GroupPositionSetpointGet(cntrl->pollsocket,
                                          cntrl->groupname,
                                          groupsize,
                                          position); /* Re-re use Array! */
        if (status != 0) {
            printf(" Error performing GroupPositionSetpointGet\n");
            statuserror =1;
        } else {
        /* Place the position data into the axis structure */
        cntrl->setpointposition = position[axisingroup];}

        status = PositionerErrorGet(cntrl->pollsocket,
                                    cntrl->positionername,
                                    &cntrl->positionererror);
        if (status != 0) {
            printf(" Error performing PositionerErrorGet\n");
            statuserror =1;
        }

        /* We probably don't need to poll this because the limits are set to 
         * the database values on startup*/

        status = PositionerUserTravelLimitsGet(cntrl->pollsocket,
                                               cntrl->positionername,
                                               &cntrl->minlimit,
                                               &cntrl->maxlimit);
        if (status != 0) {
            printf(" Error performing PositionerUserTravelLimitsGet\n");
            statuserror =1;
        }

        Debug(10, "readXPSC8Status, socket=%d, groupname=%s, minlim=%f\n",
                    cntrl->socket,cntrl->groupname,cntrl->minlimit);    

        if (status == 1)
            cntrl->status = XPSC8_COMM_ERR; /* This is checked in set_status */
        else
            cntrl->status = OK;
    }
 
    /* Free the lock */
    epicsMutexUnlock(control->XPSC8Lock);
}


/**************************************************************
 * Parse status and position for a card and signal
 * set_status()
 ************************************************************/

STATIC int set_status(int card, int signal)
{
    struct XPSC8controller *control;
    struct XPSC8axis         *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;

    int  positionererror;
    int rtn_state, groupstatus;
    double motorData,pos,target,resolution;
    bool ls_active = false;
    msta_field status;

    control = (struct XPSC8controller *) motor_state[card]->DevicePrivate;
    cntrl = (struct XPSC8axis *)&control->axis[signal];
    
    resolution = cntrl->resolution;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    /* Lock access to global data structure */
    epicsMutexLock(control->XPSC8Lock);
    
    Debug(10, "XPSC8:set_status entry: card=%d, signal=%d\n", card, signal);
    
    /* Parse the error and position values read within the readXPSC8Status 
       function */
    positionererror = cntrl->positionererror;
    pos = cntrl->currentposition;
    target = cntrl->targetposition;
    groupstatus = cntrl->groupstatus;

    /*if (cntrl->velocity >= 0)
        status.Bits.RA_DIRECTION = 1;
    else
        status.Bits.RA_DIRECTION=0;*/

    if (groupstatus > 9 && groupstatus < 20) { 
        /* These states mean ready from move/home/jog etc*/
        status.Bits.RA_DONE=1; /* 1 means cd ready */
        Debug(10, "Set_status: Done -->groupstatus=%d\n", groupstatus);
    } else {
        status.Bits.RA_DONE=0;
        Debug(10, "Set_status: Not Done -->groupstatus=%d\n", groupstatus);
    }
    status.Bits.RA_PLUS_LS=0;
    status.Bits.RA_MINUS_LS=0;
    
    cntrl->moving = 0 ; /* not moving */
    
    if (groupstatus > 42 && groupstatus < 49) {
        /* These states mean it is moving/homeing/jogging etc*/
        cntrl->moving = 1;
        
        if (target >= pos) {
            status.Bits.RA_DIRECTION = 1;
            Debug(2, "Set_status: Positive Direction\n");
        } else {
            status.Bits.RA_DIRECTION = 0;
            Debug(2, "Set_status: Negative Direction\n");
        }

    }
 
    /* These are hard limits */   
    if (positionererror & XPSC8_END_OF_RUN_MINUS ) {
 
        status.Bits.RA_MINUS_LS=1;   /* defined in drvXPSC8.h */
        ls_active = true;
    }

    if (positionererror & XPSC8_END_OF_RUN_PLUS ) {
        status.Bits.RA_PLUS_LS=1;
        ls_active = true;
    }

    status.Bits.RA_HOME=0;
    if (groupstatus == 11) status.Bits.RA_HOME=1; /* ready from home state */
    
    status.Bits.EA_POSITION=0;   /* position maintenence disabled */

    /* encoder status */
    status.Bits.EA_SLIP=0;
    status.Bits.EA_SLIP_STALL=0;
    status.Bits.EA_HOME=0;

    /* Motor position */
    if (resolution == 0.) resolution = 1.;
    motorData = pos / resolution;

    if (motorData == motor_info->position)
        motor_info->no_motion_count++;
    else
    {
        motor_info->position = NINT(motorData);  
        if (motor_state[card]->motor_info[signal].encoder_present == YES)
            motor_info->encoder_position = (int) motorData;
        else
            motor_info->encoder_position = 0;

        motor_info->no_motion_count = 0;
    }

    status.Bits.RA_PROBLEM=0;

        /* not initialized, homed or disabled */
    if ((groupstatus >= 0 && groupstatus < 10) || (groupstatus >= 20 && groupstatus < 43)) { 

        /* Set the Hard limits To show that it is unable to move. This means when you
           home the motor record will only let you home away from the current shown
           limit */
 
        status.Bits.RA_MINUS_LS=1;
        status.Bits.RA_PLUS_LS=1;
        status.Bits.RA_PROBLEM=1;  /*This variable is to do with polling*/
        Debug(1, "--------Set Status Not initialised state!  \n");    
    
    }

   motor_info->velocity = (int)cntrl->velocity; 

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
                 (status.Bits.RA_DONE | status.Bits.RA_PROBLEM)) ? 1 : 0;
 

    if (cntrl->status == XPSC8_COMM_ERR)
        /* I defined this to be -1! */
        status.Bits.CNTRL_COMM_ERR=1;
    else
        status.Bits.CNTRL_COMM_ERR=0;

    /* Free the lock */
    epicsMutexUnlock(control->XPSC8Lock);
    motor_info->status.All = status.All;
    
    Debug(12, "status.Bits.RA_PROBLEM: %i status.Bits.RA_DONE: %i",
            status.Bits.RA_PROBLEM,status.Bits.RA_DONE);
    
    return(rtn_state);
}


/*****************************************************/
/* send a message to the XPS board                  */
/* send_mess()                                       */
/*****************************************************/
STATIC RTN_STATUS send_mess(int card, char const *com, char *c)
{
    struct XPSC8controller *control;
    struct XPSC8axis         *cntrl;
    struct XPSC8group   *groupcntrl;        /*XPS group specific data */
    int groupnumber, groupsize;
    int axisingroup, groupstatus;
    int status;
    int waitcount;
    
    /*******************************************************************************/

    if (motor_state[0] == NULL)
        return(ERROR);
 
    int signal = -1; /* This is incremented up to 0 further down */

    control = (struct XPSC8controller *) motor_state[card]->DevicePrivate;
    epicsMutexLock(control->XPSC8Lock);
    /*control = (struct XPSC8controller *) brdptr->DevicePrivate;*/
    
    /* Loop until you find the axis/group with a move queued up! */ 
    do
    {
        ++signal;
        cntrl = (struct XPSC8axis *)&control->axis[signal];
        groupstatus = cntrl->groupstatus;    /* From XPS controller */
        axisingroup = cntrl->axisingroup;    /* Pull in group info */
        groupnumber = cntrl->groupnumber;
        groupcntrl = (struct XPSC8group *)&control->group[groupnumber];
        groupsize = groupcntrl->groupsize;         /* Number of motors in group */ 
    }
    while ((groupcntrl->queuesize == 0) && (signal < (XPSC8_NUM_CHANNELS-1)));
    
    Debug(5,"Send_mess After do loop axisingp=%d, groupstatus=%i socket=%i queue=%i flag=%i \n",
             axisingroup,groupstatus,cntrl->socket,groupcntrl->queuesize,groupcntrl->queueflag);

    if((signal == (XPSC8_NUM_CHANNELS-1)) && (groupcntrl->queuesize == 0)) 
        goto send_mess_end;                                /*No moves to perform */
    
    status = GroupStatusGet(cntrl->pollsocket, cntrl->groupname,
                                &cntrl->groupstatus); /* Update Status */
    groupstatus = cntrl->groupstatus;

    if (status != 0) 
            printf(" SendMess Error performing GroupStatusGet status=%d\n", status);

    if (groupstatus > 9 && groupstatus < 20){        /* Ready from move */

        if (groupcntrl->queueflag == 1) {               /* First motor called */
            epicsMutexUnlock(control->XPSC8Lock);/* Free up for the other motors*/
            epicsThreadSleep(XPSC8_QUE_PAUSE_READY);        /* Wait for other motors */
            epicsMutexLock(control->XPSC8Lock);
            groupcntrl->queuesize = 0;                /* Reset queue */
            groupcntrl->queueflag = 0;

            status = GroupMoveAbsolute(cntrl->socket,
                                       cntrl->groupname,
                                       groupsize,
                                       groupcntrl->positionarray); /*Pointer to array*/

            if (status != 0 && status != -27) 
                printf(" Error performing GroupMoveAbsolute %i\n",status);
            /* Error -27 is caused when the motor record changes dir i.e.
             * when it aborts a move!*/

             Debug(5,"Send_mess After move in Send_mess axisingp=%d, groupstatus=%i socket=%i psocket=%i\n",
                   cntrl->axisingroup,groupstatus,cntrl->socket,cntrl->pollsocket);
             goto send_mess_end;
         }
    }

    if (groupstatus > 42 && groupstatus < 47){        /* Moveing/Homing*/
        if (groupcntrl->queuesize > 1)
            Debug(1,"Send_mess**Added move to the queue group busy ie xps moving***\n");
        if (groupcntrl->queueflag == 1) {                /* First motor called */
            groupcntrl->queueflag = 0;        
            waitcount = 0;
            while((groupstatus > 42 && groupstatus < 49) && 
                  (waitcount <= XPSC8_MAX_WAIT)){
                epicsMutexUnlock(control->XPSC8Lock);/* Free up for the others*/
                epicsThreadSleep(XPSC8_QUE_PAUSE_MOVING);        /* Wait for end of motion */
                epicsMutexLock(control->XPSC8Lock);
                ++waitcount;
                status = GroupStatusGet(cntrl->pollsocket, cntrl->groupname,
                                        &cntrl->groupstatus);
                if (status != 0) 
                    printf("Error performing GroupStatusGet Bottom Send_mess status=%d\n", status); 

                groupstatus = cntrl->groupstatus;        /* Update groupstatus */

            }
   
            if (waitcount >= XPSC8_MAX_WAIT) {
                printf(" Error Motor Queue timed out \n");
                groupcntrl->queuesize = 0;     /* Reset queue */
                groupcntrl->queueflag = 0;
                /* Reset the queue array to current positions */
                status = GroupPositionCurrentGet(cntrl->pollsocket,
                                                 cntrl->groupname,
                                                 groupsize, 
                                                 groupcntrl->positionarray); 
                if (status != 0)
                    printf(" Error performing GroupPositionCurrentGet\n");
                goto send_mess_end;
            }

            if (groupstatus > 9 && groupstatus < 20) {    /* Ready from move */
                groupcntrl->queuesize = 0;                /* Reset queue */
                groupcntrl->queueflag = 0;
                status = GroupMoveAbsolute(cntrl->socket, 
                                           cntrl->groupname,
                                           groupsize, 
                                           groupcntrl->positionarray); /*Pointer to array*/

                /* Error -27 is caused when the motor record changes dir i.e.
                 *  when it aborts a move!*/
                if (status != 0 && status != -27 ) 
                    printf(" Error performing GroupMoveAbsolute %i\n",status);
  
            }
        }
    }

send_mess_end:
    epicsMutexUnlock(control->XPSC8Lock);
    return (OK);
}



/*****************************************************/
/* receive a message from the XPS board           */
/* recv_mess()                                       */
/*****************************************************/
STATIC int recv_mess(int card, char *com, int flag)
{
    /* This is a no-op for the XPS, but must be present */
    /* Set the returned message to a null string */
    *com = '\0';
    return (0);
}



/*****************************************************/
/* Setup system configuration                        */
/* XPSC8Setup()                                      */
/*****************************************************/
RTN_STATUS XPSC8Setup(int num_cards,   /* number of controllers in system.  */
                      int scan_rate)   /* Poll rate */
{
    Debug(10, "XPSC8Setup: Controllers=%d Scan Rate=%d\n",num_cards,scan_rate);
    int itera;
    if (num_cards > XPSC8_MAX_CONTROLLERS) 
        printf(" Error in setup too many controllers\n"); 

    if (num_cards < 1)
        XPSC8_num_cards = 1;
    else
        XPSC8_num_cards = num_cards;

    /*  Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
        targs.motor_scan_rate = scan_rate;
    else
        targs.motor_scan_rate = SCAN_RATE;

   /* Allocate space for pointers to motor_state structures. */
    motor_state = (struct controller **) malloc(XPSC8_num_cards *
                                                sizeof(struct controller *));

    for (itera = 0; itera < XPSC8_num_cards; itera++)
        motor_state[itera] = (struct controller *) NULL;

    return (OK);
}



/*****************************************************/
/* Configure a Newport XPS controller one command   */
/* For each controller                              */
/* XPSC8Config()                                    */
/*****************************************************/
RTN_STATUS XPSC8Config(int card,   /* Controller number */
                       const char *ip,  /* XPS IP address*/
                       int port,       /* This may be 5001 */
                       int totalaxes)  /* Number of axis/positioners used*/

{
    struct XPSC8controller *control;
    struct XPSC8axis         *cntrl;
    int statuserror, status, socket, pollsocket, devpollsocket, axis;
    char *ipchar, List[1000];
    statuserror = 0;
    status = 0;
    ipchar = (char *)ip;
    socket = 0;
    pollsocket = 0;
    devpollsocket = 0;
    
    Debug(10, "XPSC8Config: IP=%s, Port=%d, Card=%d, totalaxes=%d\n",
          ipchar, port, card, totalaxes);
    
    if (totalaxes < 0 || totalaxes > XPSC8_NUM_CHANNELS) {return (ERROR);}

    motor_state[card] = (struct controller *) calloc(1, 
                                                     sizeof(struct controller));
    motor_state[card]->DevicePrivate = calloc(1, 
                                              sizeof(struct XPSC8controller));
    control = (struct XPSC8controller *) motor_state[card]->DevicePrivate;
    
    motor_state[card]->total_axis = totalaxes;
      
    pollsocket = TCP_ConnectToServer(ipchar,port,TIMEOUT);
    
    if (pollsocket < 0) {
            printf(" Error TCP_ConnectToServer for pollsocket\n");
            statuserror =1;
        }

    devpollsocket = TCP_ConnectToServer(ipchar,port,TIMEOUT);
    
    if (devpollsocket < 0) {
            printf(" Error TCP_ConnectToServer for pollsocket\n");
            statuserror =1;
        }

    
    for (axis=0; axis<totalaxes; axis++) { 
        socket = TCP_ConnectToServer(ipchar,port,TIMEOUT);
        /* Find a socket number */
        if (socket < 0) {
            printf(" Error TCP_ConnectToServer for move-socket\n");
            statuserror =1;
        }
 
        cntrl = (struct XPSC8axis *)&control->axis[axis];
        Debug(11, "XPSC8Config: axis=%d, cntrl=%p\n", axis, cntrl);

        cntrl->pollsocket = pollsocket;
        cntrl->devpollsocket = devpollsocket;
        cntrl->socket = socket;
        cntrl->ip = epicsStrDup(ip);

    
        Debug(10, "XPSC8Config: Socket=%d, PollSock=%d, ip=%s, port=%d,"
              " axis=%d controller=%d\n",
              cntrl->socket,cntrl->pollsocket,ip,port,axis,card);        
    }
    Debug(11, "XPSC8Config: Above OjectsListGet\n");        
    status = ObjectsListGet(socket, List);
    if (status != 0)
        printf(" Error performing Object List Get\n");
    else 
        printf("Start of Object List From Controller: %.40s\n",List);   
    
    Debug(11, "XPSC8Config: End\n");    
    return (OK);
}


/*********************************************************/
/*    Pass the group and positioner names                */
/*     Call this for each axis                           */
/*                                                       */
/*********************************************************/
RTN_STATUS XPSC8NameConfig(int card,            /* specify which controller 0-up*/
                           int axis,            /* axis number 0-7*/
                           int groupnumber,     /* 0-7*/
                           int groupsize,       /* 1-8*/
                           int axisingroup,     /* eg 4th axis in group 2 */
                           const char *gpname,  /* group name e.g. Diffractometer */
                           const char *posname) /* positioner name e.g. Diffractometer.Phi*/

{
    struct XPSC8controller *control;
    struct XPSC8axis         *cntrl;
    struct XPSC8group   *groupcntrl;        /*XPS group specific data */  

    Debug(10, "XPSC8NameConfig: card=%d axis=%d, group=%s, positioner=%s\n",
          card, axis, gpname, posname);
 
    control = (struct XPSC8controller *) motor_state[card]->DevicePrivate;
    cntrl = (struct XPSC8axis *)&control->axis[axis];
    groupcntrl = (struct XPSC8group *)&control->group[groupnumber];
    
    cntrl->groupname = epicsStrDup(gpname);
    cntrl->positionername = epicsStrDup(posname);
    cntrl->groupnumber = groupnumber;
    cntrl->axisingroup = axisingroup;
    groupcntrl->groupsize = groupsize;

    return (OK);
}


/*****************************************************/
/* initialize all software and hardware              */
/* This is called from the init()                    */
/* motor_init()                                      */
/*****************************************************/
STATIC int motor_init()
{

    struct controller *brdptr;
    struct XPSC8controller *control;
    struct XPSC8axis         *cntrl;
    struct XPSC8group   *groupcntrl;        /*XPS group specific data */
    int card_index, motor_index;
    int status = 0, totalaxes;
    int i,counter;                /* Used in for loops */
    bool errind;

    initialized = true;   /* Indicate that driver is initialized. */

    /* Check for setup */
    Debug(10, "XPSC8:motor_init: num_cards=%d\n", XPSC8_num_cards);
    if (XPSC8_num_cards <= 0){
        return (ERROR);
    }

    for (card_index = 0; card_index < XPSC8_num_cards; card_index++) {
        totalaxes = motor_state[card_index]->total_axis;
        Debug(10, "XPSC8:motor_init: Card init loop card_index=%d\n",card_index);
        brdptr = motor_state[card_index];
        total_cards = card_index + 1;
        Debug(10, "XPSC8:motor_init: Above control def card_index=%d\n",
              card_index);
        control = (struct XPSC8controller *) brdptr->DevicePrivate;

        cntrl = (struct XPSC8axis *)&control->axis[0]; 
        /* Just for the test below!*/

        control->XPSC8Lock = epicsMutexCreate();
        errind = false;

        /* Just to test status, 0 means the call worked */
        counter = 0;
        /* wait incase the socket is busy */
        while (status < 0 && counter < (TIMEOUT*10)) {
            status = GroupStatusGet(cntrl->pollsocket,cntrl->groupname,
                                    &cntrl->groupstatus);
            counter++;
            epicsThreadSleep(0.1);
        }

        if (status !=0) errind = true;
        Debug(10, "XPSC8:motor_init: card_index=%d, errind=%d\n",
              card_index, errind);
        Debug(10, "XPSC8:motor_init: psocket %i, gpname %s, gpstatus %i status %i\n",
              cntrl->pollsocket,cntrl->groupname,cntrl->groupstatus,status);
      
        /* Set the XPSC8group struct variables to zero */

        for (i=0 ; i< XPSC8_NUM_CHANNELS ; ++i) {
            groupcntrl = (struct XPSC8group *)&control->group[i];
            groupcntrl->queuesize = 0;
            groupcntrl->queueflag = 0;
        }
    

        if (errind == false) {
            brdptr->localaddr = (char *) NULL;
            brdptr->motor_in_motion = 0;

            /*brdptr->total_axis = XPSC8_NUM_CHANNELS;*/
            /* I have set this to be the no. used inConfig*/

            for (motor_index = 0; motor_index<totalaxes; motor_index++) {
                brdptr->motor_info[motor_index].motor_motion = NULL;
            }

            strcpy(brdptr->ident, "XPS_C8_controller");

            start_status(card_index);
            /*Read in all the parameters vel/accel ect */

            for (motor_index = 0; motor_index < totalaxes; motor_index++) {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];
                motor_info->status.All = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;
                /* Read status of each motor */
            }
        }
        else
            motor_state[card_index] = (struct controller *) NULL;
    }

    any_motor_in_motion = 0;

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;
    
    epicsThreadCreate((char *) "XPSC8_motor", epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium), 
                      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return (OK);
}

