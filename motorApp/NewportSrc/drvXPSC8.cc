/*
 *      drvXPSC8.cc
 *      Motor record driver level support for Newport XPSC8 motor controller.
 *
 *      Original Author: Mark Rivers
 *      Date: 10-May-2000
 *
 * Modification Log:
 * -----------------
 
 */

#include        <string.h>
#include        <stdio.h>
#include        <epicsThread.h>
#include        <epicsMutex.h>
#include        <epicsExport.h>
#include        <epicsString.h>
#include        <drvSup.h>
#include        "motor.h"
#include        "drvXPSC8.h"

#include        "xps_c8_driver.h"

#define STATIC static

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	volatile int drvXPSC8Debug = 0;
	#define Debug(L, FMT, V...) { if(L <= drvXPSC8Debug) \
                        { printf("%s(%d):",__FILE__,__LINE__); \
                          printf(FMT,##V); } }
	epicsExportAddress(int, drvXPSC8Debug);
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif


/* --- Local data. --- */
int XPSC8_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"

/* Why are not all the functions declared here? */

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

epicsExportAddress(drvet, drvXPSC8);

/* I don't know where the XPSC8_access is set?*/
STATIC struct thread_args targs = {SCAN_RATE, &XPSC8_access};

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
        Debug(1, "init(): XPSC8 driver disabled. XPSC8etup() missing from startup script.\n");
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

    Debug(2, "start_status: card=%d  total_cards=%d\n", card,total_cards);
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
    int positioner,i; /* this = 1 because we are talking to 1 axis at a time */

    struct mess_node *nodeptr; /* These are to print the DONE flag */
    register struct mess_info *motor_info;
    msta_field statusflags;

    positioner = 1; 
    statuserror = 0; /* this = 1 if an error occurs */
    XPSC8_num_axes = motor_state[card]->total_axis;

    Debug(2, "XPSC8:readXPSC8Status card=%d num_axes=%d\n",card,XPSC8_num_axes);
    
    /* Take a lock so that only 1 thread can be talking to the Neport XPSC8 
    */
    epicsMutexLock(control->XPSC8Lock);

    for (i=0; i<XPSC8_num_axes;i++) {

        motor_info = &(motor_state[card]->motor_info[i]); /*To print DONE flag*/
        nodeptr = motor_info->motor_motion;
        statusflags.All = motor_info->status.All;

        Debug(9, "XPSC8:readXPSC8Status RA_DONE=%d, RA_MOVING=%d, "\
                 "RA_PROBLEM=%d\n",\
              statusflags.Bits.RA_DONE, statusflags.Bits.RA_MOVING,\
              statusflags.Bits.RA_PROBLEM);

        control = (struct XPSC8controller *) motor_state[card]->DevicePrivate;
        cntrl = (struct XPSC8axis *)&control->axis[i];

        Debug(2, "XPSC8:readXPSC8Status card=%d axis=%d sock=%d gp=%s\n",card,i,\
              cntrl->socket,cntrl->groupname);
    
        /* Where I have used "&" the func requires an pointer */
        status = GroupStatusGet(cntrl->pollsocket, cntrl->groupname,
                                &cntrl->groupstatus);
        if (status != 0) {
            printf(" Error performing GroupStatusGet status=%d\n", status); 
            statuserror =1;
        }

        status = PositionerSGammaParametersGet(cntrl->pollsocket,
                                               cntrl->positionername,
                                               &cntrl->velocity,
                                               &cntrl->accel,
                                               &cntrl->minjerktime,
                                               &cntrl->maxjerktime);
        if (status != 0) {
            printf(" Error performing PositionerSGammaParametersGet\n");
            statuserror =1;
        }

        /* The jog function is not enabled*/
/*        
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
                                         cntrl->positionername,
                                         positioner,
                                         &cntrl->currentposition[1]);
        if (status != 0) {
            printf(" Error performing GroupPositionCurrentGet\n");
            statuserror =1;
        }


        status = GroupPositionTargetGet(cntrl->pollsocket,
                                        cntrl->positionername,
                                        positioner,
                                        &cntrl->targetposition[1]);
        if (status != 0) {
            printf(" Error performing GroupPositionTargetGet\n");
            statuserror =1;
        }


        status = GroupPositionSetpointGet(cntrl->pollsocket,
                                          cntrl->positionername,
                                          positioner,
                                          &cntrl->setpointposition[1]);
        if (status != 0) {
            printf(" Error performing GroupPositionSetpointGet\n");
            statuserror =1;
        }

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

        Debug(11, "readXPSC8Status, socket=%d, groupname=%s, minlim=%f\n",\
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
    double motorData, pos,resolution;
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

    Debug(2, "XPSC8:set_status entry: card=%d, signal=%d\n", card, signal);
    
    /* Parse the error and position values read within the readXPSC8Status 
       function */
    positionererror = cntrl->positionererror;
    pos = cntrl->currentposition[1];
    groupstatus = cntrl->groupstatus;
    Debug(2, "XPSC8:set_status entry: positionererror=%d, pos=%f,"\
          " resolution=%f\n",\
          positionererror, pos, resolution);
    Debug(11, "XPSC8:set_status entry: pos0=%f, pos1=%f\n",\
          cntrl->currentposition[0], cntrl->currentposition[1]);

    if (cntrl->velocity >= 0)
        status.Bits.RA_DIRECTION = 1;
    else
        status.Bits.RA_DIRECTION=0;

    if (groupstatus > 9 && groupstatus < 20) { 
        /* These states mean ready from move/home/jog etc*/
        status.Bits.RA_DONE=1; /* 1 means cd ready */
        Debug(2, "Set_status: Done -->groupstatus=%d\n", groupstatus);
    } else {
        status.Bits.RA_DONE=0;
        Debug(2, "Set_status: Not Done -->groupstatus=%d\n", groupstatus);
    }
    status.Bits.RA_PLUS_LS=0;
    status.Bits.RA_MINUS_LS=0;
    
    cntrl->moving = 0 ; /* not moving */
    
    if (groupstatus > 42 && groupstatus < 49) {
        /* These states mean it is moving/homeing/jogging etc*/
        cntrl->moving = 1;
    }
 
    /* These are hard limits */   
    if (positionererror & XPSC8_END_OF_RUN_MINUS ) {
        /* I am unsure of the use of & !!*/
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
            motor_info->encoder_position = (int32_t) motorData;
        else
            motor_info->encoder_position = 0;

        motor_info->no_motion_count = 0;
    }

    status.Bits.RA_PROBLEM=0;

    Debug(11, "--------above inisialisation test  \n");   
    if (groupstatus < 9 || (groupstatus > 19 && groupstatus < 43)) { 
        /* not initialized or disabled*/
        /* Set the Hard limits To show that it is unable to move */
        status.Bits.RA_MINUS_LS=1;
        status.Bits.RA_PLUS_LS=1;
        status.Bits.RA_PROBLEM=1;  /*This variable is to do with polling*/
        Debug(1, "--------Set Status Not initialised state!  \n");    
    
    }

   motor_info->velocity = (int)cntrl->velocity; 
   /* I don't know what this is used for */

/*    if (status.Bits.RA_DIRECTION==0)
        motor_info->velocity *= -1;*/

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
                 (status.Bits.RA_DONE | status.Bits.RA_PROBLEM)) ? 1 : 0;
 
    Debug(1, "--------set_status rtn_state=%d  \n",rtn_state);   


    if (cntrl->status == XPSC8_COMM_ERR)
        /* I defined this to be -1! */
        status.Bits.CNTRL_COMM_ERR=1;
    else
        status.Bits.CNTRL_COMM_ERR=0;

    /* Free the lock */
    epicsMutexUnlock(control->XPSC8Lock);
    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send a message to the XPS board                  */
/* send_mess()                                       */
/*****************************************************/
STATIC RTN_STATUS send_mess(int card, char const *com, char *name)
{
    /* This is a no-op for the XPS, but must be present */
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
                      int scan_rate)   /* I think this is for the epicsthread */
{
    Debug(1, "XPSC8Setup: Controllers=%d Scan Rate=%d\n",num_cards,scan_rate);
    int itera;
    if (num_cards > XPSC8_NUM_CHANNELS) 
        printf(" Error in setup too many channels\n"); 

    if (num_cards < 1)
        XPSC8_num_cards = 1;
    else
        XPSC8_num_cards = num_cards;

/*     Set motor polling task rate */
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
                       const char *ip,        /* XPS IP address*/
            int port,              /* This may be 5001 */
            int totalaxes)         /* Number of axis/positioners used*/

{
    struct XPSC8controller *control;
    struct XPSC8axis         *cntrl;
    int statuserror, status, socket, pollsocket, axis;
    char *ipchar, List[1000];
    statuserror = 0;
    status = 0;
    ipchar = (char *)ip;
    socket = 0;
    pollsocket = 0;
    
    
    Debug(1, "XPSC8Config: IP=%s, Port=%d, Card=%d, totalaxes=%d\n",\
          ipchar, port, card, totalaxes);
    
    if (totalaxes < 0 || totalaxes > XPSC8_NUM_CHANNELS) {return (ERROR);}

    motor_state[card] = (struct controller *) calloc(1, 
                                                     sizeof(struct controller));
    motor_state[card]->DevicePrivate = calloc(1, 
                                              sizeof(struct XPSC8controller));
    control = (struct XPSC8controller *) motor_state[card]->DevicePrivate;
    
    motor_state[card]->total_axis = totalaxes;
    
    
       
    for (axis=0; axis<totalaxes; axis++) { 
        socket = TCP_ConnectToServer(ipchar,port,TIMEOUT);
        /* Find a socket number */
        if (socket < 0) {
            printf(" Error TCP_ConnectToServer for move-socket\n");
            statuserror =1;
        }

        pollsocket = TCP_ConnectToServer(ipchar,port,TIMEOUT);
        /* Find a socket number */
        if (pollsocket < 0) {
            printf(" Error TCP_ConnectToServer for pollsocket\n");
            statuserror =1;
        } 
 
        cntrl = (struct XPSC8axis *)&control->axis[axis];
        Debug(11, "XPSC8Config: axis=%d, cntrl=%p\n", axis, cntrl);

        cntrl->pollsocket = pollsocket;
        cntrl->socket = socket;
        cntrl->ip = epicsStrDup(ip);

    
        Debug(1, "XPSC8Config: Socket=%d, PollSock=%d, ip=%s, port=%d,"\
              " axis=%d controller=%d\n",\
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
RTN_STATUS XPSC8NameConfig(int card,      /*specify which controller 0-up*/
                           int axis,      /*axis number 0-7*/
                           const char *gpname, /*group name e.g. Diffractometer */
                           const char *posname) /*positioner name e.g. Diffractometer.Phi*/

{
    struct XPSC8controller *control;
    struct XPSC8axis         *cntrl;

    Debug(1, "XPSC8NameConfig: card=%d axis=%d, group=%s, positioner=%s\n",\
          card, axis, gpname, posname);
 
    control = (struct XPSC8controller *) motor_state[card]->DevicePrivate;
    cntrl = (struct XPSC8axis *)&control->axis[axis];

    cntrl->groupname = epicsStrDup(gpname);
    cntrl->positionername = epicsStrDup(posname);

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
    int card_index, motor_index;
    int status, totalaxes;

    bool errind;

    initialized = true;   /* Indicate that driver is initialized. */

    /* Check for setup */
    Debug(1, "XPSC8:motor_init: num_cards=%d\n", XPSC8_num_cards);
    if (XPSC8_num_cards <= 0){
        printf("Error num_cards <=0");
        return (ERROR);
    }

    for (card_index = 0; card_index < XPSC8_num_cards; card_index++) {
        totalaxes = motor_state[card_index]->total_axis;
        Debug(5, "XPSC8:motor_init: Card init loop card_index=%d\n",card_index);
        brdptr = motor_state[card_index];
        total_cards = card_index + 1;
        Debug(5, "XPSC8:motor_init: Above control def card_index=%d\n",\
              card_index);
        control = (struct XPSC8controller *) brdptr->DevicePrivate;

        cntrl = (struct XPSC8axis *)&control->axis[0]; 
        /* Just for the test below!*/

        control->XPSC8Lock = epicsMutexCreate();
        errind = false;

        /* just to test status, 0 means the call worked*/
        status = GroupStatusGet(cntrl->socket,cntrl->groupname,
                                &cntrl->groupstatus); 
        if (status !=0) errind = true;
        Debug(5, "XPSC8:motor_init: card_index=%d, errind=%d\n",\
              card_index, errind);

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

            Debug(5, "XPSC8:motor_init: called start_status OK\n");
            for (motor_index = 0; motor_index < totalaxes; motor_index++) {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];
                motor_info->status.All = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;
                /* Read status of each motor */
                Debug(5, " XPSC8:motor_init: calling set_status for motor %d\n",\
                      motor_index);
                set_status(card_index, motor_index);
            }
        }
        else
            motor_state[card_index] = (struct controller *) NULL;
    }
    Debug(5, "XPSC8:motor_init: done with start_status and set_status\n");

    any_motor_in_motion = 0;

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;

    Debug(5, "XPSC8:motor_init: spawning XPSC8_motor task\n");
    
    epicsThreadCreate((char *) "XPSC8_motor", 64, 5000, 
                      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return (OK);
}
