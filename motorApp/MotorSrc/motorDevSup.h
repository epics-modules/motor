/*****************************************************************************
  Calls to device support
  Wrappers that call device support.
*****************************************************************************/
#ifndef INC_motorDevSup_H
#define INC_motorDevSup_H

#include    <epicsStdio.h>
#include    <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif
/*******************************************************************************
Support for tracking the progress of motor from one invocation of 'process()'
to the next.  The field 'pmr->mip' stores the motion in progress using these
fields.  ('pmr' is a pointer to motorRecord.)
*******************************************************************************/
#define MIP_DONE        0x0000  /* No motion is in progress. */
#define MIP_JOGF        0x0001  /* A jog-forward command is in progress. */
#define MIP_JOGR        0x0002  /* A jog-reverse command is in progress. */
#define MIP_JOG_BL1     0x0004  /* Done jogging; 1st phase take out backlash. */
#define MIP_JOG         (MIP_JOGF | MIP_JOGR | MIP_JOG_BL1 | MIP_JOG_BL2)
#define MIP_HOMF        0x0008  /* A home-forward command is in progress. */
#define MIP_HOMR        0x0010  /* A home-reverse command is in progress. */
#define MIP_HOME        (MIP_HOMF | MIP_HOMR)
#define MIP_MOVE        0x0020  /* A move not resulting from Jog* or Hom*. */
#define MIP_RETRY       0x0040  /* A retry is in progress. */
#define MIP_LOAD_P      0x0080  /* A load-position command is in progress. */
#define MIP_MOVE_BL     0x0100  /* Done moving; now take out backlash. */
#define MIP_STOP        0x0200  /* We're trying to stop.  When combined with */
/*                                 MIP_JOG* or MIP_HOM*, the jog or home     */
/*                                 command is performed after motor stops    */
#define MIP_DELAY_REQ   0x0400  /* We set the delay watchdog */
#define MIP_DELAY_ACK   0x0800  /* Delay watchdog is calling us back */
#define MIP_DELAY       (MIP_DELAY_REQ | MIP_DELAY_ACK) /* Waiting for readback
                                                         * to settle */
#define MIP_JOG_REQ     0x1000  /* Jog Request. */
#define MIP_JOG_STOP    0x2000  /* Stop jogging. */
#define MIP_JOG_BL2     0x4000  /* 2nd phase take out backlash. */
#define MIP_EXTERNAL    0x8000  /* Move started by external source */


  double devSupDialToRaw(motorRecord *pmr, double dialValue);
  void devSupStop(motorRecord *pmr);
  void devSupLoadPos(motorRecord *pmr, double newpos);
  RTN_STATUS devSupGetInfo(motorRecord *pmr);
  RTN_STATUS devSupUpdateLimitFromDial(motorRecord *pmr, motor_cmnd command,
                                       double dialValue);
  void devSupMoveAbsRaw(motorRecord *pmr, double vel, double vbase,
                        double acc, double pos);
  void devSupMoveRelRaw(motorRecord *pmr, double vel, double vbase,
                        double acc, double relpos);
  void devSupMoveDialEgu(motorRecord *, double, double, double, int);
  void devSupJogDial(motorRecord *pmr, double jogv, double jacc);
  void devSupUpdateJogDial(motorRecord *pmr, double jogv, double jacc);
  void devSupCNEN(motorRecord *pmr, double cnen);
  int  devSupSetPID(motorRecord *pmr, motor_cmnd command, double *pcoeff);
  void devSupSetEncRatio(motorRecord *pmr, double ep_mp[2]);
  void setCDIRfromRawMove(motorRecord *pmr, int directionRaw);
  void setCDIRfromDialMove(motorRecord *pmr, int directionDial);
  void doHomeSetcdir(motorRecord *pmr);

  static inline const char *mrStripPath(const char *file)
  {
    const char *ret = strrchr(file, '/');
    if (ret) return ret + 1;
#if (defined(CYGWIN32) || defined(_WIN32))
    ret = strrchr(file, '\\');
    if (ret) return ret + 1;
#endif
    return file;
  }
  static inline void mrPrint(motorRecord*,unsigned,const char *, ...) EPICS_PRINTF_STYLE(3,4);
  static inline void mrPrint(motorRecord *mr, unsigned lvl, const char *format, ...)
  {
    va_list pVar;
    va_start(pVar, format);
    vfprintf(stdout, format, pVar);
    va_end(pVar);
}

#define Debug(pmr, lvl, fmt, ...)                         \
{                                                         \
    if ((1<<lvl) & pmr->spam) {                           \
       epicsTimeStamp now;                                \
       char nowText[25];                                  \
       size_t rtn;                                        \
                                                          \
       nowText[0] = 0;                                    \
       rtn = epicsTimeGetCurrent(&now);                   \
       if (!rtn) {                                        \
         epicsTimeToStrftime(nowText,sizeof(nowText),     \
                          "%Y/%m/%d %H:%M:%S.%03f ",&now);\
       }                                                  \
       mrPrint(pmr, lvl, "%s[%s:%-4d %s] " fmt,           \
               nowText,                                   \
               mrStripPath(__FILE__), __LINE__,           \
               pmr->name,  __VA_ARGS__);                  \
    }                                                     \
}

#ifdef __cplusplus
}
#endif
#endif /* INC_motorDevSup_H */
