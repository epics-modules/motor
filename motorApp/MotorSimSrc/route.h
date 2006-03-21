#ifndef __INCrouteLibh
#define __INCrouteLibh

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_AXES 3

typedef enum
{
    ROUTE_CALC_ROUTE = 0,
    ROUTE_NEW_ROUTE  = 1,
    ROUTE_NO_NEW_ROUTE = 2
} route_reroute_t;

typedef enum
{
    ROUTE__OK = 0,
    ROUTE__BADROUTE = 1,
    ROUTE__BADPARAM = 2,
    ROUTE__NEGSQRT  = 3,
    ROUTE__NEGTIME  = 4
} route_status_t;

typedef struct route_axis_demand_str
{
    double p;                         /* Demand position for axis at a given time */ 
    double v;                         /* Demand velocity for axis at a given time */
} route_axis_demand_t;

typedef struct route_demand_str
{
    double T;                         /* Time at which demand is valid            */
    route_axis_demand_t axis[NUM_AXES];
} route_demand_t;

typedef struct route_axis_pars_str
{
    double Amax;                      /* Maximum acceleration for this axis       */
    double Vmax;                      /* Maximum velocity for this axis           */
} route_axis_pars_t;

typedef struct route_pars_str
{
    unsigned int numRoutedAxes;       /* Number of axes to be routed              */
    int routedAxisList[NUM_AXES];     /* List of the axes to be routed            */
    double Tsync;                     /* Synchronisation period for routing       */
    double Tcoast;                    /* End of route coast time for all axes     */
    route_axis_pars_t axis[NUM_AXES];
} route_pars_t;

typedef struct route_str * ROUTE_ID;

ROUTE_ID routeNew( route_demand_t * initialDemand, route_pars_t * initial_parameters );
route_status_t routeFind( ROUTE_ID, route_reroute_t, route_demand_t * end_demand, route_demand_t * next_demand );
void routePrint( ROUTE_ID route, route_reroute_t reroute, route_demand_t * endp, route_demand_t * nextp, FILE * logfile );
void routeDelete( ROUTE_ID );

route_status_t routeSetDemand( ROUTE_ID, route_demand_t * demand );
route_status_t routeSetParams( ROUTE_ID, route_pars_t * parameters );
route_status_t routeGetParams( ROUTE_ID, route_pars_t * parameters );
route_status_t routeGetNumRoutedAxes( ROUTE_ID route, unsigned int * number );

#ifdef __cplusplus
}
#endif
 
#endif /* __INCrouteLibh */
