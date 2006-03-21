#include <math.h>
#include <float.h>
#include <stdio.h>     /* For definition of the NULL pointer! */
#include <stdlib.h>    /* For definition of malloc            */
#include <route.h>

#define LOCAL static

typedef struct path_str
{
    double dist;
    double vi;
    double vf;
    double v2;
    double t1;
    double t2;
    double t3;
    double t4;
    double T;
} path_t;

typedef struct route_str
{
    route_pars_t   pars;
    route_demand_t demand;
    path_t path[NUM_AXES];
    route_demand_t endp;
} route_t;

typedef enum
{
    V2 = 1,
    T  = 2,
    T2 = 4,
    T4 = 8
} route_unknown_t;

#define ZERO_SIZE  2
#define IS_ZERO(a, scale) (fabs((a)) <= fabs(ZERO_SIZE*DBL_EPSILON*(scale)))
#define DR2D (180.0/3.141592654)


/*+                      r o u t e D e m a n d
  
   Function Name: routeDemand
  
   Function: Returns the position and velocity for a particular time on a path.
 
   Description:
      This function returns the position and velocity at a given time of something
      following the specified path.
 
   Call:
      status = routeDemand( &path_t, t, &posn, &vel )
  
   Parameters:
      ("<" input, "!" modified, "W" workspace, ">" output)

      (<) path     (path_t *)  Path structure. 
      (<) t        (double)    Time at which to find the position and velocity
      (>) demand   (route_axis_demand_t *)  Position and velocit at time t.

   Returns:
      status (route_status_t)  Always ROUTE__OK

   Author: Nick Rees

*-

   History:
*/

LOCAL route_status_t routeDemand(path_t * path, double t, route_axis_demand_t * demand )
{
    double accel1, accel2;


    if (path->t1 != 0) accel1 = (path->v2 - path->vi) / path->t1;
    else accel1 = 0;
    if (path->t3 != 0) accel2 = (path->vf - path->v2) / path->t3;
    else accel2 = 0;


    if (t >= -path->t4)
    {
        demand->v = path->vf;
        demand->p = path->vf * t;
    }
    else
    {
        t = t + path->t4;
        demand->p = -path->vf * path->t4;

        if (t >= -path->t3)
        {
            demand->v = path->vf + accel2 * t;
            demand->p += 0.5 * (demand->v + path->vf) * t;
        }
        else
        {
            t = t + path->t3;
            demand->p -= 0.5 * (path->v2 + path->vf) * path->t3;
        
            if (t >= -path->t2)
            {
                demand->v = path->v2;
                demand->p += path->v2 * t;
            }
            else
            {
                t = t + path->t2;
                demand->p -= path->v2 * path->t2;

                if (t >= -path->t1)
                {
                    demand->v = path->v2 + accel1 * t;
                    demand->p += 0.5 * (demand->v + path->v2) * t;
                }
                else
                {
                    demand->v = path->vi;
                    demand->p += 0.5 * (path->vi + path->v2) * path->t2 + (t+path->t1)*path->vi;
                }
            }
        }
    }

/*                
            
    if (t <= path->t1)
    {
        demand->v = path->vi + accel1 * t;
        demand->p = 0.5 * (path->vi + demand->v) * t;
    }
    else if (t <= (path->t1 + path->t2))
    {
        demand->v = path->v2;
        demand->p = 0.5 * (path->vi + path->v2) * path->t1 + demand->v * (t - path->t1);
    }
    else if (t <= (path->t1 + path->t2 + path->t3))
    {
        demand->v = path->v2 + accel2 * (t - (path->t1 + path->t2));
        demand->p = (0.5 * (path->vi + path->v2) * path->t1 + path->v2 * path->t2 +
                     0.5 * (path->v2 + demand->v) * (t - (path->t1 + path->t2)));
    }
    else
    {
        demand->v = path->vf;
        demand->p = path->dist + path->vf * (t - path->T);
    }
*/
/*    else if (t <= (path->T - path->t4))
    {
        demand->v = path->vf - accel2 * (path->T - path->t4 - t);
        demand->p = (path->dist - path->vf*path->t4 - 
                     0.5 * (path->vf + demand->v) * (path->T - path->t4 - t));
    }
    else
    {
        demand->v = path->vf;
        demand->p = path->dist + path->vf * (t - path->T);
    }
*/
    return ROUTE__OK;
}


/*+                      r o u t e F i n d W h i c h V 2 S q r t
 
   Function Name: routeFindWhichV2Sqrt

   Function: Returns the correct quadratic term in the path problem
  
   Description:
      This function returns the correct solution to the second phase coast velocity
      quadratic, given the linear and square root terms.

   Call:
      status = routeFindWhichV2Sqrt( &path, Ai, lin_term, sqrt_term );
  
   Parameters:
      ("<" input, "!" modified, "W" workspace, ">" output)

      (!) path      (path_t *)          Path structure.
      (<) Ai        (double)            Phase 1 cceleration.
      (<) lin_term  (double)            Linear term of quadratic solution
      (<) sqrt_term (route_unknown_t)   Square root term of quadratic solution.
  
   Returns:
      status (route_status_t)  ROUTE__OK or ROUTE__NEGSQRT
  
   Author: Nick Rees
*-
   History:
*/

LOCAL route_status_t routeFindWhichV2Sqrt( path_t * path, double Ai,
                                            double lin_term, double sqrt_term, int unknown )
{
    route_status_t status = ROUTE__OK;

/*    if (sqrt_term < -1e-10)
    {
        status = ROUTE__NEGSQRT;
    }
    else
*/    {
        if (sqrt_term < 0) sqrt_term = 0;

        sqrt_term = sqrt( sqrt_term );

        if (unknown == T2 )
        {
            if ( Ai > 0 ) path->v2 =  lin_term - sqrt_term;
            else path->v2 =  lin_term + sqrt_term;
        }
        else
        {
            if ( Ai > 0 ) path->v2 =  lin_term + sqrt_term;
            else path->v2 =  lin_term - sqrt_term;
        }

    }

    return status;
}


/*+                      r o u t e F i n d P a t h
 
   Function Name: routeFindPath

   Function: Solves the general 4-Phase path problem given a variety of unknowns.
  
   Description:
  
      This function finds a path from a given position and velocity to a final
      position and velocity via a four stage motion.
  
       - The first stage is a constant acceleration at +/- Amax, a fixed maximum
         acceleration.
       - The second stage is a coast at a constant velocity.
       - The third stage is a decelleration to the final velocity at the maximum
         acceleration.
       - The final stage is a coast at the final velocity to the final position.
  
      This is equivalent to the solving the following four equations:
  
      distance = 0.5*(vf+v2)*t1 + v2*t2 + 0.5*(vf+v2)*t3 + vf*t4
            t2 = |(v2-vi)/Amax|
            t3 = |(v2-vf)/Amax|
            T  = t1 + t2 + t3 + t4
          |Ai| = Amax
          |Af| = Amax

      Where:
  
       - t1, t2, t3, and t4 are the times for each stage of the motion, 
       - vi, v2 and vf are the initial, stage 2 and final velocities respectively,
       - T is the total time,
       - Ai and Af are the initial and final accelerations respectively and
       - Amax is the acceleration.
  
      We also are subject to the constraints t1, t2, t3, t4 > 0.
  
      We assume we are always given distance, |Amax|, vi, vf and |Vmax| and are
      never given t1 and t3. Since we have basically 4 equations we must have 4
      unknowns, which means we also must know 2 of v2, t2, t4 and T. However,
      there are sometimes 2 solutions and we also need to know the sign of Amax
      (the sign of the initial acceleration). To resolve this we must use the
      positivity constraints and the only way of doing this is to test all
      possible cases. This makes the routine fairly messy, but I don't know a
      way around this.
  
      In addition, it is also messy since if v2 is unknown, the answer is quadratic
      in v2, and both cases in the quadratic have to be decided.
  
      In the case of ambiguity, the minimum to total time is taken. If two answers
      have the same total time, the one with the longest final coast time is
      selected.

      The mathematics are solved in the JCMT TCS design note TCS/DN/10

   Call:
      status = routeFindPath( &path, accel, unknowns );
  
   Parameters:
      ("<" input, "!" modified, "W" workspace, ">" output)

      (!) path      (path_t *)  Path structure.
      (<) accel     (double)    Acceleration.
      (<) unknowns  (int)       Bit mask of unknowns
  
   Returns:
      status (route_status_t)  Status
  
   Author: Nick Rees
*-
   History:
*/

LOCAL route_status_t routeFindPath( path_t * path,
                                    double accel,
                                    int unknowns )
{
    double sqrt_term, lin_term, Ai;
    route_status_t status = ROUTE__OK;

    if (accel <= 0.0) return ROUTE__BADPARAM;

    switch (unknowns)
    {
    case ( V2 | T ):
    case ( V2 | T4 ):
    case ( V2 | T2 ):
    {
        /* Calculate:
            - the minimum time spent accelerating (min_accel_time),
            - the maximum time spent coasting at velocity v2 (max_t2_time) and
            - the minimum distance spent on other parts of the route (min_not_t2_dist) */

        double min_accel_time  = fabs((path->vi - path->vf)/accel);
        double max_t2_time = path->t2;
        double vi_dist, vf_dist, min_not_t2_dist;

        switch ( unknowns & ~ V2 )
        {
        case( T ):
        {
            min_not_t2_dist = 0.5*(path->vi + path->vf)*min_accel_time + path->vf * path->t4;
            break;
        }
        case( T4 ):
        {
            min_not_t2_dist = 0.5*(path->vi + path->vf)*min_accel_time + path->vf * (path->T - path->t2 - min_accel_time);
            break;
        }
        case (T2 ):
        {
            min_not_t2_dist = 0.5*(path->vi + path->vf)*min_accel_time + path->vf * path->t4;
            max_t2_time = path->T - path->t4 - min_accel_time;
            break;
        }
        default:
            return ROUTE__BADPARAM;
        }

        /* Calculate the two critical distances corresponding to v2=vi and v2=vf */
        vi_dist = min_not_t2_dist + path->vi * max_t2_time;
        vf_dist = min_not_t2_dist + path->vf * max_t2_time;

        /* Now calculate v2 */
        if ( path->dist == vi_dist )
        {
            path->v2 = path->vi;
        }
        else if ( path->dist == vf_dist )
        {
            path->v2 = path->vf;
        }
        else
        {
            if ( ((path->dist < vi_dist) && (path->dist > vf_dist)) ||
                 ((path->dist > vi_dist) && (path->dist < vf_dist))    )
            {
                /* Velocity is intermediate between vi and vf */
                if (path->vf > path->vi) Ai = accel;
                else Ai = -accel;

                /* Note: the division by max_t2_time in the following switch block 
                   is OK since if max_t2_time = 0, then vi_dist = vf_dist and it is
                   impossible for path->dist to be intermediate between the two     */

                switch ( unknowns & ~V2 )
                {
                case( T ):
                    path->v2 = (path->dist +
                                0.5*(path->vi*path->vi - path->vf*path->vf)/Ai - 
                                path->vf*path->t4                                ) / max_t2_time;
                    break;
                case( T4 ):
                    path->v2 = (path->dist + 
                                0.5*(path->vi - path->vf)*(path->vi - path->vf)/Ai - 
                                path->vf*(path->T-path->t2))                         / max_t2_time;
                    break;
                case( T2 ):
                    path->v2 = ((path->dist + 
                                 0.5*(path->vi*path->vi - path->vf*path->vf)/Ai - 
                                 path->vf*path->t4) / max_t2_time);
                    break;
                default:
                    break;
                }
            }
            else
            {
                /* Distance is outside range between vi_dist and vf_dist */

                if ((path->dist < vi_dist) && (path->dist < vf_dist))
                {
                    Ai = -accel;
                }
                else /*if ((path->dist > vi_dist) && (path->dist > vf_dist))*/
                {
                    Ai = accel;

                }
                switch ( unknowns & ~ V2 )
                {
                case( T ):
                    lin_term = -0.5 * Ai * path->t2;
                    sqrt_term = 0.5 * path->t2 * Ai;
                    sqrt_term = (sqrt_term * sqrt_term + 
                                 0.5 * (path->vi * path->vi + path->vf * path->vf) +
                                 Ai * (path->dist - path->vf * path->t4));
                    break;
                case( T4 ):
                    lin_term = path->vf - 0.5 * Ai * path->t2;
                    sqrt_term = 0.5 * Ai * path->t2;
                    sqrt_term = (sqrt_term * sqrt_term + 0.5 * (path->vi - path->vf) * (path->vi - path->vf) +
                                 Ai * (path->dist - path->vf*path->T));
                    break;
                case( T2 ):
                    lin_term = 0.5 * (Ai * (path->T-path->t4) + path->vi + path->vf);
                    sqrt_term = (lin_term * lin_term -
                                 0.5 * (path->vi * path->vi + path->vf * path->vf) -
                                 Ai * (path->dist - path->vf * path->t4));
                    break;
                default:
                    return ROUTE__BADPARAM;
                }
                
                /* Solve the quadratic for v2 */
                status = routeFindWhichV2Sqrt( path, 
                                               Ai, 
                                               lin_term,
                                               sqrt_term,
                                               (unknowns & ~V2) );
            }
        }

        if (status == ROUTE__OK)
        {
            path->t1 = fabs( ( path->v2 - path->vi ) / accel );
            path->t3 = fabs( ( path->vf - path->v2 ) / accel );
            if (unknowns & T)  path->T   = path->t1 + path->t2 + path->t3 + path->t4;
            if (unknowns & T4) path->t4  = path->T - (path->t1 + path->t2 + path->t3);
            if (unknowns & T2) path->t2  = path->T - (path->t1 + path->t3 + path->t4);
        }    
        break;
    }
    case ( T | T4 ):
    case ( T | T2 ):
    case ( T2| T4 ):
    {
        /* We know v2 - life is much easier since t1 and t3 are trivial to find ... */
        double dist;

        path->t1 = fabs( ( path->v2 - path->vi ) / accel );
        path->t3 = fabs( ( path->vf - path->v2 ) / accel );
        dist = path->dist - 0.5*((path->vi+path->v2)*path->t1 + (path->v2 + path->vf)*path->t3);

        if (unknowns & T )
        {
            if ((unknowns & T4) && (path->vf != 0))
            {
                path->t4 = (dist - path->v2*path->t2) / path->vf;
            }
            else if ((unknowns & T2) && (path->v2 != 0))
            {
                path->t2 = (dist - path->vf*path->t4) / path->v2;
            }
            else status = ROUTE__BADPARAM;

            path->T = path->t1 + path->t2 + path->t3 + path->t4;
        }
        else if ( path->v2 != path->vf )
        {
            path->t2 = (dist - path->vf * ( path->T - path->t1 - path->t3 ))/ (path->v2 - path->vf);
            path->t4 = path->T - path->t1 - path->t2 - path->t3;
        }
        else status = ROUTE__BADPARAM;

        break;
    }
    default:
        status = ROUTE__BADPARAM;
    }

    /* Now do a test for positive times */
    if (status == ROUTE__OK &&
        (path->t1 < 0 || path->t2 < 0 || path->t3 < 0 || path->t4 < 0 || path->T < 0))
    {
        if (IS_ZERO(path->t1, path->T)) path->t1 = 0;
        if (IS_ZERO(path->t2, path->T)) path->t2 = 0;
        if (IS_ZERO(path->t3, path->T)) path->t3 = 0;
        if (IS_ZERO(path->t4, path->T)) path->t4 = 0;

        if (path->t1 < 0 || path->t2 < 0 || path->t3 < 0 || path->t4 < 0 || path->T < 0)
        {
            status = ROUTE__NEGTIME;
            if (path->t1 < 0) path->t1 = 0;
            if (path->t2 < 0) path->t2 = 0;
            if (path->t3 < 0) path->t3 = 0;
            if (path->t4 < 0) path->t4 = 0;
            if (path->T < 0)  path->T = 0;
        }
    }
    
    if (status == ROUTE__OK &&
        !IS_ZERO( path->T - (path->t1 + path->t2 + path->t3 + path->t4), path->T))
    {
        status = ROUTE__NEGTIME;
    }

/*    printf( "Path found: T=%g, t1=%g, t2=%g, t3=%g, t4=%g, vi=%g, v2=%g, vf=%g %d %d\n",
            path->T, path->t1, path->t2, path->t3, path->t4,
            path->vi, path->v2, path->vf, unknowns, status ); */

    return status;
}

/*+                      r o u t e F i n d P a t h W i t h V m a x
 
   Function Name: routeFindPathWithVmax

   Function: Solves the 4-Phase path problem with a maximum velocity constraint.
  
   Description:
  
      This function finds a path from a given position and velocity to a final
      position and velocity via a four stage motion, subject to a maximum velocity.
      It uses routeFindPath to determine the paths.

      It initially sets the second stage coast time to zero and attempts to find a
      path. If the maximum velocity is less than Vmax, it exits. Otherwise it
      restricts the coast velocity (v2) to the maximum velocity and allows the coast
      time to vary.


   Call:
      status = routeFindPathWithVmax( &path, accel, vmax, unknowns );
  
   Parameters:
      ("<" input, "!" modified, "W" workspace, ">" output)

      (!) path      (path_t *)          Path structure.
      (<) accel     (double)            Acceleration.
      (<) vmax      (double)            Maximum velocity
      (<) unknowns  (route_unknown_t)   Unknown - either T, or t4.
  
   Returns:
      status (route_status_t)  Status from routeFindPath
  
   Author: Nick Rees
*-
   History:
*/


LOCAL route_status_t routeFindPathWithVmax(path_t * path, double Amax, double Vmax, route_unknown_t unknown)
{
    route_status_t status;

    path->t2 = 0;
    status = routeFindPath(path, Amax, V2 | unknown );

    if ((status == ROUTE__OK) && (fabs(path->v2) > Vmax))
    {
        if (path->v2 >= 0.0)
            path->v2 = Vmax;
        else
            path->v2 = -Vmax;

        status = routeFindPath( path, Amax, T2 | unknown );
    }
    return status;
}

/********************************************************************************/
/*                                                                              */
/* External Routines                                                            */
/*                                                                              */
/********************************************************************************/

/*+                      r o u t e N e w
  
   Function Name: routeNew
  
   Function: Initialises the routing mechanism
 
   Description:
      This function mallocs and initialises the internal data structures used by
      the routing algorithm.
 
   Call:
      route = routeNew( demand, params )
  
   Parameters:
      ("<" input, "!" modified, "W" workspace, ">" output)

      (<) demand    (route_demand_t *) The inital demand for the routing system. All
                                       subsequent routing will derive from this point.
      (<) params    (route_pars_t *)   The initial parameters for routing.

   Returns:
      route     (ROUTE_ID)   Either a pointer to a valid routing structure, or NULL
                             if an error occurs (Bad parameters or no space available)

   Author: Nick Rees

*-

   History:
*/

ROUTE_ID routeNew( route_demand_t * demand, route_pars_t * params )
{
    ROUTE_ID route = NULL;
    unsigned int i, ok;

    /* Check input parameters */
    ok = (params != NULL && demand != NULL && params->Tsync >= 0);

    for (i=0; i<params->numRoutedAxes && ok; i++)
    {
        int j = params->routedAxisList[i] - 1;
        ok = (params->axis[j].Amax > 0 && 
              params->axis[j].Vmax > 0 && 
              params->axis[j].Vmax > fabs(demand->axis[j].v) );
    }

    /* If input parameters are OK, malloc and initialize structure */
    if ( ok && ((route = (route_t *) calloc( sizeof(route_t),1)) != NULL) )
    {
        route->pars = *params;
        route->endp = *demand;
        route->demand = *demand;
        for (i=0; i<params->numRoutedAxes; i++)
        {
            int j = params->routedAxisList[i] - 1;
            route->path[j].vi = demand->axis[j].v;
            route->path[j].v2 = demand->axis[j].v;
            route->path[j].vf = demand->axis[j].v;
        }
     }

    return route;
}


/*+                      r o u t e F i n d
  
   Function Name: routeFind
  
   Function: Finds a route to a given target and returns the necessary information
 
   Description:
      This function calculates a route from the last demand position to a new endpoint
      and returns an appropriate route to this position that lies within the systems
      acceleration and velocity constraints.
 
   Call:
      status = routeFind( route, reroute, &endpoint, &nextpoint )
  
   Parameters:
      ("<" input, "!" modified, "W" workspace, ">" output)

      (<) route     (ROUTE_ID)         Route information
      (<) reroute   (int)              Flag set to force a total route recalculation,
                                       including a resynchronisation to the synch
                                       time, if required. It can have one of the following values:
                                       ROUTE_NEW_ROUTE    => Force a route recalculation, including
                                                             resychronization to the synch time.
                                       ROUTE_NO_NEW_ROUTE => Force routing to be turned off. The next
                                                             route position is the end-point and it
                                                             will be reached at the nextpoint time.
                                       ROUTE_CALC_ROUTE   => Calculate an aceptable route to the next
                                                             point.
      (!) endpoint  (route_demand_t *) On input, the position and velocities in this
                                       must be the final demand for the route.
                                       On output, the time in the structure will be
                                       modified to reflect the actual time the system
                                       will be at the endpoint if the calculated route
                                       is followed.
      (!) nextpoint (route_demand_t *) On output, the time in this structure must be
                                       the time for which the next demand is required.
                                       This must be a later time than the previous
                                       time this routine was called for this route.
                                       On output, the positions and velocities in the
                                       structure will be updated to give an appropriate
                                       demand at the given time

   Returns:
      status (route_status_t)  Always ROUTE__OK

   Author: Nick Rees

*-

   History:
*/

route_status_t routeFind( ROUTE_ID route, route_reroute_t reroute, route_demand_t * endp, route_demand_t * nextp )
{
    route_status_t status = ROUTE__OK;
    route_status_t ret_status = ROUTE__OK;
    unsigned int long_path, short_path, i;
    int old_path_ok;

    /* If no axes are routed, copy endp into nextp, and return */
    if (route->pars.numRoutedAxes == 0)
    {
        *nextp = *endp;

        /* Save the previous demands and endpoints */
        route->demand = *nextp;
        route->endp = *endp;
        return ret_status;
    }

    /* If we don't want routing, set the old demand to be a coast from the current endpoint */
    if (reroute == ROUTE_NO_NEW_ROUTE)
    {
        for (i = 0; i < route->pars.numRoutedAxes; i++ )
        {
            int j = route->pars.routedAxisList[i] - 1;
            route->demand.axis[j].v = endp->axis[j].v;
            route->demand.axis[j].p = endp->axis[j].p - (nextp->T - route->demand.T)*endp->axis[j].v;
        }
    }

    /* First check whether the previous path is OK to use */
    old_path_ok = (reroute == ROUTE_CALC_ROUTE) && IS_ZERO( (endp->T - route->endp.T), endp->T);
    for ( i = 0; i < route->pars.numRoutedAxes && old_path_ok; i++ )
    {
        int j = route->pars.routedAxisList[i] - 1;
        old_path_ok = (IS_ZERO( (endp->axis[j].p - route->endp.axis[j].p), 40.0) &&
                       (fabs(endp->axis[j].v - route->endp.axis[j].v) < (route->pars.axis[j].Vmax*1.0e-10)));
/*         old_path_ok = (IS_ZERO( (endp->axis[j].p - route->endp.axis[j].p), 6.0) && */
/*                        IS_ZERO( (endp->axis[j].v - route->endp.axis[j].v), route->pars.axis[j].Vmax)); */
    }

    if (!old_path_ok)
    {
        /* Initialise the Path structures */
        for (i = 0; i < route->pars.numRoutedAxes; i++ )
        {
            int j = route->pars.routedAxisList[i] - 1;
            route->path[j].dist = endp->axis[j].p - route->demand.axis[j].p;
            route->path[j].vi = route->demand.axis[j].v;
            route->path[j].vf = endp->axis[j].v;
            route->path[j].t2 = 0.0;
            route->path[j].t4 = route->pars.Tcoast;
            route->path[j].T  = endp->T - route->demand.T;
        }

        /* Calculate whether we expect to complete this route in the next coast period */
        short_path = (reroute != ROUTE_NEW_ROUTE) && (nextp->T + route->pars.Tcoast >= endp->T);
        if (short_path)
        {
            for (i = 0; ((i < route->pars.numRoutedAxes) && (status == ROUTE__OK)); i++ )
            {
                int j = route->pars.routedAxisList[i] - 1;
                status = routeFindPathWithVmax(&route->path[j],
                                               route->pars.axis[j].Amax,
                                               route->pars.axis[j].Vmax,
                                               T4 );
            }
        }

        if (!short_path || (status != ROUTE__OK))
        {
            /* Either we didn't find a route using the above or the time is longer */

            long_path = 0;
            for (i = 0; i < route->pars.numRoutedAxes; i++ )
            {
                int j = route->pars.routedAxisList[i] - 1;
                route->path[j].t4 = route->pars.Tcoast;
                status = routeFindPathWithVmax(&route->path[j],
                                               route->pars.axis[j].Amax,
                                               route->pars.axis[j].Vmax,
                                               T );
                switch (status)
                {
                case ROUTE__OK:
                case ROUTE__NEGSQRT:
                case ROUTE__NEGTIME:
                    break;
                default:
                    return status;
                }

                if (route->path[j].T > route->path[long_path].T) long_path = j;
            }

            /* Set the time for the path - synchronising it to an integral 
               number of Tsync units, if required */
            if (route->pars.Tsync > 0)
            {
                endp->T = ceil((route->demand.T + route->path[long_path].T) / 
                               route->pars.Tsync) * route->pars.Tsync;
                route->path[long_path].T = endp->T - route->demand.T;
            }
            else
            {
                endp->T = route->demand.T + route->path[long_path].T;
            }

            /* Recalculate the paths for the new total path time. */
            for (i = 0; i < route->pars.numRoutedAxes; i++ )
            {
                int j = route->pars.routedAxisList[i] - 1;
                route->path[j].T = route->path[long_path].T;

                /* If we are synchronising to an integral number of tic's all
                   paths have to be recalculated, otherwise all except the
                   longest path needs recaculating */
                if ( route->pars.Tsync > 0 || j != long_path )
                {
                    status = routeFindPath(&route->path[j],
                                           route->pars.axis[j].Amax,
                                           T2 | V2 );
                }

                if (status != ROUTE__OK) ret_status = status;
            }
        }
    }

    /* Now all the paths are valid - calculate the demand position at the next time */
    for (i=0; i< route->pars.numRoutedAxes; i++)
    {
        int j = route->pars.routedAxisList[i] - 1;
        status = routeDemand(&route->path[j],
                             (nextp->T - endp->T),
                             &(nextp->axis[j]) );
        nextp->axis[j].p += endp->axis[j].p;
        if (status != ROUTE__OK) return status;
    }

    /* Save the previous demands and endpoints */
    route->demand = *nextp;
    route->endp = *endp;

    return ret_status;
}

/*+                      r o u t e D e l e t e
  
   Function Name: routeDelete
  
   Function: Deletes all structures associated with the given route
 
   Description:
      This function frees the internal data structures used by the routing
      algorithm.
 
   Call:
      (void) routeNew( route )
  
   Parameters:
      ("<" input, "!" modified, "W" workspace, ">" output)

      (<) route     (ROUTE_ID)         The ID of the route to be deleted.

   Author: Nick Rees

*-

   History:
*/

void routeDelete( ROUTE_ID route )
{
    /* Check input parameters */
    free( route );
    return;
}


/*+                      r o u t e S e t D e m a n d
  
   Function Name: routeSetDemand
  
   Function: Re-initialises the current demand of the routing mechanism
 
   Description:
      This function re-initialises the current demand of the routing algorithm.
      Note that this function is normally not required because the algorithm 
      assumes routing is always on, and so the current demand is maintained by
      the algorithm internally.

   Call:
      route = routeSetDemand( route, demand )
  
   Parameters:
      ("<" input, "!" modified, "W" workspace, ">" output)

      (<) route     (ROUTE_ID)         The route identifier.
      (<) demand    (route_demand_t *) The inital demand for the routing system. All
                                       subsequent routing will derive from this point.

   Returns:
      status    (route_status_t)   Always ROUTE__OK - you can set the demand to
                                   anything you like.

   Author: Nick Rees

*-

   History:
*/

route_status_t routeSetDemand( ROUTE_ID route, route_demand_t * demand )
{
    route->demand = *demand;

    return ROUTE__OK;
}

/*+                      r o u t e S e t P a r a m s
  
   Function Name: routeSetParams
  
   Function: Re-initialises the routing mechanism parameters
 
   Description:
      This function allows the user to reinitialise all the routing algorithm
      parameteters on the fly.
 
   Call:
      status = routeSetParams( route, params );
  
   Parameters:
      ("<" input, "!" modified, "W" workspace, ">" output)

      (<) route     (ROUTE_ID)         The ID of the route to be changed.
      (<) params    (route_pars_t *)   The new routing parameters.

   Returns:
      status     (route_status_t)   ROUTE_BADPARAMS if the times given are
                                    negative or the velocities and accelerations are
				    negative or zero. Otherwise ROUTE__OK.

   Author: Nick Rees

*-

   History:
*/

route_status_t routeSetParams( ROUTE_ID route, route_pars_t * params )
{
    unsigned int i, ok;
    route_status_t status = ROUTE__OK;

    /* Check input parameters */
    ok = (params != NULL && 
	  params->Tsync >= 0 &&
	  params->Tcoast >= 0   );

    for (i=0; i<params->numRoutedAxes && ok; i++)
    {
        int j = params->routedAxisList[i] - 1;
        ok = (params->axis[j].Amax > 0 && 
              params->axis[j].Vmax > 0 && 
              params->axis[j].Vmax > fabs(route->demand.axis[j].v) );
    }

    /* If input parameters are OK, re-initialize the params structure */
    if ( ok )
    {
        route->pars = *params;
    }
    else
    {
	status = ROUTE__BADPARAM;
    }
    return status;
}

/*+                      r o u t e G e t P a r a m s
  
   Function Name: routeGetParams
  
   Function: Returns the routing mechanism parameters
 
   Description:
      This function allows the user to read all the routing algorithm
      parameteters.
 
   Call:
      status = routeGetParams( route, &params );
  
   Parameters:
      ("<" input, "!" modified, "W" workspace, ">" output)

      (<) route     (ROUTE_ID)         The ID of the route to be interrogated.
      (>) params    (route_pars_t *)   The current routing parameters.

   Returns:
      status     (route_status_t)   Always ROUTE__OK.

   Author: Nick Rees

*-

   History:
*/

route_status_t routeGetParams( ROUTE_ID route, route_pars_t * params )
{
    *params = route->pars;

    return ROUTE__OK;
}


/*+                      r o u t e G e t N u m R o u t e d A x e s
  
   Function Name: routeGetNumRoutedAxes
  
   Function: Returns the number of routed axes
 
   Description:
      This function allows the user to determine how many axes are routed
 
   Call:
      status = routeGetNumRoutedAxes( route, &number );
  
   Parameters:
      ("<" input, "!" modified, "W" workspace, ">" output)

      (<) route     (ROUTE_ID)         The ID of the route to be interrogated.
      (>) number    (unsigned int *)   The number of routed axes.

   Returns:
      status     (route_status_t)   Always ROUTE__OK.

   Author: Russell Kackley

*-

   History:
*/

route_status_t routeGetNumRoutedAxes( ROUTE_ID route, unsigned int * number )
{
    *number = route->pars.numRoutedAxes;

    return ROUTE__OK;
}



/* Test routines */

#ifdef TEST_FIND_PATH

#include <stdio.h>

int main( int * argc, char * argv[] )
{
    int     option = 1;
    double  value=0.0, distance;
    path_t path = {0,0,0,0,0,0,0,0,0};
    double Amax = 0.0;
    double Vmax = 0.0;
    route_status_t status = 0;
    int unknown1 = 1;
    int unknown2 = 2;
    int unknowns;
    char line[128];

    while ( option != 0 )
    {
        printf("\n \
                Choose from: \n \
                    0 - Exit \n \
                    1 - Set Amax (%f) \n \
                    2 - Set Vmax (%f) \n \
                    3 - Set dist (%f) \n \
                    4 - Set vi   (%f) \n \
                    5 - Set vf   (%f) \n \
                    6 - Set v2   (%f) \n \
                    7 - Set t1   (%f) \n \
                    8 - Set t2   (%f) \n \
                    9 - Set t3   (%f) \n \
                   10 - Set t4   (%f) \n \
                   11 - Set T    (%f) \n \
                   12 - Set unknown 1 (1 = v2, 2=T, 3=t2, 4=t4) (%d) \n \
                   13 - Set unknown 2 (1 = v2, 2=T, 3=t2, 4=t4) (%d) \n \
                   14 - Enter all values at once \n \
 \n \
               Enter option : ",
               Amax,
               Vmax,
               path.dist,
               path.vi,
               path.vf,
               path.v2,
               path.t1,
               path.t2,
               path.t3,
               path.t4,
               path.T,
               unknown1, unknown2 );

        while ( gets( line ) == NULL || line[0] == '#' || (sscanf( line, "%d", &option ) != 1));

        if (option > 0 && option < 14)
        {
            printf( "\n                Enter value  : " );
            while ( gets( line ) == NULL || line[0] == '#' || (sscanf( line, "%lf", &value ) != 1));

            switch (option)
            {
            case 1: Amax = value; break;
            case 2: Vmax = value; break;
            case 3: path.dist = value; break;
            case 4: path.vi = value; break;
            case 5: path.vf = value; break;
            case 6: path.v2 = value; break;
            case 7: path.t1 = value; break;
            case 8: path.t2 = value; break;
            case 9: path.t3 = value; break;
            case 10: path.t4 = value; break;
            case 11: path.T = value; break;
            case 12: unknown1 = (int) value; break;
            case 13: unknown2 = (int) value; break;
            default: break;
            }
        }
        else if (option == 14)
        {
            while ( gets( line ) == NULL || line[0] == '#' ||
                    (sscanf( line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d",
                             &Amax, &Vmax, &path.dist,
                             &path.vi, &path.vf, &path.v2,
                             &path.t1, &path.t2, &path.t3, &path.t4, &path.T,
                             &unknown1, &unknown2 ) != 13 ));
        }

        unknowns =(0x1 << (int)(unknown1-1)) | (0x1 << (int)(unknown2-1));
        status = routeFindPath( &path, Amax, unknowns );
        distance = (path.t1 * (path.vi+path.v2) / 2 +
                    path.t2 * path.v2 +
                    path.t3 * (path.v2+path.vf) / 2 +
                    path.t4 * path.vf);

        printf("\n" );
        printf("                routeFindPath status = %d\n", (int) status );
        printf("                distance       = %.12f\n", distance );
        printf("                distance error = %.12f\n", distance - path.dist );
    }

    return 0;
}

#endif

#ifdef TEST

#include <stdio.h>

#define DAS2R (3.141592654/(3600.0*180.0))

route_demand_t route_list[] =
{
    0.0, -100*DAS2R, 10*DAS2R,  10.0*DAS2R, 0.0, -100*DAS2R, 10*DAS2R,
    0.0, -100*DAS2R, 10*DAS2R,   0.0*DAS2R, 0.0, -100*DAS2R, 10*DAS2R,
    0.0, -100*DAS2R, 10*DAS2R, -10.0*DAS2R, 0.0, -100*DAS2R, 10*DAS2R,
    0.0, -180*3600*DAS2R, 10*DAS2R, -50*3600*DAS2R, 0.0,      -180*3600*DAS2R, 10*DAS2R,
    0.0, -180*3600*DAS2R, 10*DAS2R, (-50*3600+10)*DAS2R, 0.0, -180*3600*DAS2R, 10*DAS2R, 
    0.0, -180*3600*DAS2R, 10*DAS2R, (-50*3600+20)*DAS2R, 0.0, -180*3600*DAS2R, 10*DAS2R
};

#define NUM_DEMANDS (sizeof(route_list)/sizeof(route_demand_t))

route_demand_t sky = { 0.0, 0*DAS2R, 10*DAS2R, 0.0*DAS2R, 0.0 };

int routeTCSSim( int route_num, route_demand_t * demand, int slewing )
{
    route_demand_t * this_route = &route_list[route_num % NUM_DEMANDS];
    int i;

    if (slewing) this_route->T = demand->T;

    for (i = 0; i < NUM_AXES; i++ )
    {
        demand->axis[i].p = (sky.axis[i].p + 
                             sky.axis[i].v * demand->T +
                             this_route->axis[i].p + 
                             this_route->axis[i].v * (demand->T - this_route->T));
        demand->axis[i].v = (sky.axis[i].v + this_route->axis[i].v);
    }
     
    return 0;
}

int route_numbers[]  = {  0,  1,  2, 3, 4, 5 };
double route_times[] = { 10, 10, 10, 60, 60, 60 };

#define NUM_ROUTES  (sizeof(route_numbers)/sizeof(int))

route_demand_t initial_demand = { 0.0, 0.0, 0.0, 0.0, 0.0 0.0, 0.0};
route_pars_t initial_params =
{
    3,                 /* numRoutedAxes */
    1,2,3,             /* routedAxisList */
    0.05,              /* Tsync  */
    1.0,              /* Tcoast */
    0.12*3600.0*DAS2R, /* Amax   */
    0.60*3600.0*DAS2R, /* Vmax   */
    0.12*3600.0*DAS2R, /* Amax   */
    0.60*3600.0*DAS2R, /* Vmax   */
    0.12*3600.0*DAS2R, /* Amax   */
    0.60*3600.0*DAS2R  /* Vmax   */
};

#define DELTA_T (0.05);

#include "thi_route_if.h"

int main( int argc, char * argv[] )
{
    ROUTE_ID route = routeNew( &initial_demand, &initial_params );
    route_demand_t demand=initial_demand;
    route_demand_t next_demand=initial_demand;
    double last_route_start = initial_demand.T;
    int i, slewing, status, tel_step;

    for ( i = 0; i<NUM_ROUTES; i++ )
    {
        slewing = 1;
        last_route_start = next_demand.T;
        tel_step = 1;

        while ( slewing || (next_demand.T < (last_route_start + route_times[i])))
        {

            routeTCSSim( route_numbers[i], &demand, slewing );
            /* status = (int) routeFindFJO( route, tel_step, &demand, &next_demand ); */
            status = (int) routeFind( route, tel_step, &demand, &next_demand ); 
            tel_step = 0;

            printf( "%g %g %g %g %g %g %g %g %g %g %d %d\n",
                    next_demand.T,
                    next_demand.axis[0].p ,next_demand.axis[0].v,
                    next_demand.axis[1].p ,next_demand.axis[1].v,
                    demand.T,
                    demand.axis[0].p ,demand.axis[0].v,
                    demand.axis[1].p ,demand.axis[1].v,
                    slewing, status );

            next_demand.T += DELTA_T;

            if (slewing) last_route_start = demand.T;
            slewing = (demand.T > next_demand.T);
            if (!slewing) demand.T = next_demand.T;
        }
    }

    return 0;
}

#endif

void routePrint( ROUTE_ID route, route_reroute_t reroute, route_demand_t * endp, route_demand_t * nextp, FILE * logfile )
{
    int i;

    fprintf( logfile, "\nreroute %d\n", reroute);
    
    fprintf( logfile, "\nroute pars struct\n");
    fprintf( logfile, "Tsync, Tcoast %f %f\n", route->pars.Tsync, route->pars.Tcoast );
    for (i = 0; i < route->pars.numRoutedAxes; i++ )
    {
        fprintf( logfile, "Axis %d Amax, Vmax %f %f\n", i, route->pars.axis[i].Amax * DR2D, route->pars.axis[i].Vmax * DR2D);
    }
    
    fprintf( logfile, "\nroute path struct\n");
    for (i = 0; i < route->pars.numRoutedAxes; i++ )
    {
        fprintf( logfile, "Axis %d dist %f vi %f vf %f v2 %f t1 %f t2 %f t3 %f t4 %f T %f \n", i,
                 route->path[i].dist * DR2D, route->path[i].vi * DR2D, route->path[i].vf * DR2D, route->path[i].v2 * DR2D,
                 route->path[i].t1, route->path[i].t2, route->path[i].t3, route->path[i].t4, route->path[i].T );
    }

    fprintf( logfile, "\nroute demand struct\n");
    fprintf( logfile, "T %f\n", route->demand.T );
    for (i = 0; i < route->pars.numRoutedAxes; i++ )
    {
        fprintf( logfile, "Axis %d p, v %f %f\n", i, route->demand.axis[i].p * DR2D, route->demand.axis[i].v * DR2D );
    }
    
    fprintf( logfile, "\nroute endp struct\n");
    fprintf( logfile, "T %f\n", route->endp.T );
    for (i = 0; i < route->pars.numRoutedAxes; i++ )
    {
        fprintf( logfile, "Axis %d p, v %f %f\n", i, route->endp.axis[i].p * DR2D, route->endp.axis[i].v * DR2D );
    }
    
    fprintf( logfile, "\nendp struct\n");
    fprintf( logfile, "T %f\n", endp->T );
    for (i = 0; i < route->pars.numRoutedAxes; i++ )
    {
        fprintf( logfile, "Axis %d p, v %f %f\n", i, endp->axis[i].p * DR2D, endp->axis[i].v * DR2D );
    }
    
    fprintf( logfile, "\nnextp struct\n");
    fprintf( logfile, "T %f\n", nextp->T );
    for (i = 0; i < route->pars.numRoutedAxes; i++ )
    {
        fprintf( logfile, "Axis %d p, v %f %f\n", i, nextp->axis[i].p * DR2D, nextp->axis[i].v * DR2D );
    }
}

