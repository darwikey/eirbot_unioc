/////////////////////////////////////////////////////////////////////////////////
//      Trajectory Manager     
// 
// Derniere modification:
// Vincent, le 19/12/2011 a 18h
// 
// Commentaire:
// 
//
// A faire:
//
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\



#include <scheduler.h>
#include <vect2.h>
#include <math.h>
#include <uart.h>
#include "trajectory_manager.h"
#include "modulo.h"


static int8_t couleur_depart = 0;//devra valoir 1 ou -1 selon si on démarre en rouge ou en bleu, pour le mutick,(void*)t,ROBOT_PM_UPDATE_TIME/SCHEDULER_UNIT);


void trajectory_init(trajectory_manager_t *t,position_manager_t *pm,asserv_manager_t * ass)//,notification_manager_t *not)
{
  t->current = 0;
  t->last = 0;

  t->pm = pm;
  t->ass = ass;
  //t->not = not;
  scheduler_add_periodical_event(trajectory_tick,(void*)t,ROBOT_PM_UPDATE_TIME/SCHEDULER_UNIT);

  // EIRBUG
  t->directed_flag = 0;
}


void trajectory_reinit(trajectory_manager_t *t)
{
  t->current = 0;
  t->last = 0;


  

  // EIRBUG
  t->directed_flag = 0;
}


static void trajectory_next(trajectory_manager_t *t)
{
//  printf("NEXT ! %ld %ld\n",t->current,t->last);
  if(t->current != t->last)
  {
    t->current = (t->current +1)%TRAJECTORY_NB_POINT_MAX;
  }
}

void trajectory_removeStep(trajectory_manager_t *t)
{
  while(t->current != t->last)
  {
    if(t->points[t->current].type == STEP)
    {
      trajectory_next(t);
      return;
    }
    trajectory_next(t);
  }
}

static int8_t trajectory_add_point(trajectory_manager_t *t,trajectory_order_when_t when,trajectory_dest_t point)
{
  point.flags = 0;
  
  // Si on a atteint la taille de la pile -1
  // TODO à améliorer !!
  if((t->last + 1) % TRAJECTORY_NB_POINT_MAX == t->current) {
    return -1;
  }

  if(when == END)
  {
    t->points[t->last] = point;
    t->last = (t->last + 1) % TRAJECTORY_NB_POINT_MAX;
  }
  else if(when == BEFORE_LAST_STEP)
  {
    uint8_t i;
    if(t->current != t->last)
    {
      if(t->points[t->current].type == STEP)
      {
        trajectory_add_point(t,NOW,point);
        return 1;
      }
      else
      {
        for(i=((t->current+1) % TRAJECTORY_NB_POINT_MAX); i!=t->last;i=((i+1)%TRAJECTORY_NB_POINT_MAX))
        {
          if(t->points[i].type == STEP)
          {
            int8_t id = (i-1)%TRAJECTORY_NB_POINT_MAX;
            if(id < 0)
            {
              id += TRAJECTORY_NB_POINT_MAX;
            }
            trajectory_add_point(t,id,point);
          }
          return 1;
        }
        trajectory_add_point(t,END,point);
      }
    }
    else
    {
      trajectory_add_point(t,NOW,point);
    }
  }
  else
  {
    int8_t i;
    if(((t->last+1) % TRAJECTORY_NB_POINT_MAX) == t->current)
    {
      //TODO: env notif au gestionnaire d'erreur... plus de point dispo
      //      que faire ?
      //      pour l'instant on saute le point actuel..
      printf("BUGGGG traj pas assez de point dispo %ld %ld\n",t->last,t->current);
      trajectory_next(t);
    }
    when = (t->current + when) % TRAJECTORY_NB_POINT_MAX;
    //for(i=((t->last+1) % TRAJECTORY_NB_POINT_MAX); i!=when;i=((i-1)%TRAJECTORY_NB_POINT_MAX))
    for(i=((t->last+1) % TRAJECTORY_NB_POINT_MAX); i!=when;i=modulo_fast_8(i-1,TRAJECTORY_NB_POINT_MAX))
    {
/*      if(i < 0)
      {
        i += TRAJECTORY_NB_POINT_MAX;
      }*/
      //int8_t id = (i-1)%TRAJECTORY_NB_POINT_MAX;
      int8_t id = modulo_fast_8((i-1),TRAJECTORY_NB_POINT_MAX);
 /*     if(id < 0)
      {
        id += TRAJECTORY_NB_POINT_MAX;
      }*/
      t->points[i] = t->points[id];
    }
    t->points[when] = point;
    t->last = ((t->last+1) % TRAJECTORY_NB_POINT_MAX);
  }
  return 0;
}

uint8_t trajectory_is_paused(trajectory_manager_t *t)
{
  
  if(t->current != t->last)
  {
    trajectory_dest_t *p = t->points + t->current;
    if(p->type == WAIT)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
  return 0;
}


//Checker si un mouvement est fini
uint8_t trajectory_is_ended(trajectory_manager_t *t)
{
	if(t->current == t->last) {
		return 1;
	} else {
		return 0;
	}
}

void trajectory_resume(trajectory_manager_t *t)
{
  while(trajectory_is_paused(t))
  {
    trajectory_next(t);
  }
}

void trajectory_pause(trajectory_manager_t * t)
{
  if(!trajectory_is_paused(t))
  {
    trajectory_goto_wait(t,NOW);
  }
  trajectory_tick((void *)t);
}

int8_t trajectory_goto_d_noa(trajectory_manager_t *t,trajectory_order_when_t when,double d)
{
  trajectory_dest_t dest;
  dest.type = D_NOA;
  dest.d.d = position_cm2imp(t->pm,fxx_from_double(d));
  dest.d.prec_d = position_cm2imp(t->pm,DEFAULT_PREC_DISTANCE);
  return trajectory_add_point(t,when,dest);
}

int8_t trajectory_goto_d(trajectory_manager_t *t,trajectory_order_when_t when,double d)
{
  trajectory_dest_t dest;
  dest.type = D;
  dest.d.d = position_cm2imp(t->pm,fxx_from_double(d));
  dest.d.prec_d = position_cm2imp(t->pm,DEFAULT_PREC_DISTANCE);
  return trajectory_add_point(t,when,dest);
}

int8_t trajectory_goto_a(trajectory_manager_t *t,trajectory_order_when_t when,double a)
{
  while(a<-180)
  {
    a=a+360;
  }

  while(a>180)
  {
    a=a-360;
  }

  //a = a * get_couleur_depart(); chgt ici !!!!!!
  trajectory_dest_t dest;
  dest.type = A;
  dest.a.a = position_deg2imp(t->pm,fxx_from_double(a));
  dest.a.prec_a = position_deg2imp(t->pm,DEFAULT_PREC_ANGLE);
  return trajectory_add_point(t,when,dest);
}

int8_t trajectory_goto_arel(trajectory_manager_t *t,trajectory_order_when_t when,double a)
{
  while(a<-180)
  {
    a=a+360;
  }


  while(a>180)
  {
    a=a-360;
  }
  //a = a * get_couleur_depart();
  trajectory_dest_t dest;
  dest.type = AREL;
  dest.a.a = position_deg2imp(t->pm,fxx_from_double(a));
  dest.a.prec_a = position_deg2imp(t->pm,DEFAULT_PREC_ANGLE);
  return trajectory_add_point(t,when,dest);
}

int8_t trajectory_goto_xy(trajectory_manager_t *t,trajectory_order_when_t when,double x,double y)
{
  //y = y * get_couleur_depart();
  //printf("GOTO xy %f %f %d\n",x,y,when);
  trajectory_dest_t dest;
  dest.type = XY;
  dest.xy.x = position_cm2imp(t->pm,fxx_from_double(x));
  dest.xy.y = position_cm2imp(t->pm,fxx_from_double(y));
  dest.xy.prec_xy = position_cm2imp(t->pm,DEFAULT_PREC_DISTANCE);
  dest.xy.prec_xy = dest.xy.prec_xy * dest.xy.prec_xy;
  dest.xy.prec_ap = position_deg2imp(t->pm,DEFAULT_PREC_ANGLE_P);
  //printf("DEBUG %f %f\n",x,y);
  return trajectory_add_point(t,when,dest);
}


void trajectory_goto_reculon_xy(trajectory_manager_t *t, trajectory_order_when_t chen, double x, double y)//Nouvelle fonction pour se déplacer plus simplement à reculon
{
  //y = y * get_couleur_depart();
  double x_current = position_get_x_cm(t->pm);
  double y_current = position_get_y_cm(t->pm);//*get_couleur_depart();
  //printf("%lf\t", x - x_current);
  double angle = 180 + 180 * atan2(y - y_current, x - x_current) / M_PI;
  //printf("%lf\n", angle);
  double dist = -1 * sqrt((x - x_current)*(x - x_current) + (y - y_current)*(y - y_current));

    trajectory_goto_a(t, END, angle); // relatif ou pas?
    trajectory_goto_d(t, END, dist);
}

int8_t trajectory_goto_xya(trajectory_manager_t *t,trajectory_order_when_t when,double x,double y,double a)
{
  //y = y * get_couleur_depart();
  //a = a * get_couleur_depart();
  trajectory_dest_t dest;
  dest.type = XYA;
  dest.xya.x = position_cm2imp(t->pm,fxx_from_double(x));
  dest.xya.y = position_cm2imp(t->pm,fxx_from_double(y));
  dest.xya.a = position_deg2imp(t->pm,fxx_from_double(a));
  dest.xya.prec_xy = position_cm2imp(t->pm,DEFAULT_PREC_DISTANCE);
  dest.xya.prec_xy = dest.xya.prec_xy * dest.xya.prec_xy;
  dest.xya.prec_a = position_deg2imp(t->pm,DEFAULT_PREC_ANGLE);
  dest.xya.prec_ap = position_deg2imp(t->pm,DEFAULT_PREC_ANGLE_P);
  return trajectory_add_point(t,when,dest);
}

int8_t trajectory_goto_step(trajectory_manager_t *t,trajectory_order_when_t when)
{
  trajectory_dest_t dest;
  dest.type = STEP;
  return trajectory_add_point(t,when,dest);
}

int8_t trajectory_goto_wait(trajectory_manager_t *t,trajectory_order_when_t when)
{
  trajectory_dest_t dest;
  dest.type = WAIT;
  return trajectory_add_point(t,when,dest);
}

int32_t trajectory_get_nearest_angle(trajectory_manager_t *t,int32_t angle)
{
  int32_t ca = angle;
  int32_t cb = angle + 2L*ROBOT_IMP_PI;
  int32_t cc = angle + 2L*ROBOT_IMP_PI;
  if(abs(ca - position_get_angle(t->pm)) > abs(cb -position_get_angle(t->pm)))
  {
    ca = cb;
  }
  if(abs(ca - position_get_angle(t->pm)) > abs(cc -position_get_angle(t->pm)))
  {
    ca = cc;
  }
  return ca;
}

void trajectory_tick(void *arg)
{
  trajectory_manager_t * t = (trajectory_manager_t *)arg;

//      printf("debugdispo %ld %ld\n",t->last,t->current);
  if(t->current != t->last)
  {
    trajectory_dest_t *p = t->points + t->current;
    if((!p->flags & ALREADY_INIT))
    {
      p->startingD = position_get_distance(t->pm);
      p->startingA = position_get_angle(t->pm);
      p->flags |= ALREADY_INIT;
      if(p->type == XYPION || p->type == XYPIONBACK){
        double acible = atan2(p->xy.y-position_get_y(t->pm),p->xy.x-position_get_x(t->pm));
        p->xy.x-=position_cm2imp(t->pm,fxx_from_double(14))*cos(acible);
        p->xy.y-=position_cm2imp(t->pm,fxx_from_double(14))*sin(acible);
      }
    }
    if(p->type == D)
    {
      int32_t cons = p->d.d + p->startingD;
      if(abs(cons - position_get_distance(t->pm)) < p->d.prec_d)
      {
        trajectory_next(t);
      }
      else
      {
        asserv_set_angle(t->ass,p->startingA);
        asserv_set_distance(t->ass,cons);
      }
    }
    else if(p->type == D_NOA)
    {
      int32_t cons = p->d.d + p->startingD;
      if(abs(cons - position_get_distance(t->pm)) < p->d.prec_d)
      {
        trajectory_next(t);
      }
      else
      {
        //asserv_set_no_angle(t->ass);
        asserv_set_distance(t->ass,cons);
      }
    }

    else if(p->type == AREL)
    {
      int32_t cons = p->a.a + p->startingA;
      if(abs(cons - position_get_angle(t->pm)) < p->a.prec_a)
      {
        trajectory_next(t);
      }
      else
      {
        asserv_set_angle(t->ass,cons);
      }
    }
    else if(p->type == A)
    {
      int32_t cur = position_get_angle(t->pm) - position_get_angle_mod2pi(t->pm);
      int32_t cons = trajectory_get_nearest_angle(t,cur + p->a.a);
      if(abs(cons - position_get_angle(t->pm)) < p->a.prec_a)
      {
        trajectory_next(t);
      }
      else
      {
        asserv_set_angle(t->ass,cons);
      }

    }
    else if(p->type == XYBACK || p->type == XYPIONBACK)
    {
      int32_t tarx = p->xy.x - position_get_x(t->pm);
      int32_t tary = p->xy.y - position_get_y(t->pm);
      if(tarx * tarx + tary * tary < (p->xya.prec_xy * p->xya.prec_xy)) //yavait pas le carré sur la précision avant !
      { 
        trajectory_next(t);
      }
      else
      {
        int32_t acible = position_rad2imp(t->pm,fxx_from_double(atan2(tary,tarx)+M_PI));
        int32_t cur = position_get_angle(t->pm) - position_get_angle_mod2pi(t->pm);
        int32_t acons = trajectory_get_nearest_angle(t,cur + acible);
        asserv_set_angle(t->ass,acons);
        if(abs(acons - position_get_angle(t->pm)) < p->xy.prec_ap)
        {
          int32_t dcons = -sqrt(tarx*tarx + tary*tary);
          asserv_set_distance(t->ass,dcons + position_get_distance(t->pm));
        }
        else
        {
          asserv_set_distance(t->ass,position_get_distance(t->pm));
        }
      }
    }
    else if(p->type == XY || p->type == XYPION)
    {
      double tarx = p->xy.x - position_get_x(t->pm);
      double tary = p->xy.y - position_get_y(t->pm);
      if(tarx * tarx + tary * tary < p->xy.prec_xy) { 
        trajectory_next(t);
      }
      else {
	int32_t acible = position_rad2imp(t->pm,fxx_from_double(atan2(tary,tarx)));
	int32_t cur = position_get_angle(t->pm) - position_get_angle_mod2pi(t->pm);
	int32_t acons = trajectory_get_nearest_angle(t,cur + acible);

	if(t->directed_flag) {
	  int32_t dcons = sqrt(tarx*tarx + tary*tary);
	  asserv_set_angle(t->ass,acons);
	  asserv_set_distance(t->ass, dcons + position_get_distance(t->pm));
	}
	else {	  
	  asserv_set_angle(t->ass,acons);
	  asserv_set_distance(t->ass, position_get_distance(t->pm));
	  if(abs(acons - position_get_angle(t->pm)) < p->xy.prec_ap)
	    t->directed_flag = 1;
	}
      }
    }
    else if(p->type == XYA)
    {
      double tarx = p->xya.x - position_get_x(t->pm);
      double tary = p->xya.y - position_get_y(t->pm);
      if(tarx * tarx + tary * tary < (p->xya.prec_xy * p->xya.prec_xy)) //cété pas au carré
      { 
        int32_t cur = position_get_angle(t->pm) - position_get_angle_mod2pi(t->pm);
        int32_t cons = trajectory_get_nearest_angle(t,cur + p->xya.a);
        if(abs(cons - position_get_angle(t->pm)) < p->xya.prec_a)
        {
          trajectory_next(t);
        }
        else
        {
          asserv_set_angle(t->ass,cons);
        }
      }
      else
      {
        int32_t acible = position_rad2imp(t->pm,fxx_from_double(atan2(tary,tarx)));
        int32_t cur = position_get_angle(t->pm) - position_get_angle_mod2pi(t->pm);
        int32_t acons = trajectory_get_nearest_angle(t,cur + acible);
        asserv_set_angle(t->ass,acons);
        if(abs(acons - position_get_angle(t->pm)) < p->xya.prec_ap)
        {
          int32_t dcons = sqrt(tarx*tarx + tary*tary);
          asserv_set_distance(t->ass,dcons + position_get_distance(t->pm));
        }
        else
        {
          asserv_set_distance(t->ass,position_get_distance(t->pm));
        }
      }
    }
    else if(p->type == WAIT)
    {
//          asserv_set_distance(t->ass,position_get_distance(t->pm));
  //        asserv_set_angle(t->ass,position_get_angle(t->pm));
            asserv_set_distance(t->ass,p->startingD);
            asserv_set_angle(t->ass,p->startingA);
      //bloutre !
    }
    else if(p->type == STEP)
    {
//      asserv_set_distance(t->ass,position_get_distance(t->pm));
//      asserv_set_angle(t->ass,position_get_angle(t->pm));
      asserv_set_distance(t->ass,p->startingD);
      asserv_set_angle(t->ass,p->startingA);

      trajectory_next(t);
      printf("END STEOP\n");
      //notification_notify(t->not,NOTIFICATION_TRAJECTORY_END_STEP,NULL);
    }
    else //step
    {
      trajectory_next(t);
    }
  }
  else
  {

  //  asserv_set_distance(t->ass,position_get_distance(t->pm));
  //  asserv_set_angle(t->ass,position_get_angle(t->pm));
    //notification_notify(t->not,NOTIFICATION_TRAJECTORY_END_FINAL,NULL);
  }
}

int8_t get_couleur_depart(void)
{
  return couleur_depart;
}

void traj_set_couleur_depart(int8_t couleur)
{
  couleur_depart = couleur;
}

void trajectory_goto_xy_pion(trajectory_manager_t *t,trajectory_order_when_t when, double x, double y)
{
  //y = y * get_couleur_depart();
  //printf("GOTO xy %f %f %d\n",x,y,when);
  trajectory_dest_t dest;
  dest.type = XYPION;
  dest.xy.x = position_cm2imp(t->pm,fxx_from_double(x));
  dest.xy.y = position_cm2imp(t->pm,fxx_from_double(y));
  dest.xy.prec_xy = position_cm2imp(t->pm,DEFAULT_PREC_DISTANCE);
  dest.xy.prec_xy = dest.xy.prec_xy * dest.xy.prec_xy;
  dest.xy.prec_ap = position_deg2imp(t->pm,DEFAULT_PREC_ANGLE_P);
  //printf("DEBUG %f %f\n",x,y);
  return trajectory_add_point(t,when,dest);
}

void trajectory_goto_xy_backward(trajectory_manager_t *t,trajectory_order_when_t when, double x, double y)
{
  //y = y * get_couleur_depart();
  //printf("GOTO xy %f %f %d\n",x,y,when);
  trajectory_dest_t dest;
  dest.type = XYBACK;
  dest.xy.x = position_cm2imp(t->pm,fxx_from_double(x));
  dest.xy.y = position_cm2imp(t->pm,fxx_from_double(y));
  dest.xy.prec_xy = position_cm2imp(t->pm,DEFAULT_PREC_DISTANCE);
  dest.xy.prec_xy = dest.xy.prec_xy * dest.xy.prec_xy;
  dest.xy.prec_ap = position_deg2imp(t->pm,DEFAULT_PREC_ANGLE_P);
  //printf("DEBUG %f %f\n",x,y);
  return trajectory_add_point(t,when,dest);

}

void trajectory_goto_xy_pion_backward(trajectory_manager_t *t,trajectory_order_when_t when, double x, double y)
{
  //y = y * get_couleur_depart();
  //printf("GOTO xy %f %f %d\n",x,y,when);
  trajectory_dest_t dest;
  dest.type = XYPIONBACK;
  dest.xy.x = position_cm2imp(t->pm,fxx_from_double(x));
  dest.xy.y = position_cm2imp(t->pm,fxx_from_double(y));
  dest.xy.prec_xy = position_cm2imp(t->pm,DEFAULT_PREC_DISTANCE);
  dest.xy.prec_xy = dest.xy.prec_xy * dest.xy.prec_xy;
  dest.xy.prec_ap = position_deg2imp(t->pm,DEFAULT_PREC_ANGLE_P);
  //printf("DEBUG %f %f\n",x,y);
  return trajectory_add_point(t,when,dest);
}


// EIRBUG //

int8_t trajectory_goto_pos(trajectory_manager_t* t, trajectory_order_when_t when, int16_t x, int16_t y, int16_t a) {
  int16_t actual_pos_x,actual_pos_y,actual_pos_a;
  position_abs(t->pm, &actual_pos_x, &actual_pos_y, &actual_pos_a);
  x -= actual_pos_x;
  y -= actual_pos_y;
  y = -y;
  //a = ( a - actual_pos_a + 360)%360; // Euuhh.. à supprimer

  double ia = atan2((double)y,(double)x)*180/M_PI - actual_pos_a;
  double d = sqrt(x*x + y*y);
  double fa = (double)a;

  trajectory_goto_arel(t, when, ia);
  trajectory_goto_d(t, when, d);
  trajectory_goto_a(t, when, fa);
}
