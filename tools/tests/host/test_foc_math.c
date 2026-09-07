#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "config.h"
#include "motor/foc_math.h"
static int fail(const char *m){fprintf(stderr,"FAIL %s\n",m);return 1;}

int main(void){
  int16_t s=0,c=0; foc_sin_cos_q15(0,&s,&c); if(s!=0||c!=32767)return fail("sin/cos cardinal 0");
  foc_sin_cos_q15(16384u,&s,&c); if(s!=32767||c!=0)return fail("sin/cos cardinal 90");
  foc_sin_cos_q15(32768u,&s,&c); if(s!=0||c!=-32767)return fail("sin/cos cardinal 180");
  foc_sin_cos_q15(49152u,&s,&c); if(s!=-32767||c!=0)return fail("sin/cos cardinal 270");
  int max_trig_err=0;
  for(uint32_t ph=0;ph<65536u;ph++){
    foc_sin_cos_q15((uint16_t)ph,&s,&c);
    const double a=6.28318530717958647692*(double)ph/65536.0;
    const int sr=(int)lround(sin(a)*32767.0), cr=(int)lround(cos(a)*32767.0);
    const int es=abs((int)s-sr), ec=abs((int)c-cr);
    if(es>max_trig_err)max_trig_err=es;
    if(ec>max_trig_err)max_trig_err=ec;
    if(es>4||ec>4)return fail("sin/cos LUT interpolation error");
  }
  foc_ab_t ab={1200,-700},ab2={0,0}; foc_dq_t dq={0,0};
  foc_park_q4(&ab,10000,&dq); foc_inv_park(&dq,10000,&ab2);
  if(abs(ab.alpha-ab2.alpha)>3||abs(ab.beta-ab2.beta)>3)return fail("park roundtrip");
  foc_dq_t v={20000,20000};foc_vector_limit(&v,14400);
  uint32_t mag=(uint32_t)((int32_t)v.d*v.d+(int32_t)v.q*v.q); if(mag>(uint32_t)14420u*14420u)return fail("vector limit");
  foc_abc_t pwm;foc_centered_svpwm(&v,12345,&pwm);
  if(abs(pwm.a)>1000||abs(pwm.b)>1000||abs(pwm.c)>1000)return fail("svpwm range");
  /* Prove both limits separately: the EFeru electrical ceiling itself and the
   * board-specific VESC normalization chosen in config.h. */
  int max_abs=0, max_span=0;
  foc_dq_t vmax={FOC_SVPWM_VECTOR_FULL_SAFE,0};
  for(int deg=0;deg<360;deg++){
    foc_abc_t x;
    uint16_t ph=(uint16_t)(((uint32_t)deg*65536u)/360u);
    foc_centered_svpwm(&vmax,ph,&x);
    int vals[3]={x.a,x.b,x.c};
    int mx=vals[0],mn=vals[0];
    for(int j=0;j<3;j++){int av=abs(vals[j]);if(av>max_abs)max_abs=av;if(vals[j]>mx)mx=vals[j];if(vals[j]<mn)mn=vals[j];}
    if(mx-mn>max_span)max_span=mx-mn;
    if(mx>890||mn<-890)return fail("EFeru full-safe PWM exceeds +/-890");
  }
  if(max_abs<887||max_abs>890)return fail("EFeru full-safe PWM does not reach physical ceiling");
  int scaled_abs=0;
  foc_dq_t vscaled={FOC_SVPWM_VECTOR_MAX,0};
  for(int deg=0;deg<360;deg++){
    foc_abc_t x; uint16_t ph=(uint16_t)(((uint32_t)deg*65536u)/360u);
    foc_centered_svpwm(&vscaled,ph,&x);
    int vals[3]={x.a,x.b,x.c};
    for(int j=0;j<3;j++){int av=abs(vals[j]);if(av>scaled_abs)scaled_abs=av;}
  }
  if(FOC_SVPWM_VECTOR_MAX!=(FOC_SVPWM_VECTOR_FULL_SAFE*VESC_DUTY_PHYSICAL_SCALE_PERMILLE)/1000)
    return fail("config duty scale arithmetic");
  if(scaled_abs<850||scaled_abs>856)return fail("0.960 physical duty scale PWM range");
  printf("FOC_FIXEDPOINT_RUNTIME_PASS lut_max_err=%d pwm=%d,%d,%d eferu_max_abs=%d scaled960_abs=%d span=%d\n",max_trig_err,pwm.a,pwm.b,pwm.c,max_abs,scaled_abs,max_span);
  return 0;
}
