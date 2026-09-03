#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "motor/foc_math.h"
static int fail(const char *m){fprintf(stderr,"FAIL %s\n",m);return 1;}

static int16_t ref_sat(int32_t x){return x>32767?32767:(x<-32768?-32768:(int16_t)x);}
static int16_t ref_pi(foc_pi_fixed_t *pi,int16_t err,uint16_t kp,uint16_t ki,int16_t hi,int16_t lo){
  const int32_t iadd=(int32_t)err*ki;
  const int32_t isum=pi->integrator+(pi->sat_hold?0:iadd);
  const int16_t p=ref_sat(((int32_t)err*kp)>>11);
  const int16_t raw=ref_sat((((isum>>16)<<1)+(int32_t)p)>>1);
  const int hi_sat=raw>hi,lo_sat=raw<lo;
  const int16_t out=hi_sat?hi:(lo_sat?lo:raw);
  const int si=(iadd>0)-(iadd<0), sr=(raw>0)-(raw<0);
  pi->sat_hold=(uint8_t)((si==sr)&&(hi_sat||lo_sat));
  pi->integrator=isum;
  return out;
}

int main(void){
  int16_t s=0,c=0; foc_sin_cos_q15(0,&s,&c); if(abs(s)>2||c<32760)return fail("sin0");
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
  foc_pi_fixed_t pi={0,0},ref={0,0};int16_t out=0;
  for(int k=0;k<1000;k++){
    const int16_t e=(int16_t)(((k*977)%24001)-12000);
    const int16_t a=foc_pi_run(&pi,e,1229,1229,14400,-14400);
    const int16_t b=ref_pi(&ref,e,1229,1229,14400,-14400);
    if(a!=b||pi.integrator!=ref.integrator||pi.sat_hold!=ref.sat_hold)return fail("PI generated-model equivalence");
    out=a;
  }
  foc_pi_fixed_t first={0,0}; if(foc_pi_run(&first,800,1229,1229,14400,-14400)!=255)return fail("PI first-step expected 255");
  printf("FOC_FIXEDPOINT_RUNTIME_PASS pwm=%d,%d,%d eferu_max_abs=%d scaled960_abs=%d span=%d pi_last=%d pi_first=255\n",pwm.a,pwm.b,pwm.c,max_abs,scaled_abs,max_span,out);
  return 0;
}
