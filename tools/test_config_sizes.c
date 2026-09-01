#include <stdio.h>
#include <string.h>
#include "vesc/mcconf_serial.h"
int main(void){
  static uint8_t b[2048]; mc_configuration m; app_configuration a;
  memset(&m,0,sizeof(m)); memset(&a,0,sizeof(a));
  int32_t mn=confgenerator_serialize_mcconf(b,&m);
  int32_t an=confgenerator_serialize_appconf(b,&a);
  printf("mcconf=%ld appconf=%ld\n",(long)mn,(long)an);
  return (mn>0 && mn<=700 && an>0 && an<=700)?0:1;
}
