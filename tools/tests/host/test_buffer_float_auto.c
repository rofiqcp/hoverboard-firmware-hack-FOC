#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "vesc/buffer.h"

static int fail(const char *m,uint32_t u){fprintf(stderr,"FAIL %s 0x%08lx\n",m,(unsigned long)u);return 1;}
static uint32_t bitsf(float f){uint32_t u;memcpy(&u,&f,sizeof(u));return u;}
static float fbits(uint32_t u){float f;memcpy(&f,&u,sizeof(f));return f;}

int main(void){
    const uint32_t fixed[]={0x00000000u,0x3f800000u,0xbf800000u,
        0x41200000u,0xc2480000u,0x01000000u,0x7f7fffffu,0x3dcccccdu,0x447a0000u};
    uint8_t b[4];
    for(unsigned n=0;n<sizeof(fixed)/sizeof(fixed[0]);++n){
        const uint32_t u=fixed[n]; int32_t i=0; buffer_append_float32_auto(b,fbits(u),&i);
        const uint32_t enc=((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3];
        if(enc!=u)return fail("fixed encode",u);
        i=0;if(bitsf(buffer_get_float32_auto(b,&i))!=u)return fail("fixed decode",u);
    }
    uint32_t r=0x13579bdfu; unsigned checked=0u;
    for(unsigned n=0;n<200000u;++n){
        r=r*1664525u+1013904223u;
        const uint32_t exp=(r>>23)&0xffu;
        if(exp==0u||exp==0xffu||fabsf(fbits(r))<1.5e-38f)continue; /* VESC codec sengaja mengkanonisasi nilai sangat kecil dan tidak mewakili NaN/Inf. */
        int32_t i=0;buffer_append_float32_auto(b,fbits(r),&i);
        const uint32_t enc=((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3];
        if(enc!=r)return fail("random encode",r);
        i=0;if(bitsf(buffer_get_float32_auto(b,&i))!=r)return fail("random decode",r);
        ++checked;
    }
    int32_t i=0;buffer_append_float32_auto(b,fbits(0x00000001u),&i);
    if(b[0]||b[1]||b[2]||b[3])return fail("subnormal canonical zero",1u);
    i=0;buffer_append_float32_auto(b,fbits(0x80000000u),&i);
    if(b[0]||b[1]||b[2]||b[3])return fail("negative zero canonical zero",0x80000000u);
    printf("BUFFER_FLOAT_AUTO_PASS checked=%u\n",checked);
    return 0;
}
