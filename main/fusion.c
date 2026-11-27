#include "fusion.h"
#include "config.h"
#include <math.h>
#include <string.h>
void fusion_reset(fusion_t *s){memset(s,0,sizeof(*s));s->q0=1.0f;}
static void madgwick_update(fusion_t *state,float gx,float gy,float gz,float ax,float ay,float az,float dt){
    float norm,s0,s1,s2,s3;
    float qDot0=0.5f*(-state->q1*gx-state->q2*gy-state->q3*gz);
    float qDot1=0.5f*(state->q0*gx+state->q2*gz-state->q3*gy);
    float qDot2=0.5f*(state->q0*gy-state->q1*gz+state->q3*gx);
    float qDot3=0.5f*(state->q0*gz+state->q1*gy-state->q2*gx);
    norm=sqrtf(ax*ax+ay*ay+az*az);
    if(norm>0.000001f){
        ax/=norm;ay/=norm;az/=norm;
        float _2q0=2.0f*state->q0,_2q1=2.0f*state->q1,_2q2=2.0f*state->q2,_2q3=2.0f*state->q3;
        float _4q0=4.0f*state->q0,_4q1=4.0f*state->q1,_4q2=4.0f*state->q2;
        float _8q1=8.0f*state->q1,_8q2=8.0f*state->q2;
        float q0q0=state->q0*state->q0,q1q1=state->q1*state->q1,q2q2=state->q2*state->q2,q3q3=state->q3*state->q3;
        s0=_4q0*q2q2+_2q2*ax+_4q0*q1q1-_2q1*ay;
        s1=_4q1*q3q3-_2q3*ax+4.0f*q0q0*state->q1-_2q0*ay-_4q1+_8q1*q1q1+_8q1*q2q2+_4q1*az;
        s2=4.0f*q0q0*state->q2+_2q0*ax+_4q2*q3q3-_2q3*ay-_4q2+_8q2*q1q1+_8q2*q2q2+_4q2*az;
        s3=4.0f*q1q1*state->q3-_2q1*ax+4.0f*q2q2*state->q3-_2q2*ay;
        norm=sqrtf(s0*s0+s1*s1+s2*s2+s3*s3);
        if(norm>0.000001f){
            s0/=norm;s1/=norm;s2/=norm;s3/=norm;
            qDot0-=MADGWICK_BETA*s0;qDot1-=MADGWICK_BETA*s1;
            qDot2-=MADGWICK_BETA*s2;qDot3-=MADGWICK_BETA*s3;
        }
    }
    state->q0+=qDot0*dt;state->q1+=qDot1*dt;state->q2+=qDot2*dt;state->q3+=qDot3*dt;
    norm=sqrtf(state->q0*state->q0+state->q1*state->q1+state->q2*state->q2+state->q3*state->q3);
    if(norm>0.000001f){state->q0/=norm;state->q1/=norm;state->q2/=norm;state->q3/=norm;}
}
static void calculate_linear_acceleration(fusion_t *s,float ax,float ay,float az){
    float gravity_x=2.0f*(s->q1*s->q3-s->q0*s->q2);
    float gravity_y=2.0f*(s->q0*s->q1+s->q2*s->q3);
    float gravity_z=1.0f-2.0f*(s->q1*s->q1+s->q2*s->q2);
    s->linear_accel_x=(ax-gravity_x)*GRAVITY;
    s->linear_accel_y=(ay-gravity_y)*GRAVITY;
    s->linear_accel_z=(az-gravity_z)*GRAVITY;
}
void fusion_update(fusion_t *s,const imu_sample_t *raw,float dt){
    if(dt<=0.0f||dt>0.1f)dt=0.02f;
    madgwick_update(s,(raw->gx-s->gyro_offset[0])*DEG_TO_RAD,
        (raw->gy-s->gyro_offset[1])*DEG_TO_RAD,
        (raw->gz-s->gyro_offset[2])*DEG_TO_RAD,
        raw->ax,raw->ay,raw->az,dt);
    calculate_linear_acceleration(s,raw->ax,raw->ay,raw->az);
}
