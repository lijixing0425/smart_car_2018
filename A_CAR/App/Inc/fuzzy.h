#ifndef _FUZZ_H_
#define _FUZZ_H_

extern float FTri(float x,float a,float b,float c);
extern float FTraL(float x,float a,float b);
extern float FTraR(float x,float a,float b);
extern float fand(float a,float b);
extern float FUZZP_GET(int a,int b);
extern float  FUZZD_GET(int a,int b);
extern float FUZZ_SPEED_GET(int a, int b);
#endif