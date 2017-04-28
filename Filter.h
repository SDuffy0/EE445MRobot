
#ifndef __Filter_H__
#define __Filter_H__ 1

#define MEDIAN_FILTER 1


#ifndef MEDIAN_FILTER
#define AddFilter(NAME) \
uint16_t Filter ## NAME(uint16_t data){ \
static long x ## NAME[6]; \
static long y ## NAME[6]; \
static unsigned long n ## NAME=3; \
  n ## NAME++; \
  if(n ## NAME==6) n ## NAME=3;     \
  x ## NAME[n ## NAME] = x ## NAME[n ## NAME-3] = data; \
  y ## NAME[n ## NAME] = (256*(x ## NAME[n ## NAME]+x ## NAME[n ## NAME-2])-503*x ## NAME[n ## NAME-1]+498*y ## NAME[n ## NAME-1]-251*y ## NAME[n ## NAME-2]+128)/256; \
  y ## NAME[n ## NAME-3] = y ## NAME[n ## NAME];    \
  return y ## NAME[n ## NAME]; \
} \

#else

#define AddFilter(NAME)\
long Filter ## NAME(long y){ \
	static long x ## NAME[2];\
  long result ## NAME;\
  if(y>x ## NAME[0])\
    if(x ## NAME[0]>x ## NAME[1])   result ## NAME=x ## NAME[0];     \
      else\
        if(y>x ## NAME[1]) result ## NAME=x ## NAME[1];   \
        else      result ## NAME=y;   \
  else \
    if(x ## NAME[1]>x ## NAME[0])   result ## NAME=x ## NAME[0];     \
      else\
        if(y>x ## NAME[1]) result ## NAME=y;   \
        else      result ## NAME=x ## NAME[1];   \
	x ## NAME[1] = x ## NAME[0];\
	x ## NAME[0] = y;\
  return(result ## NAME);\
}\

#endif

#endif
