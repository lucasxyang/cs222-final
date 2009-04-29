
#ifndef _TRANSFORM_H
#define _TRANSFORM_H

enum {
  CDF22=22,
  DEPTH=2,
  LEVELS=DEPTH+1,
  TL = 128,
  PADDING = 1<<(DEPTH),
  LENGTH=TL + 2*PADDING,

  TRANSFORM_MASK = TL-1,

  SYM=1, // symmetric boundary value extension (default)
  ZPD=2, // zero padding
  EXP=3  // explicit padding
};

#endif
