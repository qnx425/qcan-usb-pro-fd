#include <stdint.h>

const uint8_t pcan_fd_dlc2len[] = 
{
  0, 1, 2, 3, 4, 5, 6, 7,
  8, 12, 16, 20, 24, 32, 48, 64
};

const uint8_t pcan_fd_len2dlc[] = 
{
  0, 1, 2, 3, 4, 5, 6, 7, 8,	/* 0 - 8 */
  9, 9, 9, 9,	    /* 9 - 12 */
  10, 10, 10, 10,	    /* 13 - 16 */
  11, 11, 11, 11,	    /* 17 - 20 */
  12, 12, 12, 12,	    /* 21 - 24 */
  13, 13, 13, 13, 13, 13, 13, 13,	/* 25 - 32 */
  14, 14, 14, 14, 14, 14, 14, 14,	/* 33 - 40 */
  14, 14, 14, 14, 14, 14, 14, 14,	/* 41 - 48 */
  15, 15, 15, 15, 15, 15, 15, 15,	/* 49 - 56 */
  15, 15, 15, 15, 15, 15, 15, 15	/* 57 - 64 */
};
