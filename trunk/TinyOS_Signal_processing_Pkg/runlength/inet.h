#ifndef __INET_H
#define __INET_H

#define __LITTLE_ENDIAN 1234
#define __BIG_ENDIAN    4321

#ifndef PLATFORM_PC

/* the avr is little-endian */
#define __BYTE_ORDER __LITTLE_ENDIAN

#define __bswap_16(x) \
     ((((x) >> 8) & 0xff) | (((x) & 0xff) << 8))

#define __bswap_32(x) \
     ((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) | \
      (((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24))


# if __BYTE_ORDER == __BIG_ENDIAN
/* The host byte order is the same as network byte order,
   so these functions are all just identity.  */
# define ntohl(x)       (x)
# define ntohs(x)       (x)
# define htonl(x)       (x)
# define htons(x)       (x)
# else
#  if __BYTE_ORDER == __LITTLE_ENDIAN
#   define ntoh32(x)     __bswap_32 (x)
#   define ntoh16(x)     __bswap_16 (x)
#   define hton32(x)     __bswap_32 (x)
#   define hton16(x)     __bswap_16 (x)
#  endif
# endif

#else

#   define ntoh32(x)     ntohl(x)
#   define ntoh16(x)     ntohs(x)
#   define hton32(x)     htonl(x)
#   define hton16(x)     htons(x)

#endif

#endif
