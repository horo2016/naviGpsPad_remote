#ifndef GPS_H
#define GPS_H
#ifdef __cplusplus
extern "C" {
#endif

extern double latitude ;

extern double longitude;
extern unsigned int gpsheading ;
extern unsigned int gpsvelocity ;

extern 
void *GpsThread(void *);
#ifdef __cplusplus
}
#endif

#endif
