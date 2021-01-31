#ifndef GPS_HAL_h
#define GPS_HAL_h


#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    double lng;
    double lat;
} Location;

extern void GpsHandle();


#ifdef __cplusplus
}
#endif
#endif
