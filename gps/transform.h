#ifndef TRANSFORM_HEADER
#define TRANSFORM_HEADER

void wgs2gcj(double wgsLat, double wgsLng, double *gcjLat, double *gcjLng);
void gcj2wgs(double gcjLat, double gcjLng, double *wgsLat, double *wgsLng);
void gcj2bd(double gcjLat, double gcjLng, double *bdLat, double *bdLng);
void bd2gcj(double bdLat, double bdLng, double *gcjLat, double *gcjLng);
void wgs2bd(double wgsLat, double wgsLng, double *bdLat, double *bdLng);
void bd2wgs(double bdLat, double bdLng, double *wgsLat, double *wgsLng);

void gcj2wgs_exact(double gcjLat, double gcjLng, double *wgsLat, double *wgsLng);
double distance(double latA, double lngA, double latB, double lngB);

#endif