/* 
 * File:   gps.h
 * Author: Christopher Madrigal
 *
 * Created on August 1, 2019, 11:49 AM
 */

#include "mcc_generated_files/i2c2.h"

#ifndef GPS_H
#define	GPS_H

#ifdef	__cplusplus
extern "C" {
#endif

// I2C address of the GPS device (M8)
#define GPS_ADDRESS  0x42
#define REG_NUM_HIGH 0xFD
#define REG_NUM_LOW  0xFE
#define REG_DATA     0xFF
    
/* These are the formats for the GPS data
 * 
 * $GPGGA,time,lat,NS,lon,EW,quality,numSV,HDOP,alt,altUnit,sep,sepUnit,diffAge,diffStation*cs<CR><LF>
 * $GPRMC,time,status,lat,NS,lon,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>
 */

typedef struct {
  double  latitude;   //latitude in DDDmm.mm
  char    lat_dir;    //N or S
  double  longitude;  //longitude in same format
  char    long_dir;   //E or W
  uint8_t quality;    //quality on a scale of 0 to 2
  uint8_t satellites; //number of sattelites used
  double  altitude;   //altitude in meters above mean sea level
  
  double  speed;      //speed over ground in knots
  double  course;     //track angle in deg. True
  uint8_t hour;       //UTC hour
  uint8_t minute;     //UTC minute
  uint8_t second;     //UTC second
} GPSFIX;

typedef enum {
    GGA,
    RMC,
    INVALID
} NMEA_TYPE;

/* Init the GPS with the desired configuration */
void gps_init();

/* Update the GPS as it can */
bool gps_update();

/* Get the last valid GPS data from the GPS */
void gps_fix( GPSFIX* fix );

/* Read an NMEA sentence from the incoming stream */
bool gps_readline( char* line, uint8_t n );

/* Check if the given NMEA sentence is valid,
 * and return its type */
NMEA_TYPE gps_validate( char* sentence );

/* Parse the GGA and RMC sentences */
bool gps_parse_gga( char* sentence, GPSFIX* fix );
bool gps_parse_rmc( char* sentence, GPSFIX* fix );

#ifdef	__cplusplus
}
#endif

#endif	/* GPS_H */

