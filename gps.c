#include "gps.h"
#include <string.h>
#include <stdlib.h>

//configuration settings
#define TIMEOUT_TICKS 1000000000
char CONF_PROTOCOL_BAUD[] = "$PUBX,41,0,0003,0002,19200,0*21\r\n";
char CONF_ENABLE_GPRMC[]  = "$PUBX,40,RMC,1,0,0,0,0,0*46\r\n";
char CONF_ENABLE_GPGGA[]  = "$PUBX,40,GGA,1,0,0,0,0,0*5B\r\n";
char CONF_DISABLE_GPGBS[] = "$PUBX,40,GBS,0,0,0,0,0,0*4D\r\n";
char CONF_DISABLE_GPGLL[] = "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n";
char CONF_DISABLE_GPGNS[] = "$PUBX,40,GNS,0,0,0,0,0,0*41\r\n";
char CONF_DISABLE_GPGRS[] = "$PUBX,40,GRS,0,0,0,0,0,0*5D\r\n";
char CONF_DISABLE_GPGSA[] = "$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n";
char CONF_DISABLE_GPGST[] = "$PUBX,40,GST,0,0,0,0,0,0*5B\r\n";
char CONF_DISABLE_GPGSV[] = "$PUBX,40,GSV,0,0,0,0,0,0*59\r\n";
char CONF_DISABLE_GPTXT[] = "$PUBX,40,TXT,0,0,0,0,0,0*43\r\n";
char CONF_DISABLE_GPVLW[] = "$PUBX,40,VLW,0,0,0,0,0,0*56\r\n";
char CONF_DISABLE_GPVTG[] = "$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n";
char CONF_DISABLE_GPZDA[] = "$PUBX,40,ZDA,0,0,0,0,0,0*44\r\n";

GPSFIX lastFix;

bool blocking_read( uint8_t *data, uint8_t n ) {
  
  //status that we check during transmission
  //uint32_t tick = 0;
  I2C2_MESSAGE_STATUS reqStatus = I2C2_MESSAGE_PENDING;
  I2C2_MasterRead( data, n, GPS_ADDRESS, &reqStatus );
  while( reqStatus == I2C2_MESSAGE_PENDING /*&& tick < TIMEOUT_TICKS*/ ) {
    //tick++;
  }
  return ( reqStatus == I2C2_MESSAGE_COMPLETE );
}

bool blocking_write( uint8_t *data, uint8_t n ) {
  
  //status that we check during transmission
  //uint32_t tick = 0;
  I2C2_MESSAGE_STATUS reqStatus = I2C2_MESSAGE_PENDING;
  I2C2_MasterWrite( data, n, GPS_ADDRESS, &reqStatus );
  while( reqStatus == I2C2_MESSAGE_PENDING /*&& tick < TIMEOUT_TICKS*/ ) {
    //tick++;
  }
  return ( reqStatus == I2C2_MESSAGE_COMPLETE );
}

/* Update the GPS as it can */
bool gps_update() {

  GPSFIX nextFix; //the next fix of the GPS
  char buffer[100]; //a buffer to hold the sentences
  
  //first we grab the line
  gps_readline( buffer, 100 );
  
  //printf( buffer ); printf( "\r\n" );
  
  //if this line is an GNRMC sentence, then decode it
  if( gps_validate( buffer ) == RMC ) {
    
    //if it was a valid fix, move onto GNGGA
    if( gps_parse_rmc( buffer, &nextFix ) ) {
      
      //grab the next sentence
      gps_readline( buffer, 100 );
      //printf( buffer ); printf( "\r\n" );
      
      //if the line is a GGA sentence, then decode it
      if( gps_validate( buffer ) == GGA ) {
        
        //if it was a valid fix, then we're done!
        if( gps_parse_gga( buffer, &nextFix) ) {
          lastFix = nextFix;
          return true;
        } //else printf( "INVALID GGA AFTER RMC\n" );
      } //else printf( "DID NOT GET GGA AFTER RMC\n" );
    } //else printf( "INVALID RMC FIX\n");
  }
  
  //if this line is a GNGGA sentence, then decode it
  else if( gps_validate( buffer ) == GGA ) {
    
    //if it was a valid fix, move onto GNRMC
    if( gps_parse_gga( buffer, &nextFix ) ) {
      
      //grab the next sentence
      gps_readline( buffer, 100 );
      //printf( buffer ); printf( "\r\n" );
      
      //if the line is an RMC sentence, then decode it
      if( gps_validate( buffer ) == RMC ) {
        
        //if it was a valid fix, then we're done!
        if( gps_parse_rmc( buffer, &nextFix) ) {
          lastFix = nextFix;
          return true;
        } //else printf( "INVALID RMC AFTER GGA\n" );
      } //else printf( "DID NOT GET RMC AFTER GGA\n" );
    } //else printf( "INVALID GGA FIX\n");
  } //else printf( "STEPPED INTO INVALID FIX\n" );
  
  //we will always fall back here if the update failed
  return false;
}

/* Read an NMEA sentence from the incoming stream 
 * Recommended at least 83 characters
 */
bool gps_readline( char* line, uint8_t n ) {
  
  //keep track of the prior exchange, a $ may have been gobbled up
  static uint8_t lastChar = 0;
  
  if( n == 0 ) return true; //nonzero size
  if( n == 1 ) { //only a null terminator saved
    line[0] = 0;
    return true;
  }
  
  //keep track of the last character read, and the index into the line
  uint8_t i = 0;
  n--;
  
  //keep skipping until the first $
  while( lastChar != '$' ) {
    if( !blocking_read( &lastChar, 1 ) ) return false;
  }
  
  //now read the sentence, which ends in \r\n
  do {
    //skip empty data... ÿ should never appear
    if( lastChar != 0xFF ) {
      line[i] = lastChar;
      i++;
      line[i] = 0; //follow line with null
    }
    
    //one char at a time
    if( !blocking_read( &lastChar, 1 ) ) return false;
  } while( lastChar != '\r' && lastChar != '$' && i < n );
  
  //either i < n, lastChar = \r, or lastChar = $
  
  return true;
}

/* Init the GPS with the desired configuration */
void gps_init() {
  lastFix.latitude = 0.0;
  lastFix.longitude = 0.0;
  lastFix.altitude = 0.0;
  lastFix.course = 0.0;
  lastFix.speed = 0.0;
  lastFix.hour = 0;
  lastFix.minute = 0;
  lastFix.second = 0;
  lastFix.quality = 0;
  lastFix.satellites = 0;
  lastFix.long_dir = '?';
  lastFix.lat_dir = '?';
  
  blocking_write( CONF_PROTOCOL_BAUD, sizeof( CONF_PROTOCOL_BAUD ) - 1 );
  blocking_write( CONF_DISABLE_GPGBS, sizeof( CONF_DISABLE_GPGBS ) - 1 );
  blocking_write( CONF_DISABLE_GPGLL, sizeof( CONF_DISABLE_GPGLL ) - 1 );
  blocking_write( CONF_DISABLE_GPGNS, sizeof( CONF_DISABLE_GPGNS ) - 1 );
  blocking_write( CONF_DISABLE_GPGRS, sizeof( CONF_DISABLE_GPGRS ) - 1 );
  blocking_write( CONF_DISABLE_GPGSA, sizeof( CONF_DISABLE_GPGSA ) - 1 );
  blocking_write( CONF_DISABLE_GPGST, sizeof( CONF_DISABLE_GPGST ) - 1 );
  blocking_write( CONF_DISABLE_GPGSV, sizeof( CONF_DISABLE_GPGSV ) - 1 );
  blocking_write( CONF_DISABLE_GPTXT, sizeof( CONF_DISABLE_GPTXT ) - 1 );
  blocking_write( CONF_DISABLE_GPVLW, sizeof( CONF_DISABLE_GPVLW ) - 1 );
  blocking_write( CONF_DISABLE_GPVTG, sizeof( CONF_DISABLE_GPVTG ) - 1 );
  blocking_write( CONF_DISABLE_GPZDA, sizeof( CONF_DISABLE_GPZDA ) - 1 );
}

/* Get the last valid GPS data from the GPS */
void gps_fix( GPSFIX* fix ) {
  fix->latitude = lastFix.latitude;
  fix->longitude = lastFix.longitude;
  fix->altitude = lastFix.altitude;
  fix->course = lastFix.course;
  fix->speed = lastFix.speed;
  fix->hour = lastFix.hour;
  fix->minute = lastFix.minute;
  fix->second = lastFix.second;
  fix->quality = lastFix.quality;
  fix->satellites = lastFix.satellites;
  fix->long_dir = lastFix.long_dir;
  fix->lat_dir = lastFix.lat_dir;
}

/* Check if the given NMEA sentence is valid,
 * and return its type */
NMEA_TYPE gps_validate( char* sentence ) {
  
  //all NMEA sentences start with $
  if( sentence[0] != '$' ) return INVALID;
  
  //find the checksum
  char *b = strchr( sentence, '*' );
  if( b == 0 ) return INVALID;
  uint8_t checksum = (uint8_t) strtol( b + 1, 0, 16 );
  
  //calculate the checksum from the rest of the string
  uint8_t sum = 0;
  b = sentence + 1; //skip '$'
  char c;
  while( (c = *b) != 0 && c != '*' ) {
    if( c != ',' ) sum ^= c;
    b++;
  }
  
  //compare the checksums
  if( sum != checksum ) return INVALID;
  
  //is this an RMC or GGA sentence?
  if( strstr( sentence, "$GNGGA" ) != 0 ) return GGA;
  else return RMC;
}

/* Parse the GGA and RMC sentences
 */

bool gps_parse_gga( char* sentence, GPSFIX* fix ) {
  
  //a ptr instead of messing with the sentence
  const char *p = sentence;
  
  //skip the time data in this sentence
  p = strchr( p, ',' ) + 1;
  
  //parse latitude
  p = strchr( p, ',' ) + 1;
  fix->latitude = atof( p );
  if( fix->latitude == 0.0 ) return false;
  
  //parse latitude direction
  p = strchr( p, ',' ) + 1;
  switch( *p ) {
    case 'N': fix->lat_dir = 'N'; break;
    case 'S': fix->lat_dir = 'E'; break;
    default: return false;
  }
  
  //parse longitude
  p = strchr( p, ',' ) + 1;
  fix->longitude = atof( p );
  if( fix->longitude == 0.0 ) return false;
  
  //parse longitude direction
  p = strchr( p, ',' ) + 1;
  switch( *p ) {
    case 'E': fix->long_dir = 'E'; break;
    case 'W': fix->long_dir = 'W'; break;
    default: return false;
  }
  
  //prase signal quality
  p = strchr( p, ',' ) + 1;
  fix->quality = (uint8_t) atoi( p );
  if( fix->quality == 0 ) return false;

  //parse satellite list
  p = strchr( p, ',' ) + 1;
  fix->satellites = (uint8_t) atoi( p );
  if( fix->satellites == 0 ) return false;

  //skip accuracy
  p = strchr( p, ',' ) + 1;

  //parse altitude
  p = strchr( p, ',' ) + 1;
  fix->altitude = atof( p );
  
  return true;
}

bool gps_parse_rmc( char* sentence, GPSFIX* fix ) {
  
  //a ptr instead of messing with the sentence
  const char *p = sentence;

  //parse time (UTC)
  p = strchr( p, ',' ) + 1;
  char time[3] = { 0, 0, 0 };
  char *res;

  //hour
  time[0] = p[0];
  time[1] = p[1];
  fix->hour = strtol( time, &res, 10 );

  //minute
  time[0] = p[2];
  time[1] = p[3];
  fix->minute = strtol( time, &res, 10 );

  //second
  time[0] = p[4];
  time[1] = p[5];
  fix->second = strtol( time, &res, 10 );

  //check status
  p = strchr( p, ',' ) + 1;
  if( *p != 'A' ) return false;

  //skip latitude
  p = strchr( p, ',' ) + 1;

  //skip direction
  p = strchr( p, ',' ) + 1;

  //skip longitude
  p = strchr( p, ',' ) + 1;

  //skip direction
  p = strchr( p, ',' ) + 1;

  //parse speed
  p = strchr( p, ',' ) + 1;
  fix->speed = atof( p );

  //parse course
  p = strchr( p, ',' ) + 1;
  fix->course = atof( p );
  
  return true;
}