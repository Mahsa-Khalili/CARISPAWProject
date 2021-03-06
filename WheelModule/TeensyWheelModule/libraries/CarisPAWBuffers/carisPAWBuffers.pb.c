/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.9.3 at Tue Jul  2 15:08:05 2019. */

#include "carisPAWBuffers.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t frameUnit_fields[20] = {
    PB_FIELD(  1, DOUBLE  , OPTIONAL, STATIC  , FIRST, frameUnit, time_stamp, time_stamp, 0),
    PB_FIELD(  2, UENUM   , OPTIONAL, STATIC  , OTHER, frameUnit, sensorType, time_stamp, 0),
    PB_FIELD(  3, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, acc_x, sensorType, 0),
    PB_FIELD(  4, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, acc_y, acc_x, 0),
    PB_FIELD(  5, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, acc_z, acc_y, 0),
    PB_FIELD(  6, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, angular_x, acc_z, 0),
    PB_FIELD(  7, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, angular_y, angular_x, 0),
    PB_FIELD(  8, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, angular_z, angular_y, 0),
    PB_FIELD(  9, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, mag_x, angular_z, 0),
    PB_FIELD( 10, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, mag_y, mag_x, 0),
    PB_FIELD( 11, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, mag_z, mag_y, 0),
    PB_FIELD( 12, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, heading, mag_z, 0),
    PB_FIELD( 13, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, pitch, heading, 0),
    PB_FIELD( 14, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, roll, pitch, 0),
    PB_FIELD( 15, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, USensorForward, roll, 0),
    PB_FIELD( 16, FLOAT   , OPTIONAL, STATIC  , OTHER, frameUnit, USensorDownward, USensorForward, 0),
    PB_FIELD( 17, BYTES   , OPTIONAL, CALLBACK, OTHER, frameUnit, piCamImage, USensorDownward, 0),
    PB_FIELD( 18, INT32   , OPTIONAL, STATIC  , OTHER, frameUnit, imageHeight, piCamImage, 0),
    PB_FIELD( 19, INT32   , OPTIONAL, STATIC  , OTHER, frameUnit, imageWidth, imageHeight, 0),
    PB_LAST_FIELD
};

const pb_field_t wheelUnit_fields[9] = {
    PB_FIELD(  1, FLOAT   , OPTIONAL, STATIC  , FIRST, wheelUnit, time_stamp, time_stamp, 0),
    PB_FIELD(  2, BOOL    , OPTIONAL, STATIC  , OTHER, wheelUnit, isStamp, time_stamp, 0),
    PB_FIELD(  3, FLOAT   , OPTIONAL, STATIC  , OTHER, wheelUnit, acc_x, isStamp, 0),
    PB_FIELD(  4, FLOAT   , OPTIONAL, STATIC  , OTHER, wheelUnit, acc_y, acc_x, 0),
    PB_FIELD(  5, FLOAT   , OPTIONAL, STATIC  , OTHER, wheelUnit, acc_z, acc_y, 0),
    PB_FIELD(  6, FLOAT   , OPTIONAL, STATIC  , OTHER, wheelUnit, angular_x, acc_z, 0),
    PB_FIELD(  7, FLOAT   , OPTIONAL, STATIC  , OTHER, wheelUnit, angular_y, angular_x, 0),
    PB_FIELD(  8, FLOAT   , OPTIONAL, STATIC  , OTHER, wheelUnit, angular_z, angular_y, 0),
    PB_LAST_FIELD
};



/* On some platforms (such as AVR), double is really float.
 * These are not directly supported by nanopb, but see example_avr_double.
 * To get rid of this error, remove any double fields from your .proto.
 */
PB_STATIC_ASSERT(sizeof(double) == 8, DOUBLE_MUST_BE_8_BYTES)

/* @@protoc_insertion_point(eof) */
