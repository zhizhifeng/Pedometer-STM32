#ifndef PEDOMETER_H
#define PEDOMETER_H
#include <stdint.h>
#define FILTER_BUFFER_SIZE 3
#define BUFFER_SIZE 3
#define THRESHOLD1 0.08
#define THRESHOLD2 0.05
typedef struct
{
    float AccX;
    float AccY;
    float AccZ;
} AccVector;

typedef struct{
    AccVector acc_data;
    AccVector grav_data;
    AccVector user_data;
} Acc;

typedef struct{
    float unfiltered[FILTER_BUFFER_SIZE];
    float filtered[FILTER_BUFFER_SIZE];
} FilterBuffer;

typedef struct{
    FilterBuffer x;
    FilterBuffer y;
    FilterBuffer z;
} FilterCoordBuffer;

typedef struct{
    FilterCoordBuffer hp_user_data;
    FilterBuffer lp_dot_data;
} FilterAccBuffer;

typedef struct {
    float alpha[3];
    float beta[3];
} FilterCoefficients;

typedef struct{
    float processed_data[BUFFER_SIZE];
    int step;
    float max;
    float min;
    int flag[4];
} StepDetectHandler;

// export functions
void pedometer_update(AccVector acc, Acc *data, FilterAccBuffer *coord_data, StepDetectHandler *hdetect);

#endif