#include <stdint.h>
#define FILTER_BUFFER_SIZE 20
#define BUFFER_SIZE 5
#define THRESHOLD 0.1

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
    FilterCoordBuffer lp_grav_data;
    FilterBuffer lp_dot_data;
    FilterBuffer hp_dot_data;
} FilterAccBuffer;

typedef struct {
    float alpha[3];
    float beta[3];
} FilterCoefficients;

// void pedometer_init(Acc *data, AccBuffer *coord_data);
void pedometer_update(AccVector acc, Acc *data, FilterAccBuffer *coord_data, float *processed_data);
void filter_coord_buffer_update(FilterCoordBuffer *buffer, AccVector data);
void filter_buffer_update(FilterBuffer *buffer, float data);
float single_step_filter(float *unfiltered, float *filtered, FilterCoefficients coef, uint8_t buffer_size);
float single_step_averaging_filter(float* data, float* filtered_data, uint8_t data_length);
void measure_steps(int *steps, float *data);