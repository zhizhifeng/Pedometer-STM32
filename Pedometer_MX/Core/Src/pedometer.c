#include "pedometer.h"

// filter coefficients
static const FilterCoefficients LP_USER_COEF = { // fc=1
    .alpha = {1, (float)-1.561018075800718, (float)0.641351538057563},
    .beta = {(float)0.020083365564211, (float)0.040166731128422, (float)0.020083365564211}};
static const FilterCoefficients HP_USER_COEF = { // fc=0.2
    .alpha = {1, (float)-1.911197067426073, (float)0.914975834801434},
    .beta = {(float)0.956543225556877, (float)-1.913086451113754, (float)0.956543225556877}};

// internal functions
static void measure_steps(StepDetectHandler *hdetect);
static void filter_coord_buffer_update(FilterCoordBuffer *buffer, AccVector data);
static void filter_buffer_update(FilterBuffer *buffer, float data);
static float single_step_filter(float *unfiltered, float *filtered, FilterCoefficients coef, uint8_t buffer_size);

void pedometer_update(AccVector acc, Acc *data, FilterAccBuffer *coord_data, StepDetectHandler *hdetect)
{
    // Update the accelerometer data
    data->acc_data.AccX = acc.AccX;
    data->acc_data.AccY = acc.AccY;
    data->acc_data.AccZ = acc.AccZ;
    // Update the user data by filtering the accelerometer data
    filter_coord_buffer_update(&coord_data->hp_user_data, data->acc_data);
    data->user_data.AccX = single_step_filter(coord_data->hp_user_data.x.unfiltered, coord_data->hp_user_data.x.filtered, HP_USER_COEF, FILTER_BUFFER_SIZE);
    data->user_data.AccY = single_step_filter(coord_data->hp_user_data.y.unfiltered, coord_data->hp_user_data.y.filtered, HP_USER_COEF, FILTER_BUFFER_SIZE);
    data->user_data.AccZ = single_step_filter(coord_data->hp_user_data.z.unfiltered, coord_data->hp_user_data.z.filtered, HP_USER_COEF, FILTER_BUFFER_SIZE);
    // Update the gravity data by subtracting the user data from the accelerometer data
    data->grav_data.AccX = data->acc_data.AccX - data->user_data.AccX;
    data->grav_data.AccY = data->acc_data.AccY - data->user_data.AccY;
    data->grav_data.AccZ = data->acc_data.AccZ - data->user_data.AccZ;
    // Dot product of user data and gravity data
    filter_buffer_update(&coord_data->lp_dot_data, data->user_data.AccX * data->grav_data.AccX +
                                                       data->user_data.AccY * data->grav_data.AccY +
                                                       data->user_data.AccZ * data->grav_data.AccZ);
    // Low pass filter the dot product and update the step detect handler
    hdetect->processed_data[2] = hdetect->processed_data[1];
    hdetect->processed_data[1] = hdetect->processed_data[0];
    hdetect->processed_data[0] = single_step_filter(coord_data->lp_dot_data.unfiltered, coord_data->lp_dot_data.filtered, LP_USER_COEF, FILTER_BUFFER_SIZE);
    // Detect step
    measure_steps(hdetect);
}

static void filter_coord_buffer_update(FilterCoordBuffer *buffer, AccVector data)
{
    filter_buffer_update(&buffer->x, data.AccX);
    filter_buffer_update(&buffer->y, data.AccY);
    filter_buffer_update(&buffer->z, data.AccZ);
}

static void filter_buffer_update(FilterBuffer *buffer, float data)
{
    for (uint8_t i = 0; i < FILTER_BUFFER_SIZE - 1; i++)
    {
        buffer->unfiltered[i] = buffer->unfiltered[i + 1];
        // buffer->filtered[i] = buffer->filtered[i+1];
    }
    buffer->unfiltered[FILTER_BUFFER_SIZE - 1] = data;
}

static float single_step_filter(float *data, float *filtered_data, FilterCoefficients coefficients, uint8_t data_length)
{
    uint8_t i = 0;
    // Shift the data
    for (i = 0; i < data_length - 1; i++)
    {
        filtered_data[i] = filtered_data[i + 1];
    }
    // Filter the data
    filtered_data[i] = coefficients.alpha[0] *
                       (data[i] * coefficients.beta[0] +
                        data[i - 1] * coefficients.beta[1] +
                        data[i - 2] * coefficients.beta[2] -
                        filtered_data[i - 1] * coefficients.alpha[1] -
                        filtered_data[i - 2] * coefficients.alpha[2]);
    return filtered_data[i];
}

static void measure_steps(StepDetectHandler *hdetect)
{
    float difference = 0;
    int count_steps = 1;
    // To detect a step, we check the following conditions:
    // - The current data is a local maxima or minima
    // - We detect a slope that is greater than a threshold
    // - The polarity of the previous slope is different from the current slope
    // - The difference between the maxima and minima is greater than a threshold
    // - The previous slope was not detected as a step
    if (hdetect->processed_data[2] < hdetect->processed_data[1] &&
        hdetect->processed_data[0] < hdetect->processed_data[1] &&
        hdetect->processed_data[1] - hdetect->min > THRESHOLD2)
    {                                              // if we detected a local maxima and it is a slope
        hdetect->max = hdetect->processed_data[1]; // update the maxima
        hdetect->flag[1] = 0;                      // a rising slope was detected
        difference = hdetect->max - hdetect->min;
        if (hdetect->max != 0 && hdetect->min != 0 &&
            hdetect->flag[0] == 0 && difference > THRESHOLD1)
        { // if the range defines a step and the previous slope was a falling slope
            if (hdetect->flag[3])
            { // if the previous falling slope was detected as a step
                // only reset the flag to allowthe next slope to be detected as a step
                hdetect->flag[3] = 0;
            }
            else
            { // update the step count
                hdetect->step += 1;
                // set the flag that the last slope was a rising slope so that we don't count the same step twice
                hdetect->flag[0] = 1;
                // set the flag that the last slope was a step so that we don't count the same step twice
                hdetect->flag[2] = 1;
            }
        }
    }
    else if (hdetect->processed_data[2] > hdetect->processed_data[1] &&
             hdetect->processed_data[0] > hdetect->processed_data[1] &&
             hdetect->max - hdetect->processed_data[1] > THRESHOLD2)
    {
        hdetect->min = hdetect->processed_data[1];
        hdetect->flag[0] = 0;
        difference = hdetect->max - hdetect->min;
        if (hdetect->max != 0 && hdetect->min != 0 && hdetect->flag[1] == 0 && difference > THRESHOLD1)
        {
            if (hdetect->flag[2])
            {
                hdetect->flag[2] = 0;
            }
            else
            {
                hdetect->step += 1;
                hdetect->flag[1] = 1;
                hdetect->flag[3] = 1;
            }
        }
    }
}
static float single_step_averaging_filter(float *data, float *filtered_data, uint8_t data_length)
{
    uint8_t i = 0;
    // Shift the data
    for (i = 0; i < data_length - 1; i++)
    {
        filtered_data[i] = filtered_data[i + 1];
    }
    // Filter the data
    filtered_data[i] = filtered_data[i - 1] - data[1] / (data_length - 1) + data[i] / (data_length - 1);
    return filtered_data[i];
}
