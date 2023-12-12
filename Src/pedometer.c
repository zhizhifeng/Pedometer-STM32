#include "pedometer.h"

// void pedometer_init(Acc *data, FilterAccBuffer *coord_data, float *processed_data) {
//     data->acc_data.AccX = 0;
//     data->acc_data.AccY = 0;
//     data->acc_data.AccZ = 0;
//     data->grav_data.AccX = 0;
//     data->grav_data.AccY = 0;
//     data->grav_data.AccZ = 0;
//     data->user_data.AccX = 0;
//     data->user_data.AccY = 0;
//     data->user_data.AccZ = 0;
//     filter_coord_buffer_init(&coord_data->lp_grav_data);
//     filter_buffer_init(&coord_data->lp_dot_data);
//     filter_buffer_init(&coord_data->hp_dot_data);
//     processed_data[0] = 0;
// }

// void filter_buffer_init(FilterBuffer *buffer) {
//     for(uint8_t i = 0; i < FILTER_BUFFER_SIZE; i++){
//         buffer->unfiltered[i] = 0;
//         buffer->filtered[i] = 0;
//     }
// }

// void filter_coord_buffer_init(FilterCoordBuffer *buffer) {
//     filter_buffer_init(&buffer->x);
//     filter_buffer_init(&buffer->y);
//     filter_buffer_init(&buffer->z);
// }

void pedometer_update(AccVector acc, Acc *data, FilterAccBuffer *coord_data, float *processed_data) {

    const FilterCoefficients LOW_GRAV_COEF = {
    .alpha = {1, -1.99555712434579, 0.995566972065975},
    .beta = {0.000241359049041981, 0.000482718098083961, 0.000241359049041981}
    };
    const FilterCoefficients LP_USER_COEF = {
        //fc=3
    // .alpha = {1, -0.747789178258503, 0.272214937925007},
    // .beta = {0.131106439916626,0.262212879833252, 0.131106439916626}
    .alpha = {1, -1.142980502539901, 0.412801598096189},
    .beta = {0.067455273889072,0.134910547778144, 0.067455273889072}
    };
    const FilterCoefficients HP_USER_COEF = {
    .alpha = {1, -1.911197067426073, 0.914975834801434},
    .beta = {0.956543225556877, -1.913086451113754, 0.956543225556877}
    };
    // Update the accelerometer data
    data->acc_data.AccX = acc.AccX;
    data->acc_data.AccY = acc.AccY;
    data->acc_data.AccZ = acc.AccZ;
    // Update the gravity data
    filter_coord_buffer_update(&coord_data->lp_grav_data, data->acc_data);
    data->grav_data.AccX = 0.01*single_step_filter(coord_data->lp_grav_data.x.unfiltered, coord_data->lp_grav_data.x.filtered, LOW_GRAV_COEF, FILTER_BUFFER_SIZE);
    data->grav_data.AccY = 0.01*single_step_filter(coord_data->lp_grav_data.y.unfiltered, coord_data->lp_grav_data.y.filtered, LOW_GRAV_COEF, FILTER_BUFFER_SIZE);
    data->grav_data.AccZ = 0.01*single_step_filter(coord_data->lp_grav_data.z.unfiltered, coord_data->lp_grav_data.z.filtered, LOW_GRAV_COEF, FILTER_BUFFER_SIZE);
    // Update the user data
    data->user_data.AccX = data->acc_data.AccX - data->grav_data.AccX;
    data->user_data.AccY = data->acc_data.AccY - data->grav_data.AccY;
    data->user_data.AccZ = data->acc_data.AccZ - data->grav_data.AccZ;
    // Dot product of user data and gravity data
    filter_buffer_update(&coord_data->lp_dot_data, data->user_data.AccX * data->grav_data.AccX +
                                                   data->user_data.AccY * data->grav_data.AccY +
                                                   data->user_data.AccZ * data->grav_data.AccZ);
    // Low pass filter the dot product  
    filter_buffer_update(&coord_data->hp_dot_data, 
                        single_step_filter(coord_data->lp_dot_data.unfiltered, coord_data->lp_dot_data.filtered, LP_USER_COEF, FILTER_BUFFER_SIZE));  
    processed_data[0]=single_step_filter(coord_data->lp_dot_data.unfiltered, coord_data->lp_dot_data.filtered, LP_USER_COEF, FILTER_BUFFER_SIZE);
    // High pass filter the dot product
    //processed_data[0] = single_step_filter(coord_data->hp_dot_data.unfiltered, coord_data->hp_dot_data.filtered, HP_USER_COEF, FILTER_BUFFER_SIZE);
    // Detect step
    
}

void filter_coord_buffer_update(FilterCoordBuffer *buffer, AccVector data) {
    filter_buffer_update(&buffer->x, data.AccX);
    filter_buffer_update(&buffer->y, data.AccY);
    filter_buffer_update(&buffer->z, data.AccZ);
}

void filter_buffer_update(FilterBuffer *buffer, float data) {
    for(uint8_t i = 0; i < FILTER_BUFFER_SIZE - 1; i++){
        buffer->unfiltered[i] = buffer->unfiltered[i+1];
        // buffer->filtered[i] = buffer->filtered[i+1];
    }
    buffer->unfiltered[FILTER_BUFFER_SIZE - 1] = data;
}

float single_step_filter(float* data, float* filtered_data, FilterCoefficients coefficients, uint8_t data_length) {
    uint8_t i = 0;
    // Shift the data
    for (i = 0; i < data_length - 1; i++){
        filtered_data[i] = filtered_data[i+1];
    }
    // Filter the data
    filtered_data[i] = coefficients.alpha[0] *
                         (data[i]            * coefficients.beta[0] +
                          data[i-1]          * coefficients.beta[1] +
                          data[i-2]          * coefficients.beta[2] -
                          filtered_data[i-1] * coefficients.alpha[1] -
                          filtered_data[i-2] * coefficients.alpha[2]);
    return filtered_data[i];
}

void measure_steps(int *steps, float *data) {
    int count_steps = 1;
        if (data[1] >= THRESHOLD && data[0] < THRESHOLD) {
            if (count_steps) {
                *steps += 1;
                count_steps = 0;
            }
        }
}
// float filter(float* data, FilterCoefficients coefficients, uint8_t data_length) {
//     float filtered_data[data_length];
//     filtered_data[0] = 0;
//     filtered_data[1] = 0;

//     for (uint8_t i = 2; i < data_length; i++) {
//         filtered_data[i] = coefficients.alpha[0] *
//                              (data[i]            * coefficients.beta[0] +
//                               data[i-1]          * coefficients.beta[1] +
//                               data[i-2]          * coefficients.beta[2] -
//                               filtered_data[i-1] * coefficients.alpha[1] -
//                               filtered_data[i-2] * coefficients.alpha[2]);
//     }
//     return filtered_data[data_length - 2];
// }