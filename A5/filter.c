#include "filter.h"
#include <math.h>

/* High-Pass Filter */
void HighPassFilter_Init(HighPassFilter* filter, float cutoff, float sampling_frequency) {
    float samples = sampling_frequency / (cutoff * 2 * M_PI);
    filter->kX = exp(-1 / samples);
    filter->kA0 = (1 + filter->kX) / 2;
    filter->kA1 = -filter->kA0;
    filter->kB1 = filter->kX;
    filter->last_filter_value = NAN;
    filter->last_raw_value = NAN;
}

float HighPassFilter_Process(HighPassFilter* filter, float value) {
    if (isnan(filter->last_filter_value) || isnan(filter->last_raw_value)) {
        filter->last_filter_value = 0.0;
    } else {
        filter->last_filter_value =
            filter->kA0 * value +
            filter->kA1 * filter->last_raw_value +
            filter->kB1 * filter->last_filter_value;
    }
    filter->last_raw_value = value;
    return filter->last_filter_value;
}

void HighPassFilter_Reset(HighPassFilter* filter) {
    filter->last_filter_value = NAN;
    filter->last_raw_value = NAN;
}

/* Low-Pass Filter */
void LowPassFilter_Init(LowPassFilter* filter, float cutoff, float sampling_frequency) {
    float samples = sampling_frequency / (cutoff * 2 * M_PI);
    filter->kX = exp(-1 / samples);
    filter->kA0 = 1 - filter->kX;
    filter->kB1 = filter->kX;
    filter->last_value = NAN;
}

float LowPassFilter_Process(LowPassFilter* filter, float value) {
    if (isnan(filter->last_value)) {
        filter->last_value = value;
    } else {
        filter->last_value = filter->kA0 * value + filter->kB1 * filter->last_value;
    }
    return filter->last_value;
}

void LowPassFilter_Reset(LowPassFilter* filter) {
    filter->last_value = NAN;
}

/* Differentiator */
void Differentiator_Init(Differentiator* differentiator, float sampling_frequency) {
    differentiator->kSamplingFrequency = sampling_frequency;
    differentiator->last_value = NAN;
}

float Differentiator_Process(Differentiator* differentiator, float value) {
    float diff = (value - differentiator->last_value) * differentiator->kSamplingFrequency;
    differentiator->last_value = value;
    return diff;
}

void Differentiator_Reset(Differentiator* differentiator) {
    differentiator->last_value = NAN;
}

/* Moving Average Filter */
void MovingAverageFilter_Init(MovingAverageFilter* filter) {
    filter->index = 0;
    filter->count = 0;
}

float MovingAverageFilter_Process(MovingAverageFilter* filter, float value) {
    filter->values[filter->index] = value;
    filter->index = (filter->index + 1) % AVERAGING_BUFFER_SIZE;
    if (filter->count < AVERAGING_BUFFER_SIZE) {
        filter->count++;
    }

    float sum = 0;
    for (int i = 0; i < filter->count; i++) {
        sum += filter->values[i];
    }
    return sum / filter->count;
}

void MovingAverageFilter_Reset(MovingAverageFilter* filter) {
    filter->index = 0;
    filter->count = 0;
}

int MovingAverageFilter_Count(MovingAverageFilter* filter) {
    return filter->count;
}


// min/max/average Statistic Functions
void MinMaxAvgStatistic_Init(MinMaxAvgStatistic* stat) {
    stat->min = NAN;
    stat->max = NAN;
    stat->sum = 0.0;
    stat->count = 0;
}

void MinMaxAvgStatistic_Process(MinMaxAvgStatistic* stat, float value) {
    if (isnan(stat->min)) {
        stat->min = value;
    } else {
        stat->min = fminf(stat->min, value);
    }

    if (isnan(stat->max)) {
        stat->max = value;
    } else {
        stat->max = fmaxf(stat->max, value);
    }

    stat->sum += value;
    stat->count++;
}

void MinMaxAvgStatistic_Reset(MinMaxAvgStatistic* stat) {
    stat->min = NAN;
    stat->max = NAN;
    stat->sum = 0.0;
    stat->count = 0;
}

float MinMaxAvgStatistic_Minimum(MinMaxAvgStatistic* stat) {
    return stat->min;
}

float MinMaxAvgStatistic_Maximum(MinMaxAvgStatistic* stat) {
    return stat->max;
}

float MinMaxAvgStatistic_Average(MinMaxAvgStatistic* stat) {
    return stat->sum / stat->count;
}
