#ifndef FILTERS_H
#define FILTERS_H

#include <math.h>

/* high-Pass Filter */
typedef struct {
    float kX;
    float kA0;
    float kA1;
    float kB1;
    float last_filter_value;
    float last_raw_value;
} HighPassFilter;

/* low-Pass Filter */
typedef struct {
    float kX;
    float kA0;
    float kB1;
    float last_value;
} LowPassFilter;

/* differentiator */
typedef struct {
    float kSamplingFrequency;
    float last_value;
} Differentiator;

/* moving Average Filter */
#define AVERAGING_BUFFER_SIZE 50
typedef struct {
    int index;
    int count;
    float values[AVERAGING_BUFFER_SIZE];
} MovingAverageFilter;

/* Function Prototypes */
void HighPassFilter_Init(HighPassFilter* filter, float cutoff, float sampling_frequency);
float HighPassFilter_Process(HighPassFilter* filter, float value);
void HighPassFilter_Reset(HighPassFilter* filter);

void LowPassFilter_Init(LowPassFilter* filter, float cutoff, float sampling_frequency);
float LowPassFilter_Process(LowPassFilter* filter, float value);
void LowPassFilter_Reset(LowPassFilter* filter);

void Differentiator_Init(Differentiator* differentiator, float sampling_frequency);
float Differentiator_Process(Differentiator* differentiator, float value);
void Differentiator_Reset(Differentiator* differentiator);

void MovingAverageFilter_Init(MovingAverageFilter* filter);
float MovingAverageFilter_Process(MovingAverageFilter* filter, float value);
void MovingAverageFilter_Reset(MovingAverageFilter* filter);
int MovingAverageFilter_Count(MovingAverageFilter* filter);



// Statistic structure for Min/Max/Average calculation
typedef struct {
    float min;
    float max;
    float sum;
    int count;
} MinMaxAvgStatistic;

void MinMaxAvgStatistic_Init(MinMaxAvgStatistic* stat);
void MinMaxAvgStatistic_Process(MinMaxAvgStatistic* stat, float value);
void MinMaxAvgStatistic_Reset(MinMaxAvgStatistic* stat);
float MinMaxAvgStatistic_Minimum(MinMaxAvgStatistic* stat);
float MinMaxAvgStatistic_Maximum(MinMaxAvgStatistic* stat);
float MinMaxAvgStatistic_Average(MinMaxAvgStatistic* stat);

#endif
