/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"

extern "C" {
#include "pico/analog_microphone.h"
}

#include "tflite_model.h"

#include "ml_model.h"

#include "ringBuffer.h"

// constants
#define SAMPLE_RATE       16000
#define FFT_SIZE          256
#define SPECTRUM_SHIFT    4
#define INPUT_BUFFER_SIZE 16
#define INPUT_SIZE        1323
#define CAPTURE_THRESHOLD 0.01

// microphone configuration
const struct analog_microphone_config analog_config = {
    // GPIO to use for input, must be ADC compatible (GPIO 26 - 28)
    .gpio = 26,

    // bias voltage of microphone in volts
    .bias_voltage = 0.8,

    // sample rate in Hz
    .sample_rate = SAMPLE_RATE,

    // number of samples to buffer
    .sample_buffer_size = INPUT_BUFFER_SIZE,
};

q15_t sample[INPUT_BUFFER_SIZE];
volatile int new_samples_captured = 0;

bool sample_ready = false;

MLModel ml_model(tflite_model, 128 * 1323);


void on_analog_samples_ready();

int main( void )
{
    // initialize stdio
    stdio_init_all();

    printf("pico bullet detection\n");

    if (!ml_model.init()) {
        printf("Failed to initialize ML model!\n");
        while (1) { tight_loop_contents(); }
    }

    if (!dsp_pipeline.init()) {
        printf("Failed to initialize DSP Pipeline!\n");
        while (1) { tight_loop_contents(); }
    }

    int8_t* input = (int8_t*)ml_model.input_data();
    bool recording = false;
    int curIndex = 0;
    float rising = 0;
    float filtered = 0;
    float alpha = 0.98;
    int historySize = 100;

    // initialize the Analog microphone
    if (analog_microphone_init(&analog_config) < 0) {
        printf("Analog microphone initialization failed!\n");
        while (1) { tight_loop_contents(); }
    }

    // set callback that is called when all the samples in the library
    // internal sample buffer are ready for reading
    analog_microphone_set_samples_ready_handler(on_analog_samples_ready);

    // start capturing data from the Analog microphone
    if (analog_microphone_start() < 0) {
        printf("Analog microphone start failed!\n");
        while (1) { tight_loop_contents(); }
    }

    while (1) {
        // wait for new samples
        while (new_samples_captured == 0) {
            tight_loop_contents();
        }
        new_samples_captured = 0;

        float max = 0;
		for(int n = 0; n < 16; n++){
			max = abs(sample[n])>max?abs(sample[n]):max;
			if((pow(sample[n],2) > CAPTURE_THRESHOLD) && (!recording) && (rising > 0)){
				recording = true;
			}
			if(recording){
				input[curIndex++] = history.front();
				if(curIndex == INPUT_SIZE){
					recording = false;
					curIndex = 0;
                    sample_ready = true;
				}
			}

            history.push(sample[n]);
		}

		// std::cout << max << std::endl;
		float tempfiltered = filtered * alpha + max * (1-alpha);
		float delta = tempfiltered - filtered;
		float temprising = rising + ((delta > 0)*(rising < 5) - (delta < 0)*(rising > - 5));
		
		rising = rising * (0.5) + temprising*(0.5);
		rising *= (filtered > 0.001);
		filtered = tempfiltered;

        if(sample_ready){
            float prediction = ml_model.predict();

            if (prediction < 0.5) {
            printf("\t🔥 🔔\tdetected!\t(prediction = %f)\n\n", prediction);
            } else {
            printf("\t🔕\tNOT detected\t(prediction = %f)\n\n", prediction);
            }
        }
    }

    return 0;
}

void on_analog_samples_ready()
{
    // callback from library when all the samples in the library
    // internal sample buffer are ready for reading 

    // read in the new samples
    new_samples_captured = analog_microphone_read(sample, INPUT_BUFFER_SIZE);
}
