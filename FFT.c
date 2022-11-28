#include "FFT.h"

/* Credit
 * _fft(), fft(), and show() functions were found on https://rosettacode.org/wiki/Fast_Fourier_transform
 *
 */
double PI;

void _fft(cplx buf[], cplx out[], int n, int step)
{
	if (step < n) {
		_fft(out, buf, n, step * 2);
		_fft(out + step, buf + step, n, step * 2);

		for (int i = 0; i < n; i += 2 * step) {
			cplx t = cexp(-I * PI * i / n) * out[i + step];
			buf[i / 2]     = out[i] + t;
			buf[(i + n)/2] = out[i] - t;
		}
	}
}

void fft(cplx buf[], int n)
{
	cplx out[n];
	for (int i = 0; i < n; i++){
		out[i] = buf[i];
	}

	_fft(buf, out, n, 1);
}


void show(const char * s, cplx buf[], int arraySize) {
	printf("%s", s);
	for (int i = 0; i < arraySize; i++)
		printf("(%g, %g) ", creal(buf[i]), cimag(buf[i]));
	printf("\n");
}

// Finds the frequency but finding the FFT index with the largest magnitude
int findFreqIndex(cplx buf[], int arraySize){
    int maxIndex = 0;
    int max = 0;
    int temp = 0;
    for(int i=1; i < arraySize/2; i++){ // Start at 2nd index because first is DC voltage (freq = 0 Hz)
        temp = creal(buf[i]) * creal(buf[i]) + cimag(buf[i]) * cimag(buf[i]);
        if(temp > max){
            max = temp;
            maxIndex = i;
        }
    }

    return maxIndex;
}

int calcFreq(int maxIndex, int arraySize, int samplingFreq){
    /* -------------Notes---------------------
    Frequency = k (index of max magnitude FFT result from above) / N (number of samples) * R (sampling frequency)
    Test Functions:
        //printf("k / N = %f\n", k / N);
        //printf("SAMP_FREQ / N = %f\n", SAMPLING_FREQ / N);
        //freq = k / N * SAMPLING_FREQ;

    */

    // Adjust variables for calculations
    int freq = 0;
    double k = (double)maxIndex;

    // Calculate Frequency
    freq = k / arraySize * samplingFreq;

    return freq;
}

int findFreq(int arraySize, int samplingFreq, uint16_t ADC_arr[])
{
	PI = atan2(1, 1) * 4;
	cplx buf[arraySize];

	// Copy Array
	for(int i = 0; i < arraySize; i++){
		buf[i] = (cplx)(ADC_arr[i]);
	}

	// Perform the FFT
	fft(buf, arraySize);

	// Test Functions
	//show("Data: ", buf, arraySize);
	//show("\nFFT : ", buf, arraySize);

	// Functions to calculate frequency
	int maxIndex = 0;
	int freq = 0;

	maxIndex = findFreqIndex(buf, arraySize);
	freq = calcFreq(maxIndex, arraySize, samplingFreq);

	// Print Results
	//printf("Frequency: %d\n", freq);
	//printf("Max Index: %d\n", maxIndex);

    /*---Notes
    - You only care about 0-N/2 in the array because the 2nd half is just a reflection of the first half
    - The frequency of the wave will be the frequency with the highest magnitude
        that is not at index 0 (DC w/ freq=0, although it will be off and is unneeded)
    - the frequency is given by the index in the array
    Equation: Frequency = k (index of max magnitude FFT result from above) / N (number of samples) * R (sampling frequency)

    Resources: https://www.youtube.com/watch?v=3aOaUv3s8RY, https://stackoverflow.com/questions/6740545/understanding-fft-output
    */

    // FFT will need a slow sampling rate (3000 Hz is a good value)
	return freq;
}
