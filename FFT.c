#include <stdio.h>
#include <math.h>
#include <complex.h>

#define N 16    // Sample Array Size

double PI;
typedef double complex cplx;

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
	for (int i = 0; i < n; i++) out[i] = buf[i];

	_fft(buf, out, n, 1);
}


void show(const char * s, cplx buf[]) {
	printf("%s", s);
	for (int i = 0; i < N; i++)
		//if (!cimag(buf[i]))
		//	printf("%g ", creal(buf[i]));
		//else
			printf("(%g, %g) ", creal(buf[i]), cimag(buf[i]));
}

int runFFT()
{
	PI = atan2(1, 1) * 4;
	//cplx buf[] = {3, 4, 5, 4, 3, 4, 5, 4, 3, 4, 5, 4, 3, 4, 5, 4}; // 4 Hz
	cplx buf[] = {3, 3, 4, 4, 5, 5, 4, 4, 3, 3, 4, 4, 5, 5, 4, 4}; // 2 Hz (2nd item in array has largest magnitude)

	show("Data: ", buf);
	fft(buf, N);
	show("\nFFT : ", buf);

    /*---Notes
    - You only care about 0-N/2 in the array because the 2nd half is just a reflection of the first half
    - The frequency of the wave will be the frequency with the highest magnitude
        that is not at index 0 (DC w/ freq=0, although it will be off and is unneeded)
    - the frequency is given by the index in the array
    Equation: Frequency = k (index of max magnitude FFT result from above) / N (number of samples) * R (sampling frequency)

    Resources: https://www.youtube.com/watch?v=3aOaUv3s8RY, https://stackoverflow.com/questions/6740545/understanding-fft-output
    */
	return 0;
}
