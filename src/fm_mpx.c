/*
    PiFmAdv - Advanced FM transmitter for the Raspberry Pi
    Copyright (C) 2017 Miegl

    See https://github.com/Miegl/PiFmAdv
*/

#include <sndfile.h>
#include <stdlib.h>
#include <strings.h>
#include <math.h>

#include <soxr.h>

#include "rds.h"

#define FIR_HALF_SIZE 30
#define FIR_SIZE (2*FIR_HALF_SIZE-1)

// coefficients of the low-pass FIR filter
double low_pass_fir[FIR_HALF_SIZE];

double carrier_38[] = {0, 0.866025403784438589324, 0.866025403784438761604, 0, -0.866025403784438417099, -0.866025403784438600762};
double carrier_19[] = {0, 0.499999999999999950262, 0.866025403784438589324, 1, 0.866025403784438761604, 0.499999999999999960183, 0, -0.500000000000000132652, -0.866025403784438417099, -1, -0.866025403784438600762, -0.499999999999999681651};

int phase_38 = 0;
int phase_19 = 0;

float *input_buffer;
size_t ilength;
double *audio_buffer;
size_t alength;

int audio_index = 0;
int audio_len = 0;

double fir_buffer_mono[FIR_SIZE] = {0};
double fir_buffer_stereo[FIR_SIZE] = {0};
int fir_index = 0;
int channels;

float *last_buffer_val;
float preemphasis_prewarp;
float preemphasis_coefficient;

SNDFILE *inf;

soxr_t soxr;

double *alloc_empty_buffer(size_t length) {
    double *p = malloc(length * sizeof(double));
    if(p == NULL) return NULL;

    bzero(p, length * sizeof(double));

    return p;
}

int fm_mpx_open(char *filename, size_t buf_len, int cutoff_freq, int preemphasis_corner_freq, int srate, int nochan) {
	if(filename != NULL) {
		// Open the input file
		SF_INFO sfinfo;

                if(filename[0] == '-' && srate > 0 && nochan > 0) {
                        printf("Using stdin for raw audio input at %d Hz.\n",srate);
        		sfinfo.samplerate = srate;
        		sfinfo.format = SF_FORMAT_RAW | SF_FORMAT_PCM_16 | SF_ENDIAN_LITTLE;
        		sfinfo.channels = nochan;
        		sfinfo.frames = 0;
                } 

		// stdin or file on the filesystem?
		if(filename[0] == '-') {
			if(!(inf = sf_open_fd(fileno(stdin), SFM_READ, &sfinfo, 0))) {
				fprintf(stderr, "Error: could not open stdin for audio input.\n");
				return -1;
			} else {
				printf("Using stdin for audio input.\n");
			}
		} else {
			if(!(inf = sf_open(filename, SFM_READ, &sfinfo))) {
				fprintf(stderr, "Error: could not open input file %s.\n", filename);
				return -1;
			} else {
				printf("Using audio file: %s\n", filename);
			}
		}

		int in_samplerate = sfinfo.samplerate;
		channels = sfinfo.channels;

		soxr_error_t soxr_error;
		soxr_quality_spec_t q_spec = soxr_quality_spec(SOXR_LQ, 0);
		double passband_end = (double)cutoff_freq/in_samplerate;
		//q_spec.passband_end = passband_end > 0.5 ? passband_end : 0.5;
		soxr_io_spec_t io_spec = soxr_io_spec((soxr_datatype_t)SOXR_FLOAT32_I, (soxr_datatype_t)SOXR_FLOAT64_I);
		soxr_runtime_spec_t runtime_spec = soxr_runtime_spec(!1);

		soxr = soxr_create((double)in_samplerate, 228000, channels, &soxr_error, &io_spec, &q_spec, &runtime_spec);
		if(soxr_error) {
			fprintf(stderr, "Error: could not create SoX Resampler.\n");
			return -1;
		}

		printf("Input: %d Hz, created SoX Resampler.\n", in_samplerate);

		if(channels > 1) {
			printf("%d channels, generating stereo multiplex.\n", channels);
		} else {
			printf("1 channel, monophonic operation.\n");
		}

		// Create the preemphasis
		last_buffer_val = malloc(sizeof(float)*channels);
		for(int i=0; i<channels; i++) last_buffer_val[i] = 0;

		preemphasis_prewarp = tan(M_PI*preemphasis_corner_freq/in_samplerate);
		preemphasis_coefficient = (1.0 + (1.0 - preemphasis_prewarp)/(1.0 + preemphasis_prewarp))/2.0;
		printf("Created preemphasis with cutoff at %.1i Hz\n", preemphasis_corner_freq);

		// Create the low-pass FIR filter
		if(in_samplerate < cutoff_freq) cutoff_freq = in_samplerate;
		// Here we divide this coefficient by two because it will be counted twice
		// when applying the filter
		low_pass_fir[FIR_HALF_SIZE-1] = 2 * cutoff_freq / 228000 /2;

		// Only store half of the filter since it is symmetric
		for(int i=1; i<FIR_HALF_SIZE; i++) {
			low_pass_fir[FIR_HALF_SIZE-1-i] =
				sin(2 * M_PI * cutoff_freq * i / 228000) / (M_PI * i) // sinc
				//* (.54 - .46 * cos(2*M_PI * (i+FIR_HALF_SIZE) / (2*FIR_HALF_SIZE))); // Hamming window
				//* (.42 - .5 * cos(2*M_PI * (i+FIR_HALF_SIZE) / (2*FIR_HALF_SIZE)) + .08 * cos(4*M_PI * (i+FIR_HALF_SIZE) / (2*FIR_HALF_SIZE))); // Blackman window
				* (.35875 - .48829 * cos(2*M_PI * (i+FIR_HALF_SIZE) / (2*FIR_HALF_SIZE)) + .14128 * cos(4*M_PI * (i+FIR_HALF_SIZE) / (2*FIR_HALF_SIZE)) - .01168 * cos(6*M_PI * (i+FIR_HALF_SIZE) / (2*FIR_HALF_SIZE))); // Blackman-Harris window
		}
		printf("Created low-pass FIR filter for audio channels, with cutoff at %.1i Hz\n", cutoff_freq);


		alength = (size_t)(228000. * buf_len / (in_samplerate + 228000.) + .5);
		audio_buffer = alloc_empty_buffer(alength * channels);
		if(audio_buffer == NULL) return -1;

		ilength = buf_len - alength;
		input_buffer = malloc(ilength * channels * sizeof(float));
		bzero(input_buffer, ilength * channels * sizeof(float));
		if(input_buffer == NULL) return -1;
	}
	else {
		inf = NULL;
	}

	return 0;
}


// samples provided by this function are in 0..10: they need to be divided by
// 10 after.
int fm_mpx_get_samples(double *mpx_buffer, int mpx_buffer_len, float mpx, int rds, int wait) {

	if(rds) get_rds_samples(mpx_buffer, mpx_buffer_len);

	if(inf == NULL) return 0;

	for(size_t i = 0; i < mpx_buffer_len; i++) {

		if(audio_len <= channels) {
			for(int j=0; j<2; j++) { // one retry
				audio_len = sf_readf_float(inf, input_buffer, ilength);
				if (audio_len < 0) {
					fprintf(stderr, "Error reading audio\n");
					return -1;
				}
				if(audio_len == 0) {
					if( sf_seek(inf, 0, SEEK_SET) < 0 ) {
						if(wait) {
							printf("waiting\n");
							return 0;
						} else {
							fprintf(stderr, "Could not rewind in audio file, terminating\n");
							return -1;
						}
					}
				} else {
					//apply preemphasis
					int k;
					int l;
					float tmp;
					for(k=0; k<audio_len*channels; k+=channels) {
						for(l=0; l<channels; l++) {
							tmp = input_buffer[k+l];
							input_buffer[k+l] = input_buffer[k+l] - preemphasis_coefficient*last_buffer_val[l];
							last_buffer_val[l] = tmp;
						}
					}

					size_t odone;
					soxr_process(soxr, input_buffer, audio_len, NULL, audio_buffer, alength, &odone);
					audio_len = (int)odone;

					break;
				}
			}
			audio_index = 0;
		} else {
			audio_index += channels;
			audio_len--;
		}

		// First store the current sample(s) into the FIR filter's ring buffer
		if(channels == 0) {
			fir_buffer_mono[fir_index] = audio_buffer[audio_index];
		} else {
			// In stereo operation, generate sum and difference signals
			fir_buffer_mono[fir_index] =
				audio_buffer[audio_index] + audio_buffer[audio_index+1];
			fir_buffer_stereo[fir_index] =
				audio_buffer[audio_index] - audio_buffer[audio_index+1];
		}
		fir_index++;
		if(fir_index >= FIR_SIZE) fir_index = 0;

		// Now apply the FIR low-pass filter

		/* As the FIR filter is symmetric, we do not multiply all
		   the coefficients independently, but two-by-two, thus reducing
		   the total number of multiplications by a factor of two
		 */
		double out_mono = 0;
		double out_stereo = 0;
		int ifbi = fir_index; // ifbi = increasing FIR Buffer Index
		int dfbi = fir_index; // dfbi = decreasing FIR Buffer Index
		for(int fi=0; fi<FIR_HALF_SIZE; fi++) { // fi = Filter Index
			dfbi--;
			if(dfbi < 0) dfbi = FIR_SIZE-1;
			out_mono +=
				low_pass_fir[fi] *
				(fir_buffer_mono[ifbi] + fir_buffer_mono[dfbi]);
			if(channels > 1) {
				out_stereo +=
					low_pass_fir[fi] *
					(fir_buffer_stereo[ifbi] + fir_buffer_stereo[dfbi]);
			}
			ifbi++;
			if(ifbi >= FIR_SIZE) ifbi = 0;
		}
		// End of FIR filter

		if(!rds) mpx_buffer[i] = 0;

		if (channels>1) {
			mpx_buffer[i] +=
			((mpx-2)/2) * out_mono +
			((mpx-2)/2) * out_stereo * carrier_38[phase_38] + //todo: filtering
			carrier_19[phase_19];

			phase_19++;
			phase_38++;
			if(phase_19 >= 12) phase_19 = 0;
			if(phase_38 >= 6) phase_38 = 0;
		}
		else {
			mpx_buffer[i] +=
			((mpx-1)/2) * out_mono;
		}
	}

	return mpx_buffer_len;
}


int fm_mpx_close() {
	if(sf_close(inf) ) {
		fprintf(stderr, "Error closing audio file");
	}

	if(audio_buffer != NULL) free(audio_buffer);
	if(soxr != NULL) soxr_delete(soxr);

	return 0;
}
