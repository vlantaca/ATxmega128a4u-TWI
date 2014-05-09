/* **************************************
 * File: sensor_data.c
 * Author: Michael Bennett
 * Purpose: Convert incoming GE ChipCap 2 Humidity and Temprature Sensor TWI
 *          comunication into its respective values.
 * ***************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(void) {
	int readData[4] = {57,97,204,18};

	int rawHumidData[2];
	int rawTempData[2];
	float humidData = 0;
	float tempData = 0;
	int i, j;
	int checkSum = 0;
	unsigned char mask = 0x01;

	if (!(readData[3] & 0xC0)) {
		// Grab humidity and temp data
		for (j=0; j < 6; j++) {
			rawHumidData[1] += readData[3] & (mask << j);
			rawTempData[0] += readData[0] & (mask << (j+2));
		}
		rawHumidData[0] = readData[2];
		rawTempData[1] = readData[1];

		humidData = (rawHumidData[1]*256 + rawHumidData[0])/pow(2,14) * 100;
		tempData = (rawTempData[1]*64 + rawTempData[0]/4)/pow(2,14) * 165 -40;

		printf("\nExpected- \n");
		printf("Humidity:   29.37 RH\n");
		printf("Temprature: 22.70 C\n\n");
		printf("Calculated- \n");
		printf("Humidity:   %f RH\n", humidData);
		printf("Temprature: %f C\n\n", tempData);
	} else
		printf("\nBAD DATA\n\n");

	return 0;
}
