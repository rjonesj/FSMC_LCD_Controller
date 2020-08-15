/*
 * ColorCycle.h
 *
 *  Created on: Aug 5, 2020
 *      Author: rjonesj
 */

#ifndef INC_COLORCYCLE_H_
#define INC_COLORCYCLE_H_

typedef unsigned char BYTE; //define an "integer" that only stores 0-255 value

typedef struct _CRGB //Define a struct to store the 3 color values
{
    BYTE r;
    BYTE g;
    BYTE b;
    BYTE r2;
    BYTE g2;
    BYTE b2;
}CRGB;

CRGB TransformH(const CRGB *in, const float fHue);

#endif /* INC_COLORCYCLE_H_ */
