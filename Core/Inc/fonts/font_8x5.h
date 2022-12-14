/*
 * font_8x5.h
 *
 *  The MIT License.
 *  Created on: 25.05.2017
 *      Author: Mateusz Salamon
 *      www.msalamon.pl
 *      mateusz@msalamon.pl
 */

#ifndef FONT_8X5_H_
#define FONT_8X5_H_

// Font definition

const uint8_t font_5x4[] =
{
		5,4,
		0x00,0x00,0x00,0x00,	//
		0x00,0x00,0x00,0x0b,	//!
		0x00,0x03,0x00,0x03,	//"
		0x00,0x0f,0x06,0x0f, 	//#
		0x00,0x0b,0x0b,0x05,	//$

		0x00,0x00,0x00,0x00, //%
		0x00,0x00,0x00,0x00, //&
		0x00,0x00,0x00,0x00, //'
		0x00,0x00,0x00,0x00, //(
		0x00,0x00,0x00,0x00, //)
		0x00,0x00,0x00,0x00, //*
		0x00,0x00,0x00,0x00, //+
		0x00,0x00,0x00,0x00, //,
		0x00,0x00,0x00,0x00, //-
		0x00,0x00,0x00,0x00, //.
		0x00,0x00,0x00,0x00, // /

		0x1f,0x11,0x11,0x1f, //0
		0x04,0x02,0x01,0x1f, //1
		0x09,0x15,0x15,0x13, //2
		0x11,0x15,0x15,0x1f, //3
		0x07,0x04,0x04,0x1f, //4
		0x13,0x15,0x15,0x09, //5
		0x1f,0x15,0x15,0x1d, //6
		0x11,0x09,0x05,0x03, //7
		0x1f,0x15,0x15,0x1f, //8
		0x17,0x15,0x15,0x1f, //9
		0x00,0x00,0x1b,0x00, //:
		0x00,0x00,0x1b,0x00, //;
		0x00,0x00,0x00,0x00, //<
		0x00,0x00,0x00,0x00, //=
		0x00,0x00,0x00,0x00, //>
		0x00,0x00,0x00,0x00, //?
		0x00,0x00,0x00,0x00, //@
		0x1e,0x05,0x05,0x1e, //A
		0x1f,0x15,0x15,0x0a, //B
		0x1f,0x11,0x11,0x11, //C
		0x1f,0x11,0x11,0x0e, //D
		0x1f,0x15,0x15,0x11, //E
		0x1f,0x05,0x05,0x01, //F
		0x1f,0x11,0x15,0x19, //G
		0x1f,0x04,0x04,0x1f, //H
		0x11,0x1f,0x1f,0x11, //I
		0x09,0x11,0x11,0x0f, //J
		0x1f,0x04,0x0a,0x11, //K
		0x1f,0x10,0x10,0x10, //L
		0x1f,0x06,0x06,0x1f, //M
		0x1f,0x03,0x0c,0x1f, //N
		0x1f,0x11,0x11,0x1f, //O
		0x1f,0x05,0x05,0x02, //P
		0x1f,0x09,0x11,0x1f, //Q
		0x1f,0x05,0x0d,0x12, //R
		0x12,0x15,0x15,0x09, //S
		0x01,0x1f,0x1f,0x01, //T
		0x1f,0x10,0x10,0x1f, //U
		0x0f,0x10,0x10,0x0f, //V
		0x1f,0x0c,0x0c,0x1f, //W
		0x1b,0x04,0x04,0x1b, //X
		0x03,0x1c,0x1c,0x03, //Y
		0x19,0x15,0x15,0x13, //Z
		0x1f,0x11,0x00,0x00, //[

};
const uint8_t font_8x5[] =
{
			8, 5, //height, width
			0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x5F, 0x00, 0x00,
			0x00, 0x07, 0x00, 0x07, 0x00,
			//0x14, 0x7F, 0x14, 0x7F, 0x14,//#
			//0x24, 0x2A, 0x7F, 0x2A, 0x12,//$

			0x3c,0x42,0x42,0x42,0x00,
			0x26,0x16,0x68,0x64,0x00,


			0x23, 0x13, 0x08, 0x64, 0x62,//%
			0x36, 0x49, 0x56, 0x20, 0x50,//&
			0x00, 0x08, 0x07, 0x03, 0x00,//
			0x00, 0x1C, 0x22, 0x41, 0x00,//
			0x00, 0x41, 0x22, 0x1C, 0x00,//
			0x2A, 0x1C, 0x7F, 0x1C, 0x2A,// *
			0x08, 0x08, 0x3E, 0x08, 0x08,// +
			0x00, 0x80, 0x70, 0x30, 0x00,// ,
			0x08, 0x08, 0x08, 0x08, 0x08,// _
			0x00, 0x00, 0x60, 0x60, 0x00,// .
			0x20, 0x10, 0x08, 0x04, 0x02,// /
			0x3E, 0x51, 0x49, 0x45, 0x3E,//0
			0x00, 0x42, 0x7F, 0x40, 0x00,//1
			0x72, 0x49, 0x49, 0x49, 0x46,//2
			0x21, 0x41, 0x49, 0x4D, 0x33,//3
			0x18, 0x14, 0x12, 0x7F, 0x10,//4
			0x27, 0x45, 0x45, 0x45, 0x39,//4
			0x3C, 0x4A, 0x49, 0x49, 0x31,//6
			0x41, 0x21, 0x11, 0x09, 0x07,//7
			0x36, 0x49, 0x49, 0x49, 0x36,//8
			0x46, 0x49, 0x49, 0x29, 0x1E,//9
			0x00, 0x00, 0x14, 0x00, 0x00,//:
			0x00, 0x40, 0x34, 0x00, 0x00,//;
			0x00, 0x08, 0x14, 0x22, 0x41,//<
			0x14, 0x14, 0x14, 0x14, 0x14,//=
			0x00, 0x41, 0x22, 0x14, 0x08,//>
			0x02, 0x01, 0x59, 0x09, 0x06,//?
			0x3E, 0x41, 0x5D, 0x59, 0x4E,//@
			0x7C, 0x12, 0x11, 0x12, 0x7C,//A
			0x7F, 0x49, 0x49, 0x49, 0x36,//B
			//0x3E, 0x41, 0x41, 0x41, 0x22,//C
			0x3c, 0x42, 0x42, 0x42, 0x00,
			0x7F, 0x41, 0x41, 0x41, 0x3E,//D
			0x7F, 0x49, 0x49, 0x49, 0x41,//E
			0x7F, 0x09, 0x09, 0x09, 0x01,
			0x3E, 0x41, 0x41, 0x51, 0x73,
			0x7F, 0x08, 0x08, 0x08, 0x7F,
			0x00, 0x41, 0x7F, 0x41, 0x00,
			0x20, 0x40, 0x41, 0x3F, 0x01,
			0x7F, 0x08, 0x14, 0x22, 0x41,
			0x7F, 0x40, 0x40, 0x40, 0x40,
			0x7F, 0x02, 0x1C, 0x02, 0x7F,
			0x7F, 0x04, 0x08, 0x10, 0x7F,
			0x3E, 0x41, 0x41, 0x41, 0x3E,
			0x7F, 0x09, 0x09, 0x09, 0x06,
			0x3E, 0x41, 0x51, 0x21, 0x5E,
			0x7F, 0x09, 0x19, 0x29, 0x46,
			0x26, 0x49, 0x49, 0x49, 0x32,
			0x03, 0x01, 0x7F, 0x01, 0x03,
			0x3F, 0x40, 0x40, 0x40, 0x3F,
			0x1F, 0x20, 0x40, 0x20, 0x1F,
			0x3F, 0x40, 0x38, 0x40, 0x3F,
			0x63, 0x14, 0x08, 0x14, 0x63,
			0x03, 0x04, 0x78, 0x04, 0x03,
			0x61, 0x59, 0x49, 0x4D, 0x43,
			0x00, 0x7F, 0x41, 0x41, 0x41,
			0x02, 0x04, 0x08, 0x10, 0x20,
			0x00, 0x41, 0x41, 0x41, 0x7F,
			0x04, 0x02, 0x01, 0x02, 0x04,
			0x40, 0x40, 0x40, 0x40, 0x40,
			0x00, 0x03, 0x07, 0x08, 0x00,
			0x20, 0x54, 0x54, 0x78, 0x40,
			0x7F, 0x28, 0x44, 0x44, 0x38,
			0x38, 0x44, 0x44, 0x44, 0x28,
			0x38, 0x44, 0x44, 0x28, 0x7F,
			0x38, 0x54, 0x54, 0x54, 0x18,
			0x00, 0x08, 0x7E, 0x09, 0x02,
			0x18, 0xA4, 0xA4, 0x9C, 0x78,
			0x7F, 0x08, 0x04, 0x04, 0x78,
			0x00, 0x44, 0x7D, 0x40, 0x00,
			0x20, 0x40, 0x40, 0x3D, 0x00,
			0x7F, 0x10, 0x28, 0x44, 0x00,
			0x00, 0x41, 0x7F, 0x40, 0x00,
			0x7C, 0x04, 0x78, 0x04, 0x78,
			0x7C, 0x08, 0x04, 0x04, 0x78,
			0x38, 0x44, 0x44, 0x44, 0x38,
			0xFC, 0x18, 0x24, 0x24, 0x18,
			0x18, 0x24, 0x24, 0x18, 0xFC,
			0x7C, 0x08, 0x04, 0x04, 0x08,
			0x48, 0x54, 0x54, 0x54, 0x24,
			0x04, 0x04, 0x3F, 0x44, 0x24,
			0x3C, 0x40, 0x40, 0x20, 0x7C,
			0x1C, 0x20, 0x40, 0x20, 0x1C,
			0x3C, 0x40, 0x30, 0x40, 0x3C,
			0x44, 0x28, 0x10, 0x28, 0x44,
			0x4C, 0x90, 0x90, 0x90, 0x7C,
			0x44, 0x64, 0x54, 0x4C, 0x44,
			0x00, 0x08, 0x36, 0x41, 0x00,
			0x00, 0x00, 0x77, 0x00, 0x00,
			0x00, 0x41, 0x36, 0x08, 0x00,
			0x02, 0x01, 0x02, 0x04, 0x02,
};

#endif /* FONT_8X5_H_ */
