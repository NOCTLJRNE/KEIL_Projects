/*
  main.cpp
  2014 Copyright (c) Seeed Technology Inc.  All right reserved.

  Author:lawliet zou(lawliet.zou@gmail.com)
  2014-02-17

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "mbed.h"
#include "SeeedStudioTFTv2.h"
#include "Arial12x12.h"
#include "Arial24x23.h"
#include "Arial28x28.h"
#include "font_big.h"

#define PIN_XP          A3
#define PIN_XM          A1
#define PIN_YP          A2
#define PIN_YM          A0
#define PIN_MOSI        D11
#define PIN_MISO        D12
#define PIN_SCLK        D13
#define PIN_CS_TFT      D5
#define PIN_DC_TFT      D6
#define PIN_BL_TFT      D7
#define PIN_CS_SD       D4

SeeedStudioTFTv2 TFT(PIN_XP, PIN_XM, PIN_YP, PIN_YM, PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS_TFT, PIN_DC_TFT, PIN_BL_TFT, PIN_CS_SD);

int main()
{
    //Configure the display driver
    TFT.background(Black);
    TFT.foreground(White);
    TFT.cls();

    //Print a welcome message
    TFT.set_font((unsigned char*) Arial12x12);
    TFT.locate(0,0);
    TFT.printf("Hello Mbed");

    //Wait for 5 seconds
    wait(5.0);

    //Draw some graphics
    TFT.cls();
    TFT.set_font((unsigned char*) Arial24x23);
    TFT.locate(100,100);
    TFT.printf("Graphic");

    TFT.line(0,0,100,0,Green);
    TFT.line(0,0,0,200,Green);
    TFT.line(0,0,100,200,Green);

    TFT.rect(100,50,150,100,Red);
    TFT.fillrect(180,25,220,70,Blue);

    TFT.circle(80,150,33,White);
    TFT.fillcircle(160,190,20,Yellow);

    double s;
    for (int i = 0; i < 320; i++) {
        s = 20 * sin((long double)i / 10);
        TFT.pixel(i, 100 + (int)s, Red);
    }

    //Wait for 5 seconds
    wait(5.0);

    //Multiple fonts
    TFT.foreground(White);
    TFT.background(Blue);
    TFT.cls();
    TFT.set_font((unsigned char*) Arial24x23);
    TFT.locate(0,0);
    TFT.printf("Different Fonts:");
    TFT.set_font((unsigned char*) Neu42x35);
    TFT.locate(0,30);
    TFT.printf("Hello Mbed 1");
    TFT.set_font((unsigned char*) Arial24x23);
    TFT.locate(20,80);
    TFT.printf("Hello Mbed 2");
    TFT.set_font((unsigned char*) Arial12x12);
    TFT.locate(35,120);
    TFT.printf("Hello Mbed 3");
}
