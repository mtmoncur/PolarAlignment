//
// SPDX-FileCopyrightText: Copyright 2023 Arm Limited and/or its affiliates <open-source-office@arm.com>
// SPDX-License-Identifier: MIT
//

#include <stdio.h>

#include "pico/stdlib.h"
#include "hm01b0.h"
#include "LCD_Test.h"
#include "LCD_1in3.h"

#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 120

const struct hm01b0_config hm01b0_config = {
    .i2c           = i2c1,//PICO_DEFAULT_I2C_INSTANCE(),
    .sda_pin       = 14,//PICO_DEFAULT_I2C_SDA_PIN,
    .scl_pin       = 15,//PICO_DEFAULT_I2C_SCL_PIN,

    .vsync_pin     = 19,//6,
    .hsync_pin     = 18,//7,
    .pclk_pin      = 16,//8,
    .data_pin_base = 17,//9,
    .data_bits     = 1,
    .pio           = pio0,
    .pio_sm        = 0,
    .reset_pin     = -1,   // Not connected
    .mclk_pin      = -1,   // Not connected

    .width         = SCREEN_WIDTH,
    .height        = SCREEN_HEIGHT,
};

void convert(uint8_t* buffer, size_t length) {
    for (int i=length-1; i>=0; i--) {
        const uint8_t orig = buffer[i];
        const uint16_t red = (orig>>3) << 11;
        const uint16_t green = (orig>>2) << 5;
        const uint16_t blue = (orig>>3);
        const uint16_t rgb = red|green|blue;
        buffer[2*i+1] = rgb >> 8;
        buffer[2*i] = rgb&0xff;
    }
}

uint8_t pixels[SCREEN_WIDTH*SCREEN_HEIGHT*2];

int CameraDisplay()
{
    stdio_init_all();
    DEV_Delay_ms(5000);
    printf("Hello, I'm the pico!\n");

    // Initialize the camera
    if (hm01b0_init(&hm01b0_config) != 0) {
        printf("failed to initialize camera!\n");

        while (1) { tight_loop_contents(); }
    }
    DEV_Delay_ms(5000);

    printf("Test read\n");
    hm01b0_read_frame(pixels, SCREEN_WIDTH*SCREEN_HEIGHT);

    DEV_Delay_ms(500);
    printf("LCD_1in3_test Demo\r\n");
    if(DEV_Module_Init()!=0){
        return -1;
    }

    hm01b0_read_frame(pixels, SCREEN_WIDTH*SCREEN_HEIGHT);

    DEV_SET_PWM(50);
    /* LCD Init */
    printf("1.3inch LCD demo...\r\n");
    LCD_1IN3_Init(HORIZONTAL);
    LCD_1IN3_Clear(WHITE);
    
    //LCD_SetBacklight(1023);
    UDOUBLE Imagesize = LCD_1IN3_HEIGHT*LCD_1IN3_WIDTH*2;
    UWORD *BlackImage;
    if((BlackImage = (UWORD *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        exit(0);
    }
    // /*1.Create a new image cache named IMAGE_RGB and fill it with white*/
    Paint_NewImage((UBYTE *)BlackImage,LCD_1IN3.WIDTH,LCD_1IN3.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_Clear(WHITE);
    Paint_SetRotate(ROTATE_0);
    Paint_Clear(WHITE);

#if 1
    // /*3.Refresh the picture in RAM to LCD*/
    // LCD_1IN3_Display(BlackImage);
    DEV_Delay_ms(2000);

    Paint_DrawImage(gImage_1inch3_1,0,0,240,240);
    LCD_1IN3_Display(BlackImage);
    DEV_Delay_ms(5000);
#endif

#if 1

    // initialize stdio and wait for USB CDC connect
    // while (!stdio_usb_connected()) {
    //     tight_loop_contents();
    // }
    // printf("Hello, I'm the pico!");

    // // Initialize the camera
    // if (hm01b0_init(&hm01b0_config) != 0) {
    //     printf("failed to initialize camera!\n");

    //     while (1) { tight_loop_contents(); }
    // }

    // optional, set course integration time in number of lines: 2 to 0xFFFF
    // this controls the exposure
    //
    // hm01b0_set_coarse_integration(2);

    // uint8_t pixels[SCREEN_WIDTH*SCREEN_HEIGHT*2];

    convert(pixels, SCREEN_WIDTH*SCREEN_HEIGHT);

    int count = 0;
    while (true) {
        printf("Start loop %d\n", count);
        // Read frame from camera
        hm01b0_read_frame(pixels, SCREEN_WIDTH*SCREEN_HEIGHT);
        // hm01b0_read_frame(pixels, sizeof(pixels));

        printf("Read camera\n");

        convert(pixels, SCREEN_WIDTH*SCREEN_HEIGHT);

        printf("converted pixels\n");

        Paint_DrawImage((unsigned char*)pixels,0,0,SCREEN_WIDTH,SCREEN_HEIGHT);
        LCD_1IN3_Display(BlackImage);
        printf("painted camera image\n");
        DEV_Delay_ms(2000);
        // sleep_ms(500);


        Paint_DrawImage(gImage_1inch3_1,0,0,240,240);
        LCD_1IN3_Display(BlackImage);
        printf("painted saved image\n");
        DEV_Delay_ms(2000);

        count++;
    }

    /* Module Exit */
    free(BlackImage);
    BlackImage = NULL;
    
    
    DEV_Module_Exit();
#endif
}
