//
// SPDX-FileCopyrightText: Copyright 2023 Arm Limited and/or its affiliates <open-source-office@arm.com>
// SPDX-License-Identifier: MIT
//

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hm01b0.h"
#include "LCD_Test.h"
#include "LCD_1in3.h"

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

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

void convertAndScale(uint8_t* buffer, size_t wd_in, size_t ht_in, size_t wd_out, size_t ht_out) {
    // accept any image size, and scales by 2x
    for (int j=ht_in-1; j>=0; j--) {
        if (2*j >= ht_out) continue;
        for (int i=wd_in-1; i>=0; i--) {
            if (2*i >= wd_out) continue;
            const size_t loc = j*wd_in + i;
            const size_t locX2_1 = 2*j*wd_out + 2*i;
            const size_t locX2_2 = locX2_1 + 1;
            const size_t locX2_3 = locX2_1 + wd_out;
            const size_t locX2_4 = locX2_1 + wd_out + 1;
            if (2*locX2_4+1 >= 240*240*2) continue;
            const uint8_t orig = buffer[loc];
            const uint16_t red = (orig>>3) << 11;
            const uint16_t green = (orig>>2) << 5;
            const uint16_t blue = (orig>>3);
            const uint16_t rgb = red|green|blue;

            const uint8_t upper = rgb >> 8;
            const uint8_t lower = rgb&0xff;
            buffer[2*locX2_1] = lower;
            buffer[2*locX2_2] = lower;
            buffer[2*locX2_3] = lower;
            buffer[2*locX2_4] = lower;

            buffer[2*locX2_1+1] = upper;
            buffer[2*locX2_2+1] = upper;
            buffer[2*locX2_3+1] = upper;
            buffer[2*locX2_4+1] = upper;
        }
    }
}

void convertAndClip(uint8_t* buffer, size_t wd_in, size_t ht_in, size_t wd_out, size_t ht_out) {
    // accept any image size, and clips to less than 240x240
    for (int j=ht_in-1; j>=0; j--) {
        if (j >= ht_out) continue;
        for (int i=wd_in-1; i>=0; i--) {
            if (i >= wd_out) continue;
            const size_t loc = j*wd_in + i;
            const size_t loc2 = j*wd_out + i;

            if (loc2+1 >= 240*240*2) continue;
            const uint8_t orig = buffer[loc];
            const uint16_t red = (orig>>3) << 11;
            const uint16_t green = (orig>>2) << 5;
            const uint16_t blue = (orig>>3);
            const uint16_t rgb = red|green|blue;

            buffer[2*loc2] = rgb&0xff;
            buffer[2*loc2+1] = rgb >> 8;
        }
    }
}

void convert(uint8_t* buffer, size_t length) {
    for (int i=length-1; i>=0; i--) {
        const uint8_t orig = buffer[i];
        // const uint16_t red = (orig>>3) << 11;
        // const uint16_t green = (orig>>2) << 5;
        const uint16_t blue = (orig>>3);
        // const uint16_t rgb = red|green|blue;

        // const uint8_t upper = rgb >> 8;
        // const uint8_t lower = rgb&0xff;
        buffer[2*i+1] = 0;//upper;
        buffer[2*i] = blue;//lower;
    }
}

// uint8_t pixels[SCREEN_WIDTH*SCREEN_HEIGHT*2*2*2];
uint8_t pixels[240*240*2];
uint8_t pixels2[240*240*2];

int CameraDisplay()
{
    stdio_init_all();
    DEV_Delay_ms(100);
    printf("Hello, I'm the pico!\n");

    // Initialize the camera
    if (hm01b0_init(&hm01b0_config) != 0) {
        printf("failed to initialize camera!\n");

        while (1) { tight_loop_contents(); }
    }

    printf("Test read\n");
    hm01b0_read_frame(pixels, SCREEN_WIDTH*SCREEN_HEIGHT);

    DEV_Delay_ms(100);
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
    DEV_Delay_ms(100);

    Paint_DrawImage(gImage_1inch3_1,0,0,240,240);
    LCD_1IN3_Display(BlackImage);
    DEV_Delay_ms(100);
#endif

    hm01b0_set_coarse_integration(1000);

    uint8_t* pixelsPtr = pixels;
    uint8_t* pixels2Ptr = pixels2;

    int count = 0;
    printf("Start loop %d\n", count);
    static const int SIZE = 16;
    uint32_t times[SIZE];
    while (true) {
        const uint32_t curMs = to_ms_since_boot(get_absolute_time());
        const uint32_t prevMs = times[count%SIZE];
        const uint32_t diffMs = (curMs > prevMs) ? curMs - prevMs : 0;
        times[count%SIZE] = curMs;

        // Read frame from camera
        hm01b0_read_frame(pixelsPtr, SCREEN_WIDTH*SCREEN_HEIGHT);
        // hm01b0_read_frame(pixelsPtr, sizeof(pixelsPtr));

        #if 1
        convertAndClip(pixelsPtr, SCREEN_WIDTH, SCREEN_HEIGHT, 240, 240);
        // convertAndScale(pixelsPtr, SCREEN_WIDTH, SCREEN_HEIGHT, 240, 240);
        // printf("converted pixelsPtr\n");
        Paint_DrawImage((unsigned char*)pixelsPtr,0,0,240,240);
        #else
        convert(pixelsPtr, SCREEN_WIDTH*SCREEN_HEIGHT);
        // printf("converted pixelsPtr\n");
        Paint_DrawImage((unsigned char*)pixelsPtr,0,0,SCREEN_WIDTH,SCREEN_HEIGHT);
        #endif

        double diffMs_db = diffMs;
        diffMs_db /= SIZE;
        // Paint_DrawNum (20, 160 , curMs, &Font20,3,  WHITE,  BLACK);
        // Paint_DrawNum (20, 180 , prevMs, &Font20,3,  WHITE,  BLACK);
        Paint_DrawNum (20, 200 , diffMs_db, &Font20,3,  WHITE,  BLACK);

        LCD_1IN3_Display(BlackImage);

        uint8_t* tmp = pixelsPtr;
        pixelsPtr = pixels2Ptr;
        pixels2Ptr = tmp;

        count++;
    }

    /* Module Exit */
    free(BlackImage);
    BlackImage = NULL;

    DEV_Module_Exit();
    hm01b0_deinit();
}
