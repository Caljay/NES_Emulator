//
//

#include "nes_debug.h"
#include "../Bus.h"
#include "../6502cpu.h"

#include <stdio.h>
#include <stdlib.h>

#include "raylib.h"

void init_debug_window() {
    InitWindow(720*2, 480*2, "Emulator");
    SetTargetFPS(60);
}

//currently all windows are debug windows
int write_nes_debug(struct cpu* cpu) {

if (WindowShouldClose()) {
    CloseWindow();
    return 1;
}

    BeginDrawing();
    ClearBackground(BLACK);
    char* format = malloc(sizeof(char) * 250);
    sprintf(format, "A: 0x%04x [%d]\nX: 0x%04x [%d]\nY: 0x%04x [%d]\nPC: 0x%04x\nSP: 0x%04x\nStatus: %b" , cpu->a, cpu->a, cpu->x, cpu->x, cpu->y, cpu->y, cpu->pc, cpu->sp, cpu->status);
    DrawText(format, 1200, 0, 28, WHITE);
    Image target = GenImageColor(32, 32, WHITE);

    for (int i = 0; i < 32; i++) {
        for (int j = 0; j < 32; j++) {
            ImageDrawPixel(&target, i,j, i % 2 == 0 ? (j%3 == 0 ? RED : WHITE) : (i % 3 == 0 ? BLUE : WHITE));

        }
    }






    ExportImage(target, "picture.jpg");
    DrawTextureEx(LoadTextureFromImage(target), (Vector2){250, 250},0, 20, WHITE);
    //target = LoadImageFromTexture()









    EndDrawing();
    return 0;


}




