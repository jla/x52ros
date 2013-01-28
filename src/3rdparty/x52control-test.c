#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>


#include "x52interface.h"

int main(int argc, char* argv[])
{
    int32_t res = x52i_open_device();

    x52i_set_led(x52i_led_A_red);

    x52i_clr_led(x52i_led_A_green);

    x52i_set_text(x52i_text_line1,"HOLA!");
    x52i_commit();
    getchar();
    x52i_close_device();
}
