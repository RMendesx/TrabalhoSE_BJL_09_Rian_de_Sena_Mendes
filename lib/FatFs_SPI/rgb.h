#ifndef RGB_H
#define RGB_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <string.h>

#define LED_GREEN_PIN 11 // Pino do LED verde
#define LED_BLUE_PIN 12  // Pino do LED azul
#define LED_RED_PIN 13   // Pino do LED vermelho

void rgb_init()
{
    // Configuração do LED verde
    gpio_init(LED_GREEN_PIN);              // Inicializa o pino do LED verde
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT); // Configura o pino como saída
    gpio_put(LED_GREEN_PIN, false);        // Desliga o LED verde

    // Configuração do LED azul
    gpio_init(LED_BLUE_PIN);              // Inicializa o pino do LED azul
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT); // Configura o pino como saída
    gpio_put(LED_BLUE_PIN, false);        // Desliga o LED azul

    // Configuração do LED vermelho
    gpio_init(LED_RED_PIN);              // Inicializa o pino do LED vermelho
    gpio_set_dir(LED_RED_PIN, GPIO_OUT); // Configura o pino como saída
    gpio_put(LED_RED_PIN, false);        // Desliga o LED vermelho
}

void set_rgb(bool red, bool green, bool blue)
{
    gpio_put(LED_RED_PIN, red);   // Define o estado do LED vermelho
    gpio_put(LED_GREEN_PIN, green); // Define o estado do LED verde
    gpio_put(LED_BLUE_PIN, blue);  // Define o estado do LED azul
}

void rgb_set_color(char *cor)
{
    if (strcmp(cor, "vermelho") == 0)
        set_rgb(1, 0, 0);
    else if (strcmp(cor, "verde") == 0)
        set_rgb(0, 1, 0);
    else if (strcmp(cor, "azul") == 0)
        set_rgb(0, 0, 1);
    else if (strcmp(cor, "amarelo") == 0)
        set_rgb(1, 1, 0);
    else if (strcmp(cor, "ciano") == 0)
        set_rgb(0, 1, 1);
    else if (strcmp(cor, "magenta") == 0)
        set_rgb(1, 0, 1);
    else if (strcmp(cor, "branco") == 0)
        set_rgb(1, 1, 1);
    else
        set_rgb(0, 0, 0); // Desliga tudo se não reconhecer a cor
}

void blinking_rgb(int times, int delay_ms, char *cor)
{
    for (int i = 0; i < times; i++)
    {
        rgb_set_color(cor);
        sleep_ms(delay_ms);
        set_rgb(0, 0, 0); // Desliga os LEDs
        sleep_ms(delay_ms);
    }
}

#endif